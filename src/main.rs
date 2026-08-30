#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::{Config, bind_interrupts, can, peripherals, usart};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel};
use mavlink::embedded::Write;
use panic_probe as _;

use mavlink;
use mavlink::dialects::swingby::MavMessage;
use mavlink::{MAVLinkV2MessageRaw, read_v2_raw_message_async};

use static_cell::StaticCell;

static UART_TO_CAN: Channel<ThreadModeRawMutex, MAVLinkV2MessageRaw, 8> = Channel::new();
static CAN_TO_UART: Channel<ThreadModeRawMutex, MAVLinkV2MessageRaw, 8> = Channel::new();

bind_interrupts!(struct Irqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<peripherals::FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<peripherals::FDCAN1>;
    USART3 => usart::BufferedInterruptHandler<peripherals::USART3>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.pll1 = Some(Pll {
            source: PllSource::CSI,// 4MHz
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL108,
            divp: Some(PllDiv::DIV2),//216MHz
            divq: Some(PllDiv::DIV9),// 48MHz
            divr: None,
        });
        config.rcc.csi = true;
        config.rcc.mux.fdcan12sel = mux::Fdcansel::PLL1_Q;
        config.rcc.sys = Sysclk::PLL1_P;//
    }
    let p = embassy_stm32::init(config);

    let led = Output::new(p.PA5, Level::High, Speed::High);

    let mut config = usart::Config::default();
    config.baudrate = 115200;

    static RX_BUF : StaticCell<[u8;1024]>=StaticCell::new();
    static TX_BUF : StaticCell<[u8;1024]>=StaticCell::new();
    let rx_buf = &mut RX_BUF.init([0u8;1024])[..];
    let tx_buf = &mut TX_BUF.init([0u8;1024])[..];

    let uart3 =
        usart::BufferedUart::new(p.USART3, p.PA3, p.PA4, tx_buf, rx_buf, Irqs, config).unwrap();
    // Break serial in TX and RX (not used)
    let (uart_tx, uart_rx) = uart3.split();

    let mut can = can::CanConfigurator::new(p.FDCAN1, p.PC6, p.PC7, Irqs);

    can.properties().set_extended_filter(
        can::filter::ExtendedFilterSlot::_0,
        can::filter::ExtendedFilter::accept_all_into_fifo1(),
    );

    // nominal : 250k bps
    can.set_bitrate(250_000);
    // data : 1M bps
    can.set_fd_data_bitrate(1_000_000, false);

    // nominal : 250k bps
    can.set_bitrate(250_000);
    // data : 1M bps
    can.set_fd_data_bitrate(1_000_000, false);

    info!("Configured");

    let can = can.start(can::OperatingMode::NormalOperationMode);

    let (mut can_tx, can_rx, _props) = can.split();

    spawner.spawn(unwrap!(can_rx_task(can_rx, led)));
    spawner.spawn(unwrap!(uart2_rx_task(uart_rx)));
    spawner.spawn(unwrap!(uart2_tx_task(uart_tx)));
    // With split
    loop {
        let msg = UART_TO_CAN.receive().await;
        let raw_bytes = msg.raw_bytes();
        let mut buffer: [u8; 64] = [0u8; 64];
        if let Some(len) = pad_can_fd_payload(raw_bytes, &mut buffer) {
            let frame =
                can::frame::FdFrame::new_extended(msg.component_id() as u32, &buffer[..len])
                    .unwrap();
            _ = can_tx.write_fd(&frame).await;
        }
    }
}

#[embassy_executor::task]
pub async fn can_rx_task(mut rx: can::CanRx<'static>, mut led: Output<'static>) {
    let mut last_read_ts = embassy_time::Instant::now();
    led.set_low();
    loop {
        match rx.read_fd().await {
            Ok(envelope) => {
                let (ts, rx_frame) = (envelope.ts, envelope.frame);
                let delta = (ts - last_read_ts).as_millis();
                last_read_ts = ts;
                info!(
                    "CAN Rx: {} --- {}ms",
                    rx_frame.header().len(),
                    delta,
                );
                let mut buffer = rx_frame.data();
                let msg = read_v2_raw_message_async::<MavMessage>(&mut buffer).await;
                if let Ok(msg) = msg {
                    defmt::info!("CAN recieved raw message: msg_id={}", msg.message_id());
                    CAN_TO_UART.send(msg).await;
                    led.toggle();
                }
            }
            Err(_err) => error!("Error in frame"),
        }
    }
}

#[embassy_executor::task]
pub async fn uart2_rx_task(mut rx: usart::BufferedUartRx<'static>) {
    debug!("rx task started");

    loop {
        // Read raw message to reduce firmware flash size (using read_v2_msg_async will be add ~80KB because
        // all *_DATA::deser methods will be add to firmware).
        let raw = read_v2_raw_message_async::<MavMessage>(&mut rx).await;
        match raw {
            Ok(msg) => {
                defmt::info!("UART  recieved raw message: msg_id={}", msg.message_id());
                if !UART_TO_CAN.is_full(){
                    UART_TO_CAN.send(msg).await;
                }else{
                    error!("Channel for UART to CAN is full");
                }
            }
            Err(_) => {}
        }
    }
}

#[embassy_executor::task]
pub async fn uart2_tx_task(mut tx: usart::BufferedUartTx<'static>) {
    // Main loop
    loop {
        let msg = CAN_TO_UART.receive().await;
        tx.write_all(msg.raw_bytes()).unwrap();
    }
}

fn can_fd_len(len: usize) -> usize {
    match len {
        0..=8 => len,
        9..=12 => 12,
        13..=16 => 16,
        17..=20 => 20,
        21..=24 => 24,
        25..=32 => 32,
        33..=48 => 48,
        49..=64 => 64,
        _ => {
            error!("MAVLink packet is too large for CAN FD (len={})",len);
            0xFF
        }
    }
}

fn pad_can_fd_payload(data: &[u8], buffer: &mut [u8; 64]) -> Option<usize> {
    let len = can_fd_len(data.len());
    if len <= 64 {
        buffer[..len].fill(0);
        buffer[..data.len()].copy_from_slice(data);
        Some(len)
    } else {
        None
    }
}