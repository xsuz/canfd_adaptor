#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::mode::Async;
use embassy_stm32::spi::Spi;
use embassy_stm32::spi::mode::Master;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel};
use embassy_time::WithTimeout;
use mavlink::embedded::Write;
use panic_probe as _;

use mavlink;
use mavlink::dialects::swingby::MavMessage;
use mavlink::{MAVLinkV2MessageRaw, read_v2_raw_message_async};

use static_cell::StaticCell;

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::time::Hertz;
use embassy_stm32::{Config, bind_interrupts, can, dma, peripherals, spi, usart};
use embedded_sdmmc::SdCard;

use embedded_hal_bus::spi::ExclusiveDevice;

type MAVLinkSender<'ch> =
    embassy_sync::channel::DynamicSender<'ch, (embassy_time::Instant, MAVLinkV2MessageRaw)>;
type MAVLinkReceiver<'ch> =
    embassy_sync::channel::DynamicReceiver<'ch, (embassy_time::Instant, MAVLinkV2MessageRaw)>;

struct Publisher<'ch> {
    subscribers: [Option<MAVLinkSender<'ch>>; 5],
}

impl<'ch> Publisher<'ch> {
    pub const fn new() -> Self {
        let subscribers: [Option<MAVLinkSender<'ch>>; 5] = [None, None, None, None, None];
        Self { subscribers }
    }
    pub fn add_subscriber(&mut self, channel: MAVLinkSender<'ch>) -> Result<(), ()> {
        while let Some(pos) = self.subscribers.iter().position(|x| x.is_none()) {
            self.subscribers[pos] = Some(channel);
            return Ok(());
        }
        Err(())
    }
    pub async fn publish(
        &self,
        msg: MAVLinkV2MessageRaw,
        timestamp: embassy_time::Instant,
        timeout: embassy_time::Duration,
    ) {
        for sub in self.subscribers.iter() {
            if let Some(channel) = sub {
                let result = channel.send((timestamp, msg)).with_timeout(timeout).await;
                if let Err(_) = result {
                    defmt::error!("Failed to send message to subscriber");
                }
            }
        }
    }
}

bind_interrupts!(struct Irqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<peripherals::FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<peripherals::FDCAN1>;
    USART3 => usart::BufferedInterruptHandler<peripherals::USART3>;
    GPDMA1_CHANNEL0 => dma::InterruptHandler<peripherals::GPDMA1_CH0>;
    GPDMA1_CHANNEL1 => dma::InterruptHandler<peripherals::GPDMA1_CH1>;
});

struct DummyTimesource();

impl embedded_sdmmc::TimeSource for DummyTimesource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.pll1 = Some(Pll {
            source: PllSource::CSI, // 4MHz
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL108,
            divp: Some(PllDiv::DIV2), //216MHz
            divq: Some(PllDiv::DIV9), // 48MHz
            divr: None,
        });
        config.rcc.csi = true;
        config.rcc.mux.fdcan12sel = mux::Fdcansel::PLL1_Q;
        config.rcc.sys = Sysclk::PLL1_P; //
    }
    let p = embassy_stm32::init(config);

    let led = Output::new(p.PA5, Level::High, Speed::High);

    let mut config = usart::Config::default();
    config.baudrate = 115200;

    static RX_BUF: StaticCell<[u8; 2048]> = StaticCell::new();
    static TX_BUF: StaticCell<[u8; 512]> = StaticCell::new();
    let rx_buf = &mut RX_BUF.init([0u8; 2048])[..];
    let tx_buf = &mut TX_BUF.init([0u8; 512])[..];

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

    let (can_tx, can_rx, _props) = can.split();

    let mut config = spi::Config::default();
    config.frequency = Hertz::hz(400_000);
    let spi1: Spi<'_, Async, Master> = spi::Spi::new(
        p.SPI1,
        p.PC0,
        p.PC3,
        p.PC2,
        p.GPDMA1_CH0,
        p.GPDMA1_CH1,
        Irqs,
        config,
    );
    let cs1 = Output::new(p.PC1, Level::High, Speed::High);

    static UART_CHANNEL: Channel<
        ThreadModeRawMutex,
        (embassy_time::Instant, MAVLinkV2MessageRaw),
        10,
    > = Channel::new();
    static CAN_CHANNEL: Channel<
        ThreadModeRawMutex,
        (embassy_time::Instant, MAVLinkV2MessageRaw),
        50,
    > = Channel::new();
    static LOG_CHANNEL: Channel<
        ThreadModeRawMutex,
        (embassy_time::Instant, MAVLinkV2MessageRaw),
        10,
    > = Channel::new();

    let mut publisher_can_rx_task = Publisher::new();
    let mut publisher_uart_rx_task = Publisher::new();
    publisher_can_rx_task
        .add_subscriber(UART_CHANNEL.dyn_sender())
        .unwrap();
    publisher_can_rx_task
        .add_subscriber(LOG_CHANNEL.dyn_sender())
        .unwrap();
    publisher_uart_rx_task
        .add_subscriber(CAN_CHANNEL.dyn_sender())
        .unwrap();
    publisher_uart_rx_task
        .add_subscriber(LOG_CHANNEL.dyn_sender())
        .unwrap();

    spawner.spawn(unwrap!(can_rx_task(can_rx, led, publisher_can_rx_task)));
    spawner.spawn(unwrap!(uart2_rx_task(uart_rx, publisher_uart_rx_task)));
    spawner.spawn(unwrap!(uart2_tx_task(uart_tx, UART_CHANNEL.dyn_receiver())));
    spawner.spawn(unwrap!(can_tx_task(can_tx, CAN_CHANNEL.dyn_receiver())));
    spawner.spawn(unwrap!(sdmmc_task(spi1, cs1, LOG_CHANNEL.dyn_receiver())));

    loop {
        defmt::info!("Task running");
        embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
pub async fn can_rx_task(
    mut rx: can::CanRx<'static>,
    mut led: Output<'static>,
    publisher: Publisher<'static>,
) {
    let mut last_read_ts = embassy_time::Instant::now();
    led.set_low();
    loop {
        match rx.read_fd().await {
            Ok(envelope) => {
                let (ts, rx_frame) = (envelope.ts, envelope.frame);
                let delta = (ts - last_read_ts).as_millis();
                last_read_ts = ts;
                info!("CAN Rx: {} --- {}ms", rx_frame.header().len(), delta,);
                let mut buffer = rx_frame.data();
                let msg = read_v2_raw_message_async::<MavMessage>(&mut buffer).await;
                if let Ok(msg) = msg {
                    defmt::info!("CAN recieved raw message: msg_id={}", msg.message_id());
                    publisher
                        .publish(
                            msg,
                            embassy_time::Instant::now(),
                            embassy_time::Duration::from_millis(0),
                        )
                        .await;
                    led.toggle();
                }
            }
            Err(_err) => error!("Error in frame"),
        }
    }
}

#[embassy_executor::task]
pub async fn can_tx_task(mut tx: can::CanTx<'static>, receiver: MAVLinkReceiver<'static>) {
    // Main loop
    loop {
        let (_timestamp, msg) = receiver.receive().await;
        let raw_bytes = msg.raw_bytes();
        let mut buffer: [u8; 64] = [0u8; 64];
        if let Some(len) = pad_can_fd_payload(raw_bytes, &mut buffer) {
            let frame =
                can::frame::FdFrame::new_extended(msg.component_id() as u32, &buffer[..len])
                    .unwrap();
            _ = tx.write_fd(&frame).await;
        }
    }
}

#[embassy_executor::task]
pub async fn uart2_rx_task(mut rx: usart::BufferedUartRx<'static>, publisher: Publisher<'static>) {
    debug!("rx task started");

    loop {
        // Read raw message to reduce firmware flash size (using read_v2_msg_async will be add ~80KB because
        // all *_DATA::deser methods will be add to firmware).
        let raw = read_v2_raw_message_async::<MavMessage>(&mut rx).await;
        match raw {
            Ok(msg) => {
                defmt::info!("UART  recieved raw message: msg_id={}", msg.message_id());
                publisher
                    .publish(
                        msg,
                        embassy_time::Instant::now(),
                        embassy_time::Duration::from_millis(0),
                    )
                    .await;
            }
            Err(_) => {}
        }
    }
}

#[embassy_executor::task]
pub async fn uart2_tx_task(
    mut tx: usart::BufferedUartTx<'static>,
    receiver: MAVLinkReceiver<'static>,
) {
    // Main loop
    loop {
        let (_timestamp, msg) = receiver.receive().await;
        tx.write_all(msg.raw_bytes()).unwrap();
    }
}

#[embassy_executor::task]
pub async fn sdmmc_task(
    spi: Spi<'static, Async, Master>,
    cs: Output<'static>,
    receiver: MAVLinkReceiver<'static>,
) {
    let spi = ExclusiveDevice::new_no_delay(spi, cs);
    let sdcard = SdCard::new(spi, embassy_time::Delay);
    while let Err(_) = sdcard.num_bytes() {
        info!("Waiting for SD card initialization...");
        embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await;
    }
    info!("Card size is {} bytes", sdcard.num_bytes().unwrap());

    let volume_mgr = embedded_sdmmc::VolumeManager::new(sdcard, DummyTimesource {});
    let volume0 = volume_mgr
        .open_volume(embedded_sdmmc::VolumeIdx(0))
        .unwrap();
    let root_dir = volume0.open_root_dir().unwrap();
    let file = root_dir
        .open_file_in_dir(
            "12345678.log",
            embedded_sdmmc::Mode::ReadWriteCreateOrTruncate,
        )
        .unwrap();
    while !file.is_eof() {
        let mut buf = [0u8; 32];
        if let Ok(n) = file.read(&mut buf) {
            info!("{:a}", buf[..n]);
        } else {
            error!("error");
        }
    }
    let mut buffer = [0u8; 512];
    let mut pos = 0;

    loop {
        let (timestamp, message) = receiver.receive().await;

        let timestamp = timestamp.as_micros().to_be_bytes();
        let raw_bytes = message.raw_bytes();

        // timestamp と raw_bytes を順番に buffer へ書き込む
        for data in [&timestamp[..], raw_bytes] {
            let mut data_pos = 0;

            while data_pos < data.len() {
                let copy_len = (buffer.len() - pos).min(data.len() - data_pos);

                buffer[pos..pos + copy_len].copy_from_slice(&data[data_pos..data_pos + copy_len]);

                pos += copy_len;
                data_pos += copy_len;

                if pos == buffer.len() {
                    if let Err(_) = file.write(&buffer) {
                        error!("Failed to write to file");
                    } else {
                        if let Err(_) = file.flush() {
                            error!("Failed to flush file");
                        }else{
                            info!("Wrote {} bytes to file", buffer.len());
                        }
                    }

                    pos = 0;
                }
            }
        }
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
            error!("MAVLink packet is too large for CAN FD (len={})", len);
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
