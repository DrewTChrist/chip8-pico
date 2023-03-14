#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::timer::{Cancel, CountDown};
use embedded_time::duration::Extensions;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::Extensions as RateExtensions;
use panic_probe as _;

use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::dynpin::DynPin,
    pac,
    rosc::RingOscillator,
    sio::Sio,
    timer::Timer,
    watchdog::Watchdog,
};

use chip8::fonts;
use chip8::keypad::KeyPad;
use chip8::Chip8;
use st7735_lcd::Orientation;

enum Rom<'a> {
    Bytes(&'a [u8]),
}

impl<'a> Rom<'a> {
    fn bytes(&self) -> &'a [u8] {
        let Rom::Bytes(b) = self;
        b
    }
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut countdown = timer.count_down();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio6.into_mode::<bsp::hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio7.into_mode::<bsp::hal::gpio::FunctionSpi>();
    let _spi_miso = pins.gpio4.into_mode::<bsp::hal::gpio::FunctionSpi>();
    let spi = bsp::hal::Spi::<_, _, 8>::new(pac.SPI0);

    let mut lcd_led = pins.gpio12.into_push_pull_output();
    let dc = pins.gpio13.into_push_pull_output();
    let rst = pins.gpio14.into_push_pull_output();

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    let mut display = st7735_lcd::ST7735::new(spi, dc, rst, true, false, 160, 128);

    display.init(&mut delay).unwrap();
    display.set_orientation(&Orientation::Landscape).unwrap();

    let keypad = KeyPad::<DynPin, DynPin>::new(
        [
            pins.gpio19.into_push_pull_output().into(),
            pins.gpio18.into_push_pull_output().into(),
            pins.gpio17.into_push_pull_output().into(),
            pins.gpio16.into_push_pull_output().into(),
        ],
        [
            pins.gpio26.into_pull_up_input().into(),
            pins.gpio22.into_pull_up_input().into(),
            pins.gpio21.into_pull_up_input().into(),
            pins.gpio20.into_pull_up_input().into(),
        ],
    );

    let rosc = RingOscillator::new(pac.ROSC);
    let rng = rosc.initialize();
    let mut chip8 = Chip8::new(display, keypad, rng, delay);
    let roms: [Rom; 3] = [
        Rom::Bytes(include_bytes!("roms/test_opcode.ch8")),
        Rom::Bytes(include_bytes!("roms/ibm_logo.ch8")),
        Rom::Bytes(include_bytes!("roms/breakout.ch8")),
    ];
    let program_index: usize = 0;
    chip8.load_program(roms[program_index].bytes());
    chip8.load_font(fonts::DEFAULT);
    chip8.set_scale((2, 4));
    chip8.set_padding(16);

    lcd_led.set_high().unwrap();

    loop {
        chip8.tick();
        countdown.start(5_u32.milliseconds());
        countdown.cancel().unwrap();
        let _ = nb::block!(countdown.wait());
    }
}
