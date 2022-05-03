#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::Extensions;
use panic_probe as _;

use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::dynpin::DynPin,
    rosc::RingOscillator,
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use st7735_lcd::Orientation;
use chip8::Chip8;
use chip8::keypad::KeyPad;
use chip8::roms::testroms;
use chip8::fonts::fonts;

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

    let mut disp = st7735_lcd::ST7735::new(spi, dc, rst, true, false, 160, 128);

    disp.init(&mut delay).unwrap();
    disp.set_orientation(&Orientation::Landscape).unwrap();

    lcd_led.set_high().unwrap();

    let keypad = KeyPad::<DynPin, DynPin>::new(
        [
            pins.gpio26.into_push_pull_output().into(),
            pins.gpio22.into_push_pull_output().into(),
            pins.gpio21.into_push_pull_output().into(),
            pins.gpio20.into_push_pull_output().into(),
        ], 
        [
            pins.gpio19.into_pull_down_input().into(),
            pins.gpio18.into_pull_down_input().into(),
            pins.gpio17.into_pull_down_input().into(),
            pins.gpio16.into_pull_down_input().into(),
        ]
    );

    let rosc = RingOscillator::new(pac.ROSC);
    let rng = rosc.initialize();
    let mut chip8 = Chip8::new(disp, keypad, rng, false);
    chip8.load_program(testroms::IBM_LOGO);
    chip8.load_font(fonts::DEFAULT);

    loop {
        chip8.tick();
        info!("Opcode: {:x}", chip8.get_current_op());
        //info!("{}", chip8.get_pixels());
        delay.delay_ms(40);
        info!("Registers: {}", chip8.get_registers());
        //info!("PC: {}", chip8.get_program_counter());
        //info!("Index: {}", chip8.get_index());
    }
}
