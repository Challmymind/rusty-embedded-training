#![no_std]
#![no_main]

// Set up allocator
extern crate alloc;

use embedded_alloc::Heap;
#[global_allocator]
static HEAP: Heap = Heap::empty();

use panic_halt as _;
use rp2040_hal as hal;
use hal::pac;
// Import useful traits
use embedded_hal::blocking::delay::DelayMs;
use alloc::{vec, vec::Vec, boxed::Box};

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[rp2040_hal::entry]
fn main() -> ! {

    // Initlialize allocator
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe {HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE)};

    }

    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    ).ok().unwrap();

    let mut timer = rp2040_hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut counter = 0;

    let mut leds : Vec<Box<dyn embedded_hal::digital::v2::OutputPin<Error = hal::gpio::Error>>> = vec![
        Box::new(pins.gpio18.into_push_pull_output()),
        Box::new(pins.gpio19.into_push_pull_output()),
        Box::new(pins.gpio20.into_push_pull_output()),
        Box::new(pins.gpio21.into_push_pull_output())];

    loop {
        for (index, led) in leds.iter_mut().enumerate(){
            if ((counter >> index) & 1) == 1 {
                led.set_high().unwrap();
            }
            else {
                led.set_low().unwrap();
            }

        }
        counter += 1;

        timer.delay_ms(500);
    }
}
