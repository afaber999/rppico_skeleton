// Basic skeleton for RP2040 development
// A.L. Faber (c) 2022
// License MIT

#![no_std]
#![no_main]

#![feature(alloc_error_handler)]
extern crate alloc;
use alloc_cortex_m::CortexMHeap;
use embedded_hal::digital::v2::{ToggleableOutputPin, OutputPin};
use core::alloc::Layout;

use alloc::boxed::Box;
use log::LevelFilter;
use rp_pico::entry;
use panic_halt as _;
use rp_pico::hal::pac;
use rp_pico::hal;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;
use rp_pico::hal::rom_data::reset_to_usb_boot;

pub mod logger;
use log::{info, warn, error, trace, debug};

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();
use core::mem::MaybeUninit;
const HEAP_SIZE: usize = 15 * 1024;
static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}

pub mod stopwatch;
use stopwatch::Stopwatch;

pub mod countdown_timer;
use countdown_timer::CountdownTimer;

#[entry]
fn main() -> ! {
    // setup heap
    unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }

    // setup logger
    let logger = logger::Logger::new();
    let logger = Box::leak(Box::new(logger));
    logger.set_log_level(LevelFilter::Trace);

    unsafe {
        log::set_logger_racy(logger).unwrap();
    }

    // setup minimal peripherals
    let mut pac = pac::Peripherals::take().unwrap();
    let cp = pac::CorePeripherals::take().unwrap();    
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // setup the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // get the delay object
    let mut delay = cortex_m::delay::Delay::new(cp.SYST, 125_000_000);

    // setup the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // create serial (CDC) USB device
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("NoName")
        .product("Nothing")
        .serial_number("11")
        .device_class(2)
        .build();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);
        
    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set the LED to be an output
    let mut led_pin = pins.led.into_push_pull_output();
    drop( led_pin.set_high());
    
    let mut said_hello = false;

    info!("Start the main loop");    
    let mut blink_timer = CountdownTimer::new();

    loop {

        if blink_timer.is_expired(&timer) {
            blink_timer.start(&timer, 1500_000);
            drop( led_pin.toggle());
        }

        if usb_dev.state() == UsbDeviceState::Configured {
            // display a welcome message at startup
            if !said_hello {
                said_hello = true;
                drop( serial.write(b"RP2040 pico skeleton version 0.1 !\r\n"));
    
            }
            logger.write_to_serial(&mut serial);
        }

        // check for incoming serial over USB data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {}
                Ok(0) => {}
                Ok(count) => {
                    info!("GOT SOMETHING");
                    for b_idx in 0..count { 
                        let ub = buf[b_idx];

                        match  ub as char  {
                            'a' | 'A' => {
                                drop( serial.write(b"Check: I'm alive !\r\n") );
                            },
                            'd' | 'D' => {
                                drop( serial.write(b"Check: delay !\r\n") );

                                let mut stopwatch = Stopwatch::new();
                                stopwatch.start( &timer );
                                delay.delay_us(100);
                                let delta = stopwatch.delta(&timer);
                                info!("Time check for 100us :{} us", delta);
                                
                                stopwatch.start( &timer );
                                delay.delay_ms(100);
                                let delta = stopwatch.delta(&timer);
                                info!("Time check for 100ms :{} us", delta);

                            },                             
                            'f' | 'F' => {
                                drop( serial.write(b"Boot to Flash mode!\r\n") );
                                reset_to_usb_boot(0,0);    
                            },   
                            'p' | 'P' => {
                                info!("this is an info, timer {}", timer.get_counter());
                                error!("this is an error, timer {}",timer.get_counter());
                                warn!("this is a warn, timer {}",timer.get_counter());
                                debug!("this is a debug, timer {}",timer.get_counter());
                                trace!("this is a trace, timer {}",timer.get_counter());
                                warn!("purge logs!!");
                            },
                            _=>{ 
                                error!("unknown command: {}", ub);
                            },
                        }
                    }
                }
            }
        }
    }
}
