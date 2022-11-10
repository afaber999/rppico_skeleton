// Basic skeleton for RP2040 development
// A.L. Faber (c) 2022
// License MIT

#![no_std]
#![no_main]

#![feature(alloc_error_handler)]
extern crate alloc;

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

//use cortex_m::prelude::_embedded_hal_blocking_delay_DelayUs;
use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout;
//use cortex_m::delay::Delay;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();
use core::mem::MaybeUninit;
const HEAP_SIZE: usize = 15 * 1024;
static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}

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

    // setup the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("NoName")
        .product("Nothing")
        .serial_number("11")
        .device_class(2)
        .build();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut said_hello = false;

    info!("Start the main loop");    

    loop {
        // display a welcome message at startup
        if !said_hello && timer.get_counter() >= 2_000_000 {
            said_hello = true;
            let _ = serial.write(b"RP skeleton version 0.1 !\r\n");
        }

        // check for incoming serial over USB data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {}
                Ok(0) => {}
                Ok(count) => {
                    for b_idx in 0..count { 
                        let ub = buf[b_idx];
                        //drop( serial.write(&ub) );

                        match  ub as char  {
                            'a' | 'A' => {
                                drop( serial.write(b"Check: I'm alive !\r\n") );
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
                            _=>{},
                        }
                    }
                }
            }
        }
        if usb_dev.state() == UsbDeviceState::Configured {
            // if !said_hello && timer.get_counter() >= 2_000_000 {
            //     said_hello = true;
            //     let _ = serial.write(b"RP skeleton version 0.1 !\r\n");
            // }
                // purge the logging
            logger.write_to_serial(&mut serial);
        }

    }
}
