//! # Philips XUS phantom
//!
//! Microcontroller code (RP2040) for the Philips XUS phantom 

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
use log::info;
use log::warn;

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
    unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }

    let logger = logger::Logger::new();
    let mut logger = Box::leak(Box::new(logger));

    log::set_max_level(LevelFilter::Trace);
    unsafe {
        log::set_logger_racy(logger).unwrap();
    }

    //logger.init();

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

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Philips")
        .product("XUS US Phantom")
        .serial_number("1111")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut said_hello = false;

    info!("Start loop");    

    loop {
        // A welcome message at the beginning
        if !said_hello && timer.get_counter() >= 2_000_000 {
            said_hello = true;
            let _ = serial.write(b"Philips XUS phantom 0.1 !\r\n");

      //      info!("LOGGGGGGGGGGGGGGGGGG \r\n");
       //     info!("MORE LOGGING \r\n");

        //    logger.write_to_serial(&mut serial);

        }

        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(0) => {
                    // Do nothing
                }
                Ok(count) => {
                    // Convert to upper case
                    buf.iter().for_each(|b| {
                        let mut ub = *b;
                        ub.make_ascii_uppercase();
                        match  ub as char  {
                            'A' => {
                                drop( serial.write(b"I'm alive !\r\n") );
                            },   
                            'F' => {
                                drop( serial.write(b"Boot to Flash mode!\r\n") );
                                reset_to_usb_boot(0,0);    
                            },   
                            'P' => {
                                warn!("warning error....");
                                warn!("purge logs!!");
                                drop( serial.write(b"PURGE LOGGING!\r\n") );

                                logger.write_to_serial(&mut serial);
                            },
                            _=>{},
                        }
                    });
                    // Send back to the host
                    let mut wr_ptr = &buf[..count];
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            // On error, just drop unwritten data.
                            // One possible error is Err(WouldBlock), meaning the USB
                            // write buffer is full.
                            Err(_) => break,
                        };
                    }
                }
            }
        }
    }
}

// End of file
