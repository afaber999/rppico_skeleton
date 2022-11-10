
use alloc::boxed::Box;
use arrayvec::ArrayString;
use core::{alloc::Layout, cell::RefCell, fmt::Write, ops::DerefMut};
use log::{Record, LevelFilter};
use ringbuffer::{AllocRingBuffer, RingBufferWrite, RingBufferExt, RingBufferRead};
use usb_device::UsbError;


pub struct Logger {
    buffer: RefCell<AllocRingBuffer<u8>>,
    level : LevelFilter,
    strbuf: RefCell<ArrayString<128>>,
}

unsafe impl Sync for Logger {}
unsafe impl Send for Logger {}

impl Logger {
    pub fn new() -> Self {
        Self {
            buffer: RefCell::new(AllocRingBuffer::with_capacity(128)),
            strbuf: RefCell::new(ArrayString::<128>::new()),
            level : LevelFilter::Trace,
        }
    }

    // pub fn init(&'static self) {
    //     log::set_max_level(self.level);
    //     unsafe {
    //         log::set_logger_racy(Box::leak(Box::new(self))).unwrap();
    //     }
    // }

    pub fn write_to_serial(&self, writer: &mut dyn embedded_hal::serial::Write<u8, Error = UsbError>) {

        let mut rb_binding = self.buffer.borrow_mut();
        let rb = rb_binding.deref_mut();

        for bt in rb.drain() {
            writer.write(bt);
        }
    }
}

impl log::Log for Logger {
    fn enabled(&self, _metadata: &log::Metadata) -> bool {
        true
    }

    fn log(&self, record: &Record) {
        let mut strbuf_binding = self.strbuf.borrow_mut();
        let strbuf = strbuf_binding.deref_mut();
        strbuf.clear();
        //let mut strbuf = ArrayString::<128>::new();        
        drop(writeln!(strbuf, "Log::> {}", record.args()));

        let mut rb_binding = self.buffer.borrow_mut();
        let rb = rb_binding.deref_mut();

        for b in strbuf.as_bytes() {
            rb.push(*b);
        }
    }

    fn flush(&self) {}
}
