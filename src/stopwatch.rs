use rp_pico::hal;


pub struct Stopwatch {
    start : u64,
}

impl Stopwatch {
    pub fn new() -> Self {
        Self {
            start : 0,
        }
    }

    pub fn start(&mut self, timer : &hal::Timer) {
        self.start = timer.get_counter();
    }
    
    pub fn delta(&self, timer : &hal::Timer) -> u64 {
        timer.get_counter() - self.start
    }
}

