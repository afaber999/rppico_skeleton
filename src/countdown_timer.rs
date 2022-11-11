use rp_pico::hal;


pub struct CountdownTimer {
    stop : u64,
}

impl CountdownTimer {
    pub fn new() -> Self {
        Self {
            stop : 0,
        }
    }

    pub fn start(&mut self, timer : &hal::Timer, duration_in_us: u64) {
        self.stop = timer.get_counter() + duration_in_us;
    }
    
    pub fn is_expired(&self, timer : &hal::Timer) -> bool {
        timer.get_counter() >= self.stop
    }
}
