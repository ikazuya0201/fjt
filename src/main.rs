#![no_std]
#![no_main]

use cortex_m_rt::entry;

extern crate panic_rtt;

#[entry]
fn main() -> ! {
    loop {}
}
