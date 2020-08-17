#![no_std]
#![no_main]

extern crate panic_rtt;
mod alias;
mod init;
mod macros;
mod sensors;

use cortex_m_rt::entry;
use once_cell::unsync::Lazy;
// use lazy_static::lazy_static;

use init::{init, Storage};

static mut STORAGE: Lazy<Storage> = Lazy::new(|| init());

#[entry]
fn main() -> ! {
    loop {}
}
