#![feature(asm,concat_idents,const_fn)]
#![no_std]

extern crate kernel;

extern "C" {
    pub fn init();
}

mod peripheral_registers;
mod peripheral_interrupts;
mod nvic;

pub mod chip;
pub mod gpio;
pub mod rtc;
pub mod timer;
pub mod clock;
pub mod uart;
pub use chip::NRF51;

#[repr(C)]
pub struct PinCnf(usize);

impl PinCnf {
    pub const unsafe fn new(pin: usize) -> PinCnf {
        PinCnf(pin)
    }
}

