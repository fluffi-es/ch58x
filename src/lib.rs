#![no_std]
#![feature(abi_riscv_interrupt)]
#![allow(mismatched_lifetime_syntaxes, non_camel_case_types)]

mod generic;
pub use generic::*;

pub mod raw;
pub use raw::*;

extern crate embedded_hal as hal;

pub mod adc;
pub mod gpio;
pub mod pfic;
pub mod sys;
pub mod sysclk;
pub mod usb;

struct CriticalSection;
critical_section::set_impl!(CriticalSection);

unsafe impl critical_section::Impl for CriticalSection {
    unsafe fn acquire() -> critical_section::RawRestoreState {
        let gintenr: usize;
        core::arch::asm!("csrrci {}, 0x800, 0b1000", out(reg) gintenr);
        gintenr & 0b1000 != 0
    }

    unsafe fn release(restore_state: critical_section::RawRestoreState) {
        if restore_state {
            core::arch::asm!("csrs 0x800, 0b1000")
        }
    }
}

pub struct Executor {
    inner: embassy_executor::raw::Executor,
    not_send: core::marker::PhantomData<*mut ()>,
}

impl Executor {
    pub fn new() -> Self {
        Self {
            inner: embassy_executor::raw::Executor::new(core::ptr::null_mut()),
            not_send: core::marker::PhantomData,
        }
    }

    pub fn run(&'static mut self, init: impl FnOnce(embassy_executor::Spawner)) -> ! {
        init(self.inner.spawner());

        loop {
            // SAFETY: property initialized, not reentrant on the same executor, not called
            // in pender
            unsafe { self.inner.poll() };

            wfe();
        }
    }
}

#[export_name = "__pender"]
fn __pender(_: *mut ()) {
    sev();
}

pub fn wfi() {
    unsafe { Pfic::steal() }
        .sctlr()
        .modify(|_, w| w.wfitowfe().clear_bit());
    riscv::asm::wfi();
}

pub fn wfe() {
    /*unsafe { Pfic::steal() }
        .sctlr()
        .modify(|_, w| w.setevent().set_bit().wfitowfe().set_bit());
    riscv::asm::wfi();*/

    unsafe { Pfic::steal() }
        .sctlr()
        .modify(|_, w| w.wfitowfe().set_bit());
    riscv::asm::wfi();
}

pub fn sev() {
    unsafe { Pfic::steal() }
        .sctlr()
        .modify(|_, w| w.setevent().set_bit());
}
