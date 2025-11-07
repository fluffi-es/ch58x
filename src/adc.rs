use crate::{gpio::{Floating, Input}, interrupt::CoreInterrupt};
pub use crate::raw::adc::cfg::{ClkDiv, PgaGain as Gain};
use core::{future::poll_fn, task::Poll};
use embassy_sync::waitqueue::AtomicWaker;

static WAKER: AtomicWaker = AtomicWaker::new();

pub struct Adc {
    raw: crate::raw::Adc,
}

pub trait AdcChannel {
    fn channel(&self) -> u8;
}

impl AdcChannel for crate::gpio::PA4<Input<Floating>> {
    fn channel(&self) -> u8 {
        0
    }
}

impl AdcChannel for crate::gpio::PA5<Input<Floating>> {
    fn channel(&self) -> u8 {
        1
    }
}

impl AdcChannel for crate::gpio::PA12<Input<Floating>> {
    fn channel(&self) -> u8 {
        2
    }
}

impl AdcChannel for crate::gpio::PA13<Input<Floating>> {
    fn channel(&self) -> u8 {
        3
    }
}

impl AdcChannel for crate::gpio::PA14<Input<Floating>> {
    fn channel(&self) -> u8 {
        4
    }
}

impl AdcChannel for crate::gpio::PA15<Input<Floating>> {
    fn channel(&self) -> u8 {
        5
    }
}

impl AdcChannel for crate::gpio::PA3<Input<Floating>> {
    fn channel(&self) -> u8 {
        6
    }
}

impl AdcChannel for crate::gpio::PA2<Input<Floating>> {
    fn channel(&self) -> u8 {
        7
    }
}

impl AdcChannel for crate::gpio::PA1<Input<Floating>> {
    fn channel(&self) -> u8 {
        8
    }
}

impl AdcChannel for crate::gpio::PA0<Input<Floating>> {
    fn channel(&self) -> u8 {
        9
    }
}

impl AdcChannel for crate::gpio::PA6<Input<Floating>> {
    fn channel(&self) -> u8 {
        10
    }
}

impl AdcChannel for crate::gpio::PA7<Input<Floating>> {
    fn channel(&self) -> u8 {
        11
    }
}

impl AdcChannel for crate::gpio::PA8<Input<Floating>> {
    fn channel(&self) -> u8 {
        12
    }
}

impl AdcChannel for crate::gpio::PA9<Input<Floating>> {
    fn channel(&self) -> u8 {
        13
    }
}

pub struct Vbat;
impl AdcChannel for Vbat {
    fn channel(&self) -> u8 {
        14
    }
}

pub struct Temperature;
impl AdcChannel for Temperature {
    fn channel(&self) -> u8 {
        15
    }
}

impl Adc {
    pub fn new(raw: crate::raw::Adc, gain: Gain, clk_div: ClkDiv) -> Self {
        raw.cfg().write(|w| {
            w.power_on()
                .set_bit()
                .buf_en()
                .set_bit()
                .ofs_test()
                .set_bit()
                .pga_gain()
                .variant(gain)
                .clk_div()
                .variant(clk_div)
        });

        raw.convert().write(|w| w.start().set_bit());
        for _ in 0..16 {
            while raw.convert().read().start().bit() {}
        }
        raw.cfg().modify(|_, w| w.ofs_test().clear_bit());

        Self { raw }
    }

    pub fn read_one(&mut self, channel: &mut impl AdcChannel) -> u16 {
        self.raw
            .channel()
            .write(|w| unsafe { w.ch_inx().bits(channel.channel()) });
        self.raw.convert().write(|w| w.start().set_bit());
        while self.raw.convert().read().start().bit() {}
        self.raw.data().read().bits()
    }

    pub async fn read(&mut self, channel: &mut impl AdcChannel, values: &mut [u16]) {
        self.raw
            .channel()
            .write(|w| unsafe { w.ch_inx().bits(channel.channel()) });
        self.raw
            .dma_beg()
            .write(|w| unsafe { w.dma_beg().bits(values.as_ptr() as u16) });
        self.raw
            .dma_end()
            .write(|w| unsafe { w.dma_end().bits(values.as_ptr().add(values.len()) as u16) });
        self.raw
            .ctrl_dma()
            .write(|w| w.ie_dma_end().set_bit().dma_enable().set_bit());
        poll_fn(|cx| {
            WAKER.register(cx.waker());
            if self.raw.dma_if().read().if_dma_end().bit() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
    }
}

#[riscv_rt::core_interrupt(CoreInterrupt::ADC)]
fn adc() {
    let adc = unsafe { crate::raw::Adc::steal() };
    if adc.dma_if().read().if_dma_end().bit_is_clear() {
        return;
    }
    WAKER.wake();
}
