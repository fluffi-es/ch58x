use crate::interrupt::CoreInterrupt;
use core::{convert::Infallible, marker::PhantomData};
use hal::digital::{ErrorType, InputPin, OutputPin, PinState, StatefulOutputPin};

pub type DefaultMode = Input<Floating>;

pub trait GpioExt {
    type Parts;

    fn split(self) -> Self::Parts;
}

trait GpioRegExt {
    fn is_low(&self, pos: u8) -> bool;
    fn is_set_low(&self, pos: u8) -> bool;
    fn set_high(&self, pos: u8);
    fn set_low(&self, pos: u8);
    fn set_floating(&self, pos: u8);
}

/// Input mode (type state)
pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}

/// Floating input (type state)
pub struct Floating;

/// Pulled down input (type state)
pub struct PullDown;

/// Pulled up input (type state)
pub struct PullUp;

/// Output mode (type state)
pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}

/// Push pull output (type state)
pub struct PushPull;

/// Fully erased pin
pub struct Pin<MODE> {
    i: u8,
    port: *const dyn GpioRegExt,
    _mode: PhantomData<MODE>,
}

macro_rules! gpio_trait {
    ($gpiox:ident) => {
        impl GpioRegExt for crate::raw::$gpiox::RegisterBlock {
            fn is_low(&self, pos: u8) -> bool {
                self.pin().read().bits() & (1 << pos) == 0
            }

            fn is_set_low(&self, pos: u8) -> bool {
                self.out().read().bits() & (1 << pos) == 0
            }

            fn set_high(&self, pos: u8) {
                self.dir()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << pos)) });
                self.out()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << pos)) });
            }

            fn set_low(&self, pos: u8) {
                self.dir()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << pos)) });
                self.clr()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << pos)) });
            }

            fn set_floating(&self, pos: u8) {
                self.dir()
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << pos)) });
            }
        }
    };
}

gpio_trait!(gpioa);
gpio_trait!(gpiob);

// NOTE(unsafe) The only write acess is to BSRR, which is thread safe
unsafe impl<MODE> Sync for Pin<MODE> {}
// NOTE(unsafe) this only enables read access to the same pin from multiple
// threads
unsafe impl<MODE> Send for Pin<MODE> {}

impl<MODE> ErrorType for Pin<Output<MODE>> {
    type Error = Infallible;
}

impl<MODE> StatefulOutputPin for Pin<Output<MODE>> {
    #[inline(always)]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        self.is_set_low().map(|v| !v)
    }

    #[inline(always)]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(unsafe { (*self.port).is_set_low(self.i) })
    }
}

impl<MODE> OutputPin for Pin<Output<MODE>> {
    #[inline(always)]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        unsafe { (*self.port).set_high(self.i) }
        Ok(())
    }

    #[inline(always)]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        unsafe { (*self.port).set_low(self.i) }
        Ok(())
    }
}

impl<MODE> ErrorType for Pin<Input<MODE>> {
    type Error = Infallible;
}

impl<MODE> InputPin for Pin<Input<MODE>> {
    #[inline(always)]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        self.is_low().map(|v| !v)
    }

    #[inline(always)]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(unsafe { (*self.port).is_low(self.i) })
    }
}

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $PXx:ident, $Pxn:expr, [
        $($PXi:ident: ($pxi:ident, $i:expr),)+
    ]) => {
        pub mod $gpiox {
            use crate::$GPIOX;
            use super::*;
            use core::convert::Infallible;
            use core::marker::PhantomData;

            pub struct Parts {
                $(
                    pub $pxi: $PXi<DefaultMode>,
                )+
            }

            impl GpioExt for $GPIOX {
                type Parts = Parts;

                fn split(self) -> Parts {
                    Parts {
                        $(
                            $pxi: $PXi { _mode: PhantomData },
                        )+
                    }
                }
            }

            /// Partially erased pin
            pub struct $PXx<MODE> {
                i: u8,
                _mode: PhantomData<MODE>,
            }

            impl<MODE> ErrorType for $PXx<Output<MODE>> {
                type Error = Infallible;
            }

            impl<MODE> OutputPin for $PXx<Output<MODE>> {
                fn set_high(&mut self) -> Result<(), Self::Error> {
                    // NOTE(unsafe) atomic write to a stateless register
                    unsafe { (*$GPIOX::ptr()).out().modify(|r, w| w.bits(r.bits() | (1 << self.i))) };
                    Ok(())
                }

                fn set_low(&mut self) -> Result<(), Self::Error> {
                    // NOTE(unsafe) atomic write to a stateless register
                    unsafe { (*$GPIOX::ptr()).clr().modify(|r, w| w.bits(r.bits() | (1 << self.i))) };
                    Ok(())
                }
            }

            impl<MODE> StatefulOutputPin for $PXx<Output<MODE>> {
                fn is_set_high(&mut self) -> Result<bool, Self::Error> {
                    let is_set_high = !self.is_set_low()?;
                    Ok(is_set_high)
                }

                fn is_set_low(&mut self) -> Result<bool, Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let is_set_low = unsafe { (*$GPIOX::ptr()).out().read().bits() & (1 << self.i) == 0 };
                    Ok(is_set_low)
                }
            }

            impl<MODE> InputPin for $PXx<Output<MODE>> {
                fn is_high(&mut self) -> Result<bool, Self::Error> {
                    let is_high = !self.is_low()?;
                    Ok(is_high)
                }

                fn is_low(&mut self) -> Result<bool, Self::Error>  {
                    // NOTE(unsafe) atomic read with no side effects
                    let is_low = unsafe { (*$GPIOX::ptr()).pin().read().bits() & (1 << self.i) == 0 };
                    Ok(is_low)
                }
            }

            impl<MODE> ErrorType for $PXx<Input<MODE>> {
                type Error = Infallible;
            }

            impl<MODE> InputPin for $PXx<Input<MODE>> {
                fn is_high(&mut self) -> Result<bool, Self::Error> {
                    let is_high = !self.is_low()?;
                    Ok(is_high)
                }

                fn is_low(&mut self) -> Result<bool, Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let is_low = unsafe { (*$GPIOX::ptr()).pin().read().bits() & (1 << self.i) == 0 };
                    Ok(is_low)
                }
            }

            $(
                pub struct $PXi<MODE> {
                    _mode: PhantomData<MODE>,
                }

                impl<MODE> $PXi<MODE> {
                    /// Configures the pin to operate as a floating input pin
                    pub fn into_floating_input(self) -> $PXi<Input<Floating>> {
                        unsafe {
                            let gpio = &(*$GPIOX::ptr());
                            gpio.dir().modify(|r, w| w.bits(r.bits() & !(1 << $i)));
                            gpio.pu().modify(|r, w| w.bits(r.bits() & !(1 << $i)));
                            gpio.pd_drv().modify(|r, w| w.bits(r.bits() & !(1 << $i)));
                        };
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a pulled down input pin
                    pub fn into_pull_down_input(self) -> $PXi<Input<PullDown>> {
                        unsafe {
                            let gpio = &(*$GPIOX::ptr());
                            gpio.dir().modify(|r, w| w.bits(r.bits() & !(1 << $i)));
                            gpio.pu().modify(|r, w| w.bits(r.bits() & !(1 << $i)));
                            gpio.pd_drv().modify(|r, w| w.bits(r.bits() | (1 << $i)));
                        };
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a pulled up input pin
                    pub fn into_pull_up_input(self) -> $PXi<Input<PullUp>> {
                        unsafe {
                            let gpio = &(*$GPIOX::ptr());
                            gpio.dir().modify(|r, w| w.bits(r.bits() & !(1 << $i)));
                            gpio.pu().modify(|r, w| w.bits(r.bits() | (1 << $i)));
                            gpio.pd_drv().modify(|r, w| w.bits(r.bits() & !(1 << $i)));
                        };
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a push pull output pin
                    /// with `initial_state` specifying whether the pin should
                    /// initially be high or low
                    pub fn into_push_pull_output_in_state(mut self, initial_state: PinState) -> $PXi<Output<PushPull>> {
                        self.internal_set_state(initial_state);
                        self.into_push_pull_output()
                    }

                    /// Configures the pin to operate as a push pull output pin
                    pub fn into_push_pull_output(self) -> $PXi<Output<PushPull>> {
                        unsafe {
                            let gpio = &(*$GPIOX::ptr());
                            gpio.dir().modify(|r, w| w.bits(r.bits() | (1 << $i)));
                            gpio.pd_drv().modify(|r, w| w.bits(r.bits() & !(1 << $i)));
                        };
                        $PXi { _mode: PhantomData }
                    }

                    fn internal_set_state(&mut self, state: PinState) {
                        match state {
                            PinState::High => {
                                // NOTE(unsafe) atomic write to a stateless register
                                unsafe { (*$GPIOX::ptr()).out().modify(|r, w| w.bits(r.bits() | (1 << $i))) };
                            }
                            PinState::Low => {
                                // NOTE(unsafe) atomic write to a stateless register
                                unsafe { (*$GPIOX::ptr()).clr().modify(|r, w| w.bits(r.bits() | (1 << $i))) };
                            }
                        }
                    }
                }

                impl<MODE> $PXi<Output<MODE>> {
                    /// Erases the pin number from the type
                    ///
                    /// This is useful when you want to collect the pins into
                    /// an array where you need all the elements to have the same type
                    pub fn downgrade(self) -> $PXx<Output<MODE>> {
                        $PXx { i: $i, _mode: self._mode }
                    }
                }

                impl<MODE> ErrorType for $PXi<Output<MODE>> {
                    type Error = Infallible;
                }

                impl<MODE> OutputPin for $PXi<Output<MODE>> {
                    fn set_high(&mut self) -> Result<(), Self::Error> {
                        self.internal_set_state(PinState::High);
                        Ok(())
                    }

                    fn set_low(&mut self) -> Result<(), Self::Error>{
                        self.internal_set_state(PinState::Low);
                        Ok(())
                    }
                }

                impl<MODE> StatefulOutputPin for $PXi<Output<MODE>> {
                    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
                        let is_set_high = !self.is_set_low()?;
                        Ok(is_set_high)
                    }

                    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
                        // NOTE(unsafe) atomic read with no side effects
                        let is_set_low = unsafe { (*$GPIOX::ptr()).out().read().bits() & (1 << $i) == 0 };
                        Ok(is_set_low)
                    }
                }

                impl<MODE> InputPin for $PXi<Output<MODE>> {
                    fn is_high(&mut self) -> Result<bool, Self::Error> {
                        let is_high = !self.is_low()?;
                        Ok(is_high)
                    }

                    fn is_low(&mut self) -> Result<bool, Self::Error>  {
                        // NOTE(unsafe) atomic read with no side effects
                        let is_low = unsafe { (*$GPIOX::ptr()).pin().read().bits() & (1 << $i) == 0 };
                        Ok(is_low)
                    }
                }

                impl<MODE> $PXi<Input<MODE>> {
                    /// Erases the pin number from the type
                    ///
                    /// This is useful when you want to collect the pins into
                    /// an array where you need all the elements to have the same type
                    pub fn downgrade(self) -> $PXx<Input<MODE>> {
                        $PXx { i: $i, _mode: self._mode }
                    }
                }

                impl<MODE> ErrorType for $PXi<Input<MODE>> {
                    type Error = Infallible;
                }

                impl<MODE> InputPin for $PXi<Input<MODE>> {
                    fn is_high(&mut self) -> Result<bool, Self::Error> {
                        let is_high = !self.is_low()?;
                        Ok(is_high)
                    }

                    fn is_low(&mut self) -> Result<bool, Self::Error> {
                        // NOTE(unsafe) atomic read with no side effects
                        let is_low = unsafe { (*$GPIOX::ptr()).pin().read().bits() & (1 << $i) == 0 };
                        Ok(is_low)
                    }
                }
            )+

            impl<TYPE> $PXx<TYPE> {
                pub fn get_id (&self) -> u8 {
                    self.i
                }
            }

            impl<MODE> $PXx<Output<MODE>> {
                /// Erases the port number from the type
                ///
                /// This is useful when you want to collect the pins into an
                /// array where you need all the elements to have the same type
                pub fn downgrade(self) -> Pin<Output<MODE>> {
                    Pin {
                        i: self.get_id(),
                        port: $GPIOX::ptr() as *const dyn GpioRegExt,
                        _mode: self._mode,
                    }
                }
            }

            impl<MODE> $PXx<Input<MODE>> {
                /// Erases the port number from the type
                ///
                /// This is useful when you want to collect the pins into an
                /// array where you need all the elements to have the same type
                pub fn downgrade(self) -> Pin<Input<MODE>> {
                    Pin {
                        i: self.get_id(),
                        port: $GPIOX::ptr() as *const dyn GpioRegExt,
                        _mode: self._mode,
                    }
                }
            }
        }

        pub use $gpiox::{ $($PXi,)+ };
    }
}

gpio!(Gpioa, gpioa, PA, 0, [
    PA0: (pa0, 0),
    PA1: (pa1, 1),
    PA2: (pa2, 2),
    PA3: (pa3, 3),
    PA4: (pa4, 4),
    PA5: (pa5, 5),
    PA6: (pa6, 6),
    PA7: (pa7, 7),
    PA8: (pa8, 8),
    PA9: (pa9, 9),
    PA10: (pa10, 10),
    PA11: (pa11, 11),
    PA12: (pa12, 12),
    PA13: (pa13, 13),
    PA14: (pa14, 14),
    PA15: (pa15, 15),
]);

gpio!(Gpiob, gpiob, PB, 1, [
    PB0: (pb0, 0),
    PB1: (pb1, 1),
    PB2: (pb2, 2),
    PB3: (pb3, 3),
    PB4: (pb4, 4),
    PB5: (pb5, 5),
    PB6: (pb6, 6),
    PB7: (pb7, 7),
    PB8: (pb8, 8),
    PB9: (pb9, 9),
    PB10: (pb10, 10),
    PB11: (pb11, 11),
    PB12: (pb12, 12),
    PB13: (pb13, 13),
    PB14: (pb14, 14),
    PB15: (pb15, 15),
    PB16: (pb16, 16),
    PB17: (pb17, 17),
    PB18: (pb18, 18),
    PB19: (pb19, 19),
    PB20: (pb20, 20),
    PB21: (pb21, 21),
    PB22: (pb22, 22),
    PB23: (pb23, 23),
]);

#[riscv_rt::core_interrupt(CoreInterrupt::GPIOA)]
fn gpioa() {}

#[riscv_rt::core_interrupt(CoreInterrupt::GPIOB)]
fn gpiob() {}
