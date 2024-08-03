//! # Pulse Width Modulation
use core::marker::PhantomData;

use crate::rcc::*;
use crate::stm32::*;
use crate::time::Hertz;
use crate::timer::pins::{polarity, TimerPin};
use crate::timer::*;

pub enum OutputCompareMode {
    Frozen = 0,
    MatchPos = 1,
    MatchNeg = 2,
    MatchToggle = 3,
    ForceLow = 4,
    ForceHigh = 5,
    PwmMode1 = 6,
    PmwMode2 = 7,
    OpmMode1 = 8,
    OomMode2 = 9,
    CombinedMode1 = 12,
    CombinedMode2 = 13,
    AsyncMode1 = 14,
    AsyncMode2 = 15,
}

mod pwm_state {
    pub struct Unbound;
    pub struct Bound;
}

pub struct Pwm<Tim, State> {
    clk: Hertz,
    tim: Tim,
    _state: PhantomData<State>,
}

impl<Tim> Into<Pwm<Tim, pwm_state::Bound>> for Pwm<Tim, pwm_state::Unbound> {
    fn into(self: Self) -> Pwm<Tim, pwm_state::Bound> {
        let Self { clk, tim, .. } = self;

        Pwm {
            clk,
            tim,
            _state: PhantomData,
        }
    }
}

pub struct PwmCh<Tim, Ch: Channel> {
    tim: PhantomData<Tim>,
    channel: PhantomData<Ch>,
}

pub struct PwmPin<Tim, Pin, Ch>
where
    Pin: TimerPin<Tim, Channel = Ch, Polarity = polarity::Normal>,
    Ch: Channel,
{
    pin: Pin,
    channel: PwmCh<Tim, Ch>,
}

pub struct ComplementaryPwmPins<Tim, NPin, CPin, Ch>
where
    NPin: TimerPin<Tim, Channel = Ch, Polarity = polarity::Normal>,
    CPin: TimerPin<Tim, Channel = Ch, Polarity = polarity::Complementary>,
    Ch: Channel,
{
    normal_pin: NPin,
    complementary_pin: CPin,
    channel: PwmCh<Tim, Ch>,
}

enum ClockSource {
    ApbTim,
    #[allow(dead_code)]
    Pllq,
}

pub trait PwmExt: Sized {
    fn pwm(self, freq: Hertz, rcc: &mut Rcc) -> Pwm<Self, pwm_state::Unbound>;
}

pub trait PwmQExt: Sized {
    // Configures PWM using PLLQ as a clock source. Panics if PLLQ was not
    // enabled when RCC was configured.
    fn pwm_q(self, freq: Hertz, rcc: &mut Rcc) -> Pwm<Self, pwm_state::Unbound>;
}

pub trait PwmPinMode {
    fn set_compare_mode(&mut self, mode: OutputCompareMode);
}

impl<Tim, Ch: Channel> PwmCh<Tim, Ch> {
    pub fn bind_pin<Pin>(self, pin: Pin) -> PwmPin<Tim, Pin, Ch>
    where
        Pin: TimerPin<Tim, Channel = Ch, Polarity = polarity::Normal>,
    {
        pin.setup();
        PwmPin { pin, channel: self }
    }

    pub fn bind_complementary_pins<NPin, CPin>(
        self,
        normal_pin: NPin,
        complementary_pin: CPin,
    ) -> ComplementaryPwmPins<Tim, NPin, CPin, Ch>
    where
        NPin: TimerPin<Tim, Channel = Ch, Polarity = polarity::Normal>,
        CPin: TimerPin<Tim, Channel = Ch, Polarity = polarity::Complementary>,
    {
        normal_pin.setup();
        complementary_pin.setup();

        ComplementaryPwmPins {
            normal_pin,
            complementary_pin,
            channel: self,
        }
    }
}

macro_rules! pwm {
    ($($TIMX:ident: ($timX:ident, $arr:ident $(,$arr_h:ident)*),)+) => {
        $(
            impl PwmExt for $TIMX {
                fn pwm(self, freq: Hertz, rcc: &mut Rcc) -> Pwm<Self, pwm_state::Unbound> {
                    $timX(self, freq, rcc, ClockSource::ApbTim)
                }
            }

            fn $timX(tim: $TIMX, freq: Hertz, rcc: &mut Rcc, clock_source: ClockSource) -> Pwm<$TIMX, pwm_state::Unbound> {
                $TIMX::enable(rcc);
                $TIMX::reset(rcc);

                let clk = match clock_source {
                    ClockSource::ApbTim => {
                        rcc.ccipr.modify(|_, w| w.tim1sel().clear_bit());
                        rcc.clocks.apb_tim_clk
                    }
                    ClockSource::Pllq => {
                        rcc.ccipr.modify(|_, w| w.tim1sel().set_bit());
                        rcc.clocks.pll_clk.q.unwrap()
                    }
                };

                let mut pwm = Pwm {
                    clk,
                    tim,
                    _state: PhantomData,
                };
                pwm.set_freq(freq);
                pwm
            }

            impl Pwm<$TIMX, pwm_state::Unbound> {
                /// Set the PWM frequency. Actual frequency may differ from
                /// requested due to precision of input clock. To check actual
                /// frequency, call freq.
                pub fn set_freq(&mut self, freq: Hertz) {
                    let ratio = self.clk / freq;
                    let psc = (ratio - 1) / 0xffff;
                    let arr = ratio / (psc + 1) - 1;

                    unsafe {
                        self.tim.psc.write(|w| w.psc().bits(psc as u16));
                        self.tim.arr.write(|w| w.$arr().bits(arr as u16));
                        $(
                            self.tim.arr.modify(|_, w| w.$arr_h().bits((arr >> 16) as u16));
                        )*
                        self.tim.cr1.write(|w| w.cen().set_bit())
                    }
                }
                /// Starts listening
                pub fn listen(&mut self) {
                    self.tim.dier.write(|w| w.uie().set_bit());
                }

                /// Stops listening
                pub fn unlisten(&mut self) {
                    self.tim.dier.write(|w| w.uie().clear_bit());
                }
                /// Clears interrupt flag
                pub fn clear_irq(&mut self) {
                    self.tim.sr.modify(|_, w| w.uif().clear_bit());
                }

                /// Resets counter value
                pub fn reset(&mut self) {
                    self.tim.cnt.reset();
                }

                /// Returns the currently configured frequency
                pub fn freq(&self) -> Hertz {
                    Hertz::from_raw(self.clk.raw()
                        / (self.tim.psc.read().bits() + 1)
                        / (self.tim.arr.read().bits() + 1))
                }
            }
        )+
    }
}

#[allow(unused_macros)]
macro_rules! pwm_q {
    ($($TIMX:ident: $timX:ident,)+) => {
        $(
            impl PwmQExt for $TIMX {
                fn pwm_q(self, freq: Hertz, rcc: &mut Rcc) -> Pwm<Self, pwm_state::Unbound> {
                    $timX(self, freq, rcc, ClockSource::Pllq)
                }
            }
        )+
    }
}

macro_rules! pwm_channels {
    ($($TIMX:ident: ($($CH:ty),+),)+) => {
        $(
            impl Pwm<$TIMX, pwm_state::Unbound> {
                pub fn split(self) -> (Pwm<$TIMX, pwm_state::Bound>, $(PwmCh<$TIMX, $CH>),+) {
                    (
                        self.into(),
                        $(
                            PwmCh::<_, $CH> {
                                tim: PhantomData,
                                channel: PhantomData,
                            }
                        ),+
                    )
                }
            }

            impl Pwm<$TIMX, pwm_state::Bound> {
                pub fn recombine(self, _channels: ($(PwmCh<$TIMX, $CH>),+)) -> Pwm<$TIMX, pwm_state::Unbound> {
                    let Self { clk, tim, .. } = self;
                    Pwm {
                        clk,
                        tim,
                        _state: PhantomData,
                    }
                }
            }
        )+
    };
}

macro_rules! pwm_general_purpose {
    ($($TIMX:ident,)+) => {
        $(
            impl<State> Pwm<$TIMX, State> {
                pub fn get_max_duty(&self) -> u32 {
                    self.tim.arr.read().bits()
                }
            }
        )+
    };
}

macro_rules! pwm_advanced {
    ($($TIMX:ident,)+) => {
        $(
            impl<State> Pwm<$TIMX, State> {
                pub fn get_max_duty(&self) -> u16 {
                    self.tim.arr.read().arr().bits()
                }
            }
        )+
    };
}

macro_rules! release_normal_funcs {
    ($PIN:ident, $TIMX:ty, $CH:ty) => {
        pub unsafe fn release(self) -> ($PIN, PwmCh<$TIMX, $CH>) {
            (self.pin, self.channel)
        }

        pub fn disable_and_release(mut self) -> ($PIN, PwmCh<$TIMX, $CH>) {
            self.disable();
            (self.pin.release(), self.channel)
        }
    };
}

macro_rules! release_complementary_funcs {
    ($NPIN:ident, $CPIN:ident, $TIMX:ty, $CH:ty) => {
        /// Release the underlying resources as a noop.
        ///
        /// # Safety
        ///
        /// Calling this function *will not* disable the PWM
        /// channel. Refer to `disable_and_release` for a safe
        /// implementation.
        pub unsafe fn release(self) -> (($NPIN, $CPIN), PwmCh<$TIMX, $CH>) {
            ((self.normal_pin, self.complementary_pin), self.channel)
        }

        /// Release the underlying resources after disabling them.
        ///
        /// Refer to `release` for an unsafe noop implementation.
        pub fn disable_and_release(mut self) -> (($NPIN, $CPIN), PwmCh<$TIMX, $CH>) {
            self.disable();
            (
                (self.normal_pin.release(), self.complementary_pin.release()),
                self.channel,
            )
        }
    };
}

#[cfg(any(feature = "stm32g0x1", feature = "stm32g070"))]
macro_rules! pwm_pin_funcs {
    (
        $TIMX:ident: ($CH:ty, $ccxe:ident, $ccmrx_output:ident, $ocxpe:ident, $ocxm:ident, $ccrx:ident, $ccrx_l:ident, $ccrx_h:ident)
    ) => {
        pub fn disable(&mut self) {
            let tim = unsafe { &*$TIMX::ptr() };
            tim.ccer.modify(|_, w| w.$ccxe().clear_bit());
        }

        pub fn enable(&mut self) {
            let tim = unsafe { &*$TIMX::ptr() };
            tim.$ccmrx_output()
                .modify(|_, w| w.$ocxpe().set_bit().$ocxm().bits(6));
            tim.ccer.modify(|_, w| w.$ccxe().set_bit());
        }

        pub fn get_duty(&self) -> u32 {
            let tim = unsafe { &*$TIMX::ptr() };
            tim.$ccrx.read().bits()
        }

        pub fn set_duty(&mut self, duty: u32) {
            let tim = unsafe { &*$TIMX::ptr() };
            tim.$ccrx.write(|w| unsafe { w.bits(duty) })
        }
    };
}

#[cfg(any(feature = "stm32g0x1", feature = "stm32g070"))]
macro_rules! pwm_pins {
    (
        $(
            $TIMX:ident: ($CH:ty, $ccxe:ident, $ccmrx_output:ident, $ocxpe:ident, $ocxm:ident, $ccrx:ident, $ccrx_l:ident, $ccrx_h:ident)
        ),+
    ) => {
        $(
            impl<Pin> PwmPin<$TIMX, Pin, $CH>
            where
                Pin: TimerPin<$TIMX, Channel = $CH, Polarity = polarity::Normal>
            {
                pwm_pin_funcs!{ $TIMX: ($CH, $ccxe, $ccmrx_output, $ocxpe, $ocxm, $ccrx, $ccrx_l, $ccrx_h) }
                release_normal_funcs! { Pin, $TIMX, $CH }
            }

            impl<NPin, CPin> ComplementaryPwmPins<$TIMX, NPin, CPin, $CH>
            where
                NPin: TimerPin<$TIMX, Channel = $CH, Polarity = polarity::Normal>,
                CPin: TimerPin<$TIMX, Channel = $CH, Polarity = polarity::Complementary>,
            {
                pwm_pin_funcs!{ $TIMX: ($CH, $ccxe, $ccmrx_output, $ocxpe, $ocxm, $ccrx, $ccrx_l, $ccrx_h) }
                release_complementary_funcs! { NPin, CPin, $TIMX, $CH }
            }
        )+
    };
}

macro_rules! pwm_advanced_pin_funcs {
    (
        $TIMX:ident: ($CH:ty, $ccxe:ident $(: $ccxne:ident)*, $ccmrx_output:ident, $ocxpe:ident, $ocxm:ident, $ccrx:ident $(, $moe:ident)*)
    ) => {
        pub fn disable(&mut self) {
            let tim = unsafe { &*$TIMX::ptr() };
            tim.ccer.modify(|_, w| w.$ccxe().clear_bit());
        }

        pub fn enable(&mut self) {
            let tim = unsafe { &*$TIMX::ptr() };
            tim.$ccmrx_output().modify(|_, w| w.$ocxpe().set_bit().$ocxm().bits(6));
            tim.ccer.modify(|_, w| w.$ccxe().set_bit());
            $(
                tim.ccer.modify(|_, w| w.$ccxne().bit(true));
            )*
            $(
                tim.bdtr.modify(|_, w| w.$moe().set_bit());
            )*
        }

        pub fn get_duty(&self) -> u16 {
            let tim = unsafe { &*$TIMX::ptr() };
            tim.$ccrx.read().$ccrx().bits()
        }

        pub fn set_duty(&mut self, duty: u16) {
            let tim = unsafe { &*$TIMX::ptr() };
            tim.$ccrx.write(|w| unsafe { w.$ccrx().bits(duty) })
        }

        pub fn set_compare_mode(&mut self, mode: OutputCompareMode) {
            let tim = unsafe { &*$TIMX::ptr() };
            tim.$ccmrx_output().modify(|_, w| w.$ocxm().bits(mode as u8));
        }
    };
}

macro_rules! pwm_advanced_pins {
    (
        $(
            $TIMX:ident: ($CH:ty, $ccxe:ident $(: $ccxne:ident)*, $ccmrx_output:ident, $ocxpe:ident, $ocxm:ident, $ccrx:ident $(, $moe:ident)*)
        ),+
    ) => {
        $(
            impl<Pin> PwmPin<$TIMX, Pin, $CH>
            where
                Pin: TimerPin<$TIMX, Channel = $CH, Polarity = polarity::Normal>
            {
                pwm_advanced_pin_funcs!{ $TIMX: ($CH, $ccxe $(: $ccxne)*, $ccmrx_output, $ocxpe, $ocxm, $ccrx $(, $moe)*) }
                release_normal_funcs! { Pin, $TIMX, $CH }
            }

            impl<NPin, CPin> ComplementaryPwmPins<$TIMX, NPin, CPin, $CH>
            where
                NPin: TimerPin<$TIMX, Channel = $CH, Polarity = polarity::Normal>,
                CPin: TimerPin<$TIMX, Channel = $CH, Polarity = polarity::Complementary>,
            {
                pwm_advanced_pin_funcs!{ $TIMX: ($CH, $ccxe $(: $ccxne)*, $ccmrx_output, $ocxpe, $ocxm, $ccrx $(, $moe)*) }
                release_complementary_funcs! { NPin, CPin, $TIMX, $CH }
            }
        )+
    };
}

macro_rules! dead_time {
    ($($TIMX:ident),+) => {
        $(
            impl Pwm<$TIMX, pwm_state::Unbound> {
                /// Set the (raw) dead-time of the timer.
                pub fn set_dead_time(&mut self, raw: u8) {
                    self.tim.bdtr.modify(|_, w| unsafe { w.dtg().bits(raw) });
                }

                /// Get the (raw) dead-time of the timer.
                pub fn get_dead_time(&self) -> u8 {
                    self.tim.bdtr.read().dtg().bits()
                }
            }
        )+
    };
}

pwm_general_purpose! {
    TIM2,
    TIM3,
}

pwm_advanced! {
    TIM1,
    TIM14,
    TIM16,
    TIM17,
}

#[cfg(any(featre = "stm32g070", feature = "stm32g071", feature = "stm32g081"))]
pwm_advanced! {
    TIM15,
}

pwm_channels! {
    TIM1: (Channel1, Channel2, Channel3, Channel4),
    TIM2: (Channel1, Channel2, Channel3, Channel4),
    TIM3: (Channel1, Channel2, Channel3, Channel4),
}

dead_time! {
    TIM1,
    TIM17
}

pwm_advanced_pins! {
    TIM1: (Channel1, cc1e: cc1ne, ccmr1_output, oc1pe, oc1m, ccr1, moe),
    TIM1: (Channel2, cc2e: cc2ne, ccmr1_output, oc2pe, oc2m, ccr2, moe),
    TIM1: (Channel3, cc3e: cc3ne, ccmr2_output, oc3pe, oc3m, ccr3, moe),
    TIM1: (Channel4, cc4e, ccmr2_output, oc4pe, oc4m, ccr4, moe),
    TIM14: (Channel1, cc1e, ccmr1_output, oc1pe, oc1m, ccr1),
    TIM16: (Channel1, cc1e: cc1ne, ccmr1_output, oc1pe, oc1m, ccr1, moe),
    TIM17: (Channel1, cc1e: cc1ne, ccmr1_output, oc1pe, oc1m, ccr1, moe)
}

#[cfg(feature = "stm32g070")]
pwm_advanced_pins! {
    TIM15: (Channel1, cc1e: cc1ne, ccmr1_output, oc1pe, oc1m1, ccr1, moe),
}

#[cfg(any(feature = "stm32g071", feature = "stm32g081"))]
pwm_advanced_pins! {
    TIM15: (Channel1, cc1e: cc1ne, ccmr1_output, oc1pe, oc1m, ccr1, moe),
}

#[cfg(feature = "stm32g0x1")]
pwm_pins! {
    TIM2: (Channel1, cc1e, ccmr1_output, oc1pe, oc1m, ccr1, ccr1_l, ccr1_h),
    TIM2: (Channel2, cc2e, ccmr1_output, oc2pe, oc2m, ccr2, ccr2_l, ccr2_h),
    TIM2: (Channel3, cc3e, ccmr2_output, oc3pe, oc3m, ccr3, ccr3_l, ccr3_h),
    TIM2: (Channel4, cc4e, ccmr2_output, oc4pe, oc4m, ccr4, ccr4_l, ccr4_h),
    TIM3: (Channel1, cc1e, ccmr1_output, oc1pe, oc1m, ccr1, ccr1_l, ccr1_h),
    TIM3: (Channel2, cc2e, ccmr1_output, oc2pe, oc2m, ccr2, ccr2_l, ccr2_h),
    TIM3: (Channel3, cc3e, ccmr2_output, oc3pe, oc3m, ccr3, ccr3_l, ccr3_h),
    TIM3: (Channel4, cc4e, ccmr2_output, oc4pe, oc4m, ccr4, ccr4_l, ccr4_h)
}

#[cfg(feature = "stm32g070")]
pwm_pins! {
    TIM3: (Channel1, cc1e, ccmr1_output, oc1pe, oc1m, ccr1, ccr1_l, ccr1_h),
    TIM3: (Channel2, cc2e, ccmr1_output, oc2pe, oc2m, ccr2, ccr2_l, ccr2_h),
    TIM3: (Channel3, cc3e, ccmr2_output, oc3pe, oc3m, ccr3, ccr3_l, ccr3_h),
    TIM3: (Channel4, cc4e, ccmr2_output, oc4pe, oc4m, ccr4, ccr4_l, ccr4_h),
}

pwm! {
    TIM1: (tim1, arr),
    TIM3: (tim3, arr_l, arr_h),
    TIM14: (tim14, arr),
    TIM16: (tim16, arr),
    TIM17: (tim17, arr),
}

#[cfg(feature = "stm32g0x1")]
pwm! {
    TIM2: (tim2, arr_l, arr_h),
}

#[cfg(any(feature = "stm32g070", feature = "stm32g071", feature = "stm32g081"))]
pwm! {
    TIM15: (tim15, arr),
}

#[cfg(feature = "stm32g0x1")]
pwm_q! {
    TIM1: tim1,
}

#[cfg(any(feature = "stm32g071", feature = "stm32g081"))]
pwm_q! {
    TIM15: tim15,
}
