use crate::gpio::*;
use crate::gpio::{AltFunction, DefaultMode};
use crate::stm32::*;
use crate::timer::*;

pub mod polarity {
    pub struct Normal;
    pub struct Complementary;
}

pub trait TimerPin<TIM> {
    type Channel: Channel;
    type Polarity;

    fn setup(&self);
    fn release(self) -> Self;
}

pub struct TriggerPin<TIM, PIN: TimerPin<TIM>> {
    pin: PIN,
    tim: PhantomData<TIM>,
}

impl<TIM, PIN: TimerPin<TIM>> ExternalClock for TriggerPin<TIM, PIN> {
    fn mode(&self) -> ExternalClockMode {
        ExternalClockMode::Mode1
    }
}

impl<TIM, PIN: TimerPin<TIM>> TriggerPin<TIM, PIN> {
    pub fn release(self) -> PIN {
        self.pin
    }
}

macro_rules! timer_pins {
    ($TIMX:ident, [ $(($ch:ty, $pol:ty, $pin:tt, $af_mode:expr),)+ ]) => {
        $(
            impl TimerPin<$TIMX> for $pin<Analog> {
                type Channel = $ch;
                type Polarity = $pol;

                fn setup(&self) {
                    self.set_alt_mode($af_mode);
                }

                fn release(self) -> Self {
                    self.into_analog()
                }
            }

            impl TimerPin<$TIMX> for $pin<Output<OpenDrain>> {
                type Channel = $ch;
                type Polarity = $pol;

                fn setup(&self) {
                    self.set_alt_mode($af_mode);
                }

                fn release(self) -> Self {
                    self.into_open_drain_output()
                }
            }

        )+
    };
}

macro_rules! trigger_pins {
    ($TIMX:ident, [ $(($pin:ty, $ccp:ident $(,$icf:ident)*),)+ ]) => {
        $(
            impl TriggerPin<$TIMX, $pin> {
                pub fn new(pin: $pin, edge: SignalEdge) -> Self {
                    TimerPin::<$TIMX>::setup(&pin);
                    let tim = unsafe { &(*$TIMX::ptr()) };
                    let ts = match edge {
                        SignalEdge::Any => 0b100,
                        SignalEdge::Falling => {
                            tim.ccer.modify(|_, w| w.$ccp().set_bit());
                            0b101
                        },
                        SignalEdge::Rising => {
                            tim.ccer.modify(|_, w| w.$ccp().clear_bit());
                            0b101
                        }
                    };

                    tim.smcr.modify(|_, w| unsafe { w.ts().bits(ts) });

                    Self {
                        pin,
                        tim: PhantomData,
                    }
                }

                $(
                    pub fn with_filter(pin: $pin, edge: SignalEdge, capture_filter: u8) -> Self {
                        unsafe {
                            let tim =  &(*$TIMX::ptr()) ;
                            tim.ccmr1_input().modify(|_, w| w.$icf().bits(capture_filter));
                        }
                        Self::new(pin, edge)
                    }
                )*
            }
        )+
    };
}

trigger_pins!(TIM1, [
    (PA8<DefaultMode>, cc1p),
    (PC8<DefaultMode>, cc1p),
    (PA9<DefaultMode>, cc2p),
    (PB3<DefaultMode>, cc2p),
    (PC9<DefaultMode>, cc2p),
]);

#[cfg(feature = "stm32g0x1")]
trigger_pins!(TIM2, [
    (PA0<DefaultMode>, cc1p, ic1f),
    (PA5<DefaultMode>, cc1p, ic1f),
    (PA15<DefaultMode>, cc1p, ic1f),
    (PC4<DefaultMode>, cc1p, ic1f),
    (PA1<DefaultMode>, cc2p, ic2f),
    (PB3<DefaultMode>, cc2p, ic2f),
    (PC5<DefaultMode>, cc2p, ic2f),
]);

trigger_pins!(TIM3, [
    (PA6<DefaultMode>, cc1p, ic1f),
    (PB4<DefaultMode>, cc1p, ic1f),
    (PC6<DefaultMode>, cc1p, ic1f),
    (PA7<DefaultMode>, cc2p, ic2f),
    (PB5<DefaultMode>, cc2p, ic2f),
    (PC7<DefaultMode>, cc2p, ic2f),
]);

timer_pins!(
    TIM1,
    [
        (Channel1, polarity::Normal, PA8, AltFunction::AF2),
        (Channel1, polarity::Normal, PC8, AltFunction::AF2),
        (Channel2, polarity::Normal, PA9, AltFunction::AF2),
        (Channel2, polarity::Normal, PB3, AltFunction::AF1),
        (Channel2, polarity::Normal, PC9, AltFunction::AF2),
        (Channel3, polarity::Normal, PA10, AltFunction::AF2),
        (Channel3, polarity::Normal, PB6, AltFunction::AF1),
        (Channel3, polarity::Normal, PC10, AltFunction::AF2),
        (Channel4, polarity::Normal, PA11, AltFunction::AF2),
        (Channel4, polarity::Normal, PC11, AltFunction::AF2),
    ]
);

// Inverted pins
timer_pins!(
    TIM1,
    [
        (Channel1, polarity::Complementary, PA7, AltFunction::AF2),
        (Channel1, polarity::Complementary, PB13, AltFunction::AF2),
        (Channel1, polarity::Complementary, PD2, AltFunction::AF2),
        (Channel2, polarity::Complementary, PB0, AltFunction::AF2),
        (Channel2, polarity::Complementary, PB14, AltFunction::AF2),
        (Channel2, polarity::Complementary, PD3, AltFunction::AF2),
        (Channel3, polarity::Complementary, PB1, AltFunction::AF2),
        (Channel3, polarity::Complementary, PB15, AltFunction::AF2),
        (Channel3, polarity::Complementary, PD4, AltFunction::AF2),
    ]
);

#[cfg(feature = "stm32g0x1")]
timer_pins!(
    TIM2,
    [
        (Channel1, polarity::Normal, PA0, AltFunction::AF2),
        (Channel1, polarity::Normal, PA5, AltFunction::AF2),
        (Channel1, polarity::Normal, PA15, AltFunction::AF2),
        (Channel1, polarity::Normal, PC4, AltFunction::AF2),
        (Channel2, polarity::Normal, PA1, AltFunction::AF2),
        (Channel2, polarity::Normal, PB3, AltFunction::AF2),
        (Channel2, polarity::Normal, PC5, AltFunction::AF2),
        (Channel3, polarity::Normal, PA2, AltFunction::AF2),
        (Channel3, polarity::Normal, PB10, AltFunction::AF2),
        (Channel3, polarity::Normal, PC6, AltFunction::AF2),
        (Channel4, polarity::Normal, PA3, AltFunction::AF2),
        (Channel4, polarity::Normal, PB11, AltFunction::AF2),
        (Channel4, polarity::Normal, PC7, AltFunction::AF2),
    ]
);

timer_pins!(
    TIM3,
    [
        (Channel1, polarity::Normal, PA6, AltFunction::AF1),
        (Channel1, polarity::Normal, PB4, AltFunction::AF1),
        (Channel1, polarity::Normal, PC6, AltFunction::AF1),
        (Channel2, polarity::Normal, PA7, AltFunction::AF1),
        (Channel2, polarity::Normal, PB5, AltFunction::AF1),
        (Channel2, polarity::Normal, PC7, AltFunction::AF1),
        (Channel3, polarity::Normal, PB0, AltFunction::AF1),
        (Channel3, polarity::Normal, PC8, AltFunction::AF1),
        (Channel4, polarity::Normal, PB1, AltFunction::AF1),
        (Channel4, polarity::Normal, PC9, AltFunction::AF1),
    ]
);

timer_pins!(
    TIM14,
    [
        (Channel1, polarity::Normal, PA4, AltFunction::AF4),
        (Channel1, polarity::Normal, PA7, AltFunction::AF4),
        (Channel1, polarity::Normal, PB1, AltFunction::AF0),
        (Channel1, polarity::Normal, PC12, AltFunction::AF2),
        (Channel1, polarity::Normal, PF0, AltFunction::AF2),
    ]
);

#[cfg(any(feature = "stm32g070", feature = "stm32g071", feature = "stm32g081"))]
timer_pins!(
    TIM15,
    [
        (Channel1, polarity::Normal, PA2, AltFunction::AF5),
        (Channel1, polarity::Normal, PB14, AltFunction::AF5),
        (Channel1, polarity::Normal, PC1, AltFunction::AF2),
        (Channel2, polarity::Normal, PA3, AltFunction::AF5),
        (Channel2, polarity::Normal, PB15, AltFunction::AF5),
        (Channel2, polarity::Normal, PC2, AltFunction::AF2),
    ]
);

// Inverted pins
#[cfg(any(feature = "stm32g070", feature = "stm32g071", feature = "stm32g081"))]
timer_pins!(
    TIM15,
    [
        (Channel1, polarity::Complementary, PA1, AltFunction::AF5),
        (Channel1, polarity::Complementary, PB13, AltFunction::AF5),
        (Channel1, polarity::Complementary, PF1, AltFunction::AF2),
    ]
);

timer_pins!(
    TIM16,
    [
        (Channel1, polarity::Normal, PA6, AltFunction::AF5),
        (Channel1, polarity::Normal, PB8, AltFunction::AF2),
        (Channel1, polarity::Normal, PD0, AltFunction::AF2),
    ]
);

// Inverted pins
timer_pins!(
    TIM16,
    [(Channel1, polarity::Complementary, PB6, AltFunction::AF2),]
);

timer_pins!(
    TIM17,
    [
        (Channel1, polarity::Normal, PA7, AltFunction::AF6),
        (Channel1, polarity::Normal, PB9, AltFunction::AF2),
        (Channel1, polarity::Normal, PD1, AltFunction::AF2),
    ]
);

//  Inverted pins
timer_pins!(
    TIM17,
    [(Channel1, polarity::Complementary, PB7, AltFunction::AF2),]
);
