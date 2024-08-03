//! # Analog to Digital converter
use core::ptr;

use crate::rcc::{Enable, Rcc};
use crate::stm32::ADC;
use crate::{fmt, gpio::*};
use hal::adc::{Channel, OneShot};
use proto::stm32::adc::AdcPin;

/// ADC Result Alignment
#[derive(Eq, PartialEq)]
pub enum Align {
    /// Right aligned results (least significant bits)
    ///
    /// Results in all precisions returning values from 0-(2^bits-1) in
    /// steps of 1.
    Right,
    /// Left aligned results (most significant bits)
    ///
    /// Results in all precisions returning a value in the range 0-65535.
    /// Depending on the precision the result will step by larger or smaller
    /// amounts.
    Left,
}

/// ADC Sampling Precision
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Precision {
    /// 12 bit precision
    B12 = 0b00,
    /// 10 bit precision
    B10 = 0b01,
    /// 8 bit precision
    B8 = 0b10,
    /// 6 bit precision
    B6 = 0b11,
}

/// ADC Sampling time
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum SampleTime {
    T1_5 = 0b000,
    T3_5 = 0b001,
    T7_5 = 0b010,
    T12_5 = 0b011,
    T19_5 = 0b100,
    T39_5 = 0b101,
    T79_5 = 0b110,
    T160_5 = 0b111,
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum SampleTimeSelect {
    One,
    Two,
}

// ADC Oversampling ratio
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum OversamplingRatio {
    X2 = 0b000,
    X4 = 0b001,
    X8 = 0b010,
    X16 = 0b011,
    X32 = 0b100,
    X64 = 0b101,
    X128 = 0b110,
    X256 = 0b111,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ClockSource {
    Pclk(PclkDiv),
    Async(AsyncClockDiv),
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PclkDiv {
    Div1 = 3,
    Div2 = 1,
    Div4 = 2,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AsyncClockDiv {
    Div1 = 0,
    Div2 = 1,
    Div4 = 2,
    Div8 = 3,
    Div16 = 4,
    Div32 = 5,
    Div64 = 6,
    Div128 = 7,
    Div256 = 8,
}

/// ADC injected trigger source selection
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum InjTrigSource {
    /// Trigger 0
    TIM1_TRGO2 = 0b000,
    /// Trigger 1
    TIM1_CC4 = 0b001,
    /// Trigger 2
    TIM2_TRGO = 0b010,
    /// Trigger 3
    TIM3_TRGO = 0b011,
    /// Trigger 4
    TIM15_TRGO = 0b100,
    /// Trigger 5
    TIM6_TRGO = 0b101,
    /// Trigger 6
    TIM4_TRGO = 0b110,
    /// Trigger 7
    EXTI11 = 0b111,
}

/// $RM0444 15.11 Table 77
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Event {
    /// End of calibration
    EOCAL,
    /// ADC ready
    ADRDY,
    /// End of conversion
    EOC,
    /// End of sequence
    EOS,
    /// Analog watchdog 1
    AWD1,
    /// Analog watchdog 2
    AWD2,
    /// Analog watchdog 3
    AWD3,
    /// Channel configuration ready
    CCRDY,
    /// End of sampling phase
    EOSMP,
    /// Overrun
    OVR,
}

/*
let sequence = (
    pin1.as_sequence_item(SampleTimeSelect::One),
    pin2.as_sequence_item(SampleTimeSelect::Two),
    pin3.as_sequence_item(SampleTimeSelect::One),
).into_basic_sequence(Direction::Forward);

let mut conversion: Conversion<Continuous> = adc.create_conversion(sequence).into_continuous();
conversion.start();
loop {
    let result = block!(conversion.get_all()).unwrap();
}

// on drop: conversion.stop();

---

let sequence = (pin.as_sequence_item(SampleTimeSelect::One),).into_basic_sequence(Direction::Forward);

let mut conversion: Conversion<OneShot> = adc.create_conversion(sequence);
conversion.start();

block!(conversion.get());

*/

/// Analog to Digital converter interface
pub struct Adc {
    rb: ADC,
    align: Align,
    precision: Precision,
    vref_cache: Option<u16>,
}

/// Contains the calibration factors for the ADC which can be reused with [`Adc::set_calibration()`]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct CalibrationFactor(pub u8);

impl Adc {
    pub fn new(adc: ADC, rcc: &mut Rcc) -> Self {
        // Enable ADC clocks
        ADC::enable(rcc);

        adc.cr.modify(|_, w| w.advregen().set_bit());

        Self {
            rb: adc,
            align: Align::Right,
            precision: Precision::B12,
            vref_cache: None,
        }
    }

    /// Sets ADC source
    pub fn set_clock_source(&mut self, clock_source: ClockSource) {
        match clock_source {
            ClockSource::Pclk(div) => self
                .rb
                .cfgr2
                .modify(|_, w| unsafe { w.ckmode().bits(div as u8) }),
            ClockSource::Async(div) => {
                self.rb.cfgr2.modify(|_, w| unsafe { w.ckmode().bits(0) });
                self.rb
                    .ccr
                    .modify(|_, w| unsafe { w.presc().bits(div as u8) });
            }
        }
    }

    /// Runs the calibration routine on the ADC
    ///
    /// Wait for tADCVREG_SETUP (20us on STM32G071x8) after calling [`Self::new()`] before calibrating, to wait for the
    /// ADC voltage regulator to stabilize.
    ///
    /// Do not call if an ADC reading is ongoing.
    pub fn calibrate(&mut self) {
        self.rb.cr.modify(|_, w| w.adcal().set_bit());
        while self.rb.cr.read().adcal().bit_is_set() {}
    }

    /// Returns the calibration factors used by the ADC
    ///
    /// The ADC does not have a factory-stored calibration, [`Self::calibrate()`] must be run before calling this
    /// for the returned value to be useful.
    ///
    /// The ADC loses its calibration factors when Standby or Vbat mode is entered. Saving and restoring the calibration
    /// factors can be used to recalibrate the ADC after waking up from sleep more quickly than re-running calibraiton.
    /// Note that VDDA changes and to a lesser extent temperature changes affect the ADC operating conditions and
    /// calibration should be run again for the best accuracy.
    pub fn get_calibration(&self) -> CalibrationFactor {
        CalibrationFactor(self.rb.calfact.read().calfact().bits())
    }

    /// Writes the calibration factors used by the ADC
    ///
    /// See [`Self::get_calibration()`].
    ///
    /// Do not call if an ADC reading is ongoing.
    pub fn set_calibration(&mut self, calfact: CalibrationFactor) {
        self.rb
            .calfact
            .write(|w| unsafe { w.calfact().bits(calfact.0) });
    }

    /// Set the Adc sampling time
    pub fn set_sample_time(&mut self, mux: SampleTimeSelect, sample_time: SampleTime) {
        self.rb.smpr.modify(|_, w| match mux {
            SampleTimeSelect::One => unsafe { w.smp1().bits(sample_time as u8) },
            SampleTimeSelect::Two => unsafe { w.smp2().bits(sample_time as u8) },
        });
    }

    /// Set the Adc result alignment
    pub fn set_align(&mut self, align: Align) {
        self.align = align;
    }

    /// Set the Adc precision
    pub fn set_precision(&mut self, precision: Precision) {
        self.precision = precision;
    }

    /// The nuber of bits, the oversampling result is shifted in bits at the end of oversampling
    pub fn set_oversampling_shift(&mut self, nrbits: u8) {
        self.rb
            .cfgr2
            .modify(|_, w| unsafe { w.ovss().bits(nrbits) });
    }

    /// Oversampling of adc according to datasheet of stm32g0, when oversampling is enabled
    pub fn set_oversampling_ratio(&mut self, ratio: OversamplingRatio) {
        self.rb
            .cfgr2
            .modify(|_, w| unsafe { w.ovsr().bits(ratio as u8) });
    }

    pub fn oversampling_enable(&mut self, enable: bool) {
        self.rb.cfgr2.modify(|_, w| w.ovse().bit(enable));
    }

    pub fn start_injected(&mut self) {
        self.rb.cr.modify(|_, w| w.adstart().set_bit());
        // ADSTART bit is cleared to 0 bevor using this function
        // enable self.rb.isr.eos() flag is set after each converstion
        self.rb.ier.modify(|_, w| w.eocie().set_bit()); // end of sequence interupt enable
    }

    pub fn stop_injected(&mut self) {
        // ?????? or is it reset after each conversion?
        // ADSTART bit is cleared to 0 bevor using this function
        // disable EOS interrupt
        // maybe self.rb.cr.adstp().set_bit() must be performed before interrupt is disabled + wait abortion
        self.rb.ier.modify(|_, w| w.eocie().clear_bit()); // end of sequence interupt disable
    }

    /// Read actual VREF voltage using the internal reference
    ///
    /// If oversampling is enabled, the return value is scaled down accordingly.
    /// The product of the return value and any ADC reading always gives correct voltage in 4096ths of mV
    /// regardless of oversampling and shift settings provided that these settings remain the same.
    pub fn read_vref(&mut self) -> nb::Result<u16, ()> {
        let mut vref = VRef::new();
        let vref_val: u32 = if vref.enabled(self) {
            self.read(&mut vref)?
        } else {
            vref.enable(self);
            let vref_val = self.read(&mut vref)?;
            vref.disable(self);
            vref_val
        };

        let vref_cal: u32 = unsafe {
            // DS12766 3.13.2
            ptr::read_volatile(0x1FFF_75AA as *const u16) as u32
        };

        // RM0454 14.9 Calculating the actual VDDA voltage using the internal reference voltage
        // V_DDA = 3 V x VREFINT_CAL / VREFINT_DATA
        let vref = (vref_cal * 3_000_u32 / vref_val) as u16;
        self.vref_cache = Some(vref);
        Ok(vref)
    }

    /// Get VREF value using cached value if possible
    ///
    /// See `read_vref` for more details.
    pub fn get_vref_cached(&mut self) -> nb::Result<u16, ()> {
        if let Some(vref) = self.vref_cache {
            Ok(vref)
        } else {
            self.read_vref()
        }
    }

    pub fn read_basic_sequence<const N: usize>(
        &mut self,
        sequence: BasicSequence<N>,
    ) -> nb::Result<u16, ()> {
        self.power_up();
        // self.rb.cfgr1.modify(|_, w| unsafe {
        //     w.res()
        //         .bits(self.precision as u8)
        //         .align()
        //         .bit(self.align == Align::Left)
        // });

        // self.rb
        //     .smpr
        //     .modify(|_, w| unsafe { w.smp1().bits(self.sample_time as u8) });

        // self.rb
        //     .chselr()
        //     .modify(|_, w| unsafe { w.chsel().bits(1 << PIN::channel()) });

        // self.rb.isr.modify(|_, w| w.eos().set_bit());
        // self.rb.cr.modify(|_, w| w.adstart().set_bit());
        // while self.rb.isr.read().eos().bit_is_clear() {}

        // let res = self.rb.dr.read().bits() as u16;
        // let val = if self.align == Align::Left && self.precision == Precision::B_6 {
        //     res << 8
        // } else {
        //     res
        // };

        self.power_down();
        Ok(0)
    }

    pub fn read_voltage<PIN: Channel<Adc, ID = u8>>(
        &mut self,
        pin: &mut PIN,
    ) -> nb::Result<u16, ()> {
        let vref = self.get_vref_cached()?;

        self.read(pin).map(|raw: u32| {
            let adc_mv = (vref as u32 * raw) >> 12;
            adc_mv as u16
        })
    }

    pub fn read_temperature(&mut self) -> nb::Result<i16, ()> {
        let mut vtemp = VTemp::new();
        let vtemp_voltage: u16 = if vtemp.enabled(self) {
            self.read_voltage(&mut vtemp)?
        } else {
            vtemp.enable(self);
            let vtemp_voltage = self.read_voltage(&mut vtemp)?;
            vtemp.disable(self);
            vtemp_voltage
        };

        let ts_cal1: u32 = unsafe {
            // DS12991 3.14.1
            // at 3000 mV Vref+ and 30 degC
            ptr::read_volatile(0x1FFF_75A8 as *const u16) as u32
        };

        let v30 = (3000_u32 * ts_cal1) >> 12; // mV
                                              // 2.5 mV/degC
        let t = 30 + (vtemp_voltage as i32 - v30 as i32) * 10 / 25;

        Ok(t as i16)
    }

    pub fn start_conversion<PIN>(&mut self, pin: &mut PIN)
    where
        PIN: AdcPin<Self, ID = u8>,
    {
        self.power_up();
        self.rb
            .chselr()
            .modify(|_, w| unsafe { w.chsel().bits(1 << PIN::CHANNEL) });

        fmt::trace!("chsel: {}", self.rb.chselr().read().bits());

        self.rb.isr.modify(|_, w| w.eos().set_bit());
        self.rb.cr.modify(|_, w| w.adstart().set_bit());
    }

    pub fn listen(&mut self, event: Event) {
        self.rb.ier.modify(|_, w| match event {
            Event::EOCAL => w.eocalie().set_bit(),
            Event::ADRDY => w.adrdyie().set_bit(),
            Event::EOC => w.eocie().set_bit(),
            Event::EOS => w.eosie().set_bit(),
            Event::AWD1 => w.awd1ie().set_bit(),
            Event::AWD2 => w.awd2ie().set_bit(),
            Event::AWD3 => w.awd3ie().set_bit(),
            Event::CCRDY => w.ccrdyie().set_bit(),
            Event::EOSMP => w.eosmpie().set_bit(),
            Event::OVR => w.ovrie().set_bit(),
        });
    }

    pub fn unlisten(&mut self, event: Event) {
        self.rb.ier.modify(|_, w| match event {
            Event::EOCAL => w.eocalie().clear_bit(),
            Event::ADRDY => w.adrdyie().clear_bit(),
            Event::EOC => w.eocie().clear_bit(),
            Event::EOS => w.eosie().clear_bit(),
            Event::AWD1 => w.awd1ie().clear_bit(),
            Event::AWD2 => w.awd2ie().clear_bit(),
            Event::AWD3 => w.awd3ie().clear_bit(),
            Event::CCRDY => w.ccrdyie().clear_bit(),
            Event::EOSMP => w.eosmpie().clear_bit(),
            Event::OVR => w.ovrie().clear_bit(),
        });
    }

    pub fn is_pending(&self, event: Event) -> bool {
        let reg = self.rb.isr.read();

        match event {
            Event::EOCAL => reg.eocal().bit_is_set(),
            Event::ADRDY => reg.adrdy().bit_is_set(),
            Event::EOC => reg.eoc().bit_is_set(),
            Event::EOS => reg.eos().bit_is_set(),
            Event::AWD1 => reg.awd1().bit_is_set(),
            Event::AWD2 => reg.awd2().bit_is_set(),
            Event::AWD3 => reg.awd3().bit_is_set(),
            Event::CCRDY => reg.ccrdy().bit_is_set(),
            Event::EOSMP => reg.eosmp().bit_is_set(),
            Event::OVR => reg.ovr().bit_is_set(),
        }
    }

    pub fn unpend(&mut self, event: Event) {
        self.rb.isr.modify(|_, w| match event {
            Event::EOCAL => w.eocal().set_bit(),
            Event::ADRDY => w.adrdy().set_bit(),
            Event::EOC => w.eoc().set_bit(),
            Event::EOS => w.eos().set_bit(),
            Event::AWD1 => w.awd1().set_bit(),
            Event::AWD2 => w.awd2().set_bit(),
            Event::AWD3 => w.awd3().set_bit(),
            Event::CCRDY => w.ccrdy().set_bit(),
            Event::EOSMP => w.eosmp().set_bit(),
            Event::OVR => w.ovr().set_bit(),
        });
    }

    pub fn release(self) -> ADC {
        self.rb
    }

    fn power_up(&mut self) {
        self.rb.isr.modify(|_, w| w.adrdy().set_bit());
        self.rb.cr.modify(|_, w| w.aden().set_bit());
        while self.rb.isr.read().adrdy().bit_is_clear() {}
    }

    fn power_down(&mut self) {
        self.rb.cr.modify(|_, w| w.addis().set_bit());
        self.rb.isr.modify(|_, w| w.adrdy().set_bit());
        while self.rb.cr.read().aden().bit_is_set() {}
    }
}

pub trait AdcExt {
    fn constrain(self, rcc: &mut Rcc) -> Adc;
}

impl AdcExt for ADC {
    fn constrain(self, rcc: &mut Rcc) -> Adc {
        Adc::new(self, rcc)
    }
}

pub trait InjectMode<ADC, Pin: Channel<ADC>> {
    /// Error type returned by ADC methods
    type Error;
    fn prepare_injected(&mut self, _pin: &mut Pin, triger_source: InjTrigSource);
}

impl<PIN> InjectMode<Adc, PIN> for Adc
where
    // WORD: From<u16>,
    PIN: Channel<Adc, ID = u8>,
{
    type Error = ();

    fn prepare_injected(&mut self, _pin: &mut PIN, triger_source: InjTrigSource) {
        self.rb
            .cfgr1
            .modify(|_, w| unsafe { w.exten().bits(1).extsel().bits(triger_source as u8) });

        self.rb.cfgr1.modify(|_, w| unsafe {
            w.res() // set ADC resolution bits (ADEN must be =0)
                .bits(self.precision as u8)
                .align() // set alignment bit is  (ADSTART must be 0)
                .bit(self.align == Align::Left)
        });

        self.power_up();

        // self.rb
        //     .smpr // set sampling time set 1 (ADSTART must be 0)
        //     .modify(|_, w| unsafe { w.smp1().bits(self.sample_time as u8) });

        self.rb
            .chselr() // set activ channel acording chapter 15.12.9 (ADC_CFGR1; CHSELRMOD=0)
            .modify(|_, w| unsafe { w.chsel().bits(1 << PIN::channel()) });
    }
}

pub trait DmaMode<ADC> {
    /// Error type returned by ADC methods
    type Error;
    fn dma_enable(&mut self, enable: bool);
    fn dma_circular_mode(&mut self, enable: bool);
}

impl DmaMode<Adc> for Adc {
    type Error = ();

    fn dma_enable(&mut self, enable: bool) {
        if enable {
            self.rb.cfgr1.modify(|_, w| w.dmaen().set_bit()); //  enable dma beeing called
        } else {
            self.rb.cfgr1.modify(|_, w| w.dmaen().clear_bit()); //  disable dma beeing called
        }
    }

    fn dma_circular_mode(&mut self, enable: bool) {
        if enable {
            self.rb.cfgr1.modify(|_, w| w.dmacfg().set_bit()); // activate circular mode
        } else {
            self.rb.cfgr1.modify(|_, w| w.dmacfg().clear_bit()); // disable circular mode
        }
    }
}

impl<WORD, PIN> OneShot<Adc, WORD, PIN> for Adc
where
    WORD: From<u16>,
    PIN: Channel<Adc, ID = u8>,
{
    type Error = ();

    fn read(&mut self, _pin: &mut PIN) -> nb::Result<WORD, Self::Error> {
        self.power_up();
        self.rb.cfgr1.modify(|_, w| unsafe {
            w.res()
                .bits(self.precision as u8)
                .align()
                .bit(self.align == Align::Left)
        });

        // self.rb
        //     .smpr
        //     .modify(|_, w| unsafe { w.smp1().bits(self.sample_time as u8) });

        self.rb
            .chselr()
            .modify(|_, w| unsafe { w.chsel().bits(1 << PIN::channel()) });

        self.rb.isr.modify(|_, w| w.eos().set_bit());
        self.rb.cr.modify(|_, w| w.adstart().set_bit());
        while self.rb.isr.read().eos().bit_is_clear() {}

        let res = self.rb.dr.read().bits() as u16;
        let val = if self.align == Align::Left && self.precision == Precision::B6 {
            res << 8
        } else {
            res
        };

        self.power_down();
        Ok(val.into())
    }
}

macro_rules! int_adc {
    ($($Chan:ident: ($chan:expr, $en:ident)),+ $(,)*) => {
        $(
            pub struct $Chan;

            impl $Chan {
                pub fn new() -> Self {
                    Self {}
                }

                pub fn enable(&mut self, adc: &mut Adc) {
                    adc.rb.ccr.modify(|_, w| w.$en().set_bit());
                }

                pub fn disable(&mut self, adc: &mut Adc) {
                    adc.rb.ccr.modify(|_, w| w.$en().clear_bit());
                }

                pub fn enabled(&self, adc: &Adc) -> bool {
                    adc.rb.ccr.read().$en().bit_is_set()
                }
            }

            impl Default for $Chan {
                fn default() -> $Chan {
                    $Chan::new()
                }
            }

            impl Channel<Adc> for $Chan {
                type ID = u8;

                fn channel() -> u8 {
                    $chan
                }
            }
        )+
    };
}

int_adc! {
    VTemp: (12, tsen),
    VRef: (13, vrefen),
    VBat: (14, vbaten),
}

macro_rules! adc_pin {
    ($($Chan:ty: ($pin:ty, $chan:expr)),+ $(,)*) => {
        $(
            impl Channel<Adc> for $pin {
                type ID = u8;

                fn channel() -> u8 { $chan }
            }

            // proto-hal
            impl AdcPin<Adc> for $pin {
                type ID = u8;
                const CHANNEL: Self::ID = $chan;
            }
        )+
    };
}

adc_pin! {
    Channel0: (gpioa::PA0<Analog>, 0u8),
    Channel1: (gpioa::PA1<Analog>, 1u8),
    Channel2: (gpioa::PA2<Analog>, 2u8),
    Channel3: (gpioa::PA3<Analog>, 3u8),
    Channel4: (gpioa::PA4<Analog>, 4u8),
    Channel5: (gpioa::PA5<Analog>, 5u8),
    Channel6: (gpioa::PA6<Analog>, 6u8),
    Channel7: (gpioa::PA7<Analog>, 7u8),
    Channel8: (gpiob::PB0<Analog>, 8u8),
    Channel9: (gpiob::PB1<Analog>, 9u8),
    Channel10: (gpiob::PB2<Analog>, 10u8),
    Channel11: (gpiob::PB10<Analog>, 11u8),
    Channel15: (gpiob::PB11<Analog>, 15u8),
    Channel16: (gpiob::PB12<Analog>, 16u8),
}

#[cfg(any(feature = "stm32g030", feature = "stm32g031", feature = "stm32g041",))]
adc_pin! {
    Channel11: (gpiob::PB7<Analog>, 11u8),
    Channel15: (gpioa::PA11<Analog>, 15u8),
    Channel16: (gpioa::PA12<Analog>, 16u8),
    Channel17: (gpioa::PA13<Analog>, 17u8),
    Channel18: (gpioa::PA14<Analog>, 18u8),
}

#[cfg(any(feature = "stm32g070", feature = "stm32g071", feature = "stm32g081",))]
adc_pin! {
    Channel17: (gpioc::PC4<Analog>, 17u8),
    Channel18: (gpioc::PC5<Analog>, 18u8),
}

pub enum Direction {
    Forward,
    Backward,
}

pub struct BasicSequence<const N: usize> {
    channel_select: u32,
    direction: Direction,
}

pub struct FullSequence<const N: usize> {
    channels: [u8; N],
}

pub trait BasicSequencePins<const N: usize> {
    fn into_basic_sequence(self, direction: Direction) -> BasicSequence<N>;
}
pub trait FullSequencePins<const N: usize> {
    fn into_full_sequence(self) -> FullSequence<N>;
}

macro_rules! impl_basic_sequences {
    ( $( ($N:expr, ( $($CH:ident),+ )) ),+ $(,)? ) => {
        $(
            impl<$( $CH ),+> BasicSequencePins<$N> for ($( &mut $CH ),+)
            where
                $(
                    $CH: AdcPin<Adc, ID = u8>
                ),+
            {
                fn into_basic_sequence(self, direction: Direction) -> BasicSequence<$N> {
                    BasicSequence {
                        channel_select: [ $( $CH::CHANNEL ),+ ].iter().fold(0, |partial, next| partial + (1 << next)),
                        direction,
                    }
                }
            }
        )+
    };
}

macro_rules! impl_full_sequences {
    ( $( ($N:expr, ( $($CH:ident),+ )) ),+ $(,)? ) => {
        $(
            impl<$( $CH ),+> FullSequencePins<$N> for ($( &mut $CH ),+)
            where
                $(
                    $CH: AdcPin<Adc, ID = u8>
                ),+
            {
                fn into_full_sequence(self) -> FullSequence<$N> {
                    FullSequence {
                        channels: [ $($CH::CHANNEL),+ ]
                    }
                }
            }
        )+
    };
}

// $RM0444 15.12.9 - There are up to 19 conversions in a basic configured sequence

impl_basic_sequences!(
    (2, (A, B)),
    (3, (A, B, C)),
    (4, (A, B, C, D)),
    (5, (A, B, C, D, E)),
    (6, (A, B, C, D, E, F)),
    (7, (A, B, C, D, E, F, G)),
    (8, (A, B, C, D, E, F, G, H)),
    (9, (A, B, C, D, E, F, G, H, I)),
    (10, (A, B, C, D, E, F, G, H, I, J)),
    (11, (A, B, C, D, E, F, G, H, I, J, K)),
    (12, (A, B, C, D, E, F, G, H, I, J, K, L)),
    (13, (A, B, C, D, E, F, G, H, I, J, K, L, M)),
    (14, (A, B, C, D, E, F, G, H, I, J, K, L, M, N)),
    (15, (A, B, C, D, E, F, G, H, I, J, K, L, M, N, O)),
    (16, (A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P)),
    (17, (A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q)),
    (18, (A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R)),
    (
        19,
        (A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S)
    ),
);

// $RM0444 15.12.10 - There are up to 8 conversions in a fully configured sequence

impl_full_sequences!(
    (2, (A, B)),
    (3, (A, B, C)),
    (4, (A, B, C, D)),
    (5, (A, B, C, D, E)),
    (6, (A, B, C, D, E, F)),
    (7, (A, B, C, D, E, F, G)),
    (8, (A, B, C, D, E, F, G, H)),
);
