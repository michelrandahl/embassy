//! TSC Peripheral Interface
//!
//!
//! # Example (stm32)
//! ``` rust, ignore
//!
//! let mut device_config = embassy_stm32::Config::default();
//! {
//!     device_config.rcc.mux = ClockSrc::MSI(Msirange::RANGE_4MHZ);
//! }
//!
//! let context = embassy_stm32::init(device_config);
//!
//! let config = tsc::Config {
//!     ct_pulse_high_length: ChargeTransferPulseCycle::_2,
//!     ct_pulse_low_length: ChargeTransferPulseCycle::_2,
//!     spread_spectrum: false,
//!     spread_spectrum_deviation: SSDeviation::new(2).unwrap(),
//!     spread_spectrum_prescaler: false,
//!     pulse_generator_prescaler: PGPrescalerDivider::_4,
//!     max_count_value: MaxCount::_8191,
//!     io_default_mode: false,
//!     synchro_pin_polarity: false,
//!     acquisition_mode: false,
//!     max_count_interrupt: false,
//!     channel_ios: TscIOPin::Group2Io2 | TscIOPin::Group7Io3,
//!     shield_ios: TscIOPin::Group1Io3.into(),
//!     sampling_ios: TscIOPin::Group1Io2 | TscIOPin::Group2Io1 | TscIOPin::Group7Io2,
//! };
//!
//! let mut g1: PinGroup<embassy_stm32::peripherals::TSC, G1> = PinGroup::new();
//! g1.set_io2(context.PB13, PinType::Sample);
//! g1.set_io3(context.PB14, PinType::Shield);
//!
//! let mut g2: PinGroup<embassy_stm32::peripherals::TSC, G2> = PinGroup::new();
//! g2.set_io1(context.PB4, PinType::Sample);
//! g2.set_io2(context.PB5, PinType::Channel);
//!
//! let mut g7: PinGroup<embassy_stm32::peripherals::TSC, G7> = PinGroup::new();
//! g7.set_io2(context.PE3, PinType::Sample);
//! g7.set_io3(context.PE4, PinType::Channel);
//!
//! let mut touch_controller = tsc::Tsc::new_blocking(
//!     context.TSC,
//!     Some(g1),
//!     Some(g2),
//!     None,
//!     None,
//!     None,
//!     None,
//!     Some(g7),
//!     None,
//!     config,
//! );
//!
//! touch_controller.discharge_io(true);
//! Timer::after_millis(1).await;
//!
//! touch_controller.start();
//!
//! ```

#![macro_use]

/// Enums defined for peripheral parameters
pub mod enums;

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_hal_internal::{into_ref, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;
pub use enums::*;

use crate::gpio::{AfType, AnyPin, OutputType, Speed};
use crate::interrupt::typelevel::Interrupt;
use crate::mode::{Async, Blocking, Mode as PeriMode};
use crate::rcc::{self, RccPeripheral};
use crate::{interrupt, peripherals, Peripheral};

#[cfg(tsc_v1)]
const TSC_NUM_GROUPS: u32 = 6;
#[cfg(tsc_v2)]
const TSC_NUM_GROUPS: u32 = 7;
#[cfg(tsc_v3)]
const TSC_NUM_GROUPS: u32 = 8;

/// Error type defined for TSC
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Test error for TSC
    Test,
}

/// TSC interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        T::regs().ier().write(|w| w.set_eoaie(false));
        T::waker().wake();
    }
}

/// Pin type definition to control IO parameters
#[derive(PartialEq,Clone,Copy)]
pub enum PinType {
    /// Sensing channel pin connected to an electrode
    Channel,
    /// Sampling capacitor pin, one required for every pin group
    Sample,
    /// Shield pin connected to capacitive sensing shield
    Shield,
}

/// Peripheral state
#[derive(PartialEq, Clone, Copy)]
pub enum State {
    /// Peripheral is being setup or reconfigured
    Reset,
    /// Ready to start acquisition
    Ready,
    /// In process of sensor acquisition
    Busy,
    /// Error occured during acquisition
    Error,
}

/// Individual group status checked after acquisition reported as complete
/// For groups with multiple channel pins, may take longer because acquisitions
/// are done sequentially. Check this status before pulling count for each
/// sampled channel
#[derive(PartialEq,Clone,Copy)]
pub enum GroupStatus {
    /// Acquisition for channel still in progress
    Ongoing,
    /// Acquisition either not started or complete
    Complete,
}

/// Peripheral configuration
#[derive(Clone, Copy)]
pub struct Config {
    /// Duration of high state of the charge transfer pulse
    pub ct_pulse_high_length: ChargeTransferPulseCycle,
    /// Duration of the low state of the charge transfer pulse
    pub ct_pulse_low_length: ChargeTransferPulseCycle,
    /// Enable/disable of spread spectrum feature
    pub spread_spectrum: bool,
    /// Adds variable number of periods of the SS clk to pulse high state
    pub spread_spectrum_deviation: SSDeviation,
    /// Selects AHB clock divider used to generate SS clk
    pub spread_spectrum_prescaler: bool,
    /// Selects AHB clock divider used to generate pulse generator clk
    pub pulse_generator_prescaler: PGPrescalerDivider,
    /// Maximum number of charge transfer pulses that can be generated before error
    pub max_count_value: MaxCount,
    /// Defines config of all IOs when no ongoing acquisition
    pub io_default_mode: bool,
    /// Polarity of sync input pin
    pub synchro_pin_polarity: bool,
    /// Acquisition starts when start bit is set or with sync pin input
    pub acquisition_mode: bool,
    /// Enable max count interrupt
    pub max_count_interrupt: bool,
    /// Channel IO mask
    pub channel_ios: u32,
    /// Shield IO mask
    pub shield_ios: u32,
    /// Sampling IO mask
    pub sampling_ios: u32,
}

impl Config {
    /// Automatically configures the IO masks for channels, shields, and sampling pins based on the provided pin groups.
    ///
    /// This method sets the `channel_ios`, `shield_ios`, and `sampling_ios` fields of the `Config` struct
    /// by collecting and combining the appropriate pins from each provided pin group.
    ///
    /// # Arguments
    ///
    /// * `g1` to `g6` - References to `Option<PinGroup>` for groups 1 through 6.
    /// * `g7` - Reference to `Option<PinGroup>` for group 7 (only available for TSC v2 and v3).
    /// * `g8` - Reference to `Option<PinGroup>` for group 8 (only available for TSC v3).
    ///
    /// # Note
    ///
    /// This method automatically handles different TSC versions by conditionally including pins from groups 7 and 8
    /// based on the TSC version defined by feature flags.
    ///
    /// # Example
    ///
    /// ```
    /// let mut config = Config::default();
    /// config.configure_io_masks(
    ///     Some(&pin_group1),
    ///     None,
    ///     None,
    ///     Some(&pin_group4),
    ///     None,
    ///     None,
    ///     None,
    ///     None
    /// );
    /// ```
    pub fn configure_io_masks<'d, T: Instance>(
        &mut self,
        g1: Option<&PinGroup<'d, T, G1>>,
        g2: Option<&PinGroup<'d, T, G2>>,
        g3: Option<&PinGroup<'d, T, G3>>,
        g4: Option<&PinGroup<'d, T, G4>>,
        g5: Option<&PinGroup<'d, T, G5>>,
        g6: Option<&PinGroup<'d, T, G6>>,
        #[cfg(any(tsc_v2, tsc_v3))] g7: Option<&PinGroup<'d, T, G7>>,
        #[cfg(tsc_v3)] g8: Option<&PinGroup<'d, T, G8>>
    ) {
        self.channel_ios =
            g1.map_or(0, |g| g.make_channel_ios_mask()) |
            g2.map_or(0, |g| g.make_channel_ios_mask()) |
            g3.map_or(0, |g| g.make_channel_ios_mask()) |
            g4.map_or(0, |g| g.make_channel_ios_mask()) |
            g5.map_or(0, |g| g.make_channel_ios_mask()) |
            g6.map_or(0, |g| g.make_channel_ios_mask());
        #[cfg(any(tsc_v2, tsc_v3))]
        { self.channel_ios |= g7.map_or(0, |g| g.make_channel_ios_mask()); }
        #[cfg(tsc_v3)]
        { self.channel_ios |= g8.map_or(0, |g| g.make_channel_ios_mask()); }

        self.shield_ios =
            g1.map_or(0, |g| g.make_shield_ios_mask()) |
            g2.map_or(0, |g| g.make_shield_ios_mask()) |
            g3.map_or(0, |g| g.make_shield_ios_mask()) |
            g4.map_or(0, |g| g.make_shield_ios_mask()) |
            g5.map_or(0, |g| g.make_shield_ios_mask()) |
            g6.map_or(0, |g| g.make_shield_ios_mask());
        #[cfg(any(tsc_v2, tsc_v3))]
        { self.shield_ios |= g7.map_or(0, |g| g.make_shield_ios_mask()); }
        #[cfg(tsc_v3)]
        { self.shield_ios |= g8.map_or(0, |g| g.make_shield_ios_mask()); }

        self.sampling_ios =
            g1.map_or(0, |g| g.make_sample_ios_mask()) |
            g2.map_or(0, |g| g.make_sample_ios_mask()) |
            g3.map_or(0, |g| g.make_sample_ios_mask()) |
            g4.map_or(0, |g| g.make_sample_ios_mask()) |
            g5.map_or(0, |g| g.make_sample_ios_mask()) |
            g6.map_or(0, |g| g.make_sample_ios_mask());
        #[cfg(any(tsc_v2, tsc_v3))]
        { self.sampling_ios |= g7.map_or(0, |g| g.make_sample_ios_mask()); }
        #[cfg(tsc_v3)]
        { self.sampling_ios |= g8.map_or(0, |g| g.make_sample_ios_mask()); }
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            ct_pulse_high_length: ChargeTransferPulseCycle::_1,
            ct_pulse_low_length: ChargeTransferPulseCycle::_1,
            spread_spectrum: false,
            spread_spectrum_deviation: SSDeviation::new(1).unwrap(),
            spread_spectrum_prescaler: false,
            pulse_generator_prescaler: PGPrescalerDivider::_1,
            max_count_value: MaxCount::_255,
            io_default_mode: false,
            synchro_pin_polarity: false,
            acquisition_mode: false,
            max_count_interrupt: false,
            channel_ios: 0,
            shield_ios: 0,
            sampling_ios: 0,
        }
    }
}

/// Pin struct that maintains usage
#[allow(missing_docs)]
pub struct TscPin<'d, T, C> {
    _pin: PeripheralRef<'d, AnyPin>,
    role: PinType,
    tsc_io_pin: TscIOPin,
    phantom: PhantomData<(T, C)>,
}

/// Represents errors that can occur when configuring or validating TSC pin groups.
#[derive(Debug)]
pub enum GroupError {
    /// Error when a group has no sampling capacitor
    NoSamplingCapacitor,
    /// Error when a group has neither channel IOs nor a shield IO
    NoChannelOrShield,
    /// Error when a group has both channel IOs and a shield IO
    MixedChannelAndShield,
    /// Error when there is more than one shield IO across all groups
    MultipleShields,
}

/// Pin group definition
/// Pins are organized into groups of four IOs, all groups with a
/// sampling channel must also have a sampling capacitor channel.
#[allow(missing_docs)]
#[derive(Default)]
pub struct PinGroup<'d, T, C> {
    d1: Option<TscPin<'d, T, C>>,
    d2: Option<TscPin<'d, T, C>>,
    d3: Option<TscPin<'d, T, C>>,
    d4: Option<TscPin<'d, T, C>>,
}

impl<'d, T: Instance, C> PinGroup<'d, T, C> {
    /// Create new sensing group
    pub fn new() -> Self {
        Self {
            d1: None,
            d2: None,
            d3: None,
            d4: None,
        }
    }

    fn contains_shield(&self) -> bool {
        let mut shield_count = 0;

        if let Some(pin) = &self.d1 {
            if let PinType::Shield = pin.role {
                shield_count += 1;
            }
        }

        if let Some(pin) = &self.d2 {
            if let PinType::Shield = pin.role {
                shield_count += 1;
            }
        }

        if let Some(pin) = &self.d3 {
            if let PinType::Shield = pin.role {
                shield_count += 1;
            }
        }

        if let Some(pin) = &self.d4 {
            if let PinType::Shield = pin.role {
                shield_count += 1;
            }
        }

        shield_count == 1
    }

    fn check_group(&self) -> Result<(), GroupError> {
        let mut channel_count = 0;
        let mut shield_count = 0;
        let mut sample_count = 0;
        if let Some(pin) = &self.d1 {
            match pin.role {
                PinType::Channel => {
                    channel_count += 1;
                }
                PinType::Shield => {
                    shield_count += 1;
                }
                PinType::Sample => {
                    sample_count += 1;
                }
            }
        }

        if let Some(pin) = &self.d2 {
            match pin.role {
                PinType::Channel => {
                    channel_count += 1;
                }
                PinType::Shield => {
                    shield_count += 1;
                }
                PinType::Sample => {
                    sample_count += 1;
                }
            }
        }

        if let Some(pin) = &self.d3 {
            match pin.role {
                PinType::Channel => {
                    channel_count += 1;
                }
                PinType::Shield => {
                    shield_count += 1;
                }
                PinType::Sample => {
                    sample_count += 1;
                }
            }
        }

        if let Some(pin) = &self.d4 {
            match pin.role {
                PinType::Channel => {
                    channel_count += 1;
                }
                PinType::Shield => {
                    shield_count += 1;
                }
                PinType::Sample => {
                    sample_count += 1;
                }
            }
        }

        // Every group requires one sampling capacitor
        if sample_count != 1 {
            return Err(GroupError::NoSamplingCapacitor);
        }

        // Each group must have at least one shield or channel IO
        if shield_count == 0 && channel_count == 0 {
            return Err(GroupError::NoChannelOrShield);
        }

        // Any group can either contain channel ios or a shield IO.
        // (An active shield requires its own sampling capacitor)
        if shield_count != 0 && channel_count != 0 {
            return Err(GroupError::MixedChannelAndShield);
        }

        // No more than one shield IO is allow per group and amongst all groups
        if shield_count > 1 {
            return Err(GroupError::MultipleShields);
        }

        Ok(())
    }

    /// Get information about the first pin in the group.
    pub fn get_io1(&self) -> Option<(TscIOPin, PinType)> {
        self.d1.as_ref().map(|pin| (pin.tsc_io_pin, pin.role))
    }

    /// Get information about the second pin in the group.
    pub fn get_io2(&self) -> Option<(TscIOPin, PinType)> {
        self.d2.as_ref().map(|pin| (pin.tsc_io_pin, pin.role))
    }

    /// Get information about the third pin in the group.
    pub fn get_io3(&self) -> Option<(TscIOPin, PinType)> {
        self.d3.as_ref().map(|pin| (pin.tsc_io_pin, pin.role))
    }

    /// Get information about the fourth pin in the group.
    pub fn get_io4(&self) -> Option<(TscIOPin, PinType)> {
        self.d4.as_ref().map(|pin| (pin.tsc_io_pin, pin.role))
    }

    /// Returns an iterator over the sample pins in the group
    pub fn sample_pins(&self) -> impl Iterator<Item = TscIOPin> + '_ {
        self.pin_iterator(PinType::Sample)
    }

    /// Returns an iterator over the shield pins in the group
    pub fn shield_pins(&self) -> impl Iterator<Item = TscIOPin> + '_ {
        self.pin_iterator(PinType::Shield)
    }

    /// Returns an iterator over the channel pins in the group
    pub fn channel_pins(&self) -> impl Iterator<Item = TscIOPin> + '_ {
        self.pin_iterator(PinType::Channel)
    }

    /// Generic iterator function
    fn pin_iterator(&self, pin_type: PinType) -> impl Iterator<Item = TscIOPin> + '_ {
        [&self.d1, &self.d2, &self.d3, &self.d4]
            .into_iter()
            .filter_map(move |pin| {
                pin.as_ref().and_then(|p| {
                    if p.role == pin_type {
                        Some(p.tsc_io_pin)
                    } else {
                        None
                    }
                })
            })
    }

    /// Creates a mask of all channel pins and combines them into a single u32 value.
    pub fn make_channel_ios_mask(&self) -> u32 {
        self.channel_pins().fold(0, |acc, pin| {acc | pin})
    }

    /// Creates a mask of all shield pins and combines them into a single u32 value.
    pub fn make_shield_ios_mask(&self) -> u32 {
        self.shield_pins().fold(0, |acc, pin| {acc | pin})
    }

    /// Creates a mask of all sample pins and combines them into a single u32 value.
    pub fn make_sample_ios_mask(&self) -> u32 {
        self.sample_pins().fold(0, |acc, pin| {acc | pin})
    }
}

macro_rules! trait_to_tsc_io_pin {
    (G1IO1Pin) => { TscIOPin::Group1Io1 };
    (G1IO2Pin) => { TscIOPin::Group1Io2 };
    (G1IO3Pin) => { TscIOPin::Group1Io3 };
    (G1IO4Pin) => { TscIOPin::Group1Io4 };

    (G2IO1Pin) => { TscIOPin::Group2Io1 };
    (G2IO2Pin) => { TscIOPin::Group2Io2 };
    (G2IO3Pin) => { TscIOPin::Group2Io3 };
    (G2IO4Pin) => { TscIOPin::Group2Io4 };

    (G3IO1Pin) => { TscIOPin::Group3Io1 };
    (G3IO2Pin) => { TscIOPin::Group3Io2 };
    (G3IO3Pin) => { TscIOPin::Group3Io3 };
    (G3IO4Pin) => { TscIOPin::Group3Io4 };

    (G4IO1Pin) => { TscIOPin::Group4Io1 };
    (G4IO2Pin) => { TscIOPin::Group4Io2 };
    (G4IO3Pin) => { TscIOPin::Group4Io3 };
    (G4IO4Pin) => { TscIOPin::Group4Io4 };

    (G5IO1Pin) => { TscIOPin::Group5Io1 };
    (G5IO2Pin) => { TscIOPin::Group5Io2 };
    (G5IO3Pin) => { TscIOPin::Group5Io3 };
    (G5IO4Pin) => { TscIOPin::Group5Io4 };

    (G6IO1Pin) => { TscIOPin::Group6Io1 };
    (G6IO2Pin) => { TscIOPin::Group6Io2 };
    (G6IO3Pin) => { TscIOPin::Group6Io3 };
    (G6IO4Pin) => { TscIOPin::Group6Io4 };

    (G7IO1Pin) => { TscIOPin::Group7Io1 };
    (G7IO2Pin) => { TscIOPin::Group7Io2 };
    (G7IO3Pin) => { TscIOPin::Group7Io3 };
    (G7IO4Pin) => { TscIOPin::Group7Io4 };

    (G8IO1Pin) => { TscIOPin::Group8Io1 };
    (G8IO2Pin) => { TscIOPin::Group8Io2 };
    (G8IO3Pin) => { TscIOPin::Group8Io3 };
    (G8IO4Pin) => { TscIOPin::Group8Io4 };
}

macro_rules! group_impl {
    ($group:ident, $trait1:ident, $trait2:ident, $trait3:ident, $trait4:ident) => {
        impl<'d, T: Instance> PinGroup<'d, T, $group> {
            #[doc = concat!("Create a new pin1 for ", stringify!($group), " TSC group instance.")]
            pub fn set_io1(&mut self, pin: impl Peripheral<P = impl $trait1<T>> + 'd, role: PinType) {
                into_ref!(pin);
                critical_section::with(|_| {
                    pin.set_low();
                    pin.set_as_af(
                        pin.af_num(),
                        AfType::output(
                            match role {
                                PinType::Channel => OutputType::PushPull,
                                PinType::Sample => OutputType::OpenDrain,
                                PinType::Shield => OutputType::PushPull,
                            },
                            Speed::VeryHigh,
                        ),
                    );
                    self.d1 = Some(TscPin {
                        _pin: pin.map_into(),
                        role: role,
                        tsc_io_pin: trait_to_tsc_io_pin!($trait1),
                        phantom: PhantomData,
                    })
                })
            }

            #[doc = concat!("Create a new pin2 for ", stringify!($group), " TSC group instance.")]
            pub fn set_io2(&mut self, pin: impl Peripheral<P = impl $trait2<T>> + 'd, role: PinType) {
                into_ref!(pin);
                critical_section::with(|_| {
                    pin.set_low();
                    pin.set_as_af(
                        pin.af_num(),
                        AfType::output(
                            match role {
                                PinType::Channel => OutputType::PushPull,
                                PinType::Sample => OutputType::OpenDrain,
                                PinType::Shield => OutputType::PushPull,
                            },
                            Speed::VeryHigh,
                        ),
                    );
                    self.d2 = Some(TscPin {
                        _pin: pin.map_into(),
                        role: role,
                        tsc_io_pin: trait_to_tsc_io_pin!($trait2),
                        phantom: PhantomData,
                    })
                })
            }

            #[doc = concat!("Create a new pin3 for ", stringify!($group), " TSC group instance.")]
            pub fn set_io3(&mut self, pin: impl Peripheral<P = impl $trait3<T>> + 'd, role: PinType) {
                into_ref!(pin);
                critical_section::with(|_| {
                    pin.set_low();
                    pin.set_as_af(
                        pin.af_num(),
                        AfType::output(
                            match role {
                                PinType::Channel => OutputType::PushPull,
                                PinType::Sample => OutputType::OpenDrain,
                                PinType::Shield => OutputType::PushPull,
                            },
                            Speed::VeryHigh,
                        ),
                    );
                    self.d3 = Some(TscPin {
                        _pin: pin.map_into(),
                        role: role,
                        tsc_io_pin: trait_to_tsc_io_pin!($trait3),
                        phantom: PhantomData,
                    })
                })
            }

            #[doc = concat!("Create a new pin4 for ", stringify!($group), " TSC group instance.")]
            pub fn set_io4(&mut self, pin: impl Peripheral<P = impl $trait4<T>> + 'd, role: PinType) {
                into_ref!(pin);
                critical_section::with(|_| {
                    pin.set_low();
                    pin.set_as_af(
                        pin.af_num(),
                        AfType::output(
                            match role {
                                PinType::Channel => OutputType::PushPull,
                                PinType::Sample => OutputType::OpenDrain,
                                PinType::Shield => OutputType::PushPull,
                            },
                            Speed::VeryHigh,
                        ),
                    );
                    self.d4 = Some(TscPin {
                        _pin: pin.map_into(),
                        role: role,
                        tsc_io_pin: trait_to_tsc_io_pin!($trait4),
                        phantom: PhantomData,
                    })
                })
            }
        }
    };
}

group_impl!(G1, G1IO1Pin, G1IO2Pin, G1IO3Pin, G1IO4Pin);
group_impl!(G2, G2IO1Pin, G2IO2Pin, G2IO3Pin, G2IO4Pin);
group_impl!(G3, G3IO1Pin, G3IO2Pin, G3IO3Pin, G3IO4Pin);
group_impl!(G4, G4IO1Pin, G4IO2Pin, G4IO3Pin, G4IO4Pin);
group_impl!(G5, G5IO1Pin, G5IO2Pin, G5IO3Pin, G5IO4Pin);
group_impl!(G6, G6IO1Pin, G6IO2Pin, G6IO3Pin, G6IO4Pin);
group_impl!(G7, G7IO1Pin, G7IO2Pin, G7IO3Pin, G7IO4Pin);
group_impl!(G8, G8IO1Pin, G8IO2Pin, G8IO3Pin, G8IO4Pin);

/// Group 1 marker type.
pub enum G1 {}
/// Group 2 marker type.
pub enum G2 {}
/// Group 3 marker type.
pub enum G3 {}
/// Group 4 marker type.
pub enum G4 {}
/// Group 5 marker type.
pub enum G5 {}
/// Group 6 marker type.
pub enum G6 {}
/// Group 7 marker type.
pub enum G7 {}
/// Group 8 marker type.
pub enum G8 {}

/// TSC driver
pub struct Tsc<'d, T: Instance, K: PeriMode> {
    _peri: PeripheralRef<'d, T>,
    _g1: Option<PinGroup<'d, T, G1>>,
    _g2: Option<PinGroup<'d, T, G2>>,
    _g3: Option<PinGroup<'d, T, G3>>,
    _g4: Option<PinGroup<'d, T, G4>>,
    _g5: Option<PinGroup<'d, T, G5>>,
    _g6: Option<PinGroup<'d, T, G6>>,
    #[cfg(any(tsc_v2, tsc_v3))]
    _g7: Option<PinGroup<'d, T, G7>>,
    #[cfg(tsc_v3)]
    _g8: Option<PinGroup<'d, T, G8>>,
    state: State,
    config: Config,
    _kind: PhantomData<K>,
}

impl<'d, T: Instance> Tsc<'d, T, Async> {
    /// Create a Tsc instance that can be awaited for completion
    pub fn new_async(
        peri: impl Peripheral<P = T> + 'd,
        g1: Option<PinGroup<'d, T, G1>>,
        g2: Option<PinGroup<'d, T, G2>>,
        g3: Option<PinGroup<'d, T, G3>>,
        g4: Option<PinGroup<'d, T, G4>>,
        g5: Option<PinGroup<'d, T, G5>>,
        g6: Option<PinGroup<'d, T, G6>>,
        #[cfg(any(tsc_v2, tsc_v3))] g7: Option<PinGroup<'d, T, G7>>,
        #[cfg(tsc_v3)] g8: Option<PinGroup<'d, T, G8>>,
        config: Config,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    ) -> Result<Self, GroupError> {
        // Need to check valid pin configuration input
        let g1 = g1.filter(|b| b.check_group().is_ok());
        let g2 = g2.filter(|b| b.check_group().is_ok());
        let g3 = g3.filter(|b| b.check_group().is_ok());
        let g4 = g4.filter(|b| b.check_group().is_ok());
        let g5 = g5.filter(|b| b.check_group().is_ok());
        let g6 = g6.filter(|b| b.check_group().is_ok());
        #[cfg(any(tsc_v2, tsc_v3))]
        let g7 = g7.filter(|b| b.check_group().is_ok());
        #[cfg(tsc_v3)]
        let g8 = g8.filter(|b| b.check_group().is_ok());

        Self::check_shields(
            &g1,
            &g2,
            &g3,
            &g4,
            &g5,
            &g6,
            #[cfg(any(tsc_v2, tsc_v3))]
            &g7,
            #[cfg(tsc_v3)]
            &g8,
        )?;
        Ok(
            Self::new_inner(
                peri,
                g1,
                g2,
                g3,
                g4,
                g5,
                g6,
                #[cfg(any(tsc_v2, tsc_v3))]
                g7,
                #[cfg(tsc_v3)]
                g8,
                config,
            )
        )
    }

    /// Asyncronously wait for the end of an acquisition
    pub async fn pend_for_acquisition(&mut self) {
        poll_fn(|cx| match self.get_state() {
            State::Busy => {
                T::waker().register(cx.waker());
                T::regs().ier().write(|w| w.set_eoaie(true));
                if self.get_state() != State::Busy {
                    T::regs().ier().write(|w| w.set_eoaie(false));
                    return Poll::Ready(());
                }
                Poll::Pending
            }
            _ => {
                T::regs().ier().write(|w| w.set_eoaie(false));
                Poll::Ready(())
            }
        })
        .await;
    }
}

impl<'d, T: Instance> Tsc<'d, T, Blocking> {
    /// Create a Tsc instance that must be polled for completion
    pub fn new_blocking(
        peri: impl Peripheral<P = T> + 'd,
        g1: Option<PinGroup<'d, T, G1>>,
        g2: Option<PinGroup<'d, T, G2>>,
        g3: Option<PinGroup<'d, T, G3>>,
        g4: Option<PinGroup<'d, T, G4>>,
        g5: Option<PinGroup<'d, T, G5>>,
        g6: Option<PinGroup<'d, T, G6>>,
        #[cfg(any(tsc_v2, tsc_v3))] g7: Option<PinGroup<'d, T, G7>>,
        #[cfg(tsc_v3)] g8: Option<PinGroup<'d, T, G8>>,
        config: Config,
    ) -> Result<Self, GroupError> {
        // Need to check valid pin configuration input
        let g1 = g1.filter(|b| b.check_group().is_ok());
        let g2 = g2.filter(|b| b.check_group().is_ok());
        let g3 = g3.filter(|b| b.check_group().is_ok());
        let g4 = g4.filter(|b| b.check_group().is_ok());
        let g5 = g5.filter(|b| b.check_group().is_ok());
        let g6 = g6.filter(|b| b.check_group().is_ok());
        #[cfg(any(tsc_v2, tsc_v3))]
        let g7 = g7.filter(|b| b.check_group().is_ok());
        #[cfg(tsc_v3)]
        let g8 = g8.filter(|b| b.check_group().is_ok());

        Self::check_shields(
            &g1,
            &g2,
            &g3,
            &g4,
            &g5,
            &g6,
            #[cfg(any(tsc_v2, tsc_v3))]
            &g7,
            #[cfg(tsc_v3)]
            &g8,
        )?;
        Ok(
            Self::new_inner(
                peri,
                g1,
                g2,
                g3,
                g4,
                g5,
                g6,
                #[cfg(any(tsc_v2, tsc_v3))]
                g7,
                #[cfg(tsc_v3)]
                g8,
                config
            )
        )
    }
    /// Wait for end of acquisition
    pub fn poll_for_acquisition(&mut self) {
        while self.get_state() == State::Busy {}
    }
}

/// Error returned when attempting to set an invalid channel pin as active in the TSC.
#[derive(Debug)]
pub enum SetActiveChannelsError {
    /// Indicates that one or more of the provided pins is not a valid channel pin.
    InvalidChannelPin,
    /// Indicates that multiple channels from the same group were provided.
    MultipleChannelsPerGroup,
}

impl<'d, T: Instance, K: PeriMode> Tsc<'d, T, K> {

    /// Sets the active channels for the next TSC acquisition.
    /// This method configures which sensor channels will be read during the next
    /// touch sensing acquisition cycle. It should be called before starting a new
    /// acquisition with the start() method.
    ///
    /// # Arguments
    ///
    /// * `channels` - A slice of `TscIOPin` representing the channels to activate.
    ///                Only one channel per group should be provided.
    ///
    /// # Returns
    ///
    /// * `Ok(())` if all channels were successfully set
    /// * `Err(SetActiveChannelsError::InvalidChannelPin)` if any provided pin is not a channel pin
    /// * `Err(SetActiveChannelsError::MultipleChannelsPerGroup)` if multiple channels from the same group are provided
    ///
    /// # Example
    /// ``` rust, ignore
    /// // Activate channel 2 of group 4 and channel 3 of group 5
    /// touch_controller.set_active_channels(&[TscIOPin::Group4Io2, TscIOPin::Group5Io3])?;
    /// ```
    pub fn set_active_channels(&mut self, channels: &[TscIOPin]) -> Result<(), SetActiveChannelsError> {
        let mut group_mask : u8 = 0;
        let mut channel_mask = 0u32;

        for &channel in channels {
            if !self.is_channel_pin(channel) {
                return Err(SetActiveChannelsError::InvalidChannelPin);
            }

            let group = channel.group();
            let group_bit : u8 = 1 << Into::<usize>::into(group);
            if group_mask & (1 << group_bit) != 0 {
                return Err(SetActiveChannelsError::MultipleChannelsPerGroup);
            }

            group_mask |= 1 << Into::<usize>::into(group);
            channel_mask |= channel;
        }

        T::regs()
            .ioccr()
            .write(|w| w.0 = channel_mask | self.config.shield_ios);

        Ok(())
    }

    // Helper method to check if a pin is a channel pin
    fn is_channel_pin(&self, pin: TscIOPin) -> bool {
        (self.config.channel_ios & pin) != 0
    }

    // TODO delete this, IFF that above works as intended
    //pub fn set_active_channels(&mut self, channels: u32) {
    //    //let shield_ios = self.config.shield_ios;
    //    T::regs()
    //        .iohcr()
    //        .write(|w| w.0 = !(channels | self.config.sampling_ios));
    //    T::regs()
    //        .ioccr()
    //        //.write(|w| w.0 = channels | shield_ios);
    //        .write(|w| w.0 = channels);
    //    T::regs().ioscr().write(|w| w.0 = self.config.sampling_ios);
    //    // Set the groups to be acquired
    //    T::regs()
    //        .iogcsr()
    //        .write(|w| w.0 = Self::extract_groups(channels));
    //    //T::regs()
    //    //    .ioccr()
    //    //    .modify(|w| w.0 = channels);
    //    //T::regs()
    //    //    .ioccr()
    //    //    .read().0
    //}

    /// Create new TSC driver
    fn check_shields(
        g1: &Option<PinGroup<'d, T, G1>>,
        g2: &Option<PinGroup<'d, T, G2>>,
        g3: &Option<PinGroup<'d, T, G3>>,
        g4: &Option<PinGroup<'d, T, G4>>,
        g5: &Option<PinGroup<'d, T, G5>>,
        g6: &Option<PinGroup<'d, T, G6>>,
        #[cfg(any(tsc_v2, tsc_v3))] g7: &Option<PinGroup<'d, T, G7>>,
        #[cfg(tsc_v3)] g8: &Option<PinGroup<'d, T, G8>>,
    ) -> Result<(), GroupError> {
        let mut shield_count = 0;

        if let Some(pin_group) = g1 {
            if pin_group.contains_shield() {
                shield_count += 1;
            }
        };
        if let Some(pin_group) = g2 {
            if pin_group.contains_shield() {
                shield_count += 1;
            }
        };
        if let Some(pin_group) = g3 {
            if pin_group.contains_shield() {
                shield_count += 1;
            }
        };
        if let Some(pin_group) = g4 {
            if pin_group.contains_shield() {
                shield_count += 1;
            }
        };
        if let Some(pin_group) = g5 {
            if pin_group.contains_shield() {
                shield_count += 1;
            }
        };
        if let Some(pin_group) = g6 {
            if pin_group.contains_shield() {
                shield_count += 1;
            }
        };
        #[cfg(any(tsc_v2, tsc_v3))]
        if let Some(pin_group) = g7 {
            if pin_group.contains_shield() {
                shield_count += 1;
            }
        };
        #[cfg(tsc_v3)]
        if let Some(pin_group) = g8 {
            if pin_group.contains_shield() {
                shield_count += 1;
            }
        };

        if shield_count > 1 {
            return Err(GroupError::MultipleShields);
        }

        Ok(())
    }

    fn extract_groups(io_mask: u32) -> u32 {
        let mut groups: u32 = 0;
        for idx in 0..TSC_NUM_GROUPS {
            if io_mask & (0x0F << idx * 4) != 0 {
                groups |= 1 << idx
            }
        }
        groups
    }

    fn new_inner(
        peri: impl Peripheral<P = T> + 'd,
        g1: Option<PinGroup<'d, T, G1>>,
        g2: Option<PinGroup<'d, T, G2>>,
        g3: Option<PinGroup<'d, T, G3>>,
        g4: Option<PinGroup<'d, T, G4>>,
        g5: Option<PinGroup<'d, T, G5>>,
        g6: Option<PinGroup<'d, T, G6>>,
        #[cfg(any(tsc_v2, tsc_v3))] g7: Option<PinGroup<'d, T, G7>>,
        #[cfg(tsc_v3)] g8: Option<PinGroup<'d, T, G8>>,
        config: Config,
    ) -> Self {
        into_ref!(peri);

        rcc::enable_and_reset::<T>();

        T::regs().cr().modify(|w| {
            w.set_tsce(true);
            w.set_ctph(config.ct_pulse_high_length.into());
            w.set_ctpl(config.ct_pulse_low_length.into());
            w.set_sse(config.spread_spectrum);
            // Prevent invalid configuration for pulse generator prescaler
            if config.ct_pulse_low_length == ChargeTransferPulseCycle::_1
                && (config.pulse_generator_prescaler == PGPrescalerDivider::_1
                    || config.pulse_generator_prescaler == PGPrescalerDivider::_2)
            {
                w.set_pgpsc(PGPrescalerDivider::_4.into());
            } else if config.ct_pulse_low_length == ChargeTransferPulseCycle::_2
                && config.pulse_generator_prescaler == PGPrescalerDivider::_1
            {
                w.set_pgpsc(PGPrescalerDivider::_2.into());
            } else {
                w.set_pgpsc(config.pulse_generator_prescaler.into());
            }
            w.set_ssd(config.spread_spectrum_deviation.into());
            w.set_sspsc(config.spread_spectrum_prescaler);

            w.set_mcv(config.max_count_value.into());
            w.set_syncpol(config.synchro_pin_polarity);
            w.set_am(config.acquisition_mode);
        });

        // Set IO configuration
        // Disable Schmitt trigger hysteresis on all used TSC IOs
        T::regs()
            .iohcr()
            .write(|w| w.0 = !(config.channel_ios | config.shield_ios | config.sampling_ios));

        // Set channel and shield IOs
        T::regs()
            .ioccr()
            .write(|w| w.0 = config.channel_ios | config.shield_ios);

        // Set sampling IOs
        T::regs().ioscr().write(|w| w.0 = config.sampling_ios);

        // Set the groups to be acquired
        T::regs()
            .iogcsr()
            .write(|w| w.0 = Self::extract_groups(config.channel_ios));

        // Disable interrupts
        T::regs().ier().modify(|w| {
            w.set_eoaie(false);
            w.set_mceie(false);
        });

        // Clear flags
        T::regs().icr().modify(|w| {
            w.set_eoaic(true);
            w.set_mceic(true);
        });

        unsafe {
            T::Interrupt::enable();
        }

        Self {
            _peri: peri,
            _g1: g1,
            _g2: g2,
            _g3: g3,
            _g4: g4,
            _g5: g5,
            _g6: g6,
            #[cfg(any(tsc_v2, tsc_v3))]
            _g7: g7,
            #[cfg(tsc_v3)]
            _g8: g8,
            state: State::Ready,
            config,
            _kind: PhantomData,
        }
    }

    /// Start charge transfer acquisition
    pub fn start(&mut self) {
        self.state = State::Busy;

        // Disable interrupts
        T::regs().ier().modify(|w| {
            w.set_eoaie(false);
            w.set_mceie(false);
        });

        // Clear flags
        T::regs().icr().modify(|w| {
            w.set_eoaic(true);
            w.set_mceic(true);
        });

        // Set the touch sensing IOs not acquired to the default mode
        T::regs().cr().modify(|w| {
            w.set_iodef(self.config.io_default_mode);
        });

        // Start the acquisition
        T::regs().cr().modify(|w| {
            w.set_start(true);
        });
    }

    /// Stop charge transfer acquisition
    pub fn stop(&mut self) {
        T::regs().cr().modify(|w| {
            w.set_start(false);
        });

        // Set the touch sensing IOs in low power mode
        T::regs().cr().modify(|w| {
            w.set_iodef(false);
        });

        // Clear flags
        T::regs().icr().modify(|w| {
            w.set_eoaic(true);
            w.set_mceic(true);
        });

        self.state = State::Ready;
    }

    /// Get current state of acquisition
    pub fn get_state(&mut self) -> State {
        if self.state == State::Busy {
            if T::regs().isr().read().eoaf() {
                if T::regs().isr().read().mcef() {
                    self.state = State::Error
                } else {
                    self.state = State::Ready
                }
            }
        }
        self.state
    }

    /// Get the individual group status to check acquisition complete
    pub fn group_get_status(&mut self, index: Group) -> GroupStatus {
        // Status bits are set by hardware when the acquisition on the corresponding
        // enabled analog IO group is complete, cleared when new acquisition is started
        let status = match index {
            Group::One => T::regs().iogcsr().read().g1s(),
            Group::Two => T::regs().iogcsr().read().g2s(),
            Group::Three => T::regs().iogcsr().read().g3s(),
            Group::Four => T::regs().iogcsr().read().g4s(),
            Group::Five => T::regs().iogcsr().read().g5s(),
            Group::Six => T::regs().iogcsr().read().g6s(),
            #[cfg(any(tsc_v2, tsc_v3))]
            Group::Seven => T::regs().iogcsr().read().g7s(),
            #[cfg(tsc_v3)]
            Group::Eight => T::regs().iogcsr().read().g8s(),
        };
        match status {
            true => GroupStatus::Complete,
            false => GroupStatus::Ongoing,
        }
    }

    /// Get the count for the acquisiton, valid once group status is set
    pub fn group_get_value(&mut self, index: Group) -> u16 {
        T::regs().iogcr(index.into()).read().cnt()
    }

    /// Discharge the IOs for subsequent acquisition
    pub fn discharge_io(&mut self, status: bool) {
        // Set the touch sensing IOs in low power mode
        T::regs().cr().modify(|w| {
            w.set_iodef(!status);
        });
    }
}

impl<'d, T: Instance, K: PeriMode> Drop for Tsc<'d, T, K> {
    fn drop(&mut self) {
        rcc::disable::<T>();
    }
}

pub(crate) trait SealedInstance {
    fn regs() -> crate::pac::tsc::Tsc;
    fn waker() -> &'static AtomicWaker;
}

/// TSC instance trait
#[allow(private_bounds)]
pub trait Instance: Peripheral<P = Self> + SealedInstance + RccPeripheral {
    /// Interrupt for this TSC instance
    type Interrupt: interrupt::typelevel::Interrupt;
}

foreach_interrupt!(
    ($inst:ident, tsc, TSC, GLOBAL, $irq:ident) => {
        impl Instance for peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }

        impl SealedInstance for peripherals::$inst {
            fn regs() -> crate::pac::tsc::Tsc {
                crate::pac::$inst
            }
            fn waker() -> &'static AtomicWaker {
                static WAKER: AtomicWaker = AtomicWaker::new();
                &WAKER
            }
        }
    };
);

pin_trait!(G1IO1Pin, Instance);
pin_trait!(G1IO2Pin, Instance);
pin_trait!(G1IO3Pin, Instance);
pin_trait!(G1IO4Pin, Instance);
pin_trait!(G2IO1Pin, Instance);
pin_trait!(G2IO2Pin, Instance);
pin_trait!(G2IO3Pin, Instance);
pin_trait!(G2IO4Pin, Instance);
pin_trait!(G3IO1Pin, Instance);
pin_trait!(G3IO2Pin, Instance);
pin_trait!(G3IO3Pin, Instance);
pin_trait!(G3IO4Pin, Instance);
pin_trait!(G4IO1Pin, Instance);
pin_trait!(G4IO2Pin, Instance);
pin_trait!(G4IO3Pin, Instance);
pin_trait!(G4IO4Pin, Instance);
pin_trait!(G5IO1Pin, Instance);
pin_trait!(G5IO2Pin, Instance);
pin_trait!(G5IO3Pin, Instance);
pin_trait!(G5IO4Pin, Instance);
pin_trait!(G6IO1Pin, Instance);
pin_trait!(G6IO2Pin, Instance);
pin_trait!(G6IO3Pin, Instance);
pin_trait!(G6IO4Pin, Instance);
pin_trait!(G7IO1Pin, Instance);
pin_trait!(G7IO2Pin, Instance);
pin_trait!(G7IO3Pin, Instance);
pin_trait!(G7IO4Pin, Instance);
pin_trait!(G8IO1Pin, Instance);
pin_trait!(G8IO2Pin, Instance);
pin_trait!(G8IO3Pin, Instance);
pin_trait!(G8IO4Pin, Instance);
