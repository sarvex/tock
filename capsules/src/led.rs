//! Provides userspace access to LEDs on a board.
//!
//! This allows for much more cross platform controlling of LEDs without having
//! to know which of the GPIO pins exposed across the syscall interface are
//! LEDs.
//!
//! This capsule takes an array of pins and the polarity of the LED (active high
//! or active low). This allows the board to configure how the underlying GPIO
//! must be controlled to turn on and off LEDs, such that the syscall driver
//! interface can be agnostic to the LED polarity.
//!
//! Usage
//! -----
//!
//! ```rust
//! # use kernel::static_init;
//!
//! let led_pins = static_init!(
//!     [(&'static sam4l::gpio::GPIOPin, kernel::hil::gpio::ActivationMode); 3],
//!     [(&sam4l::gpio::PA[13], kernel::hil::gpio::ActivationMode::ActiveLow),   // Red
//!      (&sam4l::gpio::PA[15], kernel::hil::gpio::ActivationMode::ActiveLow),   // Green
//!      (&sam4l::gpio::PA[14], kernel::hil::gpio::ActivationMode::ActiveLow)]); // Blue
//! let led = static_init!(
//!     capsules::led::LED<'static, sam4l::gpio::GPIOPin>,
//!     capsules::led::LED::new(led_pins));
//! ```
//!
//! Syscall Interface
//! -----------------
//!
//! - Stability: 2 - Stable
//!
//! ### Command
//!
//! All LED operations are synchronous, so this capsule only uses the `command`
//! syscall.
//!
//! #### `command_num`
//!
//! - `0`: Return the number of LEDs on this platform.
//!   - `data`: Unused.
//!   - Return: Number of LEDs.
//! - `1`: Turn the LED on.
//!   - `data`: The index of the LED. Starts at 0.
//!   - Return: `Ok(())` if the LED index was valid, `INVAL` otherwise.
//! - `2`: Turn the LED off.
//!   - `data`: The index of the LED. Starts at 0.
//!   - Return: `Ok(())` if the LED index was valid, `INVAL` otherwise.
//! - `3`: Toggle the on/off state of the LED.
//!   - `data`: The index of the LED. Starts at 0.
//!   - Return: `Ok(())` if the LED index was valid, `INVAL` otherwise.

use kernel::hil::led;
use kernel::syscall::{CommandReturn, SyscallDriver};
use kernel::{ErrorCode, ProcessId};

/// Syscall driver number.
use crate::driver;
pub const DRIVER_NUM: usize = driver::NUM::Led as usize;

/// Holds the array of LEDs and implements a `Driver` interface to
/// control them.
pub struct LedDriver<'a, L: led::Led, const NUM_LEDS: usize> {
    leds: &'a mut [&'a L; NUM_LEDS],
}

impl<'a, L: led::Led, const NUM_LEDS: usize> LedDriver<'a, L, NUM_LEDS> {
    pub fn new(leds: &'a mut [&'a L; NUM_LEDS]) -> Self {
        // Initialize all LEDs and turn them off
        for led in leds.iter_mut() {
            led.init();
            led.off();
        }

        Self { leds: leds }
    }
}

impl<L: led::Led, const NUM_LEDS: usize> SyscallDriver for LedDriver<'_, L, NUM_LEDS> {
    /// Control the LEDs.
    ///
    /// ### `command_num`
    ///
    /// - `0`: Returns the number of LEDs on the board. This will always be 0 or
    ///        greater, and therefore also allows for checking for this driver.
    /// - `1`: Turn the LED at index specified by `data` on. Returns `INVAL` if
    ///        the LED index is not valid.
    /// - `2`: Turn the LED at index specified by `data` off. Returns `INVAL`
    ///        if the LED index is not valid.
    /// - `3`: Toggle the LED at index specified by `data` on or off. Returns
    ///        `INVAL` if the LED index is not valid.
    fn command(&self, command_num: usize, data: usize, _: usize, _: ProcessId) -> CommandReturn {
        match command_num {
            // get number of LEDs
            0 => CommandReturn::success_u32(NUM_LEDS as u32),

            // on
            1 => {
                if data >= NUM_LEDS {
                    CommandReturn::failure(ErrorCode::INVAL) /* led out of range */
                } else {
                    self.leds[data].on();
                    CommandReturn::success()
                }
            }

            // off
            2 => {
                if data >= NUM_LEDS {
                    CommandReturn::failure(ErrorCode::INVAL) /* led out of range */
                } else {
                    self.leds[data].off();
                    CommandReturn::success()
                }
            }

            // toggle
            3 => {
                if data >= NUM_LEDS {
                    CommandReturn::failure(ErrorCode::INVAL) /* led out of range */
                } else {
                    self.leds[data].toggle();
                    CommandReturn::success()
                }
            }

            // default
            _ => CommandReturn::failure(ErrorCode::NOSUPPORT),
        }
    }

    fn allocate_grant(&self, _processid: ProcessId) -> Result<(), kernel::process::Error> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use core::cell::Cell;

    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    enum FakeLedState {
        NotConfigured,
        Unknown,
        On,
        Off,
    }

    #[derive(Clone, Debug, PartialEq, Eq)]
    struct FakeLed {
        state: Cell<FakeLedState>,
    }

    impl FakeLed {
        const fn new() -> Self {
            Self {
                state: Cell::new(FakeLedState::NotConfigured),
            }
        }
    }

    impl kernel::hil::led::Led for FakeLed {
        fn init(&self) {
            self.state.set(FakeLedState::Unknown);
        }

        fn on(&self) {
            assert_ne!(self.state.get(), FakeLedState::NotConfigured);
            self.state.set(FakeLedState::On);
        }

        fn off(&self) {
            assert_ne!(self.state.get(), FakeLedState::NotConfigured);
            self.state.set(FakeLedState::Off);
        }

        fn toggle(&self) {
            if self.read() {
                self.off();
            } else {
                self.on();
            }
        }

        fn read(&self) -> bool {
            match self.state.get() {
                FakeLedState::NotConfigured | FakeLedState::Unknown => panic!("LED state unknown"),
                FakeLedState::On => true,
                FakeLedState::Off => false,
            }
        }
    }

    #[test]
    fn basic_test() {
        // Outside of the kernel crate, there's no upstreamed way to:
        // - Compare a CommandReturn with an expected value
        // - Create a ProcessId to pass to SyscallDriver::command
        // - Create a kernel to create a ProcessId

        let fake_leds = [FakeLed::new(), FakeLed::new(), FakeLed::new()];
        let mut driver_input = [&fake_leds[0], &fake_leds[1], &fake_leds[2]];

        let driver = LedDriver::new(&mut driver_input);
        const OFF_LED: FakeLed = FakeLed {
            state: Cell::new(FakeLedState::Off),
        };
        const ON_LED: FakeLed = FakeLed {
            state: Cell::new(FakeLedState::On),
        };

        assert_eq!(fake_leds, [OFF_LED, OFF_LED, OFF_LED]);

        // No safe way to create a &'static without...leaking a Box?
        // TODO: replace with another safe static construction mechanism like OnceMut
        extern crate alloc;
        use alloc::boxed::Box;
        // Should there instead be a fake Kernel and/or ProcessId exported from the kernel crate?
        let id = kernel::process::ProcessId::new_external(
            Box::leak(Box::new(kernel::Kernel::new(&[]))),
            usize::MAX,
            usize::MAX,
            &kernel::capabilities::TestingCap,
        );

        assert_eq!(
            driver.command(0, 0, 0, id),
            CommandReturn::success_u32(3u32)
        );
        assert_eq!(fake_leds, [OFF_LED, OFF_LED, OFF_LED]);

        assert_eq!(driver.command(1, 0, 0, id), CommandReturn::success());
        assert_eq!(fake_leds, [ON_LED, OFF_LED, OFF_LED]);

        assert_eq!(driver.command(1, 0, 0, id), CommandReturn::success());
        assert_eq!(driver.command(1, 1, 0, id), CommandReturn::success());
        assert_eq!(
            driver.command(1, 3, 0, id),
            CommandReturn::failure(ErrorCode::INVAL)
        );
        assert_eq!(fake_leds, [ON_LED, ON_LED, OFF_LED]);

        assert_eq!(driver.command(2, 0, 0, id), CommandReturn::success());
        assert_eq!(
            driver.command(2, 3, 0, id),
            CommandReturn::failure(ErrorCode::INVAL)
        );
        assert_eq!(fake_leds, [OFF_LED, ON_LED, OFF_LED]);
    }

    #[test]
    fn out_of_range() {}
}
