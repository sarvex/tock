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


//XXX: Does `#[cfg(test)]` do something special that `cfg testonly` does not? Also, what is the purpose of marking a module as `#[cfg(test)]`?
//  `cfg(test)` is partially convention. It also stops the entire module from being compiled
//  `testonly` is a tock-specific feature enabled on the kernel while testing (see capsules/Cargo.toml)


//XXX: Should tests be placed in the relevant files or centralized? Maybe a separate directory for tests?
//  * Separate directory cleans things up and keeps files from getting huge and bloated
//  * Separate directory makes it easier to forget about updating the capsule tests for new functionality
//  * Both are plausible in rust, although same file is canonical per the docs
//      https://doc.rust-lang.org/book/ch11-03-test-organization.html
//      http://xion.io/post/code/rust-unit-test-placement.html


//XXX: Should we consider using external crates to simplify testing? I'm not familiar enough with any to suggest.


//TODO:
//  * make a test for a capsule with other syscalls
//  * make a test for a capsule with callbacks
//  * make a test for a virtualized capsule. Test could be just for the virtualizer, or for a stackup of capsules?


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
        // LED states to compare against
        const OFF_LED: FakeLed = FakeLed {
            state: Cell::new(FakeLedState::Off),
        };
        const ON_LED: FakeLed = FakeLed {
            state: Cell::new(FakeLedState::On),
        };

        // Create the LED driver
        let fake_leds = [FakeLed::new(), FakeLed::new(), FakeLed::new()];
        let mut driver_input = [&fake_leds[0], &fake_leds[1], &fake_leds[2]];
        let driver = LedDriver::new(&mut driver_input);

        // Test initial state of driver
        assert_eq!(fake_leds, [OFF_LED, OFF_LED, OFF_LED]);

        let board_kernel = kernel::testing::create_kernel();
        let id = kernel::process::ProcessId::new_external(
            board_kernel,
            usize::MAX,
            usize::MAX,
            &kernel::testing::TestingCap,
        );

        let result = driver.command(0, 0, 0, id);
        assert_eq!(result, CommandReturn::success_u32(3));
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
