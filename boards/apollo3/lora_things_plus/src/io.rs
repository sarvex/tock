use core::fmt::Write;
use core::panic::PanicInfo;

use crate::CHIP;
use crate::PROCESSES;
use crate::PROCESS_PRINTER;
use apollo3;
use kernel::debug;
use kernel::debug::IoWrite;
use kernel::hil::led;

/// Writer is used by kernel::debug to panic message to the serial port.
pub struct Writer {}

/// Global static for debug writer
pub static mut WRITER: Writer = Writer {};

impl Write for Writer {
    fn write_str(&mut self, s: &str) -> ::core::fmt::Result {
        self.write(s.as_bytes());
        Ok(())
    }
}

impl IoWrite for Writer {
    fn write(&mut self, buf: &[u8]) -> usize {
        let uart = apollo3::uart::Uart::new_uart_0(); // Aliases memory for uart0. Okay bc we are panicking.
        uart.transmit_sync(buf);
        buf.len()
    }
}

/// Panic handler.
#[no_mangle]
#[panic_handler]
pub unsafe extern "C" fn panic_fmt(info: &PanicInfo) -> ! {
    // just create a new pin reference here instead of using global
    let led_pin = &mut apollo3::gpio::GpioPin::new(
        kernel::utilities::StaticRef::new(
            apollo3::gpio::GPIO_BASE_RAW as *const apollo3::gpio::GpioRegisters,
        ),
        apollo3::gpio::Pin::Pin26,
    );
    let led = &mut led::LedLow::new(led_pin);
    let writer = &mut WRITER;

    debug::panic(
        &mut [led],
        writer,
        info,
        &cortexm4::support::nop,
        &PROCESSES,
        &CHIP,
        &PROCESS_PRINTER,
    )
}
