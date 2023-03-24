//! Components for Console and ConsoleOrdered. These are two alternative implementations of
//! the serial console system call interface. Console allows prints of arbitrary length but does
//! not have ordering or atomicity guarantees. ConsoleOrdered, in contrast, has limits on the
//! maximum lengths of prints but provides a temporal ordering and ensures a print is atomic at
//! least up to particular length (typically 200 bytes). Console is useful when userspace is
//! printing large messages. ConsoleOrdered is useful when you are debugging and there are
//! inter-related messages from the kernel and userspace, whose ordering is important to
//! maintain.
//!
//!
//! This provides three Components, `ConsoleComponent` and
//! `ConsoleOrderedComponent`, which implement a buffered read/write
//! console over a serial port, and `UartMuxComponent`, which provides
//! multiplexed access to hardware UART. As an example, the serial
//! port used for console on Imix is typically USART3 (the DEBUG USB
//! connector).
//!
//! Usage
//! -----
//! ```rust
//! let uart_mux = UartMuxComponent::new(&sam4l::usart::USART3,
//!                                      115200,
//!                                      deferred_caller).finalize(components::uart_mux_component_static!());
//! let console = ConsoleComponent::new(board_kernel, uart_mux)
//!    .finalize(console_component_static!());
//! ```
// Author: Philip Levis <pal@cs.stanford.edu>
// Last modified: 1/08/2020

use capsules_core::console;
use capsules_core::console_ordered::ConsoleOrdered;

use capsules_core::virtualizers::virtual_alarm::{MuxAlarm, VirtualMuxAlarm};
use capsules_core::virtualizers::virtual_uart::{MuxUart, UartDevice};
use core::mem::MaybeUninit;
use kernel::capabilities;
use kernel::component::Component;
use kernel::create_capability;
use kernel::hil;
use kernel::hil::time::{self, Alarm};
use kernel::hil::uart;

use capsules_core::console::DEFAULT_BUF_SIZE;

#[macro_export]
macro_rules! uart_mux_component_static {
    () => {{
        use capsules_core::virtualizers::virtual_uart::MuxUart;
        use kernel::static_buf;
        let UART_MUX = static_buf!(MuxUart<'static>);
        let RX_BUF = static_buf!([u8; capsules_core::virtualizers::virtual_uart::RX_BUF_LEN]);
        (UART_MUX, RX_BUF)
    }};
    ($rx_buffer_len: literal) => {{
        use capsules_core::virtualizers::virtual_uart::MuxUart;
        use kernel::static_buf;
        let UART_MUX = static_buf!(MuxUart<'static>);
        let RX_BUF = static_buf!([u8; $rx_buffer_len]);
        (UART_MUX, RX_BUF)
    }};
}

pub struct UartMuxComponent<const RX_BUF_LEN: usize> {
    uart: &'static dyn uart::Uart<'static>,
    baud_rate: u32,
}

impl<const RX_BUF_LEN: usize> UartMuxComponent<RX_BUF_LEN> {
    pub fn new(
        uart: &'static dyn uart::Uart<'static>,
        baud_rate: u32,
    ) -> UartMuxComponent<RX_BUF_LEN> {
        UartMuxComponent { uart, baud_rate }
    }
}

impl<const RX_BUF_LEN: usize> Component for UartMuxComponent<RX_BUF_LEN> {
    type StaticInput = (
        &'static mut MaybeUninit<MuxUart<'static>>,
        &'static mut MaybeUninit<[u8; RX_BUF_LEN]>,
    );
    type Output = &'static MuxUart<'static>;

    fn finalize(self, s: Self::StaticInput) -> Self::Output {
        let rx_buf = s.1.write([0; RX_BUF_LEN]);
        let uart_mux = s.0.write(MuxUart::new(self.uart, rx_buf, self.baud_rate));
        kernel::deferred_call::DeferredCallClient::register(uart_mux);

        uart_mux.initialize();
        hil::uart::Transmit::set_transmit_client(self.uart, uart_mux);
        hil::uart::Receive::set_receive_client(self.uart, uart_mux);

        uart_mux
    }
}

#[macro_export]
macro_rules! console_component_static {
    () => {{
        use capsules_core::console::{Console, DEFAULT_BUF_SIZE};
        use capsules_core::virtualizers::virtual_uart::UartDevice;
        use kernel::static_buf;
        let read_buf = static_buf!([u8; DEFAULT_BUF_SIZE]);
        let write_buf = static_buf!([u8; DEFAULT_BUF_SIZE]);
        // Create virtual device for console.
        let console_uart = static_buf!(UartDevice);
        let console = static_buf!(Console<'static>);
        (write_buf, read_buf, console_uart, console)
    }};
}

pub struct ConsoleComponent {
    board_kernel: &'static kernel::Kernel,
    driver_num: usize,
    uart_mux: &'static MuxUart<'static>,
}

impl ConsoleComponent {
    pub fn new(
        board_kernel: &'static kernel::Kernel,
        driver_num: usize,
        uart_mux: &'static MuxUart,
    ) -> ConsoleComponent {
        ConsoleComponent {
            board_kernel: board_kernel,
            driver_num: driver_num,
            uart_mux: uart_mux,
        }
    }
}

impl Component for ConsoleComponent {
    type StaticInput = (
        &'static mut MaybeUninit<[u8; DEFAULT_BUF_SIZE]>,
        &'static mut MaybeUninit<[u8; DEFAULT_BUF_SIZE]>,
        &'static mut MaybeUninit<UartDevice<'static>>,
        &'static mut MaybeUninit<console::Console<'static>>,
    );
    type Output = &'static console::Console<'static>;

    fn finalize(self, s: Self::StaticInput) -> Self::Output {
        let grant_cap = create_capability!(capabilities::MemoryAllocationCapability);

        let write_buffer = s.0.write([0; DEFAULT_BUF_SIZE]);

        let read_buffer = s.1.write([0; DEFAULT_BUF_SIZE]);

        let console_uart = s.2.write(UartDevice::new(self.uart_mux, true));
        console_uart.setup();

        let console = s.3.write(console::Console::new(
            console_uart,
            write_buffer,
            read_buffer,
            self.board_kernel.create_grant(self.driver_num, &grant_cap),
        ));
        hil::uart::Transmit::set_transmit_client(console_uart, console);
        hil::uart::Receive::set_receive_client(console_uart, console);

        console
    }
}
#[macro_export]
macro_rules! console_ordered_component_static {
    ($A:ty $(,)?) => {{
        let mux_alarm = kernel::static_buf!(VirtualMuxAlarm<'static, $A>);
        let console = kernel::static_buf!(ConsoleOrdered<'static, VirtualMuxAlarm<'static, $A>>);
        (mux_alarm, console)
    };};
}

pub struct ConsoleOrderedComponent<A: 'static + time::Alarm<'static>> {
    board_kernel: &'static kernel::Kernel,
    driver_num: usize,
    alarm_mux: &'static MuxAlarm<'static, A>,
    atomic_size: usize,
    retry_timer: u32,
    write_timer: u32,
}

impl<A: 'static + time::Alarm<'static>> ConsoleOrderedComponent<A> {
    pub fn new(
        board_kernel: &'static kernel::Kernel,
        driver_num: usize,
        alarm_mux: &'static MuxAlarm<'static, A>,
        atomic_size: usize,
        retry_timer: u32,
        write_timer: u32,
    ) -> ConsoleOrderedComponent<A> {
        ConsoleOrderedComponent {
            board_kernel: board_kernel,
            driver_num: driver_num,
            alarm_mux: alarm_mux,
            atomic_size: atomic_size,
            retry_timer: retry_timer,
            write_timer: write_timer,
        }
    }
}

impl<A: 'static + time::Alarm<'static>> Component for ConsoleOrderedComponent<A> {
    type StaticInput = (
        &'static mut MaybeUninit<VirtualMuxAlarm<'static, A>>,
        &'static mut MaybeUninit<ConsoleOrdered<'static, VirtualMuxAlarm<'static, A>>>,
    );
    type Output = &'static ConsoleOrdered<'static, VirtualMuxAlarm<'static, A>>;

    fn finalize(self, static_buffer: Self::StaticInput) -> Self::Output {
        let grant_cap = create_capability!(capabilities::MemoryAllocationCapability);

        let virtual_alarm1 = static_buffer.0.write(VirtualMuxAlarm::new(self.alarm_mux));
        virtual_alarm1.setup();

        let console = static_buffer.1.write(ConsoleOrdered::new(
            virtual_alarm1,
            self.board_kernel.create_grant(self.driver_num, &grant_cap),
            self.atomic_size,
            self.retry_timer,
            self.write_timer,
        ));

        virtual_alarm1.set_alarm_client(console);
        console
    }
}
