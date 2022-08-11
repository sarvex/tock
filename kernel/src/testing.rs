//XXX: Is it okay to not have `#cfg` in this file, since it's only included with `#cfg`?
//  * I tried to put this code in a module and put a `cfg(test)` around it, but couldn't get it working
//  * Alyssa's method is pretty reasonable here. It's mentioned elsewhere too:
//      https://stackoverflow.com/questions/41700543/can-we-share-test-utilites-between-crates
//  * Oops. Open issue in Rust:
//      https://github.com/rust-lang/cargo/issues/8379

//XXX: Would it be better to have testing infrastructure in relevant files or centralized?
//  * There are very few references online about testing helpers/infrastructure

use crate::*;

pub struct TestingCap;
unsafe impl capabilities::ProcessManagementCapability for TestingCap {}
unsafe impl capabilities::MainLoopCapability for TestingCap {}
unsafe impl capabilities::MemoryAllocationCapability for TestingCap {}
unsafe impl capabilities::ExternalProcessCapability for TestingCap {}
unsafe impl capabilities::UdpDriverCapability for TestingCap {}
unsafe impl capabilities::CreatePortTableCapability for TestingCap {}
unsafe impl capabilities::NetworkCapabilityCreationCapability for TestingCap {}

const NUM_PROCS: usize = 2;
static mut PROCESSES: [Option<&'static dyn process::Process>; NUM_PROCS] = [None; NUM_PROCS];

pub fn create_kernel() -> &'static kernel::Kernel {
    unsafe {
        //XXX: I'm not sure why the compiler warns about an unnecessary `mut` here
        static_init!(kernel::Kernel, kernel::Kernel::new(&PROCESSES))
    }
}
