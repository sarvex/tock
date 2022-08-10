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
        static_init!(kernel::Kernel, kernel::Kernel::new(&PROCESSES))
    }
}
