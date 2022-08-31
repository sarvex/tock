//XXX: See notes at top of `src/testing.rs`

use core::cell::Cell;
use core::fmt::Write;
use core::ptr::NonNull;
use core::{mem, str};

use crate::collections::queue::Queue;
use crate::collections::ring_buffer::RingBuffer;
use crate::errorcode::ErrorCode;
use crate::kernel::Kernel;
use crate::platform::mpu::{self};
use crate::process::{Error, FunctionCall, FunctionCallSource, Process, State, Task};
use crate::process::{ProcessCustomGrantIdentifer, ProcessId};
use crate::process::{ProcessAddresses, ProcessSizes};
use crate::process_utilities::ProcessLoadError;
use crate::processbuffer::{ReadOnlyProcessBuffer, ReadWriteProcessBuffer};
use crate::static_init;
use crate::storage_permissions;
use crate::syscall::{self, Syscall, SyscallReturn};
use crate::upcall::UpcallId;
use crate::utilities::cells::{MapCell};
use tock_tbf::types::CommandPermissions;

/// Entry that is stored in the grant pointer table at the top of process
/// memory.
///
/// One copy of this entry struct is stored per grant region defined in the
/// kernel. This type allows the core kernel to lookup a grant based on the
/// driver_num associated with the grant, and also holds the pointer to the
/// memory allocated for the particular grant.
#[repr(C)]
struct GrantPointerEntry {
    /// The syscall driver number associated with the allocated grant.
    ///
    /// This defaults to 0 if the grant has not been allocated. Note, however,
    /// that 0 is a valid driver_num, and therefore cannot be used to check if a
    /// grant is allocated or not.
    driver_num: usize,

    /// The start of the memory location where the grant has been allocated, or
    /// null if the grant has not been allocated.
    grant_ptr: *mut u8, //XXX: I think this should change to a box??
}

pub(crate) struct ProcessTesting<'a> {
    process_id: Cell<ProcessId>,
    kernel: &'static Kernel,
    grant_pointers: MapCell<&'static mut [GrantPointerEntry]>,
    tasks: MapCell<RingBuffer<'a, Task>>,
}

impl Process for ProcessTesting<'_> {
    fn processid(&self) -> ProcessId {
        // Debugging
        //print!("My process ID is: {} {}\n", self.process_id.get().index, self.process_id.get().identifier);
        self.process_id.get()
    }

    fn enqueue_task(&self, task: Task) -> Result<(), ErrorCode> {
        let ret = self.tasks.map_or(Err(ErrorCode::FAIL), |tasks| {
            match tasks.enqueue(task) {
                true => {
                    // The task has been successfully enqueued.
                    Ok(())
                }
                false => {
                    // The task could not be enqueued as there is
                    // insufficient space in the ring buffer.
                    Err(ErrorCode::NOMEM)
                }
            }
        });

        ret
    }

    fn ready(&self) -> bool {
        false
    }

    fn remove_pending_upcalls(&self, upcall_id: UpcallId) {
        self.tasks.map(|tasks| {
            tasks.retain(|task| match task {
                // Remove only tasks that are function calls with an id equal
                // to `upcall_id`.
                Task::FunctionCall(function_call) => match function_call.source {
                    FunctionCallSource::Kernel => true,
                    FunctionCallSource::Driver(id) => {
                        if id != upcall_id {
                            true
                        } else {
                            false
                        }
                    }
                },
                _ => true,
            });
        });
    }

    fn get_state(&self) -> State {
        State::StoppedYielded
    }

    fn set_yielded_state(&self) {}

    fn stop(&self) {}

    fn resume(&self) {}

    fn set_fault_state(&self) {}

    fn try_restart(&self, _completion_code: Option<u32>) {
        let _res = self.restart();
    }

    fn terminate(&self, _completion_code: Option<u32>) {}

    fn get_restart_count(&self) -> usize {
        0
    }

    fn has_tasks(&self) -> bool {
        self.tasks.map_or(false, |tasks| tasks.has_elements())
    }

    fn dequeue_task(&self) -> Option<Task> {
        self.tasks.map_or(None, |tasks| {
            tasks.dequeue().map(|cb| {
                cb
            })
        })
    }

    fn pending_tasks(&self) -> usize {
        self.tasks.map_or(0, |tasks| tasks.len())
    }

    fn get_command_permissions(&self, _driver_num: usize, _offset: usize) -> CommandPermissions {
        CommandPermissions::NoPermsAtAll
    }

    fn get_storage_permissions(&self) -> Option<storage_permissions::StoragePermissions> {
        None
    }

    fn number_writeable_flash_regions(&self) -> usize {
        0
    }

    fn get_writeable_flash_region(&self, _region_index: usize) -> (u32, u32) {
        (0, 0)
    }

    fn update_stack_start_pointer(&self, _stack_pointer: *const u8) {}

    fn update_heap_start_pointer(&self, _heap_pointer: *const u8) {}

    fn setup_mpu(&self) {}
    
    fn add_mpu_region(
        &self,
        _unallocated_memory_start: *const u8,
        _unallocated_memory_size: usize,
        _min_region_size: usize,
    ) -> Option<mpu::Region> {
        None
    }

    fn remove_mpu_region(&self, _region: mpu::Region) -> Result<(), ErrorCode> {
        Err(ErrorCode::INVAL)
    }
    
    fn sbrk(&self, _increment: isize) -> Result<*const u8, Error> {
        Ok(0 as *const u8)
    }

    fn brk(&self, _new_break: *const u8) -> Result<*const u8, Error> {
        Ok(0 as *const u8)
    }

    #[allow(clippy::not_unsafe_ptr_arg_deref)]
    fn build_readwrite_process_buffer(
        &self,
        buf_start_addr: *mut u8,
        size: usize,
    ) -> Result<ReadWriteProcessBuffer, ErrorCode> {
        Ok(unsafe { ReadWriteProcessBuffer::new(buf_start_addr, size, self.processid()) })
    }

    #[allow(clippy::not_unsafe_ptr_arg_deref)]
    fn build_readonly_process_buffer(
        &self,
        buf_start_addr: *const u8,
        size: usize,
    ) -> Result<ReadOnlyProcessBuffer, ErrorCode> {
        Ok(unsafe { ReadOnlyProcessBuffer::new(buf_start_addr, size, self.processid()) })
    }
    
    unsafe fn set_byte(&self, addr: *mut u8, value: u8) -> bool {
        *addr = value;
        true
    }

    fn grant_is_allocated(&self, grant_num: usize) -> Option<bool> {
        // Update the grant pointer to the address of the new allocation.
        self.grant_pointers.map_or(None, |grant_pointers| {
            // Implement `grant_pointers[grant_num]` without a chance of a
            // panic.
            grant_pointers
                .get(grant_num)
                .map_or(None, |grant_entry| Some(!grant_entry.grant_ptr.is_null()))
        })
    }

    fn allocate_grant(
        &self,
        grant_num: usize,
        driver_num: usize,
        size: usize,
        align: usize,
    ) -> bool {
        // Verify the grant_num is valid.
        if grant_num >= self.kernel.get_grant_count_and_finalize() {
            return false;
        }

        // Verify that the grant is not already allocated. If the pointer is not
        // null then the grant is already allocated.
        if let Some(is_allocated) = self.grant_is_allocated(grant_num) {
            if is_allocated {
                return false;
            }
        }

        // Verify that there is not already a grant allocated with the same
        // driver_num.
        let exists = self.grant_pointers.map_or(false, |grant_pointers| {
            // Check our list of grant pointers if the driver number is used.
            grant_pointers.iter().any(|grant_entry| {
                // Check if the grant is both allocated (its grant pointer is
                // non null) and the driver number matches.
                (!grant_entry.grant_ptr.is_null()) && grant_entry.driver_num == driver_num
            })
        });
        // If we find a match, then the driver_num must already be used and the
        // grant allocation fails.
        if exists {
            return false;
        }

        // Use the shared grant allocator function to actually allocate memory.
        // Returns `None` if the allocation cannot be created.
        if let Some(grant_ptr) = self.allocate_in_grant_region_internal(size, align) {
            // Update the grant pointer to the address of the new allocation.
            self.grant_pointers.map_or(false, |grant_pointers| {
                // Implement `grant_pointers[grant_num] = grant_ptr` without a
                // chance of a panic.
                grant_pointers
                    .get_mut(grant_num)
                    .map_or(false, |grant_entry| {
                        // Actually set the driver num and grant pointer.
                        grant_entry.driver_num = driver_num;
                        grant_entry.grant_ptr = grant_ptr.as_ptr() as *mut u8;

                        // If all of this worked, return true.
                        true
                    })
            })
        } else {
            // Could not allocate the memory for the grant region.
            false
        }
    }
 
    fn allocate_custom_grant(
        &self,
        size: usize,
        align: usize,
    ) -> Option<(ProcessCustomGrantIdentifer, NonNull<u8>)> {
        // Use the shared grant allocator function to actually allocate memory.
        // Returns `None` if the allocation cannot be created.
        if let Some(ptr) = self.allocate_in_grant_region_internal(size, align) {
            // Create the identifier that the caller will use to get access to
            // this custom grant in the future.
            let identifier = self.create_custom_grant_identifier(ptr);

            Some((identifier, ptr))
        } else {
            // Could not allocate memory for the custom grant.
            None
        }
    }

    fn enter_grant(&self, grant_num: usize) -> Result<NonNull<u8>, Error> {
        // Retrieve the grant pointer from the `grant_pointers` slice. We use
        // `[slice].get()` so that if the grant number is invalid this will
        // return `Err` and not panic.
        self.grant_pointers
            .map_or(Err(Error::KernelError), |grant_pointers| {
                // Implement `grant_pointers[grant_num]` without a chance of a
                // panic.
                match grant_pointers.get_mut(grant_num) {
                    Some(grant_entry) => {
                        // Get a copy of the actual grant pointer.
                        let grant_ptr = grant_entry.grant_ptr;

                        // Check if the grant pointer is marked that the grant
                        // has already been entered. If so, return an error.
                        if (grant_ptr as usize) & 0x1 == 0x1 {
                            // Lowest bit is one, meaning this grant has been
                            // entered.
                            Err(Error::AlreadyInUse)
                        } else {
                            // Now, to mark that the grant has been entered, we
                            // set the lowest bit to one and save this as the
                            // grant pointer.
                            grant_entry.grant_ptr = (grant_ptr as usize | 0x1) as *mut u8;

                            // And we return the grant pointer to the entered
                            // grant.
                            Ok(unsafe { NonNull::new_unchecked(grant_ptr) })
                        }
                    }
                    None => Err(Error::AddressOutOfBounds),
                }
            })
    }

    fn enter_custom_grant(
        &self,
        identifier: ProcessCustomGrantIdentifer,
    ) -> Result<*mut u8, Error> {
        // Get the address of the custom grant based on the identifier.
        let custom_grant_address = self.get_custom_grant_address(identifier);

        // We never deallocate custom grants and only we can change the
        // `identifier` so we know this is a valid address for the custom grant.
        Ok(custom_grant_address as *mut u8)
    }

    unsafe fn leave_grant(&self, grant_num: usize) {
        self.grant_pointers.map(|grant_pointers| {
            // Implement `grant_pointers[grant_num]` without a chance of a
            // panic.
            match grant_pointers.get_mut(grant_num) {
                Some(grant_entry) => {
                    // Get a copy of the actual grant pointer.
                    let grant_ptr = grant_entry.grant_ptr;

                    // Now, to mark that the grant has been released, we set the
                    // lowest bit back to zero and save this as the grant
                    // pointer.
                    grant_entry.grant_ptr = (grant_ptr as usize & !0x1) as *mut u8;
                }
                None => {}
            }
        });
    }

    fn grant_allocated_count(&self) -> Option<usize> {
        self.grant_pointers.map(|grant_pointers| {
            // Filter our list of grant pointers into just the non null ones,
            // and count those. A grant is allocated if its grant pointer is non
            // null.
            grant_pointers
                .iter()
                .filter(|grant_entry| !grant_entry.grant_ptr.is_null())
                .count()
        })
    }

    fn lookup_grant_from_driver_num(&self, driver_num: usize) -> Result<usize, Error> {
        self.grant_pointers
            .map_or(Err(Error::KernelError), |grant_pointers| {
                // Filter our list of grant pointers into just the non null
                // ones, and count those. A grant is allocated if its grant
                // pointer is non null.
                match grant_pointers.iter().position(|grant_entry| {
                    // Only consider allocated grants.
                    (!grant_entry.grant_ptr.is_null()) && grant_entry.driver_num == driver_num
                }) {
                    Some(idx) => Ok(idx),
                    None => Err(Error::OutOfMemory),
                }
            })
    }

    fn is_valid_upcall_function_pointer(&self, _upcall_fn: NonNull<()>) -> bool {
        true
    }

    fn get_process_name(&self) -> &'static str {
        "Test Process"
    }

    fn get_completion_code(&self) -> Option<Option<u32>> {
        None
    }

    fn set_syscall_return_value(&self, _return_value: SyscallReturn) {}

    fn set_process_function(&self, _callback: FunctionCall) {}

    fn switch_to(&self) -> Option<syscall::ContextSwitchReason> {
        None
    }
    
    fn debug_syscall_count(&self) -> usize {
        0
    }

    fn debug_dropped_upcall_count(&self) -> usize {
        0
    }

    fn debug_timeslice_expiration_count(&self) -> usize {
        0
    }

    fn debug_timeslice_expired(&self) {}

    fn debug_syscall_called(&self, _last_syscall: Syscall) {}

    fn debug_syscall_last(&self) -> Option<Syscall> {
        None
    }

    fn get_addresses(&self) -> ProcessAddresses {
        ProcessAddresses {
            flash_start: 0,
            flash_non_protected_start: 0,
            flash_end: 0,
            sram_start: 0,
            sram_app_brk: 0,
            sram_grant_start: 0,
            sram_end: 0,
            sram_heap_start: None,
            sram_stack_top: None,
            sram_stack_bottom: None,
        }
    }

    fn get_sizes(&self) -> ProcessSizes {
        ProcessSizes {
            grant_pointers: mem::size_of::<GrantPointerEntry>()
                * self.kernel.get_grant_count_and_finalize(),
            upcall_list: mem::size_of::<Task>() * 10,
            process_control_block: mem::size_of::<ProcessTesting>(),
        }
    }
    
    fn print_full_process(&self, _writer: &mut dyn Write) {}

    fn get_stored_state(&self, _out: &mut [u8]) -> Result<usize, ErrorCode> {
        Err(ErrorCode::FAIL)
    }
}

impl ProcessTesting<'_> {

    pub(crate) fn new<'a>(kernel: &'static Kernel, index: usize) -> Self {
        let identifier = kernel.create_process_identifier();

        //XXX: fill in grant_pointers
        //  Create grant pointers, which needs to be a static thing
        //  Could, as a hack, just create a global region that's way too big and use that. Like `PROCESSES`.
        //  Maybe could `static_init!()` it instead?
        //  This is related to eventually having memory space for the grant _contents_ as well!
        //XXX: fill in tasks

        let grant_ptrs_num = kernel.get_grant_count_and_finalize();
        //let grant_pointers: &mut [GrantPointerEntry]

        Self {
            process_id: Cell::new(ProcessId::new(kernel, identifier, index)),
            kernel: kernel,
            grant_pointers: MapCell::empty(),
            tasks: MapCell::empty(),
        }
    }

    fn restart(&self) -> Result<(), ErrorCode> {
        Err(ErrorCode::FAIL)
    }

    fn allocate_in_grant_region_internal(&self, size: usize, align: usize) -> Option<NonNull<u8>> {
        None
    }

    fn create_custom_grant_identifier(&self, ptr: NonNull<u8>) -> ProcessCustomGrantIdentifer {
        ProcessCustomGrantIdentifer {
            offset: 0,
        }
    }

    fn get_custom_grant_address(&self, identifier: ProcessCustomGrantIdentifer) -> usize {
        0
    }
}

