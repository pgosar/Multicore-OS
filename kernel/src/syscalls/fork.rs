use core::sync::atomic::Ordering;

use alloc::sync::Arc;
use x86_64::{
    structures::paging::{OffsetPageTable, Page, PageTable, PageTableFlags, PhysFrame},
    VirtAddr,
};

use crate::{
    events::current_running_event_info,
    interrupts::x2apic,
    memory::{paging::get_page_flags, HHDM_OFFSET},
    processes::process::{print_process_table, UnsafePCB, NEXT_PID, PCB, PROCESS_TABLE},
    serial_println,
};

pub fn sys_fork() -> u64 {
    panic!();
    let child_pid = NEXT_PID.fetch_add(1, Ordering::SeqCst);
    serial_println!("CHILD PID {}", child_pid);
    let cpuid: u32 = x2apic::current_core_id() as u32;
    let parent_pid = current_running_event_info(cpuid).pid;
    let process_table = PROCESS_TABLE.read();
    let process = process_table
        .get(&parent_pid)
        .expect("can't find pcb in process table");
    let parent_pcb = process.pcb.get();

    let child_pcb = Arc::new(UnsafePCB::new(unsafe { (*parent_pcb).clone() }));
    serial_println!("CHILD PCB {:?}", child_pcb.pcb.get());
    (unsafe { (*child_pcb.pcb.get()).registers.rax = 0 });

    let parent_pml4_frame = unsafe { (*parent_pcb).pml4_frame };

    unsafe { (*child_pcb.pcb.get()).pml4_frame = parent_pml4_frame };
    set_page_table_cow(unsafe { &mut *child_pcb.pcb.get() });

    PROCESS_TABLE.write().insert(child_pid, child_pcb);
    unsafe {print_process_table(&PROCESS_TABLE) };
    return child_pid as u64;
}

fn set_page_table_cow(pcb: &mut PCB) {
    let mut mapper = unsafe { pcb.create_mapper() };

    for i in 0..512 {
        let entry = &mapper.level_4_table()[i];
        if entry.is_unused() {
            continue;
        }
        let pdpt_frame = PhysFrame::containing_address(entry.addr());
        // set as copy on write
        unsafe { set_page_table_cow_helper(pdpt_frame, 3, &mut mapper) };
    }
}

unsafe fn set_page_table_cow_helper(frame: PhysFrame, level: u8, mapper: &mut OffsetPageTable) {
    let virt = HHDM_OFFSET.as_u64() + frame.start_address().as_u64();
    let table = unsafe { &mut *(virt as *mut PageTable) };

    for entry in table.iter_mut() {
        if entry.is_unused() {
            continue;
        }

        if level > 1 {
            let child_frame = PhysFrame::containing_address(entry.addr());
            set_page_table_cow_helper(child_frame, level - 1, mapper);
        }
        let mut flags: PageTableFlags =
            get_page_flags(Page::containing_address(VirtAddr::new(virt)), mapper)
                .expect("Could not get page flags");
        // ONLY modify originally writable pages to COW
        let writable = flags.contains(PageTableFlags::WRITABLE);
        if writable {
            flags.set(PageTableFlags::BIT_9, true);
            flags.set(PageTableFlags::WRITABLE, false);
        }
    }
}

#[cfg(test)]
    #[test_case]
    fn test_simple_fork() {
        use crate::{constants::processes::{FORK_SIMPLE, SYSCALL_MMAP_MEMORY}, events::schedule_process, processes::process::{create_process, print_process_table, run_process_ring3}};

        let parent_pid = create_process(FORK_SIMPLE);
        let cpuid: u32 = x2apic::current_core_id() as u32;
        schedule_process(cpuid, unsafe { run_process_ring3(parent_pid) }, parent_pid);
        let child_pid = parent_pid + 1;

        serial_println!("PARENT PID {}", parent_pid);

        // since no other processes are running or being created we assume that
        // the child pid is one more than the child pid
        let process_table = PROCESS_TABLE.read();
        unsafe {
            print_process_table(&PROCESS_TABLE);
        }
        assert!(process_table.contains_key(&child_pid), "Child process not found in table");

        let parent_pcb = process_table.get(&parent_pid).expect("Could not get parent pcb from process table").pcb.get();
        let child_pcb = process_table.get(&child_pid).expect("Could not get child pcb from process table").pcb.get();

        // check that some of the fields are equivalent
        unsafe {
        assert_eq!((*parent_pcb).fd_table, (*child_pcb).fd_table);
        assert_eq!((*parent_pcb).kernel_rip, (*child_pcb).kernel_rip);
        assert_eq!((*parent_pcb).kernel_rsp, (*child_pcb).kernel_rsp);
        assert_eq!((*parent_pcb).registers, (*child_pcb).registers);
        }

        // check that the pml4 frame is set correctly
        unsafe {
        verify_page_table_walk(&mut *parent_pcb, &mut *child_pcb);
        }
    }

fn verify_page_table_walk(parent_pcb: &mut PCB, child_pcb: &mut PCB) {
    assert_eq!((*parent_pcb).pml4_frame.start_address(), (*child_pcb).pml4_frame.start_address());
    let mut parent_mapper = unsafe { parent_pcb.create_mapper() };
    let mut child_mapper = unsafe { child_pcb.create_mapper()};

    for i in 0..512 {
        let parent_entry = &parent_mapper.level_4_table()[i];
        let child_entry = &child_mapper.level_4_table()[i];
        if parent_entry.is_unused() {
            assert!(child_entry.is_unused());
        }
        else {
            assert_eq!(parent_entry.flags(), child_entry.flags());
            assert_eq!(parent_entry.addr(), child_entry.addr());
            assert_eq!(parent_entry.frame().expect("Could not retrieve parent frame"), child_entry.frame().expect("Could not retrieve child frame"));
            let parent_pdpt_frame = PhysFrame::containing_address(parent_entry.addr());
            let child_pdpt_frame = PhysFrame::containing_address(child_entry.addr());
            recursive_walk(parent_pdpt_frame, child_pdpt_frame, 3, &mut parent_mapper, &mut child_mapper);
        }
    }
}

fn recursive_walk(parent_frame: PhysFrame, child_frame: PhysFrame, level: u8, parent_mapper: &mut OffsetPageTable, child_mapper: &mut OffsetPageTable) {
    let parent_virt = HHDM_OFFSET.as_u64() + parent_frame.start_address().as_u64();
    let child_virt = HHDM_OFFSET.as_u64() + child_frame.start_address().as_u64();
    // make sure the child and parent frame virtual addresses are the same
    assert_eq!(parent_virt, child_virt);

    let parent_table = unsafe { &mut *(parent_virt as *mut PageTable) };
    let child_table = unsafe { &mut *(parent_virt as *mut PageTable) };

    for i in 0..512 {
            let parent_entry = &parent_table[i];
            let child_entry =  &child_table[i];

            // from the parent and child tables, ensure each entry is the same
            assert_eq!(parent_entry.flags(), child_entry.flags());
            assert_eq!(parent_entry.addr(), child_entry.addr());
            assert_eq!(parent_entry.frame().expect("Could not retrieve parent frame"), child_entry.frame().expect("Could not retrieve child frame"));

            if level > 1 {
                let parent_frame: PhysFrame = PhysFrame::containing_address(parent_entry.addr());
                let child_frame: PhysFrame = PhysFrame::containing_address(child_entry.addr());
                recursive_walk(parent_frame, child_frame, level - 1, parent_mapper, child_mapper);
            }


    }
}