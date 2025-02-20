pub mod loader;
pub mod process;
pub mod registers;

// Test basic remove, as removing and then translating should fail

#[cfg(test)]
mod tests {
  use crate::interrupts::x2apic;
  use crate::processes::process::{create_process, run_process_ring3};
  use crate::constants::processes::INFINITE_LOOP;
  use crate::events::schedule_process;
use crate::processes::registers::Registers;

  #[test_case]
  fn test_simple_process() {
    let cpuid = x2apic::current_core_id() as u32;
  
    let pid = create_process(INFINITE_LOOP);
    unsafe { schedule_process(cpuid, run_process_ring3(pid), pid); }
  
    assert!(matches!(
        cpuid,
        0
    ));
  }
}