use std::os::raw::{c_int, c_long};

#[repr(C)]
struct Timespec { seconds: c_long, nanoseconds: c_long }
unsafe extern "C" {
    fn clock_gettime(clock: c_int, time: *mut Timespec) -> c_int;
}

// Linux process CPU clock. This single-threaded audit driver excludes time
// descheduled by concurrent work; frequency/cache contention can still vary.
pub fn now_ns() -> u64 {
    let mut time = Timespec { seconds: 0, nanoseconds: 0 };
    // SAFETY: clock 2 is CLOCK_PROCESS_CPUTIME_ID on the audited Linux host;
    // the writable repr(C) timespec uses the platform C long fields.
    assert_eq!(unsafe { clock_gettime(2, &mut time) }, 0);
    time.seconds as u64 * 1_000_000_000 + time.nanoseconds as u64
}
