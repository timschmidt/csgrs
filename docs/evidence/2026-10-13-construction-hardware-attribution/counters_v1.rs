//! Per-thread Linux hardware counters for a standalone diagnostic driver.

use std::io;
use std::marker::PhantomData;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::rc::Rc;
use std::time::Instant;

#[derive(Clone, Copy, Default, Debug)]
#[repr(C)]
pub struct Snapshot {
    pub enabled_ns: u64,
    pub running_ns: u64,
    pub cycles: u64,
    pub instructions: u64,
}

unsafe extern "C" {
    fn counter_open(fds: *mut i32, ids: *mut u64) -> i32;
    fn counter_reset(fd: i32) -> i32;
    fn counter_enable(fd: i32) -> i32;
    fn counter_disable(fd: i32) -> i32;
    fn counter_read(fd: i32, ids: *const u64, out: *mut Snapshot) -> i32;
    fn counter_thread_cpu_ns(out: *mut u64) -> i32;
    fn counter_snapshot_size() -> u64;
}

fn checked(code: i32) -> io::Result<()> {
    if code == 0 {
        Ok(())
    } else {
        Err(io::Error::last_os_error())
    }
}

fn thread_cpu_ns() -> io::Result<u64> {
    let mut value = 0;
    // SAFETY: the helper writes one u64 only on success.
    checked(unsafe { counter_thread_cpu_ns(&mut value) })?;
    Ok(value)
}

pub struct Group {
    leader: OwnedFd,
    _member: OwnedFd,
    ids: [u64; 2],
    // The descriptors count only the thread that opened them.
    _current_thread: PhantomData<Rc<()>>,
}

struct Running<'a> {
    group: &'a Group,
    active: bool,
}

impl Running<'_> {
    fn stop(&mut self) -> io::Result<()> {
        // SAFETY: the borrowed descriptor remains open throughout this scope.
        checked(unsafe { counter_disable(self.group.leader.as_raw_fd()) })?;
        self.active = false;
        Ok(())
    }
}

impl Drop for Running<'_> {
    fn drop(&mut self) {
        if self.active {
            // Also disable on panic. A failed normal stop remains an error in
            // the observation; this second attempt is cleanup only.
            let _ = self.stop();
        }
    }
}

#[derive(Debug)]
pub struct Observation {
    pub wall_ns: u64,
    pub thread_cpu_ns: Option<u64>,
    pub before: Option<Snapshot>,
    pub after: Option<Snapshot>,
    pub errors: Vec<String>,
}

impl Observation {
    pub fn valid(&self) -> bool {
        if !self.errors.is_empty() || self.thread_cpu_ns.is_none() {
            return false;
        }
        let (Some(before), Some(after)) = (self.before, self.after) else {
            return false;
        };
        let Some(enabled) = after.enabled_ns.checked_sub(before.enabled_ns) else {
            return false;
        };
        let Some(running) = after.running_ns.checked_sub(before.running_ns) else {
            return false;
        };
        enabled > 0 && enabled == running && before.cycles == 0 && before.instructions == 0
    }

    pub fn json(&self) -> serde_json::Value {
        let snapshot = |value: Option<Snapshot>| {
            value.map(|v| {
                serde_json::json!({"enabled_ns": v.enabled_ns, "running_ns": v.running_ns,
                    "user_cycles": v.cycles, "user_instructions": v.instructions})
            })
        };
        serde_json::json!({
            "wall_ns": self.wall_ns, "thread_cpu_ns": self.thread_cpu_ns,
            "before": snapshot(self.before), "after": snapshot(self.after),
            "errors": self.errors, "usable": self.valid(),
        })
    }
}

impl Group {
    pub fn open() -> io::Result<Self> {
        // SAFETY: this helper returns a fixed C sizeof value without pointers.
        if unsafe { counter_snapshot_size() } != std::mem::size_of::<Snapshot>() as u64 {
            return Err(io::Error::other("counter snapshot ABI size mismatch"));
        }
        let mut fds = [-1; 2];
        let mut ids = [0; 2];
        // SAFETY: both arrays have two initialized, writable elements. The C
        // helper closes all partially opened descriptors on failure.
        checked(unsafe { counter_open(fds.as_mut_ptr(), ids.as_mut_ptr()) })?;
        // SAFETY: on success the helper transfers two distinct open descriptors.
        let leader = unsafe { OwnedFd::from_raw_fd(fds[0]) };
        // SAFETY: this is the other uniquely owned descriptor from the helper.
        let member = unsafe { OwnedFd::from_raw_fd(fds[1]) };
        Ok(Self {
            leader,
            _member: member,
            ids,
            _current_thread: PhantomData,
        })
    }

    fn read(&self) -> io::Result<Snapshot> {
        let mut value = Snapshot::default();
        // SAFETY: the descriptor and both event IDs remain live, and `value`
        // has the checked C layout. The helper validates byte count and IDs.
        checked(unsafe {
            counter_read(self.leader.as_raw_fd(), self.ids.as_ptr(), &mut value)
        })?;
        Ok(value)
    }

    #[cfg(feature = "pmu-controls")]
    pub fn snapshot_for_control(&self) -> io::Result<Snapshot> {
        self.read()
    }

    pub fn observe<T>(&self, body: impl FnOnce() -> T) -> (T, Observation) {
        let mut errors = Vec::new();
        // Stop first, including after an earlier unavailable observation.
        // SAFETY: all three operations use this group's owned leader descriptor.
        let ready = checked(unsafe { counter_disable(self.leader.as_raw_fd()) })
            .and_then(|()| checked(unsafe { counter_reset(self.leader.as_raw_fd()) }))
            .and_then(|()| self.read());
        let before = match ready {
            Ok(value) => Some(value),
            Err(error) => {
                errors.push(format!("prepare: {error}"));
                None
            },
        };
        let mut running = None;
        if before.is_some() {
            // SAFETY: the owned descriptor refers to the group leader.
            match checked(unsafe { counter_enable(self.leader.as_raw_fd()) }) {
                Ok(()) => {
                    running = Some(Running {
                        group: self,
                        active: true,
                    });
                },
                Err(error) => errors.push(format!("enable: {error}")),
            }
        }
        let cpu_start = thread_cpu_ns();
        let wall = Instant::now();
        let result = body();
        let wall_ns =
            u64::try_from(wall.elapsed().as_nanos()).expect("diagnostic duration fits u64");
        let cpu_end = thread_cpu_ns();
        if let Some(running) = running.as_mut()
            && let Err(error) = running.stop()
        {
            errors.push(format!("disable: {error}"));
        }
        drop(running);
        let after = match self.read() {
            Ok(value) => Some(value),
            Err(error) => {
                errors.push(format!("read: {error}"));
                None
            },
        };
        let thread_cpu_ns = match (cpu_start, cpu_end) {
            (Ok(start), Ok(end)) => end.checked_sub(start),
            (start, end) => {
                errors.push(format!("thread clock: {start:?}, {end:?}"));
                None
            },
        };
        (
            result,
            Observation {
                wall_ns,
                thread_cpu_ns,
                before,
                after,
                errors,
            },
        )
    }
}

pub fn unavailable<T>(body: impl FnOnce() -> T, error: &str) -> (T, Observation) {
    let cpu_start = thread_cpu_ns();
    let wall = Instant::now();
    let result = body();
    let wall_ns =
        u64::try_from(wall.elapsed().as_nanos()).expect("diagnostic duration fits u64");
    let cpu_end = thread_cpu_ns();
    let thread_cpu_ns = match (cpu_start, cpu_end) {
        (Ok(start), Ok(end)) => end.checked_sub(start),
        _ => None,
    };
    (
        result,
        Observation {
            wall_ns,
            thread_cpu_ns,
            before: None,
            after: None,
            errors: vec![error.to_owned()],
        },
    )
}
