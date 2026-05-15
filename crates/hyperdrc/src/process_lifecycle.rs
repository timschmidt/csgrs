//! Process-lifecycle helpers for CLI-owned subprocesses.
//!
//! The library normally performs in-process parsing and checks. External
//! converters are different: if the terminal sends an interrupt to HyperDRC, the
//! converter and any converter-launched workers must not be left behind.

use std::process::Child;
use std::sync::Once;
use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};

static ACTIVE_CHILD_GROUP: AtomicI32 = AtomicI32::new(0);
static TERMINATION_HANDLER_INSTALLED: Once = Once::new();
static TERMINATION_HANDLER_FAILED: AtomicBool = AtomicBool::new(false);

/// Install CLI termination handling for subprocess cleanup.
///
/// The handler is intentionally scoped to the command-line wrapper rather than
/// installed by [`crate::run`]. Embedders may have their own signal policy, but
/// the standalone CLI must clean up converter process groups on interrupts,
/// terminal quit keys, service-manager termination, and similar normal signals.
pub fn install_cli_termination_handler() -> std::io::Result<()> {
    install_platform_termination_handler()?;
    if TERMINATION_HANDLER_FAILED.load(Ordering::SeqCst) {
        Err(std::io::Error::last_os_error())
    } else {
        Ok(())
    }
}

#[cfg(unix)]
fn install_platform_termination_handler() -> std::io::Result<()> {
    TERMINATION_HANDLER_INSTALLED.call_once(|| unsafe {
        for signal in [libc::SIGINT, libc::SIGTERM, libc::SIGHUP, libc::SIGQUIT] {
            let handler = handle_termination_signal as *const () as libc::sighandler_t;
            if libc::signal(signal, handler) == libc::SIG_ERR {
                TERMINATION_HANDLER_FAILED.store(true, Ordering::SeqCst);
                return;
            }
        }
    });
    Ok(())
}

#[cfg(not(unix))]
fn install_platform_termination_handler() -> std::io::Result<()> {
    Ok(())
}

/// Prepare a subprocess command so descendants can be killed as a unit.
#[cfg(unix)]
pub fn configure_child_command(command: &mut std::process::Command) {
    use std::os::unix::process::CommandExt;

    unsafe {
        command.pre_exec(|| {
            // Put the converter into a fresh process group before it can spawn
            // workers. HyperDRC can then terminate `-pgid` and clean up the
            // converter tree instead of only the direct child.
            if libc::setpgid(0, 0) != 0 {
                return Err(std::io::Error::last_os_error());
            }

            // Linux parent-death signaling covers hard parent exits that do not
            // run Rust destructors or signal handlers. Other Unix targets still
            // get process-group cleanup for normal CLI termination paths.
            #[cfg(target_os = "linux")]
            {
                if libc::prctl(libc::PR_SET_PDEATHSIG, libc::SIGTERM) != 0 {
                    return Err(std::io::Error::last_os_error());
                }
                if libc::getppid() == 1 {
                    libc::_exit(128 + libc::SIGTERM);
                }
            }

            Ok(())
        });
    }
}

/// Prepare a subprocess command so descendants can be killed as a unit.
#[cfg(not(unix))]
pub fn configure_child_command(_command: &mut std::process::Command) {}

/// RAII guard for a CLI-owned child process.
pub struct ChildProcessGuard {
    child: Child,
    active_group: i32,
    disarmed: bool,
}

impl ChildProcessGuard {
    /// Track a newly spawned child and expose it to the CLI signal handler.
    pub fn new(child: Child) -> Self {
        let active_group = child.id() as i32;
        ACTIVE_CHILD_GROUP.store(active_group, Ordering::SeqCst);
        Self {
            child,
            active_group,
            disarmed: false,
        }
    }

    /// Wait for the child to exit and unregister it from signal cleanup.
    pub fn wait(mut self) -> std::io::Result<std::process::ExitStatus> {
        let status = self.child.wait();
        self.disarm();
        status
    }

    fn disarm(&mut self) {
        if ACTIVE_CHILD_GROUP.load(Ordering::SeqCst) == self.active_group {
            ACTIVE_CHILD_GROUP.store(0, Ordering::SeqCst);
        }
        self.disarmed = true;
    }
}

impl Drop for ChildProcessGuard {
    fn drop(&mut self) {
        if !self.disarmed {
            terminate_active_child_group(self.active_group);
            let _ = self.child.kill();
            let _ = self.child.wait();
            self.disarm();
        }
    }
}

#[cfg(unix)]
extern "C" fn handle_termination_signal(signal: libc::c_int) {
    let group = ACTIVE_CHILD_GROUP.load(Ordering::SeqCst);
    if group > 0 {
        terminate_active_child_group(group);
    }

    unsafe {
        libc::_exit(128 + signal);
    }
}

#[cfg(unix)]
fn terminate_active_child_group(group: i32) {
    unsafe {
        // Negative PID targets the process group. The child is configured into
        // its own group, so this should not signal HyperDRC itself.
        libc::kill(-group, libc::SIGTERM);
        libc::kill(-group, libc::SIGCONT);
    }
}

#[cfg(not(unix))]
fn terminate_active_child_group(_group: i32) {}

#[cfg(test)]
mod tests {
    use std::process::Command;
    use std::thread;
    use std::time::Duration;

    use super::{ChildProcessGuard, configure_child_command};

    #[cfg(unix)]
    #[test]
    fn child_guard_drop_terminates_process_group() {
        let mut command = Command::new("sh");
        command.arg("-c").arg("sleep 30");
        configure_child_command(&mut command);
        let child = command.spawn().expect("sleep helper should spawn");
        let process_group = child.id() as i32;

        let guard = ChildProcessGuard::new(child);
        drop(guard);
        thread::sleep(Duration::from_millis(100));

        let still_running = unsafe { libc::kill(-process_group, 0) == 0 };
        assert!(
            !still_running,
            "dropping a CLI child guard should terminate the converter process group"
        );
    }
}
