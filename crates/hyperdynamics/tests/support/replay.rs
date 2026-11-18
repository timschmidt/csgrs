use std::{io, path::PathBuf};

/// Fresh native replays use the current test executable. The conformance
/// coordinator instead launches the same test through the supplied WASI driver.
#[inline]
pub fn binary() -> io::Result<PathBuf> {
    match option_env!("HYPERDYNAMICS_REPLAY_DRIVER") {
        Some(driver) => Ok(PathBuf::from(driver)),
        None => std::env::current_exe(),
    }
}
