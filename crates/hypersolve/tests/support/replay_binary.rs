//! Select the child that replays the source supplied by a native test parent.
// Only a native parent launches a replay child. A WASI guest executes the
// selected child case with the source supplied by that parent.
#[cfg(target_arch = "wasm32")]
pub use std::env::current_exe as path;

#[cfg(not(target_arch = "wasm32"))]
use std::{io, path::PathBuf};

#[cfg(not(target_arch = "wasm32"))]
#[inline]
pub fn path() -> io::Result<PathBuf> {
    match option_env!("HYPERSOLVE_REPLAY_DRIVER") {
        Some(driver) => Ok(PathBuf::from(driver)),
        None => std::env::current_exe(),
    }
}
