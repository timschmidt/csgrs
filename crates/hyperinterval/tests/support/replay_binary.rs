//! Select the executable that consumes a parent test's serialized interval.

#[cfg(target_arch = "wasm32")]
pub use std::env::current_exe as path;

#[cfg(not(target_arch = "wasm32"))]
pub fn path() -> std::io::Result<std::path::PathBuf> {
    match option_env!("HYPERINTERVAL_REPLAY_DRIVER") {
        Some(driver) => Ok(driver.into()),
        None => std::env::current_exe(),
    }
}
