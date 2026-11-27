//! Stack high-water measurement for resource atlases.
//!
//! A measured call runs on a private stack: an anonymous mapping with a guard
//! page at its low end, entered through `psm::on_stack`. Before each call the
//! mapping is reset with `madvise(MADV_DONTNEED)`, so every page reads as
//! untouched. Afterwards `mincore` reports which pages the call made
//! resident; the lowest one bounds the depth, and the first nonzero word in it
//! refines the estimate. The result is the depth below the probe's own entry
//! frames, accurate to within one page (4 KiB on common hosts): writes of zero
//! at the very bottom of the deepest page can be missed. Transparent huge
//! pages are disabled on the mapping so residency stays page granular.
//!
//! Only Linux on x86_64 and aarch64 is supported; elsewhere no stack depth is
//! reported. A call that overflows the whole mapping hits the guard page and
//! ends the process, as an overflow on an ordinary thread stack would.
#![allow(unsafe_code)]

/// Private stack for measured calls.
#[cfg(all(
    target_os = "linux",
    any(target_arch = "x86_64", target_arch = "aarch64")
))]
pub(crate) struct Probe {
    base: *mut u8,
    size: usize,
    page: usize,
    // Depth consumed by entering the probe with an empty callback.
    baseline: usize,
    // One residency byte per page, allocated once so that no measured
    // window sees the probe's own heap use.
    resident: Vec<u8>,
}

#[cfg(all(
    target_os = "linux",
    any(target_arch = "x86_64", target_arch = "aarch64")
))]
impl Probe {
    /// Reserve `size` bytes (rounded to pages) of address space.
    pub(crate) fn new(size: usize) -> Option<Self> {
        if psm::StackDirection::new() != psm::StackDirection::Descending {
            return None;
        }
        // SAFETY: plain libc calls on a fresh private anonymous mapping that
        // this probe owns until Drop unmaps it.
        unsafe {
            let page = usize::try_from(libc::sysconf(libc::_SC_PAGESIZE)).ok()?;
            let size = size.div_ceil(page).max(4) * page;
            let base = libc::mmap(
                std::ptr::null_mut(),
                size,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_PRIVATE | libc::MAP_ANONYMOUS | libc::MAP_NORESERVE,
                -1,
                0,
            );
            if base == libc::MAP_FAILED {
                return None;
            }
            // Transparent huge pages would make residency 2 MiB granular.
            libc::madvise(base, size, libc::MADV_NOHUGEPAGE);
            if libc::mprotect(base, page, libc::PROT_NONE) != 0 {
                libc::munmap(base, size);
                return None;
            }
            let mut probe = Self {
                base: base.cast(),
                size,
                page,
                baseline: 0,
                resident: vec![0; size / page],
            };
            let ((), depth) = probe.run(|| ());
            probe.baseline = depth?;
            Some(probe)
        }
    }

    // Usable stack: everything above the guard page.
    fn usable(&self) -> (*mut u8, usize) {
        // SAFETY: the guard page lies inside the mapping.
        (unsafe { self.base.add(self.page) }, self.size - self.page)
    }

    /// Run `f` on the private stack and return its stack depth in bytes.
    ///
    /// Panics inside `f` are caught on the private stack and resumed after
    /// switching back, so they propagate normally to the caller.
    pub(crate) fn run<R>(&mut self, f: impl FnOnce() -> R) -> (R, Option<usize>) {
        let (low, len) = self.usable();
        // SAFETY: `low..low+len` is our own writable private mapping; after
        // DONTNEED its pages read as zero and are not resident.
        unsafe { libc::madvise(low.cast(), len, libc::MADV_DONTNEED) };
        // SAFETY: page-aligned base and size within the mapping; the callback
        // cannot unwind because every panic is caught inside it.
        let outcome = unsafe {
            psm::on_stack(low, len, || {
                std::panic::catch_unwind(std::panic::AssertUnwindSafe(f))
            })
        };
        let depth = self.high_water(low, len);
        match outcome {
            Ok(value) => (value, depth.map(|d| d.saturating_sub(self.baseline))),
            Err(payload) => std::panic::resume_unwind(payload),
        }
    }

    fn high_water(&mut self, low: *mut u8, len: usize) -> Option<usize> {
        let pages = len / self.page;
        let resident = &mut self.resident[..pages];
        // SAFETY: `low..low+len` is mapped; the slice has one byte per page.
        if unsafe { libc::mincore(low.cast(), len, resident.as_mut_ptr()) } != 0 {
            return None;
        }
        let lowest = resident.iter().position(|r| r & 1 == 1)?;
        // SAFETY: the resident page lies in the mapping and is readable.
        let words = unsafe {
            std::slice::from_raw_parts(
                low.add(lowest * self.page).cast::<u64>(),
                self.page / std::mem::size_of::<u64>(),
            )
        };
        let first = words.iter().position(|&w| w != 0).unwrap_or(words.len());
        let deepest = lowest * self.page + first * std::mem::size_of::<u64>();
        Some(len - deepest)
    }
}

#[cfg(all(
    target_os = "linux",
    any(target_arch = "x86_64", target_arch = "aarch64")
))]
impl Drop for Probe {
    fn drop(&mut self) {
        // SAFETY: unmaps exactly the mapping created in `new`.
        unsafe { libc::munmap(self.base.cast(), self.size) };
    }
}

/// Unsupported hosts report no stack depth.
#[cfg(not(all(
    target_os = "linux",
    any(target_arch = "x86_64", target_arch = "aarch64")
)))]
pub(crate) struct Probe;

#[cfg(not(all(
    target_os = "linux",
    any(target_arch = "x86_64", target_arch = "aarch64")
)))]
impl Probe {
    pub(crate) fn new(_size: usize) -> Option<Self> {
        None
    }
    pub(crate) fn run<R>(&mut self, f: impl FnOnce() -> R) -> (R, Option<usize>) {
        (f(), None)
    }
}
