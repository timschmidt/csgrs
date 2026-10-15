//! Caller-controlled retention for the modeling operations in [`super`].
//!
//! These thread-local caches reuse completed values. They do not select a
//! predicate policy, change precision, cache errors, or certify geometry.
//! Context-sensitive entries retain both the selected [`crate::GeometryContext`]
//! and the result's certainty. A warm approximate entry cannot answer a strict
//! request. Source meshes are keyed by shared position/index storage identity;
//! primitive parameters and slice heights use structural scalar equality.
//!
//! Budgets count entries, **not bytes**: an entry can retain arbitrarily large
//! geometry and scalar expressions. Use [`CachePolicy::DISABLED`] to release
//! all entries and avoid retaining new ones. Kernel and scalar caches belong
//! to their respective crates and are outside this policy.
//!
//! ```
//! use csgrs::{Real, solid::{self, cache::{self, CachePolicy}}};
//!
//! let mesh = cache::with_policy(CachePolicy::DISABLED, || {
//!     solid::cube(Real::from(2)).expect("valid primitive fixture")
//! });
//! assert_eq!(mesh.triangles.len(), 12);
//! ```

use std::cell::RefCell;

mod storage;
pub(super) use storage::*;

/// Maximum retained entries in each grammar cache, on the current thread.
///
/// Zero disables a cache. Insertion evicts the oldest entry when necessary;
/// a hit does not change insertion order (FIFO, not LRU). Limits are applied
/// immediately on entering and leaving [`with_policy`]. Curve cache limits
/// have no effect when the `curve` feature is disabled.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CachePolicy {
    /// Primitive meshes keyed by structural constructor parameters.
    pub primitives: usize,
    /// Linear, grid, and arc distributions, including their predicate context.
    pub distributions: usize,
    /// Concatenated mesh arrays, keyed by ordered input storage identities.
    pub merges: usize,
    /// XY projections, including their predicate context.
    pub flattenings: usize,
    /// Horizontal slices, including their height and predicate context.
    pub slices: usize,
}

impl CachePolicy {
    /// Existing retention limits: 32 primitives and eight of every other kind.
    pub const DEFAULT: Self = Self {
        primitives: 32,
        distributions: 8,
        merges: 8,
        flattenings: 8,
        slices: 8,
    };

    /// Release existing grammar cache entries and retain no new ones.
    pub const DISABLED: Self = Self {
        primitives: 0,
        distributions: 0,
        merges: 0,
        flattenings: 0,
        slices: 0,
    };
}

impl Default for CachePolicy {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Current number of retained entries in each grammar cache.
///
/// Counts include input identities retained as part of each entry, but do not
/// count them separately. These are not allocation or byte measurements.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct CacheUsage {
    /// Retained primitive results.
    pub primitives: usize,
    /// Retained distribution results.
    pub distributions: usize,
    /// Retained concatenation results.
    pub merges: usize,
    /// Retained XY projections; zero without the `curve` feature.
    pub flattenings: usize,
    /// Retained horizontal slices; zero without the `curve` feature.
    pub slices: usize,
}

std::thread_local! {
    static STATE: RefCell<CacheState> = const { RefCell::new(CacheState::new()) };
}

/// Return the current thread's active grammar cache policy.
pub fn policy() -> CachePolicy {
    STATE.with_borrow(|state| state.policy)
}

/// Return the current thread's retained entry counts without warming any cache.
pub fn usage() -> CacheUsage {
    STATE.with_borrow(CacheState::usage)
}

/// Drop all grammar cache entries on this thread, keeping the active policy.
///
/// Returned meshes and curves remain valid: they own their shared storage.
pub fn clear() {
    STATE.with_borrow_mut(|state| {
        let policy = state.policy;
        *state = CacheState::new();
        state.policy = policy;
    });
}

/// Run a synchronous operation with explicit grammar cache retention limits.
///
/// Scopes nest and restore the previous policy even during panic unwinding.
/// They share this thread's entries: shrinking a limit immediately evicts the
/// oldest entries, and restoring a larger limit does not resurrect them.
/// Increasing a limit never reserves capacity in advance. Entries admitted by
/// an inner scope remain available only within the restored outer limits.
///
/// The policy affects retention and reuse only. Predicate context, exact
/// coordinates, topology, and certainty do not depend on these limits.
///
/// Spawned threads start with [`CachePolicy::DEFAULT`]. This scope does not
/// follow a future across suspension or thread migration; pass the policy to
/// each synchronous worker explicitly.
pub fn with_policy<T>(policy: CachePolicy, operation: impl FnOnce() -> T) -> T {
    struct RestorePolicy(CachePolicy);

    impl Drop for RestorePolicy {
        fn drop(&mut self) {
            STATE.with_borrow_mut(|state| state.set_policy(self.0));
        }
    }

    let _restore = RestorePolicy(STATE.with_borrow_mut(|state| {
        let prior = state.policy;
        state.set_policy(policy);
        prior
    }));
    operation()
}
