//! Diagnostic-only integer counters for existing calls and allocation scopes.
//!
//! Rows are inclusive: an allocation may belong to several active scopes.
//! No counter owns or clones a scalar, point, geometry object or cache value.
use std::cell::RefCell;

pub const SPANS: [&str; 16] = [
    "probe", "endpoint", "coordinate_compare", "distance_squared", "policy_point_equal",
    "is_zero", "normal_form", "normal_form_recursion", "arithmetic_reuse",
    "cache_initialize", "cache_store_laurent", "cache_load_laurent", "node_new",
    "rational_linear_retention", "rational_product_retention", "rational_negation_retention",
];
pub const EVENTS: [&str; 6] = [
    "normal_form_reused", "normal_form_fallback", "cache_created",
    "cache_published_laurent", "arithmetic_reuse_ready", "arithmetic_reuse_first",
];

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Row {
    pub entries: u64,
    pub allocations: u64,
    pub allocated_bytes: u64,
    pub deallocations: u64,
    pub deallocated_bytes: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Snapshot {
    pub rows: [Row; SPANS.len()],
    pub events: [u64; EVENTS.len()],
}

const EMPTY: Snapshot = Snapshot {
    rows: [Row { entries: 0, allocations: 0, allocated_bytes: 0, deallocations: 0, deallocated_bytes: 0 }; SPANS.len()],
    events: [0; EVENTS.len()],
};

struct State {
    active: bool,
    mask: u64,
    snapshot: Snapshot,
}

thread_local! {
    static STATE: RefCell<State> = const { RefCell::new(State { active: false, mask: 0, snapshot: EMPTY }) };
}

pub struct Scope(Option<u64>);

impl Drop for Scope {
    fn drop(&mut self) {
        if let Some(mask) = self.0 {
            STATE.with(|state| {
                let mut state = state.borrow_mut();
                assert!(state.active);
                state.mask = mask;
            });
        }
    }
}

pub fn begin() {
    STATE.with(|state| {
        let mut state = state.borrow_mut();
        assert!(!state.active);
        state.active = true;
        state.mask = 1;
        state.snapshot = EMPTY;
        state.snapshot.rows[0].entries = 1;
    });
}

pub fn finish() -> Snapshot {
    STATE.with(|state| {
        let mut state = state.borrow_mut();
        assert!(state.active);
        assert_eq!(state.mask, 1, "all measured calls must have returned");
        state.active = false;
        state.mask = 0;
        state.snapshot
    })
}

pub fn enter(span: usize) -> Scope {
    STATE.with(|state| {
        let mut state = state.borrow_mut();
        if !state.active {
            return Scope(None);
        }
        assert!(span > 0 && span < SPANS.len());
        let prior = state.mask;
        state.mask |= 1 << span;
        state.snapshot.rows[span].entries += 1;
        Scope(Some(prior))
    })
}

pub fn event(index: usize) {
    STATE.with(|state| {
        let mut state = state.borrow_mut();
        if state.active {
            state.snapshot.events[index] += 1;
        }
    });
}

pub fn allocated(bytes: usize) {
    allocation(bytes, false);
}

pub fn deallocated(bytes: usize) {
    allocation(bytes, true);
}

fn allocation(bytes: usize, deallocation: bool) {
    STATE.with(|state| {
        let mut state = state.borrow_mut();
        let mut mask = state.mask;
        while mask != 0 {
            let index = mask.trailing_zeros() as usize;
            mask &= mask - 1;
            let row = &mut state.snapshot.rows[index];
            if deallocation {
                row.deallocations += 1;
                row.deallocated_bytes += bytes as u64;
            } else {
                row.allocations += 1;
                row.allocated_bytes += bytes as u64;
            }
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn overlapping_scopes_count_once_per_active_kind_and_restore() {
        begin();
        {
            let _outer = enter(1);
            allocated(7);
            {
                let _nested = enter(1);
                let _inner = enter(2);
                allocated(11);
                deallocated(3);
                event(0);
            }
            allocated(13);
        }
        allocated(17);
        let actual = finish();
        assert_eq!(actual.rows[0].allocations, 4);
        assert_eq!(actual.rows[0].allocated_bytes, 48);
        assert_eq!(actual.rows[1].entries, 2);
        assert_eq!(actual.rows[1].allocations, 3);
        assert_eq!(actual.rows[1].allocated_bytes, 31);
        assert_eq!(actual.rows[2].allocations, 1);
        assert_eq!(actual.rows[2].allocated_bytes, 11);
        assert_eq!(actual.rows[1].deallocated_bytes, 3);
        assert_eq!(actual.events[0], 1);
        allocated(19);
        begin();
        assert_eq!(finish().rows[0].allocations, 0);
    }

    #[test]
    fn unwind_restores_the_observation_scope() {
        begin();
        assert!(std::panic::catch_unwind(|| {
            let _outer = enter(1);
            let _inner = enter(2);
            panic!("diagnostic unwind calibration");
        }).is_err());
        let actual = finish();
        assert_eq!(actual.rows[1].entries, 1);
        assert_eq!(actual.rows[2].entries, 1);
        begin();
        assert_eq!(finish().rows[1].entries, 0);
    }
}
