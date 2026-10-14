//! Counts of existing local memo copies in an isolated diagnostic build.
use std::cell::Cell;

pub const KIND_NAMES: [&str; 10] = [
    "Int",
    "One",
    "Ratio",
    "Pi",
    "InvPi",
    "Tau",
    "Sqrt2",
    "Sqrt3",
    "OtherConstant",
    "OtherNode",
];
pub const MODE_NAMES: [&str; 2] = ["retained", "original"];
pub const EVENT_NAMES: [&str; 2] = ["memo_store", "memo_hit"];
pub const FIELD_NAMES: [&str; 10] = [
    "events",
    "none",
    "empty",
    "one_rational_term",
    "one_radical_term",
    "other_closed",
    "opaque",
    "terms",
    "radicals",
    "atoms",
];
pub type Counter = [u64; FIELD_NAMES.len()];
pub type Counts = [[[Counter; EVENT_NAMES.len()]; MODE_NAMES.len()]; KIND_NAMES.len()];
const SLOTS: usize = KIND_NAMES.len() * MODE_NAMES.len() * EVENT_NAMES.len();

#[derive(Clone, Copy)]
pub(crate) enum Event {
    Store,
    Hit,
}

#[derive(Clone, Copy)]
pub(crate) enum Shape {
    None,
    Empty,
    OneRationalTerm,
    OneRadicalTerm,
    OtherClosed,
    Opaque,
}

pub(crate) struct Payload {
    pub(crate) shape: Shape,
    pub(crate) terms: usize,
    pub(crate) radicals: usize,
    pub(crate) atoms: usize,
}

thread_local! {
    static ACTIVE: Cell<bool> = const { Cell::new(false) };
    static COUNTERS: [Cell<Counter>; SLOTS] = const { [const { Cell::new([0; FIELD_NAMES.len()]) }; SLOTS] };
}

pub(crate) fn active() -> bool {
    ACTIVE.with(Cell::get)
}

pub(crate) fn record(kind: usize, retained: bool, event: Event, payload: Payload) {
    if !active() {
        return;
    }
    let mode = usize::from(!retained);
    let slot = (kind * MODE_NAMES.len() + mode) * EVENT_NAMES.len() + event as usize;
    COUNTERS.with(|cells| {
        let mut counts = cells[slot].get();
        counts[0] += 1;
        counts[1 + payload.shape as usize] += 1;
        counts[7] += payload.terms as u64;
        counts[8] += payload.radicals as u64;
        counts[9] += payload.atoms as u64;
        cells[slot].set(counts);
    });
}

pub fn with_recording<T>(body: impl FnOnce() -> T) -> (T, Counts) {
    assert!(
        !ACTIVE.with(|cell| cell.replace(true)),
        "nested memo diagnostic scope"
    );
    COUNTERS.with(|cells| {
        for cell in cells {
            cell.set([0; FIELD_NAMES.len()]);
        }
    });
    struct Finish;
    impl Drop for Finish {
        fn drop(&mut self) {
            ACTIVE.with(|cell| cell.set(false));
        }
    }
    let guard = Finish;
    let result = body();
    let counters = COUNTERS.with(|cells| {
        std::array::from_fn(|kind| {
            std::array::from_fn(|mode| {
                std::array::from_fn(|event| {
                    cells[(kind * MODE_NAMES.len() + mode) * EVENT_NAMES.len() + event].get()
                })
            })
        })
    });
    drop(guard);
    (result, counters)
}
