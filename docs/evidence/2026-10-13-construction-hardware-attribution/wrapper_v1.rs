//! Hardware attribution only; libraries and fixture/witness suffix are frozen.
#[path = "../counters_v1.rs"]
mod counters;

use hyperreal::{Rational, Real};
use std::cell::RefCell;
use std::hint::black_box;

thread_local! {
    static GROUP: RefCell<Option<Result<counters::Group, String>>> = const { RefCell::new(None) };
}

fn measure<T>(name: &str, count: usize, body: impl FnOnce() -> T) -> T {
    let (result, empty, observation) = GROUP.with(|slot| {
        let mut slot = slot.borrow_mut();
        let group = slot
            .get_or_insert_with(|| counters::Group::open().map_err(|error| error.to_string()));
        match group {
            Ok(group) => {
                let (_, empty) = group.observe(|| black_box(()));
                let (result, observation) = group.observe(body);
                (result, empty, observation)
            },
            Err(error) => {
                let (_, empty) = counters::unavailable(|| black_box(()), error);
                let (result, observation) = counters::unavailable(body, error);
                (result, empty, observation)
            },
        }
    });
    println!(
        "{}",
        serde_json::json!({
            "hardware_phase": name, "iterations": count,
            "empty_window": empty.json(), "work_window": observation.json(),
        })
    );
    black_box(result)
}
