//! Exercise the actual test helper on ordinary and unwinding worker exits.
#[path = "../../../target/normal-image-test-cleanup-v1/source/crates/hyperreal/tests/support/mpfr.rs"]
mod mpfr;

use rug::Float;
use rug::float::{self, FreeCache};

fn worker(unwind: bool) -> [String; 3] {
    mpfr::cleanup_on_thread_exit();
    mpfr::cleanup_on_thread_exit();
    let mut values: [Float; 3] = std::array::from_fn(|_| Float::with_val(512, 2));
    values[0].ln_mut();
    values[1].exp_mut();
    values[2].atan_mut();
    assert!(values.iter().all(Float::is_finite));
    let witness = values.each_ref().map(ToString::to_string);
    if unwind {
        panic!("expected MPFR cache cleanup control");
    }
    drop(values);
    witness
}

fn main() {
    let unwind = match std::env::args().nth(1).as_deref() {
        Some("normal") => false,
        Some("unwind") => true,
        _ => panic!("normal or unwind"),
    };
    if unwind {
        // The expected panic remains checked through join's payload below.
        std::panic::set_hook(Box::new(|_| {}));
    }
    let workers: [_; 2] =
        std::array::from_fn(|index| std::thread::spawn(move || worker(unwind && index == 1)));
    let results = workers.map(|handle| handle.join());
    float::free_cache(FreeCache::All);
    let mut panics = 0;
    for (index, result) in results.into_iter().enumerate() {
        match result {
            Ok(witness) => println!("worker={index} values={witness:?}"),
            Err(payload) => {
                assert!(unwind && index == 1);
                assert_eq!(
                    payload.downcast_ref::<&str>(),
                    Some(&"expected MPFR cache cleanup control")
                );
                panics += 1;
            },
        }
    }
    assert_eq!(panics, usize::from(unwind));
    if unwind {
        println!("expected_worker_panics={panics}");
    }
}
