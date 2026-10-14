//! A threaded MPFR cache-lifecycle control linking Rug, but no Hyper crates.
use rug::Float;
use rug::float::{self, FreeCache};

fn worker(cleanup: bool) -> [String; 3] {
    let mut values: [Float; 3] = std::array::from_fn(|_| Float::with_val(512, 2));
    values[0].ln_mut();
    values[1].exp_mut();
    values[2].atan_mut();
    assert!(values.iter().all(Float::is_finite));
    let witness = values.each_ref().map(ToString::to_string);
    drop(values);
    if cleanup {
        float::free_cache(FreeCache::Local);
    }
    witness
}

fn main() {
    let cleanup = match std::env::args().nth(1).as_deref() {
        Some("retain") => false,
        Some("cleanup") => true,
        _ => panic!("retain or cleanup"),
    };
    let workers: [_; 2] = std::array::from_fn(|_| std::thread::spawn(move || worker(cleanup)));
    let witnesses = workers.map(|handle| handle.join().expect("MPFR worker"));
    // Both modes release shared/main-thread caches after every worker exits.
    // Only the explicit worker-local cleanup differs between the two modes.
    float::free_cache(FreeCache::All);
    for (index, witness) in witnesses.iter().enumerate() {
        println!("worker={index} values={witness:?}");
    }
}
