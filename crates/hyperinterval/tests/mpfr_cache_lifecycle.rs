#![cfg(not(target_env = "msvc"))]

#[path = "../../hyperreal/tests/support/mpfr.rs"]
mod mpfr_cache;

use rug::{Float, Rational, float::Constant};
use std::{sync::Arc, sync::Barrier, thread};

// Under LeakSanitizer this process fails if an oracle thread leaves its MPFR
// caches behind. Exercise both exits and repeated registration while other
// threads still use their own caches; no global cache clearing is permitted.
#[test]
fn mpfr_oracle_caches_release_on_normal_and_unwinding_thread_exit() {
    let barrier = Arc::new(Barrier::new(8));
    let threads: Vec<_> = (0..8)
        .map(|index| {
            let barrier = Arc::clone(&barrier);
            thread::spawn(move || {
                mpfr_cache::cleanup_on_thread_exit();
                let pi = Float::with_val(512, Constant::Pi);
                mpfr_cache::cleanup_on_thread_exit();
                barrier.wait();
                assert!(pi > 3 && pi < 4);
                if index % 2 != 0 {
                    panic!("intentional oracle unwind");
                }
                let mut atan = Float::with_val(512, 1);
                atan.atan_mut();
                let quarter_pi = pi / 4;
                assert_eq!(atan, quarter_pi);
                atan.to_rational().unwrap()
            })
        })
        .collect();
    for (index, child) in threads.into_iter().enumerate() {
        match child.join() {
            Ok(value) => {
                assert_eq!(index % 2, 0);
                assert!(value > Rational::from((3, 4)));
                assert!(value < Rational::from((4, 5)));
            }
            Err(payload) => {
                assert_eq!(index % 2, 1);
                assert_eq!(
                    payload.downcast_ref::<&str>(),
                    Some(&"intentional oracle unwind")
                );
            }
        }
    }
}
