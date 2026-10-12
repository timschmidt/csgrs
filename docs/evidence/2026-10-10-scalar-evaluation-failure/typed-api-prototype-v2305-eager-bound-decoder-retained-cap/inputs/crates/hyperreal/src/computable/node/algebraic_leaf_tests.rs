#[cfg(test)]
mod algebraic_leaf_tests {
    use super::*;
    use crate::EvaluationBudget;
    use crate::computable::budget::BudgetScope;
    use std::alloc::{GlobalAlloc, Layout, System};
    use std::cell::Cell;

    // This allocator exists only in the scalar unit-test executable. Opt-in
    // thread-local counters isolate a synchronous query from fixture creation,
    // BudgetScope setup, test-harness work and other concurrently running tests.
    // Counts describe Rust allocation requests, not RSS or native allocations.
    struct CountingAllocator;

    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    pub(super) struct Allocations {
        pub(super) calls: usize,
        pub(super) bytes: usize,
        largest: usize,
    }

    thread_local! {
        static TRACK: Cell<bool> = const { Cell::new(false) };
        static COUNTS: Cell<Allocations> = const { Cell::new(Allocations { calls: 0, bytes: 0, largest: 0 }) };
    }

    fn record(size: usize) {
        let _ = TRACK.try_with(|track| {
            if track.get() {
                let _ = COUNTS.try_with(|counts| {
                    let old = counts.get();
                    counts.set(Allocations {
                        calls: old.calls.saturating_add(1),
                        bytes: old.bytes.saturating_add(size),
                        largest: old.largest.max(size),
                    });
                });
            }
        });
    }

    // SAFETY: every operation forwards the original pointer/layout unchanged
    // to System. The counters neither allocate nor dereference the pointers;
    // fallible TLS access also permits allocation during thread teardown.
    unsafe impl GlobalAlloc for CountingAllocator {
        unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            let pointer = unsafe { System.alloc(layout) };
            if !pointer.is_null() {
                record(layout.size());
            }
            pointer
        }

        unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
            let pointer = unsafe { System.alloc_zeroed(layout) };
            if !pointer.is_null() {
                record(layout.size());
            }
            pointer
        }

        unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, size: usize) -> *mut u8 {
            let result = unsafe { System.realloc(pointer, layout, size) };
            if !result.is_null() {
                record(size);
            }
            result
        }

        unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
            unsafe { System.dealloc(pointer, layout) };
        }
    }

    #[global_allocator]
    static ALLOCATOR: CountingAllocator = CountingAllocator;

    pub(super) fn measure<T>(query: impl FnOnce() -> T) -> (T, Allocations) {
        struct Stop;
        impl Drop for Stop {
            fn drop(&mut self) {
                TRACK.with(|track| track.set(false));
            }
        }
        assert!(!TRACK.with(Cell::get));
        COUNTS.with(|counts| counts.set(Allocations::default()));
        TRACK.with(|track| track.set(true));
        let stop = Stop;
        let result = query();
        drop(stop);
        (result, COUNTS.with(Cell::get))
    }

    fn raw(kind: Approximation) -> Computable {
        Computable::raw_periodic_operand(kind)
    }

    fn integer(bits: usize, negative: bool) -> Computable {
        let value = BigInt::one() << (bits - 1);
        raw(Approximation::Int(if negative { -value } else { value }))
    }

    fn root(value: Computable, degree: u32) -> Computable {
        raw(if degree == 2 {
            Approximation::Sqrt(value)
        } else {
            Approximation::NthRoot(value, degree)
        })
    }

    fn empty() -> AlgebraicSeparation {
        AlgebraicSeparation {
            denominator_log2: 0,
            conjugate_log2: 0,
            generators: Vec::new(),
        }
    }

    fn warm_constants() {
        for value in [
            Rational::zero(),
            Rational::one(),
            Rational::new(2),
            Rational::new(3),
        ] {
            let node = root(Computable::rational(value), 2);
            let _ = empty().rational_root_depends_on_existing(&node, 2);
        }
    }

    #[test]
    fn algebraic_leaf_rejections_do_not_allocate_coefficient_storage() {
        warm_constants();
        let mut observations = Vec::new();
        for bits in [4097, 65_537, 1_000_001] {
            for kind in ["oversized", "negative", "different_degree", "ratio"] {
                let leaf = if kind == "ratio" {
                    raw(Approximation::Ratio(
                        Rational::from_bigint_fraction(
                            BigInt::one() << (bits - 1),
                            BigUint::from(3_u32),
                        )
                        .unwrap(),
                    ))
                } else {
                    integer(bits, kind == "negative")
                };
                let node = root(leaf, if kind == "different_degree" { 3 } else { 2 });
                let budget = EvaluationBudget::new(1);
                let (answer, allocations) = {
                    let _scope = BudgetScope::enter(&budget);
                    let result = measure(|| empty().rational_root_depends_on_existing(&node, 2));
                    assert!(!evaluation_exhausted());
                    result
                };
                assert_eq!(answer, Some(false));
                assert_eq!(budget.consumed(), 1);
                eprintln!("leaf rejection kind={kind}, bits={bits}, allocations={allocations:?}");
                observations.push(allocations);
            }
        }
        assert!(
            observations.iter().all(|r| *r == Allocations::default()),
            "{observations:?}"
        );
    }

    #[test]
    fn algebraic_leaf_interrupted_copy_allocates_nothing() {
        warm_constants();
        let mut observations = Vec::new();
        for bits in [16, 256, 1024, 4096] {
            let node = root(integer(bits, false), 2);
            let budget = EvaluationBudget::new(1);
            let (answer, allocations) = {
                let _scope = BudgetScope::enter(&budget);
                let result = measure(|| empty().rational_root_depends_on_existing(&node, 2));
                assert!(evaluation_exhausted());
                result
            };
            assert_eq!(answer, None);
            assert_eq!(budget.consumed(), 1);
            // Each exponent is odd, so this positive power of two is not square.
            assert_eq!(
                empty().rational_root_depends_on_existing(&node, 2),
                Some(false)
            );
            eprintln!("leaf interrupted copy bits={bits}, allocations={allocations:?}");
            observations.push(allocations);
        }
        assert!(
            observations.iter().all(|r| *r == Allocations::default()),
            "{observations:?}"
        );
    }

    #[test]
    fn algebraic_leaf_rejected_bases_add_no_allocation() {
        warm_constants();
        let candidate = root(Computable::rational(Rational::new(5)), 2);
        let _ = empty().rational_root_depends_on_existing(&candidate, 2);
        let (expected, control) =
            measure(|| empty().rational_root_depends_on_existing(&candidate, 2));
        assert_eq!(expected, Some(false));
        let mut observations = Vec::new();
        for bits in [4097, 65_537, 1_000_001] {
            for kind in ["oversized", "negative", "different_degree"] {
                let degree = if kind == "different_degree" { 3 } else { 2 };
                let metadata = AlgebraicSeparation {
                    generators: vec![AlgebraicGenerator {
                        root: root(integer(bits, kind == "negative"), degree),
                        degree,
                    }],
                    ..empty()
                };
                let (answer, allocations) =
                    measure(|| metadata.rational_root_depends_on_existing(&candidate, 2));
                assert_eq!(answer, expected);
                eprintln!(
                    "leaf rejected base kind={kind}, bits={bits}, control={control:?}, allocations={allocations:?}"
                );
                observations.push(allocations);
            }
        }
        assert!(
            observations.iter().all(|r| *r == control),
            "{observations:?}"
        );
    }
    #[test]
    fn algebraic_leaf_interruption_cannot_append_a_generator() {
        let fixture = || AlgebraicSeparation {
            generators: [2, 3]
                .into_iter()
                .map(|value| AlgebraicGenerator {
                    root: root(Computable::rational(Rational::new(value)), 2),
                    degree: 2,
                })
                .collect(),
            ..empty()
        };
        let candidate = root(Computable::rational(Rational::new(6)), 2);
        let generous = EvaluationBudget::new(100_000);
        {
            let _scope = BudgetScope::enter(&generous);
            let mut metadata = fixture();
            assert_eq!(metadata.insert_generator(candidate.clone(), 2), Some(()));
            assert_eq!(metadata.generators.len(), 2);
        }
        let needed = generous.consumed();
        assert!(needed > 0 && needed < 4096);
        for units in 0..=needed + 2 {
            let mut metadata = fixture();
            let budget = EvaluationBudget::new(units);
            {
                let _scope = BudgetScope::enter(&budget);
                assert_eq!(
                    metadata.insert_generator(candidate.clone(), 2).is_some(),
                    units >= needed
                );
                assert_eq!(evaluation_exhausted(), units < needed);
                assert_eq!(metadata.generators.len(), 2);
            }
            assert_eq!(budget.consumed() + budget.remaining(), units);
            assert_eq!(metadata.insert_generator(candidate.clone(), 2), Some(()));
            assert_eq!(metadata.generators.len(), 2);
        }
        eprintln!("leaf dependency successful work={needed}");
    }

    #[test]
    fn algebraic_leaf_integer_metadata_avoids_coefficient_copies() {
        warm_constants();
        let one = raw(Approximation::One);
        let _ = one.algebraic_separation_bound_bits();
        let (answer, control) = measure(|| one.algebraic_separation_bound_bits());
        assert_eq!(answer, Some(0));
        let mut observations = Vec::new();
        for bits in [16, 4096, 131_073, 131_074, 1_000_001] {
            for negative in [false, true] {
                let value = integer(bits, negative);
                let expected = (bits <= 131_073).then_some(0);
                let (answer, allocations) = measure(|| value.algebraic_separation_bound_bits());
                assert_eq!(answer, expected);
                eprintln!(
                    "leaf integer metadata bits={bits}, negative={negative}, control={control:?}, allocations={allocations:?}"
                );
                observations.push(allocations);
            }
        }
        // At the exact bit-length boundary, only a power of two has an
        // admissible ceil(log2). Preserve the distinction for both leaf forms.
        let high: BigInt = BigInt::one() << 131_072;
        let just_above: BigInt = &high + 1;
        for value in [
            raw(Approximation::Int(just_above.clone())),
            raw(Approximation::Ratio(
                Rational::from_bigint_fraction(BigInt::one(), just_above.magnitude().clone())
                    .unwrap(),
            )),
        ] {
            assert_eq!(value.algebraic_separation_bound_bits(), None);
        }
        assert!(
            observations
                .iter()
                .all(|r| r.bytes <= control.bytes && r.largest <= control.largest),
            "{observations:?}"
        );
    }

    #[test]
    fn algebraic_leaf_metadata_budget_preserves_local_limit_rejection() {
        for ratio in [false, true] {
            let fixture = || {
                if ratio {
                    raw(Approximation::Ratio(
                        Rational::from_bigint_fraction(BigInt::one() << 255, BigUint::from(3_u32))
                            .unwrap(),
                    ))
                } else {
                    integer(256, false)
                }
            };
            let expected = Some(if ratio { 2 } else { 0 });
            let generous = EvaluationBudget::new(100_000);
            {
                let _scope = BudgetScope::enter(&generous);
                assert_eq!(fixture().algebraic_separation_bound_bits(), expected);
            }
            let needed = generous.consumed();
            assert!(
                needed > 1 && needed < 4096,
                "coefficient scans must consume work"
            );
            for units in 0..=needed + 2 {
                let value = fixture();
                let facts = value.internal.facts.snapshot();
                let budget = EvaluationBudget::new(units);
                {
                    let _scope = BudgetScope::enter(&budget);
                    let answer = value.algebraic_separation_bound_bits();
                    assert_eq!(answer.is_some(), units >= needed);
                    assert_eq!(evaluation_exhausted(), units < needed);
                    if answer.is_some() {
                        assert_eq!(answer, expected);
                    }
                    assert_eq!(value.internal.facts.snapshot(), facts);
                    assert!(value.internal.cache_snapshot().is_none());
                }
                assert_eq!(budget.consumed() + budget.remaining(), units);
                assert_eq!(value.algebraic_separation_bound_bits(), expected);
            }
            eprintln!("leaf metadata ratio={ratio}, successful work={needed}");
        }
        for ratio in [false, true] {
            let value = if ratio {
                raw(Approximation::Ratio(
                    Rational::from_bigint_fraction(BigInt::one(), BigUint::one() << 1_000_000)
                        .unwrap(),
                ))
            } else {
                integer(1_000_001, false)
            };
            let budget = EvaluationBudget::new(1);
            let _scope = BudgetScope::enter(&budget);
            assert_eq!(value.algebraic_separation_bound_bits(), None);
            assert!(
                !evaluation_exhausted(),
                "unsupported sizes need no coefficient scan"
            );
            assert_eq!(budget.consumed(), 1);
        }
    }
}
