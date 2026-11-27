//! Allocation-request observation across completed FOC paths and selected typed failures.
//!
//! The shared process-global counter records calls and requested bytes, not peak
//! memory, stack use or arbitrary workloads. This binary has one measured test;
//! its success is a finite observation, not a firmware resource bound.
use interval_foc_conformance::samples::{self, ACTIONS};
#[path = "../../allocations.rs"]
mod allocations;

#[test]
/// Include fixture/rotation construction, overflow and balance failures, then all five actions at 64 indices.
/// Compare every native/shared successful wrapper before digesting it. The allocation
/// scope includes comparisons and hashing; final digest/counter assertions occur after
/// recording stops. A matching checksum supplements rather than replaces full equality.
fn both_complete_transform_clients_allocate_nothing() {
    let ((native, shared), calls, bytes) = allocations::measure(|| {
        use alumina_foc::{
            AlphaBeta, DqInterval, FocError, Phase3, Q30, Q30Interval as Q, Rotation,
        };
        use interval_foc_conformance as kernel;
        let minimum = Q::point(Q30::from_bits(i32::MIN));
        let maximum = Q::point(Q30::from_bits(i32::MAX));
        let rotation = Rotation::new(Q::point(Q30::NEG_ONE), Q::ZERO, Q30::ZERO).unwrap();
        let value = AlphaBeta {
            alpha: minimum,
            beta: Q::ZERO,
        };
        assert_eq!(kernel::park(value, rotation), Err(FocError::Range));
        assert_eq!(
            kernel::park(value, rotation),
            alumina_foc::park(value, rotation)
        );
        let value = DqInterval {
            d: Q::ZERO,
            q: minimum,
        };
        assert_eq!(kernel::inverse_park(value, rotation), Err(FocError::Range));
        assert_eq!(
            kernel::inverse_park(value, rotation),
            alumina_foc::inverse_park(value, rotation)
        );
        let value = AlphaBeta {
            alpha: minimum,
            beta: maximum,
        };
        assert_eq!(kernel::inverse_clarke(value), Err(FocError::Range));
        assert_eq!(
            kernel::inverse_clarke(value),
            alumina_foc::inverse_clarke(value)
        );
        let value = Phase3 {
            a: Q::ZERO,
            b: maximum,
            c: minimum,
        };
        assert_eq!(
            kernel::clarke(value, Q30::from_bits(1)),
            Err(FocError::Range)
        );
        assert_eq!(
            kernel::clarke(value, Q30::from_bits(1)),
            alumina_foc::clarke(value, Q30::from_bits(1))
        );
        assert_eq!(
            kernel::clarke(value, Q30::from_bits(-1)),
            Err(FocError::UnbalancedPhases)
        );
        let mut native = 0xcbf29ce484222325;
        let mut shared = native;
        for index in 0..64 {
            for action in ACTIONS {
                let input = samples::sample(index);
                let a = samples::native(action, input).unwrap();
                let b = samples::shared(action, input).unwrap();
                assert_eq!(a, b);
                native = a.digest(native);
                shared = b.digest(shared);
            }
        }
        (native, shared)
    });
    assert_eq!(native, shared);
    assert_eq!((calls, bytes), (0, 0));
}
