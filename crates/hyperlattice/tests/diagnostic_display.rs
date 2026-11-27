//! Diagnostic display regression fixtures across complex, vector, and matrix carriers.
//!
//! These tests inspect strings and marker counts, not exact interchange or numerical
//! error bounds. Existing scalar proof/cache state may be shared across clones.

use hyperlattice::{Complex, Matrix3, Matrix4, Vector2, Vector3, Vector4};
use hyperreal::Real;
use std::sync::{Arc, atomic::AtomicBool};

/// Construct sin(e)^2+cos(e)^2-1, mathematically zero but opaque to the intended cheap zero path; this helper itself does not assert its knowledge status.
fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

/// Check exact expected alternate strings for rational 1/8, tiny 10^-40, and explicit matrix zeros in Complex, Vector2, and Matrix3.
#[test]
fn aggregate_alternate_display_retains_ordinary_numeric_notation() {
    let ordinary: Real = "1/8".parse().unwrap();
    let tiny: Real = "1e-40".parse().unwrap();
    assert_eq!(
        format!("{:#}", Complex::new(ordinary.clone(), tiny.clone())),
        "(0.125 + 1e-40i)"
    );
    assert_eq!(
        format!("{:#}", Vector2::new([ordinary.clone(), tiny.clone()])),
        "[0.125, 1e-40]"
    );
    assert_eq!(
        format!(
            "{:#}",
            Matrix3::new([
                [ordinary, tiny, Real::zero()],
                [Real::one(), Real::zero(), Real::one()],
                [Real::zero(), Real::zero(), Real::one()],
            ])
        ),
        "[[0.125, 1e-40, 0e0], [1, 0e0, 1], [0e0, 0e0, 1]]"
    );
}

/// Check two diagnostic failure markers in Complex, Vector2/3/4, and Matrix3/4.
///
/// For each scalar, first format clones, then attempt a -512 interval and format again.
/// This is sequential shared-cache exercise, not independent cold/warm fixtures; the
/// preparation result is discarded. Count one marker per component, including 9/16
/// matrix entries. No timeout, full-string assertion, or numeric error bound is added.
#[test]
fn every_aggregate_family_exposes_unknown_and_aborted_components() {
    let mut aborted = Real::pi();
    aborted.abort(Arc::new(AtomicBool::new(true)));
    for (value, marker) in [
        (unresolved_zero(), "[UnknownZero]"),
        (aborted, "[Exhausted]"),
    ] {
        for prepared in [false, true] {
            if prepared {
                let _ = value.certified_rational_interval(-512);
            }
            let outputs = [
                (
                    format!("{:#}", Complex::new(value.clone(), value.clone())),
                    2,
                ),
                (
                    format!("{:#}", Vector2::new(std::array::from_fn(|_| value.clone()))),
                    2,
                ),
                (
                    format!("{:#}", Vector3::new(std::array::from_fn(|_| value.clone()))),
                    3,
                ),
                (
                    format!("{:#}", Vector4::new(std::array::from_fn(|_| value.clone()))),
                    4,
                ),
                (
                    format!(
                        "{:#}",
                        Matrix3::new(std::array::from_fn(|_| std::array::from_fn(
                            |_| value.clone()
                        )))
                    ),
                    9,
                ),
                (
                    format!(
                        "{:#}",
                        Matrix4::new(std::array::from_fn(|_| std::array::from_fn(
                            |_| value.clone()
                        )))
                    ),
                    16,
                ),
            ];
            for (text, count) in outputs {
                assert_eq!(text.matches(marker).count(), count, "{text}");
            }
        }
    }
}
