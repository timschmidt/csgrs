use hyperlattice::{Complex, Matrix2, Matrix3, Matrix4, Vector2, Vector3, Vector4};
use hyperreal::Real;
use std::sync::{Arc, atomic::AtomicBool};

fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

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
            Matrix2::new([[ordinary, tiny], [Real::one(), Real::zero()]])
        ),
        "[[0.125, 1e-40], [1, 0e0]]"
    );
}

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
                        Matrix2::new(std::array::from_fn(|_| std::array::from_fn(
                            |_| value.clone()
                        )))
                    ),
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
