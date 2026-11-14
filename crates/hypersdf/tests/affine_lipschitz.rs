use hyperlattice::Matrix4;
use hyperlimit::{Point3, PredicatePolicy, compare_reals};
use hyperreal::Real;
use hypersdf::{Sdf, SdfExpr};
use std::cmp::Ordering;

#[test]
fn inverse_affine_expansion_cannot_understate_a_certified_lipschitz_bound() {
    let mut matrix = Matrix4::identity();
    matrix.0[0][0] = (Real::one() / Real::from(2)).unwrap();
    let sdf = Sdf::new(SdfExpr::x().affine_transform(matrix).unwrap());
    let lo = Point3::new(0.into(), 0.into(), 0.into());
    let hi = Point3::new(1.into(), 0.into(), 0.into());
    let report = sdf.lipschitz_cell(&lo, &hi);
    assert!(report.is_certified());
    let delta = sdf.evaluate_point(&hi).unwrap() - sdf.evaluate_point(&lo).unwrap();
    assert_eq!(delta, Real::from(2));
    // World distance is exactly one, so a certified bound must be at least 2.
    assert_ne!(
        compare_reals(
            report.bound.as_ref().unwrap(),
            &delta,
            PredicatePolicy::STRICT
        )
        .value(),
        Some(Ordering::Less),
        "certified affine bound is smaller than the exact difference quotient"
    );
}

fn point(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}
fn ratio(a: i32, b: i32) -> Real {
    (Real::from(a) / Real::from(b)).unwrap()
}
fn bound(expr: SdfExpr) -> Real {
    let sdf = Sdf::new(expr);
    let report = sdf.lipschitz_cell(&point(-2, -2, -2), &point(2, 2, 2));
    assert!(report.is_certified());
    assert!(report.is_self_consistent());
    report.bound.unwrap()
}
fn assert_equal(a: &Real, b: &Real) {
    assert_eq!(
        compare_reals(a, b, PredicatePolicy::STRICT).value(),
        Some(Ordering::Equal)
    );
}
#[test]
fn exact_similarities_axis_scales_and_shears_transport_the_bound() {
    for (scale, want) in [
        (Real::one(), Real::one()),
        (Real::from(-1), Real::one()),
        (Real::from(2), ratio(1, 2)),
        (ratio(1, 2), Real::from(2)),
    ] {
        let mut m = Matrix4::identity();
        for i in 0..3 {
            m.0[i][i] = scale.clone();
        }
        assert_equal(&bound(SdfExpr::x().affine_transform(m).unwrap()), &want);
    }
    let mut m = Matrix4::identity();
    m.0[0][0] = ratio(1, 2);
    m.0[1][1] = ratio(1, 3);
    m.0[2][2] = ratio(1, 4);
    assert_equal(
        &bound(SdfExpr::x().affine_transform(m).unwrap()),
        &Real::from(4),
    );
    let mut shear = Matrix4::identity();
    shear.0[0][1] = 2.into();
    // B=[1,-2,0;0,1,0;0,0,1], B^T B has row sums 3,7,1.
    assert_equal(
        &bound(SdfExpr::x().affine_transform(shear).unwrap()),
        &Real::from(7).sqrt().unwrap(),
    );
    let q = (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap();
    let rotation = Matrix4([
        [q.clone(), -q.clone(), 0.into(), 0.into()],
        [q.clone(), q, 0.into(), 0.into()],
        [0.into(), 0.into(), 1.into(), 0.into()],
        [0.into(), 0.into(), 0.into(), 1.into()],
    ]);
    assert_equal(
        &bound(SdfExpr::x().affine_transform(rotation).unwrap()),
        &Real::one(),
    );
}
#[test]
fn nesting_constants_and_raw_inverse_fields_keep_exact_composition() {
    let mut first = Matrix4::identity();
    first.0[0][0] = ratio(1, 2);
    let mut second = Matrix4::identity();
    second.0[0][0] = ratio(1, 4);
    let expr = SdfExpr::x()
        .affine_transform(first)
        .unwrap()
        .translate(point(1, 2, 3))
        .affine_transform(second.clone())
        .unwrap();
    assert_equal(&bound(expr), &Real::from(8));
    assert_eq!(
        bound(
            SdfExpr::constant(9.into())
                .affine_transform(second)
                .unwrap()
        ),
        Real::zero()
    );
    let mut inverse = Matrix4::identity();
    for r in 0..3 {
        for c in 0..3 {
            inverse.0[r][c] = Real::zero();
        }
        inverse.0[r][3] = Real::from(r as i32 + 1);
    }
    let raw = SdfExpr::Transform {
        child: Box::new(SdfExpr::x()),
        transform: hypersdf::SdfTransform::Affine {
            matrix: Matrix4::identity(),
            inverse,
        },
    };
    assert_eq!(bound(raw), Real::zero());
}
#[test]
fn transformed_point_pair_inequalities_use_exact_independent_world_distances() {
    let mut m = Matrix4::identity();
    m.0[0][0] = ratio(1, 2);
    m.0[0][1] = 2.into();
    m.0[1][1] = ratio(1, 3);
    m.0[2][2] = (-1).into();
    for expr in [
        SdfExpr::x(),
        SdfExpr::linear(
            hyperlattice::Vector3([1.into(), 2.into(), (-1).into()]),
            3.into(),
        ),
        SdfExpr::sphere(point(1, -1, 0), 4.into()),
    ] {
        let sdf = Sdf::new(expr.affine_transform(m.clone()).unwrap());
        let report = sdf.lipschitz_cell(&point(-2, -2, -2), &point(2, 2, 2));
        assert!(report.is_certified());
        let l = report.bound.unwrap();
        let l2 = &l * &l;
        for x in -1..=1 {
            for y in -1..=1 {
                for z in -1..=1 {
                    for [dx, dy, dz] in [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, -1, 1]] {
                        let a = point(x, y, z);
                        let b = point(x + dx, y + dy, z + dz);
                        let difference =
                            sdf.evaluate_point(&b).unwrap() - sdf.evaluate_point(&a).unwrap();
                        let lhs = &difference * &difference;
                        let rhs = &l2 * &Real::from(dx * dx + dy * dy + dz * dz);
                        assert!(matches!(
                            compare_reals(&lhs, &rhs, PredicatePolicy::STRICT).value(),
                            Some(Ordering::Less | Ordering::Equal)
                        ));
                    }
                }
            }
        }
    }
}
#[path = "support/uncertainty.rs"]
mod uncertainty;
#[test]
fn undecided_norm_order_uses_an_independent_frobenius_certificate() {
    let zero = uncertainty::terminal_zero(Real::zero());
    let mut inverse = Matrix4::identity();
    inverse.0[0][1] = zero;
    let raw = SdfExpr::Transform {
        child: Box::new(SdfExpr::x()),
        transform: hypersdf::SdfTransform::Affine {
            matrix: Matrix4::identity(),
            inverse,
        },
    };
    let report = Sdf::new(raw).lipschitz_cell(&point(0, 0, 0), &point(0, 0, 1));
    assert!(report.is_certified());
    // Exact source identity makes B the identity. The unresolved row ordering
    // may use sqrt(trace(B^T B)) instead; it must not consume an approximate max.
    let l = report.bound.unwrap();
    assert_eq!(
        compare_reals(&l, &Real::one(), PredicatePolicy::STRICT).value(),
        Some(Ordering::Greater)
    );
    assert_eq!(
        compare_reals(&l, &Real::from(2), PredicatePolicy::STRICT).value(),
        Some(Ordering::Less)
    );
}
#[test]
fn child_unknowns_are_not_repaired_by_a_transform_norm() {
    for expr in [
        SdfExpr::x().tan(),
        SdfExpr::sphere(point(0, 0, 0), (-1).into()),
    ] {
        let report = Sdf::new(expr.affine_transform(Matrix4::identity()).unwrap())
            .lipschitz_cell(&point(0, 0, 0), &point(1, 1, 1));
        assert!(report.bound.is_none());
        assert!(!report.is_certified());
        assert!(report.is_self_consistent());
    }
}
#[cfg(feature = "dispatch-trace")]
#[test]
fn affine_bound_predicate_counts_preserve_old_work_before_transport() {
    use hyperreal::dispatch_trace as trace;
    let mut m = Matrix4::identity();
    m.0[0][0] = ratio(1, 2);
    for (expr, count) in [
        (SdfExpr::x().affine_transform(m.clone()).unwrap(), 20),
        (
            SdfExpr::constant(9.into())
                .affine_transform(m.clone())
                .unwrap(),
            18,
        ),
        (SdfExpr::x().tan().affine_transform(m).unwrap(), 18),
        (SdfExpr::x().translate(point(1, 2, 3)), 0),
    ] {
        let sdf = Sdf::new(expr);
        let lo = point(0, 0, 0);
        let hi = point(1, 1, 1);
        trace::reset();
        let _report = trace::with_recording(|| sdf.lipschitz_cell(&lo, &hi));
        let stats = trace::take_trace();
        assert_eq!(
            stats.path_count("hyperlimit", "compare_reals", "exact-rational"),
            count
        );
    }
}
#[test]
fn fresh_process_recomputes_the_correct_affine_certificate() {
    const KEY: &str = "HYPERSDF_AFFINE_LIPSCHITZ_REPLAY";
    if std::env::var(KEY).is_ok() {
        inverse_affine_expansion_cannot_understate_a_certified_lipschitz_bound();
        return;
    }
    let status = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "fresh_process_recomputes_the_correct_affine_certificate",
            "--nocapture",
        ])
        .env(KEY, "1")
        .status()
        .unwrap();
    assert!(status.success());
}
