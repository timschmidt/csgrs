use std::cmp::Ordering;

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, CurveCertainty,
};
use hyperreal::Real;

fn root() -> BezierParameter2 {
    BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(
                [-1, -2, 0, 0, 0, 32].map(Real::from).to_vec(),
            )
            .unwrap(),
            BezierParameterInterval::try_new(Real::zero(), Real::one()).unwrap(),
        )
        .unwrap(),
    )
}

#[test]
fn affine_charts_retain_selected_root_identity_and_orientation() {
    let original = root();
    assert!(original.scalar().is_none());
    let mut replayed = original.clone();
    for _ in 0..64 {
        let mapped = replayed
            .affine_image(&Real::from(-4), &Real::from(9))
            .unwrap();
        assert!(mapped.scalar().is_none());
        assert_eq!(
            mapped
                .cmp_by_refinement(&BezierParameter2::Exact(Real::from(6)))
                .unwrap(),
            Ordering::Greater
        );
        assert_eq!(
            mapped
                .cmp_by_refinement(&BezierParameter2::Exact(Real::from(7)))
                .unwrap(),
            Ordering::Less
        );
        replayed = mapped
            .affine_image(
                &(Real::from(-1) / Real::from(4)).unwrap(),
                &(Real::from(9) / Real::from(4)).unwrap(),
            )
            .unwrap();
        assert_eq!(
            original.cmp_by_refinement(&replayed).unwrap(),
            Ordering::Equal
        );
        assert!(replayed.scalar().is_none());
    }
}

#[test]
fn affine_charts_require_certified_nonzero_scale_even_in_a_provisional_scope() {
    let parameter = root();
    assert!(parameter.affine_image(&Real::zero(), &Real::one()).is_err());
    let tiny: Real = "1e-2000".parse().unwrap();
    let scale =
        (Real::pi() + Real::from(2) * &tiny).exp().unwrap() - (Real::pi() + tiny).exp().unwrap();
    assert!(parameter.affine_image(&scale, &Real::one()).is_err());
    let attempted = hypercurve::provisional(|| parameter.affine_image(&scale, &Real::one()));
    assert_eq!(attempted.certainty(), CurveCertainty::Certified);
    assert!(attempted.into_unverified().is_err());
}

#[test]
fn affine_charts_preserve_an_already_learned_scalar_witness() {
    let root = BezierAlgebraicParameter2::try_isolate(
        BezierParameterPolynomial::try_new_power_basis(vec![Real::from(-1), Real::from(3)])
            .unwrap(),
        BezierParameterInterval::try_new(Real::zero(), Real::one()).unwrap(),
    )
    .unwrap();
    assert_eq!(
        root.represented_exact_point().unwrap(),
        Some((Real::one() / Real::from(3)).unwrap())
    );
    let mapped = BezierParameter2::Algebraic(root)
        .affine_image(&Real::from(3), &Real::from(2))
        .unwrap();
    assert_eq!(mapped.scalar(), Some(&Real::from(3)));
}

#[test]
fn projective_source_evidence_replays_without_refinement_history() {
    let original = root();
    let mapped = original
        .projective_image(&[Real::one(), Real::one()], &[Real::from(2), Real::one()])
        .unwrap()
        .affine_image(&Real::from(-3), &Real::from(5))
        .unwrap();
    let BezierParameter2::Algebraic(mapped_root) = &mapped else {
        panic!("selected image");
    };
    let (source, numerator, denominator) = mapped_root.source_chart().unwrap();
    assert!(source.source_chart().is_none());
    let replayed_source = BezierAlgebraicParameter2::try_isolate(
        BezierParameterPolynomial::try_new_power_basis(source.polynomial().coefficients().to_vec())
            .unwrap(),
        BezierParameterInterval::try_new(
            source.interval().start().clone(),
            source.interval().end().clone(),
        )
        .unwrap(),
    )
    .unwrap();
    let replayed = BezierParameter2::Algebraic(replayed_source)
        .projective_image(numerator, denominator)
        .unwrap();
    assert_eq!(
        mapped.cmp_by_refinement(&replayed).unwrap(),
        Ordering::Equal
    );
    assert!(mapped.scalar().is_none());
    assert!(replayed.scalar().is_none());
    let pole = hypercurve::provisional(|| {
        original.projective_image(&[Real::zero(), Real::one()], &[Real::zero(), Real::zero()])
    });
    assert_eq!(pole.certainty(), CurveCertainty::Certified);
    assert!(pole.into_unverified().is_err());
}

#[test]
fn authored_restrictions_preserve_scalar_charts_and_reversed_traversal() {
    use hypercurve::{Curve2, CurveParameter2, CurveParameterRange2, LineSeg2, Point2};
    let source = Curve2::from(
        LineSeg2::try_new(
            Point2::new(Real::zero(), Real::zero()),
            Point2::new(Real::from(4), Real::zero()),
        )
        .unwrap(),
    );
    let quarter = (Real::one() / Real::from(4)).unwrap();
    let half = (Real::one() / Real::from(2)).unwrap();
    let restricted = source
        .restrict_source(
            CurveParameterRange2::try_new(
                CurveParameter2::from(quarter.clone()),
                CurveParameter2::from(half.clone()),
            )
            .unwrap(),
        )
        .unwrap();
    assert!(restricted.geometry().is_none());
    assert_eq!(
        restricted
            .parameter_domain()
            .start()
            .compare(&quarter.into())
            .unwrap(),
        Ordering::Equal
    );
    let reversed = restricted.reversed().unwrap();
    assert_eq!(
        reversed
            .parameter_endpoints()
            .0
            .compare(&half.into())
            .unwrap(),
        Ordering::Equal
    );
    let (retained, range, direction) = reversed.source_restriction().unwrap();
    assert_eq!(retained, &source);
    assert!(direction);
    let restored = retained
        .restrict_source(range.clone())
        .unwrap()
        .reversed()
        .unwrap();
    assert!(restored.start().coincides_with(&reversed.start()).unwrap());
    assert!(restored.end().coincides_with(&reversed.end()).unwrap());
}
