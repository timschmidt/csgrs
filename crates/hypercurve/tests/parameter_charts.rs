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
