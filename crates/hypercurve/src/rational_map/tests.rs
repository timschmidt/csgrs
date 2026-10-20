use super::*;
use crate::{CurvePointDefinition2, PolynomialCurveSet2};
use hypersolve::AlgebraicFiberRootIsolationConfig;
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
};
use std::cmp::Ordering;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn poly(rows: Vec<Vec<Real>>) -> BivariatePolynomial {
    BivariatePolynomial::new(rows)
}
fn set() -> PolynomialCurveSet2 {
    let equation = poly(vec![
        vec![q(7, 16), (-1).into(), 1.into()],
        vec![(-1).into()],
        vec![1.into()],
    ]);
    let domain = BivariateCurveDomain {
        first: [0.into(), 1.into()],
        second: [0.into(), 1.into()],
    };
    PolynomialCurveSet2::from_topology(
        decompose_bivariate_curve(&equation, &domain, BivariateTopologyConfig::default()).unwrap(),
        AlgebraicFiberRootIsolationConfig::default(),
    )
    .unwrap()
}
fn shear() -> RationalMap2 {
    RationalMap2::new(
        [
            poly(vec![vec![1.into(), 2.into()], vec![1.into()]]),
            poly(vec![vec![0.into(), 1.into()]]),
        ],
        poly(vec![vec![1.into()]]),
    )
}

#[test]
fn joint_images_keep_rational_queries_and_original_guards() {
    let set = set();
    for curve in set.curves() {
        let source = curve.point_at(&q(1, 2).into()).unwrap();
        let image = source.rational_image(&shear()).unwrap();
        assert!(matches!(
            image.definition().unwrap(),
            CurvePointDefinition2::RationalImage { .. }
        ));
        // X - 2Y - 1 is the original u, in its original joint field.
        assert_eq!(
            image
                .affine_form(1.into(), (-2).into(), (-1).into())
                .compare_to_real(&q(1, 2))
                .unwrap(),
            Ordering::Equal
        );
        let replay = image.definition().unwrap().into_point().unwrap();
        assert_eq!(
            image.same_point(&replay, &CurveContext::STRICT),
            crate::Classification::Decided(true)
        );
        let negative = RationalMap2::new(
            [
                poly(vec![vec![0.into()], vec![(-1).into()]]),
                poly(vec![vec![0.into(), (-1).into()]]),
            ],
            poly(vec![vec![(-1).into()]]),
        );
        let image = source.rational_image(&negative).unwrap();
        assert_eq!(
            image.same_point(&source, &CurveContext::STRICT),
            crate::Classification::Decided(true)
        );
        let pole = poly(vec![vec![q(-1, 2)], vec![1.into()]]);
        let removable = RationalMap2::new([pole.clone(), pole.clone()], pole);
        assert!(source.rational_image(&removable).is_err());
        assert!(curve.rational_image(&removable).is_err());
        source.rational_image(&shear()).unwrap();
    }
}

#[test]
fn whole_maps_restrict_reverse_transform_and_replay_without_chart_loss() {
    let set = set();
    let reflection = Similarity2::try_from_real_affine(
        (-1).into(),
        0.into(),
        0.into(),
        1.into(),
        2.into(),
        0.into(),
    )
    .unwrap();
    for source in set.curves() {
        let mapped = source.rational_image(&shear()).unwrap();
        assert!(
            !mapped
                .polynomial_branch()
                .unwrap()
                .has_same_parameterization(source.polynomial_branch().unwrap())
        );
        for curve in [
            mapped.clone(),
            mapped.reversed().unwrap(),
            mapped
                .subcurve(source.parameter_domain().start().clone(), q(1, 2).into())
                .unwrap(),
        ] {
            assert!(
                curve
                    .polynomial_branch()
                    .unwrap()
                    .shares_source(source.polynomial_branch().unwrap())
            );
            assert!(
                curve
                    .polynomial_branch()
                    .unwrap()
                    .has_same_parameterization(mapped.polynomial_branch().unwrap())
            );
            assert_eq!(
                curve
                    .point_at(&q(1, 2).into())
                    .unwrap()
                    .affine_form(1.into(), (-2).into(), (-1).into())
                    .compare_to_real(&q(1, 2))
                    .unwrap(),
                Ordering::Equal
            );
            curve.bounds().unwrap();
        }
        let after = mapped.transform_similarity(&reflection).unwrap();
        let before = source
            .transform_similarity(&reflection)
            .unwrap()
            .rational_image(&shear())
            .unwrap();
        let after = after.point_at(&q(1, 2).into()).unwrap();
        let before = before.point_at(&q(1, 2).into()).unwrap();
        assert_eq!(
            after
                .affine_form(1.into(), 2.into(), (-1).into())
                .compare_to_real(&q(-1, 2))
                .unwrap(),
            Ordering::Equal
        );
        assert_eq!(
            before
                .affine_form(1.into(), (-2).into(), (-1).into())
                .compare_to_real(&q(3, 2))
                .unwrap(),
            Ordering::Equal
        );
        for point in [before, after] {
            let restored = point.definition().unwrap().into_point().unwrap();
            assert_eq!(
                point.same_point(&restored, &CurveContext::STRICT),
                crate::Classification::Decided(true)
            );
            point
                .affine_form(1.into(), 0.into(), 0.into())
                .compare_to_real(&0.into())
                .unwrap();
        }
        assert_eq!(mapped.family(), CurveFamily2::ImplicitPolynomial);
        assert_eq!(
            source
                .point_at(&q(1, 2).into())
                .unwrap()
                .affine_form(1.into(), 0.into(), 0.into())
                .compare_to_real(&q(1, 2))
                .unwrap(),
            Ordering::Equal
        );
    }
}

#[test]
fn injectivity_requires_a_whole_rectangle_certificate() {
    let domain = Aabb2::new_unchecked(Point2::from_values(0, 0), Point2::from_values(1, 1));
    shear().certify_injective_on(&domain).unwrap();
    let reflection = RationalMap2::new(
        [
            poly(vec![vec![0.into()], vec![(-1).into()]]),
            poly(vec![vec![0.into(), 1.into()]]),
        ],
        poly(vec![vec![1.into()]]),
    );
    reflection.certify_injective_on(&domain).unwrap();
    let folded = RationalMap2::new(
        [
            poly(vec![vec![q(1, 4)], vec![(-1).into()], vec![1.into()]]),
            poly(vec![vec![0.into(), 1.into()]]),
        ],
        poly(vec![vec![1.into()]]),
    );
    assert!(folded.certify_injective_on(&domain).is_err());
    let flat = RationalMap2::new(
        [
            poly(vec![vec![0.into()], vec![1.into()]]),
            poly(vec![vec![0.into()]]),
        ],
        poly(vec![vec![1.into()]]),
    );
    assert!(flat.certify_injective_on(&domain).is_err());
}

#[test]
fn whole_image_identity_uses_the_retained_equation_and_rejects_perturbation() {
    let set = set();
    let equation = set.curves()[0]
        .polynomial_branch()
        .unwrap()
        .topology()
        .equation();
    let x = poly(vec![Vec::new(), vec![1.into()]]);
    let y = poly(vec![vec![0.into(), 1.into()]]);
    let weight = poly(vec![vec![1.into()], vec![1.into()]]);
    let numerators = [x, y];
    let original = RationalMap2::new(numerators.clone(), weight.clone());
    let equivalent = RationalMap2::new(
        numerators.clone().map(|p| bivariate_add(&p, equation)),
        weight.clone(),
    );
    let perturbed = RationalMap2::new(
        [
            bivariate_add(&numerators[0], &poly(vec![vec![q(1, 1_000_000)]])),
            numerators[1].clone(),
        ],
        weight,
    );
    for source in set.curves() {
        let a = source.rational_image(&original).unwrap();
        let b = source.rational_image(&equivalent).unwrap();
        let c = source.rational_image(&perturbed).unwrap();
        let agrees = |a: &crate::Curve2, b: &crate::Curve2| {
            a.polynomial_branch()
                .unwrap()
                .has_same_parameterization(b.polynomial_branch().unwrap())
        };
        assert!(agrees(&a, &b));
        assert!(!agrees(&a, &c));
        let reflection = Similarity2::try_from_real_affine(
            (-1).into(),
            0.into(),
            0.into(),
            1.into(),
            2.into(),
            0.into(),
        )
        .unwrap();
        assert!(agrees(
            &a.transform_similarity(&reflection).unwrap(),
            &b.transform_similarity(&reflection).unwrap(),
        ));
    }
    assert!(
        !set.curves()[0]
            .polynomial_branch()
            .unwrap()
            .has_same_parameterization(set.curves()[1].polynomial_branch().unwrap(),)
    );
}

#[test]
fn repeated_projective_images_preserve_joint_equations_without_degree_inflation() {
    let set = set();
    let map = RationalMap2::new(
        [
            poly(vec![vec![0.into()], vec![1.into()]]),
            poly(vec![vec![0.into(), 1.into()]]),
        ],
        poly(vec![vec![1.into()], vec![1.into()]]),
    );
    for (index, source) in set.curves().iter().enumerate() {
        let mut point = source.point_at(&q(1, 2).into()).unwrap();
        for n in 1..=16 {
            point = point.rational_image(&map).unwrap();
            assert_eq!(
                point
                    .affine_form(1.into(), 0.into(), 0.into())
                    .compare_to_real(&q(1, n + 2))
                    .unwrap(),
                Ordering::Equal
            );
            let expected = q(if index == 0 { 1 } else { 3 }, 2 * (n + 2));
            assert_eq!(
                point
                    .affine_form(0.into(), 1.into(), 0.into())
                    .compare_to_real(&expected)
                    .unwrap(),
                Ordering::Equal
            );
        }
        // Inverse u=X/(1-16X), v=Y/(1-16X) in the circle equation.
        let inverse = RationalMap2::new(
            [
                poly(vec![vec![0.into()], vec![1.into()]]),
                poly(vec![vec![0.into(), 1.into()]]),
            ],
            poly(vec![vec![1.into()], vec![(-16).into()]]),
        );
        let restored = point.rational_image(&inverse).unwrap();
        assert_eq!(
            restored.same_point(
                &source.point_at(&q(1, 2).into()).unwrap(),
                &CurveContext::STRICT
            ),
            crate::Classification::Decided(true)
        );
        let equation = poly(vec![
            vec![q(7, 16), (-1).into(), 1.into()],
            vec![(-1).into()],
            vec![1.into()],
        ]);
        assert_eq!(
            restored
                .rational_form(equation, poly(vec![vec![1.into()]]))
                .unwrap()
                .compare_to_real(&0.into())
                .unwrap(),
            Ordering::Equal
        );
    }
}
