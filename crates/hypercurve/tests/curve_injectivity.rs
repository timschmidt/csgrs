use hypercurve::{Curve2, CurveCertainty, CurveParameterRange2, Point2};
use hyperreal::Real;

fn point(x: i32, y: i32) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

#[test]
fn connected_weighted_nurbs_spans_share_a_strict_monotone_coordinate() {
    let curve = Curve2::try_nurbs(
        2,
        vec![
            point(0, 0),
            point(1, -1),
            point(2, 0),
            point(3, -1),
            point(4, 0),
        ],
        [1, 2, 1, 2, 1].map(Real::from).to_vec(),
        [2, 2, 2, 3, 3, 5, 5, 5].map(Real::from).to_vec(),
    )
    .unwrap();
    assert_eq!(curve.native_bezier_fragments().unwrap().len(), 2);
    let restriction = curve
        .restrict_source(
            CurveParameterRange2::try_new(Real::from(2).into(), Real::from(4).into()).unwrap(),
        )
        .unwrap();
    assert!(restriction.has_certified_injective_image().unwrap());
    assert!(
        restriction
            .reversed()
            .unwrap()
            .has_certified_injective_image()
            .unwrap()
    );
    for curve in [curve.clone(), curve.reversed().unwrap()] {
        for _ in 0..2 {
            assert!(curve.has_certified_injective_image().unwrap());
        }
    }
}

#[test]
fn individually_injective_spans_do_not_certify_backtracking_or_disconnected_images() {
    for (points, knots) in [
        (
            vec![point(0, 0), point(1, 0), point(0, 0)],
            vec![0, 0, 1, 2, 2],
        ),
        (
            vec![point(0, 0), point(1, 0), point(1, 1), point(2, 1)],
            vec![0, 0, 1, 1, 2, 2],
        ),
        (
            vec![point(0, 0), point(1, 0), point(1, 1)],
            vec![0, 0, 1, 2, 2],
        ),
    ] {
        let curve = Curve2::try_nurbs(
            1,
            points.clone(),
            vec![Real::one(); points.len()],
            knots.into_iter().map(Real::from).collect(),
        )
        .unwrap();
        for span in curve.native_bezier_fragments().unwrap() {
            assert!(span.has_certified_injective_axis().unwrap());
        }
        assert!(!curve.has_certified_injective_image().unwrap());
        assert!(
            !curve
                .reversed()
                .unwrap()
                .has_certified_injective_image()
                .unwrap()
        );
    }
}

#[test]
fn provisional_near_joins_do_not_author_an_injectivity_certificate() {
    let tiny: Real = "1e-2000".parse().unwrap();
    let gap = (Real::pi() + &tiny + &tiny).exp().unwrap() - (Real::pi() + tiny).exp().unwrap();
    let curve = Curve2::try_nurbs(
        1,
        vec![
            point(0, 0),
            point(1, 0),
            Point2::new(Real::one(), gap),
            point(2, 1),
        ],
        vec![Real::one(); 4],
        [0, 0, 1, 1, 2, 2].map(Real::from).to_vec(),
    )
    .unwrap();
    assert!(!curve.has_certified_injective_image().unwrap());
    let provisional = hypercurve::provisional(|| curve.has_certified_injective_image());
    assert_eq!(provisional.certainty(), CurveCertainty::Certified);
    assert!(!provisional.into_unverified().unwrap());
}
