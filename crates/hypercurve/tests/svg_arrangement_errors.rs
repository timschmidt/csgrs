#![cfg(feature = "svg")]
use hypercurve::*;
use hypersolve::work_budget::{WorkBudget, with_budget};
fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
#[test]
fn svg_region_arrangement_preserves_typed_exhaustion_and_retry() {
    let text = "<svg xmlns='http://www.w3.org/2000/svg'><path d='M0 0 C0 3 3 -3 3 0 Z'/></svg>";
    for limit in [0, 1] {
        let denied = with_budget(
            WorkBudget {
                refinements: limit,
                promotions: 4096,
            },
            || {
                let error = import_svg_document(text).unwrap_err();
                assert!(
                    matches!(error, SvgError::Exact(ExactCurveError::Blocked(ref blocker)) if blocker.reason()==UncertaintyReason::BudgetExhausted),
                    "typed region arrangement error: {error}"
                );
            },
        );
        let reached = denied.result.unwrap_err();
        assert_eq!(reached.limit, limit);
        assert_eq!(reached.used, limit);
        assert!(reached.requested > 0);
    }
    let retry = with_budget(
        WorkBudget {
            refinements: 16,
            promotions: 4096,
        },
        || import_svg_document(text),
    );
    let geometry = retry.result.unwrap().unwrap();
    // The cubic has x(1/4)=15/32, y(1/4)=27/32 and strictly
    // increasing x in (0,1); halfway to the closing chord lies inside.
    let inside = Point2::new(q(15, 32), q(27, 64)).into();
    assert_eq!(
        geometry.region().classify_point(&inside).unwrap(),
        RegionPointLocation::Inside
    );
    let replay = import_svg_document(&export_svg_document(&geometry).unwrap()).unwrap();
    assert_eq!(
        replay.region().classify_point(&inside).unwrap(),
        RegionPointLocation::Inside
    );
}
#[test]
fn identity_similarity_endpoint_retains_geometric_svg_closure() {
    let start: CurvePoint2 = Point2::from_values(0, 0).into();
    let end = CurvePointDefinition2::Similarity {
        source: start.clone(),
        transform: Similarity2::try_from_real_affine(
            1.into(),
            0.into(),
            0.into(),
            1.into(),
            0.into(),
            0.into(),
        )
        .unwrap(),
    }
    .into_point()
    .unwrap();
    assert_ne!(start, end);
    assert!(start.coincides_with(&end).unwrap());
    assert!(matches!(
        end.definition().unwrap(),
        CurvePointDefinition2::Similarity { .. }
    ));
    let a: CurvePoint2 = Point2::from_values(1, 0).into();
    let b: CurvePoint2 = Point2::from_values(0, 1).into();
    let path = CurvePath2::try_new(vec![
        Curve2::try_line(start, a.clone()).unwrap(),
        Curve2::try_line(a, b.clone()).unwrap(),
        Curve2::try_line(b, end).unwrap(),
    ])
    .unwrap();
    let region = CurveRegion2::try_from_boundary_paths(&[path], FillRule::NonZero).unwrap();
    let geometry = SvgGeometry2::new(region.clone(), vec![], vec![]);
    let text = export_svg_document(&geometry).unwrap();
    let replay = import_svg_document(&text).unwrap();
    assert_eq!(
        replay
            .region()
            .classify_point(&Point2::new(q(1, 4), q(1, 4)).into())
            .unwrap(),
        RegionPointLocation::Inside
    );
    assert!(
        replay
            .region()
            .boolean_region(&region, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
}
