mod support;
use hypercurve::{
    BulgeVertex2, Contour2, CurveError, CurveString2, FillRule, Point2,
    PolylineReconstructionOptions, Real, Segment2,
};

fn r(value: f64) -> Real {
    Real::try_from(value).unwrap()
}

fn p(x: f64, y: f64) -> Point2 {
    Point2::new(r(x), r(y))
}

#[test]
fn reconstruction_merges_collinear_polyline_to_one_line() {
    let points = [p(0.0, 0.0), p(1.0, 0.0), p(2.0, 0.0), p(3.0, 0.0)];

    let curve =
        CurveString2::reconstruct_from_polyline(&points, PolylineReconstructionOptions::default())
            .unwrap();

    assert_eq!(curve.len(), 1);
    let Segment2::Line(line) = &curve.segments()[0] else {
        panic!("collinear samples should reconstruct as one line");
    };
    assert_eq!(line.start(), &points[0]);
    assert_eq!(line.end(), &points[3]);
}
#[test]
fn reconstruction_keeps_single_corner_as_two_lines_by_default() {
    let points = [p(0.0, 0.0), p(1.0, 0.0), p(1.0, 1.0)];

    let curve =
        CurveString2::reconstruct_from_polyline(&points, PolylineReconstructionOptions::default())
            .unwrap();

    assert_eq!(curve.len(), 2);
    assert!(
        curve
            .segments()
            .iter()
            .all(|segment| matches!(segment, Segment2::Line(_)))
    );
}
#[test]
fn reconstruction_splits_arcs_at_semicircle_boundary() {
    let points = [
        p(1.0, 0.0),
        p(0.0, -1.0),
        p(-1.0, 0.0),
        p(0.0, 1.0),
        p(1.0, 0.0),
    ];
    let options = PolylineReconstructionOptions {
        min_arc_points: 3,
        ..PolylineReconstructionOptions::default()
    };

    let curve = CurveString2::reconstruct_from_polyline(&points, options).unwrap();

    assert_eq!(curve.len(), 2);
    assert!(
        curve
            .segments()
            .iter()
            .all(|segment| matches!(segment, Segment2::Arc(_)))
    );
}

#[test]
fn reconstruction_accepts_closed_polyline_without_repeated_first_point() {
    let points = [p(0.0, 0.0), p(4.0, 0.0), p(4.0, 3.0), p(0.0, 3.0)];

    let contour = Contour2::reconstruct_from_closed_polyline(
        &points,
        PolylineReconstructionOptions::default(),
    )
    .unwrap();

    assert_eq!(contour.len(), 4);
    assert_eq!(contour.fill_rule(), FillRule::NonZero);
    assert!(
        contour
            .segments()
            .iter()
            .all(|segment| matches!(segment, Segment2::Line(_)))
    );
}

#[test]
fn reconstruction_accepts_closed_polyline_with_repeated_first_point() {
    let points = [
        p(0.0, 0.0),
        p(4.0, 0.0),
        p(4.0, 3.0),
        p(0.0, 3.0),
        p(0.0, 0.0),
    ];

    let contour = Contour2::reconstruct_from_closed_polyline_with_fill_rule(
        &points,
        PolylineReconstructionOptions::default(),
        FillRule::EvenOdd,
    )
    .unwrap();

    assert_eq!(contour.len(), 4);
    assert_eq!(contour.fill_rule(), FillRule::EvenOdd);
}

#[test]
fn real_line_string_rejects_zero_length_source_edges_without_import_record() {
    let points = [[r(0.0), r(0.0)], [r(0.0), r(0.0)], [r(1.0), r(0.0)]];

    assert_eq!(
        CurveString2::from_real_line_string(&points).unwrap_err(),
        CurveError::ZeroLengthLine
    );
}

#[test]
fn reconstruction_removes_adjacent_duplicate_samples() {
    let points = [p(0.0, 0.0), p(0.0, 0.0), p(1.0, 0.0), p(2.0, 0.0)];

    let vertices =
        BulgeVertex2::reconstruct_polyline(&points, PolylineReconstructionOptions::default())
            .unwrap();

    assert_eq!(vertices.len(), 2);
    assert_eq!(vertices[0].point(), &points[0]);
    assert_eq!(vertices[1].point(), &points[3]);
}

#[test]
fn reconstruction_rejects_invalid_options() {
    let points = [p(0.0, 0.0), p(1.0, 0.0)];
    let options = PolylineReconstructionOptions {
        min_arc_points: 2,
        ..PolylineReconstructionOptions::default()
    };

    let err = CurveString2::reconstruct_from_polyline(&points, options)
        .expect_err("min_arc_points below three is invalid");
    assert_eq!(err, CurveError::InvalidReconstructionOptions);
}
#[test]
fn finite_ring_import_discards_adjacent_duplicate_source_edges() {
    let contour =
        Contour2::from_finite_ring(&[[0.0, 0.0], [0.0, 0.0], [4.0, 0.0], [4.0, 3.0], [0.0, 0.0]])
            .unwrap();

    assert_eq!(contour.len(), 3);
}

#[test]
fn finite_ring_import_counts_signed_zero_duplicates_at_the_input_boundary() {
    let contour = Contour2::from_finite_ring(&[
        [-0.0, 0.0],
        [0.0, -0.0],
        [4.0, 0.0],
        [4.0, 3.0],
        [-0.0, 0.0],
    ])
    .unwrap();

    assert_eq!(contour.len(), 3);
    let expected =
        Contour2::from_real_ring(&[[r(0.0), r(0.0)], [r(4.0), r(0.0)], [r(4.0), r(3.0)]]).unwrap();
    assert_eq!(contour, expected);
}

#[test]
fn finite_ring_import_rejects_nonfinite_points_even_when_adjacent() {
    assert_eq!(
        Contour2::from_finite_ring(&[
            [0.0, 0.0],
            [f64::INFINITY, 0.0],
            [f64::INFINITY, 0.0],
            [0.0, 1.0],
        ])
        .unwrap_err(),
        CurveError::NonFiniteReconstructionPoint
    );
}

#[test]
fn finite_ring_import_rejects_all_duplicate_source_edges() {
    assert_eq!(
        Contour2::from_finite_ring(&[[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]).unwrap_err(),
        CurveError::InsufficientVertices
    );
}

fn rectangle_for_recovery(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    Contour2::from_bulge_vertices(
        &[(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]
            .map(|(x, y)| BulgeVertex2::new(Point2::from_values(x, y), Real::zero())),
    )
    .unwrap()
}

fn profiles_for_recovery(
    material: Vec<Contour2>,
    holes: Vec<Contour2>,
    policy: &hypercurve::CurveContext,
) -> Vec<hypercurve::FiniteRegionProfile2> {
    let region = crate::support::under(policy, || {
        hypercurve::CurveRegion2::try_from_native_contours(material, holes)
    })
    .unwrap()
    .into_value();
    let projection = crate::support::under(policy, || {
        region.project_to_finite_profiles(
            &hypercurve::FiniteProjectionOptions::try_new(0.01).unwrap(),
        )
    })
    .unwrap();

    projection.into_value()
}

fn recover_profiles(
    profiles: &[hypercurve::FiniteRegionProfile2],
    policy: &hypercurve::CurveContext,
) -> hypercurve::CurveRegion2 {
    let outcome = crate::support::under(policy, || {
        hypercurve::CurveRegion2::recover_from_finite_profiles(
            profiles,
            PolylineReconstructionOptions {
                min_arc_points: 8,
                ..PolylineReconstructionOptions::DEFAULT
            },
        )
    })
    .unwrap();
    assert_eq!(outcome.certainty, hypercurve::CurveCertainty::Certified);
    outcome.into_value()
}

#[test]
fn finite_profile_recovery_regularizes_overlaps_before_publication() {
    use hypercurve::{CurveContext, OffsetCornerStyle2, RegionPointLocation};
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let mut profiles = profiles_for_recovery(
            vec![rectangle_for_recovery(0, 0, 4, 4)],
            Vec::new(),
            &policy,
        );
        profiles.extend(profiles_for_recovery(
            vec![rectangle_for_recovery(2, 0, 6, 4)],
            Vec::new(),
            &policy,
        ));
        for reverse in [false, true] {
            if reverse {
                profiles.reverse();
            }
            let recovered = recover_profiles(&profiles, &policy);
            assert_eq!(recovered.len(), 1);
            let Some(area) = crate::support::under(&policy, || recovered.filled_area())
                .unwrap()
                .into_value()
            else {
                panic!("the rectangle has an exact area");
            };
            assert_eq!(
                area.partial_cmp(&Real::from(24)),
                Some(std::cmp::Ordering::Equal)
            );
            for (x, expected) in [
                (-1, RegionPointLocation::Outside),
                (0, RegionPointLocation::Boundary),
                (2, RegionPointLocation::Inside),
                (4, RegionPointLocation::Inside),
                (6, RegionPointLocation::Boundary),
                (7, RegionPointLocation::Outside),
            ] {
                assert_eq!(
                    crate::support::under(&policy, || recovered
                        .classify_point(&Point2::from_values(x, 2).into()))
                    .unwrap()
                    .into_value(),
                    expected
                );
            }
            let expanded = crate::support::under(&policy, || {
                recovered.offset(Real::one(), &OffsetCornerStyle2::Round)
            })
            .unwrap()
            .into_value();
            for (x, expected) in [
                (-2, RegionPointLocation::Outside),
                (-1, RegionPointLocation::Boundary),
                (3, RegionPointLocation::Inside),
                (7, RegionPointLocation::Boundary),
                (8, RegionPointLocation::Outside),
            ] {
                assert_eq!(
                    crate::support::under(&policy, || expanded
                        .classify_point(&Point2::from_values(x, 2).into()))
                    .unwrap()
                    .into_value(),
                    expected
                );
            }
        }
    }
}

#[test]
fn finite_profile_recovery_preserves_nested_islands_and_cancels_filled_holes() {
    use hypercurve::{CurveContext, RegionPointLocation};
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let profiles = profiles_for_recovery(
            vec![
                rectangle_for_recovery(0, 0, 10, 10),
                rectangle_for_recovery(4, 4, 6, 6),
            ],
            vec![rectangle_for_recovery(2, 2, 8, 8)],
            &policy,
        );
        let recovered = recover_profiles(&profiles, &policy);
        assert_eq!(recovered.len(), 3);
        assert_eq!(
            crate::support::under(&policy, || recovered.loop_role_counts())
                .unwrap()
                .into_value(),
            (2, 1)
        );
        let Some(area) = crate::support::under(&policy, || recovered.filled_area())
            .unwrap()
            .into_value()
        else {
            panic!("nested rectangles have an exact area");
        };
        assert_eq!(
            area.partial_cmp(&Real::from(68)),
            Some(std::cmp::Ordering::Equal)
        );
        for (x, expected) in [
            (1, RegionPointLocation::Inside),
            (2, RegionPointLocation::Boundary),
            (3, RegionPointLocation::Outside),
            (4, RegionPointLocation::Boundary),
            (5, RegionPointLocation::Inside),
            (8, RegionPointLocation::Boundary),
            (9, RegionPointLocation::Inside),
        ] {
            assert_eq!(
                crate::support::under(&policy, || recovered
                    .classify_point(&Point2::from_values(x, 5).into()))
                .unwrap()
                .into_value(),
                expected
            );
        }
        let mut filled_profiles = profiles;
        filled_profiles.extend(profiles_for_recovery(
            vec![rectangle_for_recovery(2, 2, 8, 8)],
            Vec::new(),
            &policy,
        ));
        let filled = recover_profiles(&filled_profiles, &policy);
        assert_eq!(filled.len(), 1);
        let Some(area) = crate::support::under(&policy, || filled.filled_area())
            .unwrap()
            .into_value()
        else {
            panic!("the filled rectangle has an exact area");
        };
        assert_eq!(
            area.partial_cmp(&Real::from(100)),
            Some(std::cmp::Ordering::Equal)
        );
        for x in [2, 4, 5, 6, 8] {
            assert_eq!(
                crate::support::under(&policy, || filled
                    .classify_point(&Point2::from_values(x, 5).into()))
                .unwrap()
                .into_value(),
                RegionPointLocation::Inside
            );
        }
    }
}

#[test]
fn finite_profile_recovery_accepts_empty_input_with_certified_topology() {
    for policy in [
        hypercurve::CurveContext::STRICT,
        hypercurve::CurveContext::APPROXIMATE_512,
    ] {
        let recovered = recover_profiles(&[], &policy);
        assert!(recovered.is_empty());
        let offset = crate::support::under(&policy, || {
            recovered.offset(Real::from(-1), &hypercurve::OffsetCornerStyle2::Bevel)
        })
        .unwrap();
        assert_eq!(offset.certainty, hypercurve::CurveCertainty::Certified);
        assert!(offset.value.is_empty());
    }
}
