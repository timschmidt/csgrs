use super::super::*;
use hypercurve::RegionPointLocation;

fn fraction(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn rectangle(left: Real, bottom: Real, right: Real, top: Real) -> CurvePath2 {
    let points = [
        Point2::new(left.clone(), bottom.clone()),
        Point2::new(right.clone(), bottom),
        Point2::new(right, top.clone()),
        Point2::new(left, top),
    ];
    CurvePath2::try_new(
        (0..4)
            .map(|i| {
                LineSeg2::try_new(points[i].clone(), points[(i + 1) % 4].clone())
                    .unwrap()
                    .into()
            })
            .collect(),
    )
    .unwrap()
}

fn narrow_extension() -> (CurveRegion2, Point2) {
    let small = fraction(1, 1_i64 << 60);
    let path = rectangle(
        Real::zero(),
        Real::zero(),
        Real::one() + &small,
        Real::one(),
    );
    let source = CurveRegion2::try_from_boundary_paths(&[path], FillRule::NonZero).unwrap();
    let point = Point2::new(
        Real::one() + (&small / Real::from(2)).unwrap(),
        fraction(1, 2),
    );
    (source, point)
}

#[test]
fn preview_and_segmented_display_keep_geometry_beyond_f64_resolution() {
    let (source, point) = narrow_extension();
    let display = Shape::from_curve_region(&source).unwrap().unwrap();
    let container = CurveRegion2::try_from_boundary_paths(
        &[rectangle(
            Real::from(-1),
            Real::from(-1),
            Real::from(2),
            Real::from(2),
        )],
        FillRule::NonZero,
    )
    .unwrap();
    let container = Shape::from_curve_region(&container).unwrap().unwrap();
    for displayed in [display.clone(), display.segmented_for_display()] {
        let round_trip = displayed.to_curve_region().unwrap();
        assert_eq!(round_trip, source);
        assert_eq!(
            round_trip.classify_point(&point.clone().into()).unwrap(),
            RegionPointLocation::Inside
        );
        let rebuilt = Shape::from_materials(displayed.materials.clone())
            .to_curve_region()
            .unwrap();
        assert_eq!(
            rebuilt.classify_point(&point.clone().into()).unwrap(),
            RegionPointLocation::Inside
        );
        assert_eq!(
            rebuilt.filled_area().unwrap(),
            source.filled_area().unwrap()
        );
        let intersection = displayed
            .boolean(&container, BooleanMode::Intersection)
            .unwrap()
            .unwrap();
        assert_eq!(
            intersection
                .to_curve_region()
                .unwrap()
                .classify_point(&point.clone().into())
                .unwrap(),
            RegionPointLocation::Inside
        );
    }
    let encoded = serde_json::to_value(&display).unwrap();
    assert!(encoded.get("retained").is_none());
    assert!(encoded["materials"][0].get("retained").is_none());
    // JSON is the existing finite authoring format, not an exact result archive.
    let authored: Shape = serde_json::from_value(encoded).unwrap();
    assert_eq!(
        authored
            .to_curve_region()
            .unwrap()
            .classify_point(&point.into())
            .unwrap(),
        RegionPointLocation::Outside
    );
}

#[test]
fn an_editor_handle_change_replaces_the_retained_source() {
    let (source, _) = narrow_extension();
    let mut display = Shape::from_curve_region(&source).unwrap().unwrap();
    let untouched = display.clone();
    display.materials[0].set_handle(0, -1.0, 0.0);
    let edited = display.to_curve_region().unwrap();
    assert_eq!(edited.filled_area().unwrap(), Some(fraction(3, 2)));
    assert_eq!(untouched.to_curve_region().unwrap(), source);
    assert_ne!(edited, source);
}

#[test]
fn direct_control_changes_cannot_reuse_a_stale_region_or_path() {
    let (source, _) = narrow_extension();
    let display = Shape::from_curve_region(&source).unwrap().unwrap();
    let mut disconnected = display.clone();
    disconnected.materials[0].curve_data[0].set_handle(0, -2.0, 0.0);
    assert!(disconnected.to_curve_region().is_err());
    let edited_path = disconnected.materials[0].to_curve_path().unwrap();
    assert_eq!(edited_path.start(), Point2::from_values(-2, 0).into());
    assert_ne!(edited_path.start(), edited_path.end());
    let mut nonfinite = display;
    nonfinite.materials[0].curve_data[0].set_handle(0, f64::NAN, 0.0);
    assert!(
        nonfinite
            .to_curve_region()
            .unwrap_err()
            .contains("must be finite")
    );
    assert!(
        nonfinite.materials[0]
            .to_curve_path()
            .unwrap_err()
            .contains("must be finite")
    );
}

#[test]
fn changing_loop_roles_invalidates_the_retained_region() {
    let paths = [
        rectangle(Real::zero(), Real::zero(), Real::from(4), Real::from(4)),
        rectangle(Real::one(), Real::one(), Real::from(3), Real::from(3)),
    ];
    let source = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &paths,
        &[CurveRegionLoopRole::Material, CurveRegionLoopRole::Hole],
        &[FillRule::NonZero; 2],
    )
    .unwrap();
    let mut display = Shape::from_curve_region(&source).unwrap().unwrap();
    let point = Point2::from_values(2, 2);
    assert_eq!(
        display
            .to_curve_region()
            .unwrap()
            .classify_point(&point.clone().into())
            .unwrap(),
        RegionPointLocation::Outside
    );
    display.materials.append(&mut display.holes);
    assert_eq!(
        display
            .to_curve_region()
            .unwrap()
            .classify_point(&point.into())
            .unwrap(),
        RegionPointLocation::Inside
    );
}

#[test]
fn projection_without_an_exact_path_cannot_reenter_topology() {
    let (source, _) = narrow_extension();
    let mut display = Shape::from_curve_region(&source).unwrap().unwrap();
    let polyline = &mut display.materials[0];
    polyline.retain_path(None);
    for projected in [polyline.clone(), polyline.to_sampled_polyline(0.2)] {
        assert!(
            projected
                .to_curve_path()
                .unwrap_err()
                .contains("no reusable exact path")
        );
        assert!(
            projected
                .to_contour()
                .unwrap_err()
                .contains("no reusable exact path")
        );
        assert!(
            projected
                .to_curve_string()
                .unwrap_err()
                .contains("no reusable exact path")
        );
    }
    // An actual edit authors new finite geometry and invalidates the projection.
    polyline.set_handle(0, -1.0, 0.0);
    assert!(polyline.to_curve_path().is_ok());
}
