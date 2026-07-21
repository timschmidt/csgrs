//! Native hypercurve Profile tests.
//!
//! These tests intentionally avoid the removed finite compatibility cache.
//! `Profile` should compose CAD from `hypercurve::Region2` and
//! `hypercurve::CurveString2`, with primitive `f64` coordinates appearing only
//! at API and file-format boundaries.

use super::support::*;
#[cfg(feature = "gerber-io")]
use crate::io::gerber::ToGerber;
#[cfg(feature = "svg-io")]
use crate::io::svg::ToSVG;
#[cfg(feature = "offset")]
use hypercurve::{BooleanOp, CurvePolicy, FillRule};
use hypercurve::{
    Classification, Contour2, CubicBezier2, Curve2, CurveFamily2, CurvePath2, CurveRegion2,
    CurveString2, FiniteProjectionOptions, LineSeg2, NurbsCurve2, Point2, RationalBezier2,
    RationalQuadraticBezier2, Region2, finite_ring_signed_area,
};

#[cfg(feature = "offset")]
fn first_profile_area(sketch: &Profile) -> Real {
    let profiles = sketch.region_profiles();
    assert!(!profiles.is_empty(), "expected at least one material profile");
    r(finite_ring_signed_area(profiles[0].material().points()))
}

fn profile_area_sum(sketch: &Profile) -> Real {
    sketch
        .region_profiles()
        .iter()
        .map(|profile| r(finite_ring_signed_area(profile.material().points()).abs()))
        .sum()
}

#[test]
fn sketch_owns_region_and_wires_as_hypercurve_types() {
    let region = Profile::rectangle(r(2.0), r(1.0)).as_region().clone();
    let wire = CurveString2::from_finite_line_string(&[[3.0, 0.0], [4.0, 1.0]]).unwrap();
    let sketch = Profile::from_region_and_wires(region, vec![wire]);

    assert_eq!(sketch.material_contour_count(), 1);
    assert_eq!(sketch.wires().len(), 1);
    assert!(sketch.contains_xy(hr(1.0), hr(0.5)).unwrap());
    assert!(!sketch.contains_xy(hr(3.5), hr(0.5)).unwrap());

    let polylines = sketch.wire_polylines();
    assert_eq!(polylines.len(), 1);
    assert_eq!(polylines[0].len(), 2);
}

fn cubic_cap_path(x_offset: i64) -> CurvePath2 {
    let point = |x: i64, y: i64| Point2::new(Real::from(x + x_offset), Real::from(y));
    CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(point(0, 0), point(4, 0)).unwrap()),
        Curve2::from(LineSeg2::try_new(point(4, 0), point(4, 4)).unwrap()),
        Curve2::from(CubicBezier2::new(
            point(4, 4),
            point(3, 5),
            point(1, 5),
            point(0, 4),
        )),
        Curve2::from(LineSeg2::try_new(point(0, 4), point(0, 0)).unwrap()),
    ])
    .unwrap()
}

#[test]
fn profile_preserves_higher_order_region_and_path_carriers() {
    let boundary = cubic_cap_path(0);
    let region =
        CurveRegion2::try_from_boundary_paths(std::slice::from_ref(&boundary)).unwrap();
    let open_path = CurvePath2::try_new(vec![Curve2::from(CubicBezier2::new(
        Point2::new(Real::from(10), Real::zero()),
        Point2::new(Real::from(11), Real::from(2)),
        Point2::new(Real::from(12), Real::from(-2)),
        Point2::new(Real::from(13), Real::zero()),
    ))])
    .unwrap();
    let profile = Profile::from_curve_region_and_paths(region, vec![open_path]);

    assert!(matches!(
        profile.region_geometry(),
        crate::sketch::ProfileRegionRef::Curved(_)
    ));
    assert!(profile.as_curve_region().is_some());
    assert_eq!(profile.curve_paths().len(), 1);
    assert_eq!(profile.contains_xy(Real::from(2), Real::from(2)), Some(true));
    assert_eq!(profile.region_profiles().len(), 1);
    assert!(!profile.triangulate().is_empty());
    assert!(!profile.extrude(Real::one(), ()).polygons.is_empty());
    let moved = profile.translate(Real::from(2), Real::from(3), Real::zero());
    assert!(moved.as_curve_region().is_some());
    assert_eq!(moved.contains_xy(Real::from(4), Real::from(5)), Some(true));
    assert_eq!(moved.curve_paths().len(), 1);
    let scaled = profile.scale(Real::from(2), Real::from(3), Real::one());
    assert!(scaled.as_curve_region().is_some());
    assert_eq!(scaled.contains_xy(Real::from(4), Real::from(6)), Some(true));
    assert_eq!(scaled.curve_paths().len(), 1);
    let reflected = profile.scale(Real::from(-1), Real::one(), Real::one());
    assert_eq!(
        reflected.contains_xy(Real::from(-2), Real::from(2)),
        Some(true)
    );
    assert!(
        profile
            .as_curve_region()
            .unwrap()
            .fragment_provenance()
            .unwrap()
            .iter()
            .any(|source| source.family() == CurveFamily2::CubicBezier)
    );

    #[cfg(feature = "svg-io")]
    {
        let svg = profile.to_svg().unwrap();
        assert!(svg.contains("fill=\"black\""));
        assert!(svg.contains("fill=\"none\""));
    }

    #[cfg(feature = "gerber-io")]
    {
        let gerber = profile.to_gerber().unwrap();
        assert!(gerber.windows(4).any(|bytes| bytes == b"G36*"));
        assert!(gerber.windows(3).any(|bytes| bytes == b"D02"));
    }
}

#[test]
fn profile_open_paths_preserve_every_higher_order_curve_family() {
    let quadratic = Profile::bezier(
        &[
            [Real::zero(), Real::zero()],
            [Real::one(), Real::from(2)],
            [Real::from(2), Real::zero()],
        ],
        8,
    );
    let cubic = Profile::bezier(
        &[
            [Real::zero(), Real::zero()],
            [Real::one(), Real::from(2)],
            [Real::from(2), Real::from(-2)],
            [Real::from(3), Real::zero()],
        ],
        8,
    );
    let rational_quadratic = Profile::from_curve_path(
        CurvePath2::try_new(vec![Curve2::from(
            RationalQuadraticBezier2::try_unit_end_weights(
                Point2::new(Real::zero(), Real::zero()),
                Point2::new(Real::one(), Real::from(2)),
                Point2::new(Real::from(2), Real::zero()),
                (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap(),
            )
            .unwrap(),
        )])
        .unwrap(),
    );
    let rational = Profile::from_curve_path(
        CurvePath2::try_new(vec![Curve2::from(
            RationalBezier2::try_new(
                vec![
                    Point2::new(Real::zero(), Real::zero()),
                    Point2::new(Real::one(), Real::from(2)),
                    Point2::new(Real::from(2), Real::from(-2)),
                    Point2::new(Real::from(3), Real::from(2)),
                    Point2::new(Real::from(4), Real::zero()),
                ],
                vec![
                    Real::one(),
                    Real::from(2),
                    Real::from(3),
                    Real::from(2),
                    Real::one(),
                ],
            )
            .unwrap(),
        )])
        .unwrap(),
    );
    let bspline = Profile::bspline(
        &[
            [Real::zero(), Real::zero()],
            [Real::one(), Real::from(2)],
            [Real::from(2), Real::from(2)],
            [Real::from(3), Real::zero()],
        ],
        3,
        8,
    );
    let nurbs = Profile::from_curve_path(
        CurvePath2::try_new(vec![Curve2::from(
            NurbsCurve2::try_new(
                2,
                vec![
                    Point2::new(Real::zero(), Real::zero()),
                    Point2::new(Real::one(), Real::from(2)),
                    Point2::new(Real::from(2), Real::zero()),
                ],
                vec![Real::one(), Real::from(2), Real::one()],
                vec![
                    Real::zero(),
                    Real::zero(),
                    Real::zero(),
                    Real::one(),
                    Real::one(),
                    Real::one(),
                ],
            )
            .unwrap(),
        )])
        .unwrap(),
    );

    for (profile, family) in [
        (quadratic, CurveFamily2::QuadraticBezier),
        (cubic, CurveFamily2::CubicBezier),
        (rational_quadratic, CurveFamily2::RationalQuadraticBezier),
        (rational, CurveFamily2::RationalBezier),
        (bspline, CurveFamily2::PolynomialBSpline),
        (nurbs, CurveFamily2::Nurbs),
    ] {
        assert_eq!(profile.curve_paths()[0].curves()[0].family(), family);
        assert!(profile.wire_polylines()[0].len() > 2);
    }
}

#[test]
fn mixed_boolean_promotes_line_arc_region_without_demoting_curves() {
    let curved = Profile::from_curve_region(
        CurveRegion2::try_from_boundary_paths(&[cubic_cap_path(0)]).unwrap(),
    );
    let square =
        Profile::square(Real::from(2)).translate(Real::from(10), Real::zero(), Real::zero());

    let union = curved.try_union(&square).unwrap();

    assert!(union.as_curve_region().is_some());
    assert_eq!(union.contains_xy(Real::from(2), Real::from(2)), Some(true));
    assert_eq!(union.contains_xy(Real::from(11), Real::from(1)), Some(true));
}

#[test]
fn overlapping_curved_boolean_survives_projection_extrusion_and_affine_transform() {
    let curved_path = CurvePath2::try_new(vec![
        Curve2::from(hypercurve::QuadraticBezier2::new(
            Point2::new(Real::from(-2), Real::from(4)),
            Point2::new(Real::zero(), Real::from(-4)),
            Point2::new(Real::from(2), Real::from(4)),
        )),
        Curve2::from(
            LineSeg2::try_new(
                Point2::new(Real::from(2), Real::from(4)),
                Point2::new(Real::from(-2), Real::from(4)),
            )
            .unwrap(),
        ),
    ])
    .unwrap();
    let curved = Profile::from_curve_region(
        CurveRegion2::try_from_boundary_paths(&[curved_path]).unwrap(),
    );
    let cutter = Profile::rectangle(Real::from(6), Real::from(3)).translate(
        Real::from(-3),
        Real::from(2),
        Real::zero(),
    );

    let difference = curved.try_difference(&cutter).unwrap();

    assert!(
        difference
            .as_curve_region()
            .unwrap()
            .has_algebraic_fragments()
    );
    assert!(!difference.region_profiles().is_empty());
    assert!(!difference.extrude(Real::one(), ()).polygons.is_empty());
    let scaled = difference.scale(Real::from(2), Real::from(3), Real::one());
    assert!(scaled.as_curve_region().is_some());
    assert_eq!(scaled.contains_xy(Real::zero(), Real::from(3)), Some(true));
}

#[test]
fn primitive_profile_constructors_promote_scalars_through_hyperreal() {
    let rectangle = Profile::rectangle(r(Real::from(2)), r(1));
    let circle = Profile::circle(r(Real::from(1)), 16);
    let triangle = Profile::right_triangle(r(2), Real::from(1));

    assert!(!rectangle.as_region().is_empty());
    assert!(!circle.as_region().is_empty());
    assert!(!triangle.as_region().is_empty());
}

#[test]
fn profile_extrude_promotes_height_through_hyperreal() {
    let sketch = Profile::rectangle(r(2), r(Real::from(1)));
    let hyper_height = sketch.extrude(r(Real::from(1)), ());
    let int_height = sketch.extrude(r(1), ());

    assert!(!hyper_height.polygons.is_empty());
    assert_eq!(hyper_height.polygons.len(), int_height.polygons.len());
}

#[test]
fn profile_contains_xy_accepts_hyperreal_query_coordinates() {
    let sketch = Profile::rectangle(r(2), r(Real::from(1)));
    let half = (Real::from(1) / Real::from(2)).unwrap();

    assert_eq!(sketch.contains_xy(Real::from(1), half.clone()), Some(true));
    assert_eq!(sketch.contains_xy(Real::from(3), half), Some(false));
}

#[test]
fn region_profiles_are_hypercurve_projection_products() {
    let sketch = Profile::square(r(4.0));
    let options = FiniteProjectionOptions::try_new(1.0e-3).unwrap();
    let profiles = match sketch.project_region_profiles(&options).unwrap() {
        Classification::Decided(profiles) => profiles,
        Classification::Uncertain(_) => panic!("square projection should be decided"),
    };

    assert_eq!(profiles.len(), 1);
    assert!(r(finite_ring_signed_area(profiles[0].material().points())) > r(0.0));
    assert!(profiles[0].holes().is_empty());
}

#[test]
fn region_area_rejects_only_exact_zero_area() {
    let tiny = Profile::rectangle(r(tolerance() * 0.25), r(tolerance() * 0.25));
    assert!(Profile::region_has_nonzero_area(tiny.as_region()));

    let degenerate = Profile::polygon(&[[r(0.0), r(0.0)], [r(1.0), r(0.0)], [r(2.0), r(0.0)]]);
    assert!(!Profile::region_has_nonzero_area(degenerate.as_region()));
}

#[test]
fn closed_rings_become_region_topology_not_sidecar_wires() {
    let contour = Contour2::from_finite_ring(&[
        [0.0, 0.0],
        [2.0, 0.0],
        [2.0, 2.0],
        [0.0, 2.0],
        [0.0, 0.0],
    ])
    .unwrap();
    let sketch = Profile::from_region(Region2::from_material_contours(vec![contour]));

    assert_eq!(sketch.material_contour_count(), 1);
    assert!(sketch.wires().is_empty());
    assert!(sketch.contains_xy(hr(1.0), hr(1.0)).unwrap());
}

#[test]
fn wire_projection_uses_curve_string_directly() {
    let wire =
        CurveString2::from_finite_line_string(&[[0.0, 0.0], [1.0, 1.0], [2.0, 0.0]]).unwrap();
    let sketch = Profile::from_wires(vec![wire]);

    assert!(sketch.as_region().is_empty());
    assert_eq!(sketch.wires().len(), 1);
    let polylines = sketch.wire_polylines();
    assert_eq!(polylines.len(), 1);
    assert_eq!(polylines[0], vec![[0.0, 0.0], [1.0, 1.0], [2.0, 0.0]]);
}

#[test]
fn transform_preserves_native_region_and_wire_topology() {
    let wire = CurveString2::from_finite_point_iter([[3.0, 0.0], [4.0, 0.0]]).unwrap();
    let sketch = Profile::from_region_and_wires(
        Profile::square(r(2.0)).as_region().clone(),
        vec![wire],
    );
    let moved = sketch.translate(r(5.0), r(-2.0), r(0.0));

    assert!(moved.contains_xy(hr(6.0), hr(-1.0)).unwrap());
    assert_eq!(moved.wire_polylines()[0], vec![[8.0, -2.0], [9.0, -2.0]]);
    let bounds = moved.bounding_box();
    assert_eq!(bounds.mins.x, r(5.0));
    assert_eq!(bounds.mins.y, r(-2.0));
    assert_eq!(bounds.maxs.x, r(9.0));

    let third = (Real::one() / Real::from(3_u8)).unwrap();
    let exactly_moved =
        Profile::square(Real::one()).translate(third.clone(), Real::zero(), Real::zero());
    assert_eq!(
        exactly_moved.as_region().material_contours()[0]
            .curve_string()
            .segments()[0]
            .start()
            .x(),
        &third
    );
}

#[test]
fn booleans_regularize_overlapping_rectangles_with_hypercurve_regions() {
    let left = Profile::rectangle(r(2.0), r(2.0));
    let right = Profile::rectangle(r(2.0), r(2.0)).translate(r(1.0), r(0.0), r(0.0));

    let union = left.union(&right);
    let intersection = left.intersection(&right);
    let difference = left.difference(&right);
    let xor = left.xor(&right);

    let union_bounds = union.bounding_box();
    assert_eq!(union_bounds.mins.x, r(0.0));
    assert_eq!(union_bounds.maxs.x, r(3.0));
    assert_eq!(union_bounds.mins.y, r(0.0));
    assert_eq!(union_bounds.maxs.y, r(2.0));

    let intersection_bounds = intersection.bounding_box();
    assert_eq!(intersection_bounds.mins.x, r(1.0));
    assert_eq!(intersection_bounds.maxs.x, r(2.0));
    assert!(profile_area_sum(&union) > profile_area_sum(&left));
    assert!(profile_area_sum(&intersection) < profile_area_sum(&left));
    assert!(profile_area_sum(&difference) < profile_area_sum(&left));
    assert!(profile_area_sum(&xor) > profile_area_sum(&difference));
}

#[test]
fn disjoint_boolean_preserves_native_hole_roles() {
    let outer = Contour2::from_finite_ring(&[
        [0.0, 0.0],
        [4.0, 0.0],
        [4.0, 4.0],
        [0.0, 4.0],
        [0.0, 0.0],
    ])
    .unwrap();
    let hole = Contour2::from_finite_ring(&[
        [1.0, 1.0],
        [3.0, 1.0],
        [3.0, 3.0],
        [1.0, 3.0],
        [1.0, 1.0],
    ])
    .unwrap();
    let holed = Profile::from_region(Region2::new(vec![outer], vec![hole]));
    let island = Profile::square(r(1.0)).translate(r(10.0), r(0.0), r(0.0));

    let union = holed.union(&island);

    assert_eq!(union.material_contour_count(), 2);
    assert_eq!(union.hole_contour_count(), 1);
    assert_eq!(union.contains_xy(hr(0.5), hr(0.5)), Some(true));
    assert_eq!(union.contains_xy(hr(2.0), hr(2.0)), Some(false));
    assert_eq!(union.contains_xy(hr(10.5), hr(0.5)), Some(true));
}

#[test]
fn booleans_preserve_open_wires_without_cache_fallback() {
    let area = Profile::square(r(2.0));
    let wire = CurveString2::from_finite_point_iter([[10.0, 0.0], [11.0, 1.0]]).unwrap();
    let mixed = Profile::from_region_and_wires(area.as_region().clone(), vec![wire]);
    let other = Profile::square(r(1.0)).translate(r(0.5), r(0.5), r(0.0));

    let union = mixed.union(&other);
    assert_eq!(union.wire_polylines().len(), 1);
    assert_eq!(union.wire_polylines()[0], vec![[10.0, 0.0], [11.0, 1.0]]);
}

#[test]
#[cfg(feature = "offset")]
fn offsets_return_native_region_topology() {
    let square = Profile::square(r(2.0));
    let grown = square.offset(r(0.25));
    let shrunk = square.offset(r(-0.25));

    assert!(first_profile_area(&grown) > first_profile_area(&square));
    assert!(first_profile_area(&shrunk) < first_profile_area(&square));
    assert!(grown.wires().is_empty());
    assert!(shrunk.wires().is_empty());
}

#[test]
#[cfg(feature = "offset")]
fn zero_offset_preserves_higher_order_region_and_wires() {
    let profile = Profile::from_curve_region_and_paths(
        CurveRegion2::try_from_boundary_paths(&[cubic_cap_path(0)]).unwrap(),
        vec![
            CurvePath2::try_new(vec![Curve2::from(CubicBezier2::new(
                Point2::new(Real::from(10), Real::zero()),
                Point2::new(Real::from(11), Real::one()),
                Point2::new(Real::from(12), Real::from(-1)),
                Point2::new(Real::from(13), Real::zero()),
            ))])
            .unwrap(),
        ],
    );

    let identity = profile.offset(Real::zero());

    assert!(identity.as_curve_region().is_some());
    assert_eq!(identity.curve_paths().len(), 1);
}

#[test]
#[cfg(feature = "offset")]
fn nonzero_higher_order_offset_is_an_explicit_error() {
    let profile = Profile::from_curve_region(
        CurveRegion2::try_from_boundary_paths(&[cubic_cap_path(0)]).unwrap(),
    );

    assert!(matches!(
        profile.try_offset(Real::one()),
        Err(crate::errors::ProfileOffsetError::HigherOrderCurves)
    ));
    assert!(matches!(
        profile.try_offset_rounded(Real::one()),
        Err(crate::errors::ProfileOffsetError::HigherOrderCurves)
    ));
}

#[test]
#[cfg(feature = "offset")]
fn concentric_exact_circle_offsets_form_an_outline() {
    let circle = Profile::circle(r(15.0), 40);
    let outer = circle.offset_rounded(r(1.0));
    let inner = circle.offset_rounded(r(-1.0));
    let result = outer.as_region().boolean_region_with_report(
        inner.as_region(),
        BooleanOp::Difference,
        FillRule::NonZero,
        &CurvePolicy::certified(),
    );

    let result = result.unwrap();
    assert!(
        result.region().is_some(),
        "concentric offset difference was blocked: {:?}",
        result.report()
    );
}

#[test]
#[cfg(feature = "offset")]
fn wire_offsets_build_filled_hypercurve_outlines() {
    let wire = CurveString2::from_finite_point_iter([[0.0, 0.0], [4.0, 0.0]]).unwrap();
    let sketch = Profile::from_wires(vec![wire]);
    let outline = sketch.offset_rounded(r(0.5));

    assert!(!outline.as_region().is_empty());
    assert!(outline.wires().is_empty());
    assert!(profile_area_sum(&outline) > 0.0);
}

#[test]
#[cfg(feature = "offset")]
fn wire_offsets_admit_any_exactly_positive_width() {
    let wire = CurveString2::from_finite_point_iter([[0.0, 0.0], [4.0, 0.0]]).unwrap();
    let sketch = Profile::from_wires(vec![wire]);
    let outline = sketch.offset(r(1.0e-12));

    assert!(
        !outline.as_region().is_empty(),
        "Any exactly positive offset width should reach hypercurve"
    );
    assert!(outline.wires().is_empty());
}

#[test]
#[cfg(feature = "offset")]
fn straight_skeleton_result_is_native_wire_topology() {
    let skeleton = Profile::square(r(2.0)).straight_skeleton(true);

    assert!(skeleton.as_region().is_empty());
    assert!(!skeleton.wires().is_empty());
    assert!(skeleton.wire_polylines().iter().all(|wire| wire.len() == 2));
}

#[test]
fn triangulation_extrusion_and_exports_consume_region_profiles() {
    let sketch = Profile::rectangle(r(2.0), r(1.0));

    assert!(!sketch.triangulate().is_empty());
    assert!(!sketch.extrude(r(1.0), ()).polygons.is_empty());

    #[cfg(feature = "svg-io")]
    assert!(!sketch.to_svg().unwrap().is_empty());

    #[cfg(feature = "gerber-io")]
    assert!(!sketch.to_gerber().unwrap().is_empty());
}
