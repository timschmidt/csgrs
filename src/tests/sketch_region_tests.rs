//! Native hypercurve Sketch tests.
//!
//! These tests intentionally avoid the removed finite compatibility cache.
//! `Sketch` should compose CAD from `hypercurve::Region2` and
//! `hypercurve::CurveString2`, with primitive `f64` coordinates appearing only
//! at API and file-format boundaries.

use super::support::*;
#[cfg(feature = "gerber-io")]
use crate::io::gerber::ToGerber;
#[cfg(feature = "svg-io")]
use crate::io::svg::ToSVG;
use hypercurve::{
    Classification, Contour2, CurveString2, FiniteProjectionOptions, Region2,
    finite_ring_signed_area,
};

#[cfg(feature = "offset")]
fn first_profile_area(sketch: &Sketch<()>) -> Real {
    let profiles = sketch.region_profiles();
    assert!(!profiles.is_empty(), "expected at least one material profile");
    finite_ring_signed_area(profiles[0].material().points())
}

fn profile_area_sum(sketch: &Sketch<()>) -> Real {
    sketch
        .region_profiles()
        .iter()
        .map(|profile| finite_ring_signed_area(profile.material().points()).abs())
        .sum()
}

#[test]
fn sketch_owns_region_and_wires_as_hypercurve_types() {
    let region = Sketch::<()>::rectangle(2.0, 1.0, ()).as_region().clone();
    let wire = CurveString2::from_finite_line_string(&[[3.0, 0.0], [4.0, 1.0]]).unwrap();
    let sketch = Sketch::from_region_and_wires(region, vec![wire], "native");

    assert_eq!(sketch.metadata(), &"native");
    assert_eq!(sketch.material_contour_count(), 1);
    assert_eq!(sketch.wires().len(), 1);
    assert!(sketch.contains_xy(1.0, 0.5).unwrap());
    assert!(!sketch.contains_xy(3.5, 0.5).unwrap());

    let polylines = sketch.wire_polylines();
    assert_eq!(polylines.len(), 1);
    assert_eq!(polylines[0].len(), 2);
}

#[test]
fn region_profiles_are_hypercurve_projection_products() {
    let sketch = Sketch::<()>::square(4.0, ());
    let options = FiniteProjectionOptions::try_new(1.0e-3).unwrap();
    let profiles = match sketch.project_region_profiles(&options).unwrap() {
        Classification::Decided(profiles) => profiles,
        Classification::Uncertain(_) => panic!("square projection should be decided"),
    };

    assert_eq!(profiles.len(), 1);
    assert!(finite_ring_signed_area(profiles[0].material().points()) > 0.0);
    assert!(profiles[0].holes().is_empty());
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
    let sketch = Sketch::from_region(Region2::from_material_contours(vec![contour]), ());

    assert_eq!(sketch.material_contour_count(), 1);
    assert!(sketch.wires().is_empty());
    assert!(sketch.contains_xy(1.0, 1.0).unwrap());
}

#[test]
fn wire_projection_uses_curve_string_directly() {
    let wire =
        CurveString2::from_finite_line_string(&[[0.0, 0.0], [1.0, 1.0], [2.0, 0.0]]).unwrap();
    let sketch = Sketch::from_wires(vec![wire], ());

    assert!(sketch.as_region().is_empty());
    assert_eq!(sketch.wires().len(), 1);
    let polylines = sketch.wire_polylines();
    assert_eq!(polylines.len(), 1);
    assert_eq!(polylines[0], vec![[0.0, 0.0], [1.0, 1.0], [2.0, 0.0]]);
}

#[test]
fn transform_preserves_native_region_and_wire_topology() {
    let wire = CurveString2::from_finite_point_iter([[3.0, 0.0], [4.0, 0.0]]).unwrap();
    let sketch = Sketch::from_region_and_wires(
        Sketch::<()>::square(2.0, ()).as_region().clone(),
        vec![wire],
        (),
    );
    let moved = sketch.translate(5.0, -2.0, 0.0);

    assert!(moved.contains_xy(6.0, -1.0).unwrap());
    assert_eq!(moved.wire_polylines()[0], vec![[8.0, -2.0], [9.0, -2.0]]);
    let bounds = moved.bounding_box();
    assert_eq!(bounds.mins.x, 5.0);
    assert_eq!(bounds.mins.y, -2.0);
    assert_eq!(bounds.maxs.x, 9.0);
}

#[test]
fn booleans_regularize_overlapping_rectangles_with_hypercurve_regions() {
    let left = Sketch::<()>::rectangle(2.0, 2.0, ());
    let right = Sketch::<()>::rectangle(2.0, 2.0, ()).translate(1.0, 0.0, 0.0);

    let union = left.union(&right);
    let intersection = left.intersection(&right);
    let difference = left.difference(&right);
    let xor = left.xor(&right);

    let union_bounds = union.bounding_box();
    assert_eq!(union_bounds.mins.x, 0.0);
    assert_eq!(union_bounds.maxs.x, 3.0);
    assert_eq!(union_bounds.mins.y, 0.0);
    assert_eq!(union_bounds.maxs.y, 2.0);

    let intersection_bounds = intersection.bounding_box();
    assert_eq!(intersection_bounds.mins.x, 1.0);
    assert_eq!(intersection_bounds.maxs.x, 2.0);
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
    let holed = Sketch::from_region(Region2::new(vec![outer], vec![hole]), ());
    let island = Sketch::<()>::square(1.0, ()).translate(10.0, 0.0, 0.0);

    let union = holed.union(&island);

    assert_eq!(union.material_contour_count(), 2);
    assert_eq!(union.hole_contour_count(), 1);
    assert_eq!(union.contains_xy(0.5, 0.5), Some(true));
    assert_eq!(union.contains_xy(2.0, 2.0), Some(false));
    assert_eq!(union.contains_xy(10.5, 0.5), Some(true));
}

#[test]
fn booleans_preserve_open_wires_without_cache_fallback() {
    let area = Sketch::<()>::square(2.0, ());
    let wire = CurveString2::from_finite_point_iter([[10.0, 0.0], [11.0, 1.0]]).unwrap();
    let mixed = Sketch::from_region_and_wires(area.as_region().clone(), vec![wire], ());
    let other = Sketch::<()>::square(1.0, ()).translate(0.5, 0.5, 0.0);

    let union = mixed.union(&other);
    assert_eq!(union.wire_polylines().len(), 1);
    assert_eq!(union.wire_polylines()[0], vec![[10.0, 0.0], [11.0, 1.0]]);
}

#[test]
#[cfg(feature = "offset")]
fn offsets_return_native_region_topology() {
    let square = Sketch::<()>::square(2.0, ());
    let grown = square.offset(0.25);
    let shrunk = square.offset(-0.25);

    assert!(first_profile_area(&grown) > first_profile_area(&square));
    assert!(first_profile_area(&shrunk) < first_profile_area(&square));
    assert!(grown.wires().is_empty());
    assert!(shrunk.wires().is_empty());
}

#[test]
#[cfg(feature = "offset")]
fn wire_offsets_build_filled_hypercurve_outlines() {
    let wire = CurveString2::from_finite_point_iter([[0.0, 0.0], [4.0, 0.0]]).unwrap();
    let sketch = Sketch::from_wires(vec![wire], ());
    let outline = sketch.offset_rounded(0.5);

    assert!(!outline.as_region().is_empty());
    assert!(outline.wires().is_empty());
    assert!(profile_area_sum(&outline) > 0.0);
}

#[test]
#[cfg(feature = "offset")]
fn straight_skeleton_result_is_native_wire_topology() {
    let skeleton = Sketch::<()>::square(2.0, ()).straight_skeleton(true);

    assert!(skeleton.as_region().is_empty());
    assert!(!skeleton.wires().is_empty());
    assert!(skeleton.wire_polylines().iter().all(|wire| wire.len() == 2));
}

#[test]
fn triangulation_extrusion_and_exports_consume_region_profiles() {
    let sketch = Sketch::<()>::rectangle(2.0, 1.0, ());

    assert!(!sketch.triangulate().is_empty());
    assert!(!sketch.extrude(1.0).polygons.is_empty());

    #[cfg(feature = "svg-io")]
    assert!(!sketch.to_svg().is_empty());

    #[cfg(feature = "gerber-io")]
    assert!(!sketch.to_gerber().unwrap().is_empty());
}
