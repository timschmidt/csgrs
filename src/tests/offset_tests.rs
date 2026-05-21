//! Tests for offset and buffering behavior.

use super::support::*;
use hypercurve::finite_ring_signed_area;

fn first_material_area(sketch: &Profile<()>) -> Real {
    let profiles = sketch.region_profiles();
    finite_ring_signed_area(profiles[0].material().points())
}

#[test]
fn test_square_ccw_ordering() {
    let square = Profile::<()>::square(2.0, ());
    let profiles = square.region_profiles();
    assert_eq!(profiles.len(), 1);
    let area = finite_ring_signed_area(profiles[0].material().points());
    assert!(area > 0.0, "Square vertices are not CCW ordered");
}

#[test]
#[cfg(feature = "offset")]
fn test_offset_2d_positive_distance_grows() {
    let square = Profile::<()>::square(2.0, ()); // Centered square with size 2x2
    let offset = square.offset(0.5); // Positive offset should grow the square

    // The original square has area 4.0
    // The offset square should have area greater than 4.0
    let profiles = offset.region_profiles();
    assert_eq!(profiles.len(), 1);
    let area = finite_ring_signed_area(profiles[0].material().points());
    assert!(
        area > 4.0,
        "Offset with positive distance did not grow the square"
    );
}

#[test]
#[cfg(feature = "offset")]
fn test_offset_2d_negative_distance_shrinks() {
    let square = Profile::<()>::square(2.0, ()); // Centered square with size 2x2
    let offset = square.offset(-0.5); // Negative offset should shrink the square

    // The original square has area 4.0
    // The offset square should have area less than 4.0
    let profiles = offset.region_profiles();
    assert_eq!(profiles.len(), 1);
    let area = finite_ring_signed_area(profiles[0].material().points());
    assert!(
        area < 4.0,
        "Offset with negative distance did not shrink the square"
    );
}

#[test]
#[cfg(feature = "offset")]
fn test_straight_skeleton_2d_non_empty() {
    let square = Profile::<()>::square(2.0, ());
    let skeleton = square.straight_skeleton(true);

    assert!(
        !skeleton.wires().is_empty(),
        "Straight skeleton should produce native hypercurve wire geometry for a valid square"
    );
}

#[test]
fn test_polygon_2d_enforce_ccw_ordering() {
    // Define a triangle in CW order
    let points_cw = vec![[0.0, 0.0], [1.0, 0.0], [0.5, 1.0]];
    let csg_cw = Profile::<()>::polygon(&points_cw, ());
    // Enforce CCW ordering
    let normalized = csg_cw.renormalize();
    let area = first_material_area(&normalized);
    assert!(area > 0.0, "Polygon ordering was not corrected to CCW");
}

#[test]
#[cfg(feature = "offset")]
fn test_zero_offset_preserves_region_2d() {
    let shape = Profile::<()>::circle(1.0, 32, ());
    let unchanged = shape.offset(0.0);

    let original_area = first_material_area(&shape);
    let unchanged_area = first_material_area(&unchanged);

    assert!(
        (unchanged_area - original_area).abs() < 1.0e-9,
        "Zero offset should preserve native region area"
    );
}

#[test]
#[cfg(feature = "offset")]
fn test_offset_accepts_hyperreal_primary_scalar_and_integer_promotion() {
    let shape = Profile::<()>::circle(1.0, 32, ());

    let zero_hyper = shape.offset(hyperreal::Real::from(0));
    assert!(
        (first_material_area(&zero_hyper) - first_material_area(&shape)).abs() < 1.0e-9,
        "Hyperreal zero offset should preserve native region area"
    );

    let integer_offset = shape.offset(0);
    assert!(
        (first_material_area(&integer_offset) - first_material_area(&shape)).abs() < 1.0e-9,
        "Integer zero distance should promote through the hyperreal API surface"
    );
}

#[test]
#[cfg(feature = "offset")]
fn test_non_finite_offsets_fail_closed() {
    let shape = Profile::<()>::circle(1.0, 32, ());

    for distance in [Real::NAN, Real::INFINITY, Real::NEG_INFINITY] {
        let offset = shape.offset(distance);
        assert!(
            offset.region_profiles().is_empty(),
            "Non-finite offsets must not be treated as zero-distance identity"
        );
    }
}
