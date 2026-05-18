//! Tests for offset and buffering behavior.

use super::support::*;
use hypercurve::finite_ring_signed_area;

fn first_material_area(sketch: &Sketch<()>) -> Real {
    let profiles = sketch.region_profiles();
    finite_ring_signed_area(profiles[0].material().points())
}

#[test]
fn test_square_ccw_ordering() {
    let square = Sketch::<()>::square(2.0, ());
    let profiles = square.region_profiles();
    assert_eq!(profiles.len(), 1);
    let area = finite_ring_signed_area(profiles[0].material().points());
    assert!(area > 0.0, "Square vertices are not CCW ordered");
}

#[test]
#[cfg(feature = "offset")]
fn test_offset_2d_positive_distance_grows() {
    let square = Sketch::<()>::square(2.0, ()); // Centered square with size 2x2
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
    let square = Sketch::<()>::square(2.0, ()); // Centered square with size 2x2
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
    let square = Sketch::<()>::square(2.0, ());
    let skeleton = square.straight_skeleton(true);

    assert!(
        !skeleton.geometry().0.is_empty(),
        "Straight skeleton should produce line geometry for a valid square"
    );
}

#[test]
fn test_polygon_2d_enforce_ccw_ordering() {
    // Define a triangle in CW order
    let points_cw = vec![[0.0, 0.0], [1.0, 0.0], [0.5, 1.0]];
    let csg_cw = Sketch::<()>::polygon(&points_cw, ());
    // Enforce CCW ordering
    let normalized = csg_cw.renormalize();
    let area = first_material_area(&normalized);
    assert!(area > 0.0, "Polygon ordering was not corrected to CCW");
}

#[test]
#[cfg(feature = "offset")]
fn test_circle_offset_2d() {
    let circle = Sketch::<()>::circle(1.0, 32, ());
    let offset_grow = circle.offset(0.2); // Should grow the circle
    let offset_shrink = circle.offset(-0.2); // Should shrink the circle

    let grow_area = first_material_area(&offset_grow);
    let shrink_area = first_material_area(&offset_shrink);

    // Original circle has area ~3.1416
    let original_area = 3.141592653589793;

    assert!(
        grow_area > original_area,
        "Offset with positive distance did not grow the circle"
    );
    assert!(
        shrink_area < original_area,
        "Offset with negative distance did not shrink the circle"
    );
}
