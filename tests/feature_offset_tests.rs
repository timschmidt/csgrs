#![cfg(feature = "offset")]

use csgrs::{sketch::Sketch, traits::CSG};

#[test]
fn offset_2d() {
    let square: Sketch<()> = Sketch::square(2.0, None);
    let grown = square.offset(0.5);
    let shrunk = square.offset(-0.5);
    let bb_square = square.bounding_box();
    let bb_grown = grown.bounding_box();
    let bb_shrunk = shrunk.bounding_box();

    println!("Square bb: {:#?}", bb_square);
    println!("Grown bb: {:#?}", bb_grown);
    println!("Shrunk bb: {:#?}", bb_shrunk);

    // Should be bigger
    assert!(bb_grown.maxs.x > bb_square.maxs.x + 0.4);

    // Should be smaller
    assert!(bb_shrunk.maxs.x < bb_square.maxs.x + 0.1);
}

mod offset_2d {
    use csgrs::sketch::Sketch;
    use geo::Area;

    #[test]
    fn positive_distance_grows() {
        let square = Sketch::<()>::square(2.0, None); // Centered square with size 2x2
        let offset = square.offset(0.5); // Positive offset should grow the square

        // The original square has area 4.0
        // The offset square should have area greater than 4.0
        let mp = offset.to_multipolygon();
        assert_eq!(mp.0.len(), 1);
        let poly = &mp.0[0];
        let area = poly.signed_area();
        assert!(
            area > 4.0,
            "Offset with positive distance did not grow the square"
        );
    }

    #[test]
    fn negative_distance_shrinks() {
        let square = Sketch::<()>::square(2.0, None); // Centered square with size 2x2
        let offset = square.offset(-0.5); // Negative offset should shrink the square

        // The original square has area 4.0
        // The offset square should have area less than 4.0
        let mp = offset.to_multipolygon();
        assert_eq!(mp.0.len(), 1);
        let poly = &mp.0[0];
        let area = poly.signed_area();
        assert!(
            area < 4.0,
            "Offset with negative distance did not shrink the square"
        );
    }

    #[test]
    fn circle() {
        let circle = Sketch::<()>::circle(1.0, 32, None);
        let offset_grow = circle.offset(0.2); // Should grow the circle
        let offset_shrink = circle.offset(-0.2); // Should shrink the circle

        let grow = offset_grow.to_multipolygon();
        let shrink = offset_shrink.to_multipolygon();
        let grow_area = grow.0[0].signed_area();
        let shrink_area = shrink.0[0].signed_area();

        // Original circle has area ~3.1416
        #[allow(clippy::approx_constant)]
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
}
