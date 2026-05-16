use hypercurve::{
    BooleanOp, BulgeVertex2, Classification, Contour2, ContourPointLocation, CurvePolicy,
    DefaultBackend, FillRule, Region2, RegionPointLocation, Scalar, UncertaintyReason,
};

fn s(value: i32) -> Scalar<DefaultBackend> {
    value.into()
}

fn p(x: i32, y: i32) -> hypercurve::Point2<DefaultBackend> {
    hypercurve::Point2::new(s(x), s(y))
}

fn vertex(x: i32, y: i32, bulge: i32) -> BulgeVertex2<DefaultBackend> {
    BulgeVertex2::new(p(x, y), s(bulge))
}

fn contour(vertices: &[BulgeVertex2<DefaultBackend>]) -> Contour2<DefaultBackend> {
    Contour2::from_bulge_vertices(vertices).unwrap()
}

fn rectangle(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2<DefaultBackend> {
    contour(&[
        vertex(xmin, ymin, 0),
        vertex(xmax, ymin, 0),
        vertex(xmax, ymax, 0),
        vertex(xmin, ymax, 0),
    ])
}

fn rectangle_rotated_start(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2<DefaultBackend> {
    contour(&[
        vertex(xmax, ymax, 0),
        vertex(xmin, ymax, 0),
        vertex(xmin, ymin, 0),
        vertex(xmax, ymin, 0),
    ])
}

fn rectangle_reversed(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2<DefaultBackend> {
    contour(&[
        vertex(xmin, ymin, 0),
        vertex(xmin, ymax, 0),
        vertex(xmax, ymax, 0),
        vertex(xmax, ymin, 0),
    ])
}

fn region(contours: Vec<Contour2<DefaultBackend>>) -> Region2<DefaultBackend> {
    Region2::from_material_contours(contours)
}

fn donut(
    outer: Contour2<DefaultBackend>,
    hole: Contour2<DefaultBackend>,
) -> Region2<DefaultBackend> {
    Region2::new(vec![outer], vec![hole])
}

fn touching_material_bins() -> Region2<DefaultBackend> {
    Region2::from_material_contours(vec![rectangle(0, 0, 2, 2), rectangle(2, 0, 4, 2)])
}

fn touching_material_bins_reordered() -> Region2<DefaultBackend> {
    Region2::from_material_contours(vec![rectangle(2, 0, 4, 2), rectangle(0, 0, 2, 2)])
}

fn touching_material_bins_rotated_and_reversed() -> Region2<DefaultBackend> {
    Region2::from_material_contours(vec![
        rectangle_reversed(2, 0, 4, 2),
        rectangle_rotated_start(0, 0, 2, 2),
    ])
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn assert_contour_location(
    contour: &Contour2<DefaultBackend>,
    point: hypercurve::Point2<DefaultBackend>,
    expected: ContourPointLocation,
) {
    assert_eq!(
        contour.classify_point(&point, &policy()),
        Classification::Decided(expected)
    );
}

fn assert_region_location(
    region: &Region2<DefaultBackend>,
    point: hypercurve::Point2<DefaultBackend>,
    expected: RegionPointLocation,
) {
    assert_eq!(
        region.classify_point(&point, &policy()),
        Classification::Decided(expected)
    );
}

#[test]
fn region_boolean_boundary_contours_union_overlapping_rectangles() {
    let first = region(vec![rectangle(0, 0, 4, 4)]);
    let second = region(vec![rectangle(2, -1, 6, 3)]);

    let Classification::Decided(contours) = first
        .boolean_boundary_contours(&second, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided union boundary contours");
    };

    assert_eq!(contours.len(), 1);
    assert_contour_location(&contours[0], p(1, 1), ContourPointLocation::Inside);
    assert_contour_location(&contours[0], p(5, 2), ContourPointLocation::Inside);
    assert_contour_location(&contours[0], p(5, 4), ContourPointLocation::Outside);
}

#[test]
fn region_boolean_region_union_overlapping_rectangles() {
    let first = region(vec![rectangle(0, 0, 4, 4)]);
    let second = region(vec![rectangle(2, -1, 6, 3)]);

    let Classification::Decided(result) = first
        .boolean_region(&second, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided union region");
    };

    assert_eq!(result.material_contours().len(), 1);
    assert_eq!(result.hole_contours().len(), 0);
    assert_region_location(&result, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&result, p(5, 2), RegionPointLocation::Inside);
    assert_region_location(&result, p(5, 4), RegionPointLocation::Outside);
}

#[test]
fn region_boolean_boundary_contours_intersection_overlapping_rectangles() {
    let first = region(vec![rectangle(0, 0, 4, 4)]);
    let second = region(vec![rectangle(2, -1, 6, 3)]);

    let Classification::Decided(contours) = first
        .boolean_boundary_contours(
            &second,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided intersection boundary contours");
    };

    assert_eq!(contours.len(), 1);
    assert_contour_location(&contours[0], p(3, 1), ContourPointLocation::Inside);
    assert_contour_location(&contours[0], p(1, 1), ContourPointLocation::Outside);
    assert_contour_location(&contours[0], p(5, 2), ContourPointLocation::Outside);
}

#[test]
fn region_boolean_region_difference_nested_rectangle_creates_hole() {
    let outer = region(vec![rectangle(0, 0, 10, 10)]);
    let inner = region(vec![rectangle(3, 3, 7, 7)]);

    let Classification::Decided(result) = outer
        .boolean_region(&inner, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided nested difference region");
    };

    assert_eq!(result.material_contours().len(), 1);
    assert_eq!(result.hole_contours().len(), 1);
    assert_region_location(&result, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&result, p(5, 5), RegionPointLocation::Outside);
    assert_region_location(&result, p(3, 5), RegionPointLocation::Boundary);
}

#[test]
fn region_boolean_region_intersection_nested_rectangle_keeps_inner_material() {
    let outer = region(vec![rectangle(0, 0, 10, 10)]);
    let inner = region(vec![rectangle(3, 3, 7, 7)]);

    let Classification::Decided(result) = outer
        .boolean_region(
            &inner,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided nested intersection region");
    };

    assert_eq!(result.material_contours().len(), 1);
    assert_eq!(result.hole_contours().len(), 0);
    assert_region_location(&result, p(5, 5), RegionPointLocation::Inside);
    assert_region_location(&result, p(1, 1), RegionPointLocation::Outside);
}

#[test]
fn region_boolean_region_xor_nested_rectangle_creates_hole() {
    let outer = region(vec![rectangle(0, 0, 10, 10)]);
    let inner = region(vec![rectangle(3, 3, 7, 7)]);

    let Classification::Decided(result) = outer
        .boolean_region(&inner, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided nested xor region");
    };

    assert_eq!(result.material_contours().len(), 1);
    assert_eq!(result.hole_contours().len(), 1);
    assert_region_location(&result, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&result, p(5, 5), RegionPointLocation::Outside);
}

#[test]
fn region_boolean_region_identical_rectangle_identities_are_decided() {
    let rect = region(vec![rectangle(0, 0, 4, 4)]);

    let Classification::Decided(union) = rect
        .boolean_region(&rect, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided self union");
    };
    assert_eq!(union.material_contours().len(), 1);
    assert_region_location(&union, p(2, 2), RegionPointLocation::Inside);

    let Classification::Decided(intersection) = rect
        .boolean_region(&rect, BooleanOp::Intersection, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided self intersection");
    };
    assert_eq!(intersection.material_contours().len(), 1);
    assert_region_location(&intersection, p(2, 2), RegionPointLocation::Inside);

    let Classification::Decided(difference) = rect
        .boolean_region(&rect, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided self difference");
    };
    assert!(difference.is_empty());

    let Classification::Decided(xor) = rect
        .boolean_region(&rect, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided self xor");
    };
    assert!(xor.is_empty());
}

#[test]
fn region_boolean_region_identical_donut_identities_are_decided() {
    let ring = donut(rectangle(0, 0, 10, 10), rectangle(3, 3, 7, 7));

    let Classification::Decided(union) = ring
        .boolean_region(&ring, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided donut self union");
    };
    assert_eq!(union.material_contours().len(), 1);
    assert_eq!(union.hole_contours().len(), 1);
    assert_region_location(&union, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&union, p(5, 5), RegionPointLocation::Outside);

    let Classification::Decided(intersection) = ring
        .boolean_region(&ring, BooleanOp::Intersection, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided donut self intersection");
    };
    assert_eq!(intersection.material_contours().len(), 1);
    assert_eq!(intersection.hole_contours().len(), 1);
    assert_region_location(&intersection, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&intersection, p(5, 5), RegionPointLocation::Outside);

    let Classification::Decided(difference) = ring
        .boolean_region(&ring, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided donut self difference");
    };
    assert!(difference.is_empty());

    let Classification::Decided(xor) = ring
        .boolean_region(&ring, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided donut self xor");
    };
    assert!(xor.is_empty());
}

#[test]
fn region_boolean_region_empty_identities_preserve_donut_roles() {
    let empty = Region2::<DefaultBackend>::empty();
    let ring = donut(rectangle(0, 0, 10, 10), rectangle(3, 3, 7, 7));

    let Classification::Decided(union_left) = empty
        .boolean_region(&ring, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided empty union donut");
    };
    assert_eq!(union_left.material_contours().len(), 1);
    assert_eq!(union_left.hole_contours().len(), 1);
    assert_region_location(&union_left, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&union_left, p(5, 5), RegionPointLocation::Outside);

    let Classification::Decided(union_right) = ring
        .boolean_region(&empty, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided donut union empty");
    };
    assert_eq!(union_right.material_contours().len(), 1);
    assert_eq!(union_right.hole_contours().len(), 1);
    assert_region_location(&union_right, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&union_right, p(5, 5), RegionPointLocation::Outside);

    let Classification::Decided(intersection) = ring
        .boolean_region(
            &empty,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided donut intersection empty");
    };
    assert!(intersection.is_empty());

    let Classification::Decided(difference) = ring
        .boolean_region(&empty, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided donut difference empty");
    };
    assert_eq!(difference.material_contours().len(), 1);
    assert_eq!(difference.hole_contours().len(), 1);

    let Classification::Decided(empty_difference) = empty
        .boolean_region(&ring, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided empty difference donut");
    };
    assert!(empty_difference.is_empty());
}

#[test]
fn region_boolean_region_self_identity_preserves_touching_material_bins() {
    let touching = touching_material_bins();

    assert_eq!(
        Region2::from_boundary_contours(
            vec![rectangle(0, 0, 2, 2), rectangle(2, 0, 4, 2)],
            &policy(),
        )
        .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );

    let Classification::Decided(union) = touching
        .boolean_region(&touching, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided self union for explicit touching bins");
    };
    assert_eq!(union.material_contours().len(), 2);
    assert_eq!(union.hole_contours().len(), 0);
    assert_region_location(&union, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&union, p(2, 1), RegionPointLocation::Boundary);
    assert_region_location(&union, p(3, 1), RegionPointLocation::Inside);

    let Classification::Decided(intersection) = touching
        .boolean_region(
            &touching,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided self intersection for explicit touching bins");
    };
    assert_eq!(intersection.material_contours().len(), 2);
    assert_eq!(intersection.hole_contours().len(), 0);

    let Classification::Decided(difference) = touching
        .boolean_region(
            &touching,
            BooleanOp::Difference,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided self difference for explicit touching bins");
    };
    assert!(difference.is_empty());

    let Classification::Decided(xor) = touching
        .boolean_region(&touching, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided self xor for explicit touching bins");
    };
    assert!(xor.is_empty());
}

#[test]
fn region_boolean_region_identity_accepts_reordered_touching_material_bins() {
    let first = touching_material_bins();
    let second = touching_material_bins_reordered();

    let Classification::Decided(union) = first
        .boolean_region(&second, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided union for reordered touching bins");
    };
    assert_eq!(union.material_contours().len(), 2);
    assert_eq!(union.hole_contours().len(), 0);
    assert_region_location(&union, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&union, p(2, 1), RegionPointLocation::Boundary);
    assert_region_location(&union, p(3, 1), RegionPointLocation::Inside);

    let Classification::Decided(intersection) = first
        .boolean_region(
            &second,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided intersection for reordered touching bins");
    };
    assert_eq!(intersection.material_contours().len(), 2);
    assert_eq!(intersection.hole_contours().len(), 0);

    let Classification::Decided(difference) = first
        .boolean_region(&second, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided difference for reordered touching bins");
    };
    assert!(difference.is_empty());
}

#[test]
fn region_boolean_region_identity_accepts_rotated_and_reversed_bins() {
    let first = touching_material_bins();
    let second = touching_material_bins_rotated_and_reversed();

    let Classification::Decided(union) = first
        .boolean_region(&second, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided union for rotated/reversed touching bins");
    };
    assert_eq!(union.material_contours().len(), 2);
    assert_eq!(union.hole_contours().len(), 0);
    assert_region_location(&union, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&union, p(2, 1), RegionPointLocation::Boundary);
    assert_region_location(&union, p(3, 1), RegionPointLocation::Inside);

    let Classification::Decided(xor) = first
        .boolean_region(&second, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided xor for rotated/reversed touching bins");
    };
    assert!(xor.is_empty());
}

#[test]
fn region_boolean_region_empty_identity_preserves_touching_material_bins() {
    let empty = Region2::<DefaultBackend>::empty();
    let touching = touching_material_bins();

    let Classification::Decided(union_left) = empty
        .boolean_region(&touching, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided empty union touching region");
    };
    assert_eq!(union_left.material_contours().len(), 2);
    assert_eq!(union_left.hole_contours().len(), 0);
    assert_region_location(&union_left, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&union_left, p(3, 1), RegionPointLocation::Inside);

    let Classification::Decided(union_right) = touching
        .boolean_region(&empty, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided touching union empty");
    };
    assert_eq!(union_right.material_contours().len(), 2);
    assert_eq!(union_right.hole_contours().len(), 0);

    let Classification::Decided(xor_left) = empty
        .boolean_region(&touching, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided empty xor touching region");
    };
    assert_eq!(xor_left.material_contours().len(), 2);

    let Classification::Decided(difference) = touching
        .boolean_region(&empty, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided touching difference empty");
    };
    assert_eq!(difference.material_contours().len(), 2);

    let Classification::Decided(empty_difference) = empty
        .boolean_region(
            &touching,
            BooleanOp::Difference,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided empty difference touching region");
    };
    assert!(empty_difference.is_empty());
}

#[test]
fn region_boolean_region_union_adds_island_inside_hole() {
    let ring = donut(rectangle(0, 0, 12, 12), rectangle(3, 3, 9, 9));
    let island = region(vec![rectangle(5, 5, 7, 7)]);

    let Classification::Decided(result) = ring
        .boolean_region(&island, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided donut union island");
    };

    assert_eq!(result.material_contours().len(), 2);
    assert_eq!(result.hole_contours().len(), 1);
    assert_region_location(&result, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&result, p(3, 3), RegionPointLocation::Boundary);
    assert_region_location(&result, p(4, 4), RegionPointLocation::Outside);
    assert_region_location(&result, p(6, 6), RegionPointLocation::Inside);
}

#[test]
fn region_boolean_region_difference_ignores_island_inside_hole() {
    let ring = donut(rectangle(0, 0, 10, 10), rectangle(3, 3, 7, 7));
    let island = region(vec![rectangle(4, 4, 6, 6)]);

    let Classification::Decided(result) = ring
        .boolean_region(&island, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided donut difference island-in-hole");
    };

    assert_eq!(result.material_contours().len(), 1);
    assert_eq!(result.hole_contours().len(), 1);
    assert_region_location(&result, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&result, p(5, 5), RegionPointLocation::Outside);
}

#[test]
fn region_boolean_region_intersection_with_island_inside_hole_is_empty() {
    let ring = donut(rectangle(0, 0, 10, 10), rectangle(3, 3, 7, 7));
    let island = region(vec![rectangle(4, 4, 6, 6)]);

    let Classification::Decided(result) = ring
        .boolean_region(
            &island,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided donut intersection island-in-hole");
    };

    assert!(result.is_empty());
}

#[test]
fn region_boolean_region_hole_boundary_cutter_union_clips_hole() {
    let ring = donut(rectangle(0, 0, 12, 12), rectangle(4, 4, 8, 8));
    let cutter = region(vec![rectangle(6, 2, 10, 10)]);

    let Classification::Decided(result) = ring
        .boolean_region(&cutter, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided donut union hole-boundary cutter");
    };

    assert_eq!(result.material_contours().len(), 1);
    assert_eq!(result.hole_contours().len(), 1);
    assert_region_location(&result, p(2, 2), RegionPointLocation::Inside);
    assert_region_location(&result, p(5, 5), RegionPointLocation::Outside);
    assert_region_location(&result, p(7, 7), RegionPointLocation::Inside);
    assert_region_location(&result, p(9, 9), RegionPointLocation::Inside);
    assert_region_location(&result, p(13, 13), RegionPointLocation::Outside);
}

#[test]
fn region_boolean_region_hole_boundary_cutter_intersection_keeps_notched_material() {
    let ring = donut(rectangle(0, 0, 12, 12), rectangle(4, 4, 8, 8));
    let cutter = region(vec![rectangle(6, 2, 10, 10)]);

    let Classification::Decided(result) = ring
        .boolean_region(
            &cutter,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided donut intersection hole-boundary cutter");
    };

    assert_eq!(result.material_contours().len(), 1);
    assert_eq!(result.hole_contours().len(), 0);
    assert_region_location(&result, p(2, 2), RegionPointLocation::Outside);
    assert_region_location(&result, p(7, 3), RegionPointLocation::Inside);
    assert_region_location(&result, p(7, 7), RegionPointLocation::Outside);
    assert_region_location(&result, p(9, 9), RegionPointLocation::Inside);
}

#[test]
fn region_boolean_region_hole_boundary_cutter_difference_merges_hole() {
    let ring = donut(rectangle(0, 0, 12, 12), rectangle(4, 4, 8, 8));
    let cutter = region(vec![rectangle(6, 2, 10, 10)]);

    let Classification::Decided(result) = ring
        .boolean_region(&cutter, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided donut difference hole-boundary cutter");
    };

    assert_eq!(result.material_contours().len(), 1);
    assert_eq!(result.hole_contours().len(), 1);
    assert_region_location(&result, p(2, 2), RegionPointLocation::Inside);
    assert_region_location(&result, p(7, 3), RegionPointLocation::Outside);
    assert_region_location(&result, p(7, 7), RegionPointLocation::Outside);
    assert_region_location(&result, p(9, 9), RegionPointLocation::Outside);
    assert_region_location(&result, p(11, 11), RegionPointLocation::Inside);
}

#[test]
fn region_boolean_region_hole_boundary_cutter_xor_keeps_nested_island() {
    let ring = donut(rectangle(0, 0, 12, 12), rectangle(4, 4, 8, 8));
    let cutter = region(vec![rectangle(6, 2, 10, 10)]);

    let xor_result = ring
        .boolean_region(&cutter, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap();
    let Classification::Decided(result) = xor_result else {
        panic!("expected decided donut xor hole-boundary cutter, got {xor_result:?}");
    };

    assert_eq!(result.material_contours().len(), 2);
    assert_eq!(result.hole_contours().len(), 1);
    assert_region_location(&result, p(2, 2), RegionPointLocation::Inside);
    assert_region_location(&result, p(5, 5), RegionPointLocation::Outside);
    assert_region_location(&result, p(7, 3), RegionPointLocation::Outside);
    assert_region_location(&result, p(7, 7), RegionPointLocation::Inside);
    assert_region_location(&result, p(9, 9), RegionPointLocation::Outside);
    assert_region_location(&result, p(11, 11), RegionPointLocation::Inside);
}

#[test]
fn region_boolean_boundary_contours_hole_boundary_cutter_xor_are_decided() {
    let ring = donut(rectangle(0, 0, 12, 12), rectangle(4, 4, 8, 8));
    let cutter = region(vec![rectangle(6, 2, 10, 10)]);

    let Classification::Decided(contours) = ring
        .boolean_boundary_contours(&cutter, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided xor boundary contours for hole-boundary cutter");
    };

    assert_eq!(contours.len(), 3);
    assert!(contours.iter().all(|contour| !contour.is_empty()));
}

#[test]
fn region_boolean_boundary_contours_identical_donut_are_decided() {
    let ring = donut(rectangle(0, 0, 10, 10), rectangle(3, 3, 7, 7));

    let Classification::Decided(contours) = ring
        .boolean_boundary_contours(&ring, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided donut self boundary contours");
    };

    assert_eq!(contours.len(), 2);
}

#[test]
fn region_boolean_boundary_contours_difference_overlapping_rectangles() {
    let first = region(vec![rectangle(0, 0, 4, 4)]);
    let second = region(vec![rectangle(2, -1, 6, 3)]);

    let Classification::Decided(contours) = first
        .boolean_boundary_contours(&second, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided difference boundary contours");
    };

    assert_eq!(contours.len(), 1);
    assert_contour_location(&contours[0], p(1, 1), ContourPointLocation::Inside);
    assert_contour_location(&contours[0], p(3, 1), ContourPointLocation::Outside);
}

#[test]
fn boundary_contour_nesting_alternates_material_hole_material() {
    let outer = rectangle(0, 0, 10, 10);
    let hole = rectangle(2, 2, 8, 8);
    let island = rectangle(4, 4, 6, 6);

    let Classification::Decided(result) =
        Region2::from_boundary_contours(vec![outer, hole, island], &policy()).unwrap()
    else {
        panic!("expected decided nested boundary region");
    };

    assert_eq!(result.material_contours().len(), 2);
    assert_eq!(result.hole_contours().len(), 1);
    assert_region_location(&result, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&result, p(3, 3), RegionPointLocation::Outside);
    assert_region_location(&result, p(5, 5), RegionPointLocation::Inside);
}

#[test]
fn boundary_contour_nesting_rejects_boundary_touching_loops() {
    let outer = rectangle(0, 0, 4, 4);
    let touching = rectangle(1, 0, 3, 2);

    assert_eq!(
        Region2::from_boundary_contours(vec![outer, touching], &policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn region_boolean_boundary_contours_disjoint_union_keeps_two_loops() {
    let first = region(vec![rectangle(0, 0, 2, 2)]);
    let second = region(vec![rectangle(4, 4, 6, 6)]);

    let Classification::Decided(contours) = first
        .boolean_boundary_contours(&second, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided disjoint union boundary contours");
    };

    assert_eq!(contours.len(), 2);
    assert!(
        contours
            .iter()
            .any(|contour| contour.classify_point(&p(1, 1), &policy())
                == Classification::Decided(ContourPointLocation::Inside))
    );
    assert!(
        contours
            .iter()
            .any(|contour| contour.classify_point(&p(5, 5), &policy())
                == Classification::Decided(ContourPointLocation::Inside))
    );
}

#[test]
fn region_boolean_boundary_contours_disjoint_intersection_is_empty() {
    let first = region(vec![rectangle(0, 0, 2, 2)]);
    let second = region(vec![rectangle(4, 4, 6, 6)]);

    let Classification::Decided(contours) = first
        .boolean_boundary_contours(
            &second,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided empty intersection boundary contours");
    };

    assert!(contours.is_empty());
}

#[test]
fn region_boolean_region_point_touching_rectangles_use_regularized_identities() {
    let first = region(vec![rectangle(0, 0, 2, 2)]);
    let second = region(vec![rectangle(2, 2, 4, 4)]);

    let Classification::Decided(union) = first
        .boolean_region(&second, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided point-touch union");
    };
    assert_eq!(union.material_contours().len(), 2);
    assert_eq!(union.hole_contours().len(), 0);
    assert_region_location(&union, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&union, p(2, 2), RegionPointLocation::Boundary);
    assert_region_location(&union, p(3, 3), RegionPointLocation::Inside);

    let Classification::Decided(intersection) = first
        .boolean_region(
            &second,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided point-touch intersection");
    };
    assert!(intersection.is_empty());

    let Classification::Decided(difference) = first
        .boolean_region(&second, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided point-touch difference");
    };
    assert_eq!(difference.material_contours().len(), 1);
    assert_region_location(&difference, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&difference, p(3, 3), RegionPointLocation::Outside);

    let Classification::Decided(xor) = first
        .boolean_region(&second, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided point-touch xor");
    };
    assert_eq!(xor.material_contours().len(), 2);
    assert_region_location(&xor, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&xor, p(3, 3), RegionPointLocation::Inside);
}

#[test]
fn region_boolean_boundary_contours_point_touching_rectangles_are_decided() {
    let first = region(vec![rectangle(0, 0, 2, 2)]);
    let second = region(vec![rectangle(2, 2, 4, 4)]);

    let Classification::Decided(union) = first
        .boolean_boundary_contours(&second, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided point-touch union contours");
    };
    assert_eq!(union.len(), 2);

    let Classification::Decided(intersection) = first
        .boolean_boundary_contours(
            &second,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided point-touch intersection contours");
    };
    assert!(intersection.is_empty());

    let Classification::Decided(difference) = first
        .boolean_boundary_contours(&second, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided point-touch difference contours");
    };
    assert_eq!(difference.len(), 1);

    let Classification::Decided(xor) = first
        .boolean_boundary_contours(&second, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided point-touch xor contours");
    };
    assert_eq!(xor.len(), 2);
}

#[test]
fn region_boolean_region_shared_edge_rectangles_are_regularized() {
    let first = region(vec![rectangle(0, 0, 4, 4)]);
    let second = region(vec![rectangle(2, -2, 6, 0)]);

    let Classification::Decided(union) = first
        .boolean_region(&second, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided shared-edge union");
    };
    assert_eq!(union.material_contours().len(), 1);
    assert_eq!(union.hole_contours().len(), 0);
    assert_region_location(&union, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&union, p(3, -1), RegionPointLocation::Inside);
    assert_region_location(&union, p(3, 0), RegionPointLocation::Inside);
    assert_region_location(&union, p(5, 1), RegionPointLocation::Outside);

    let Classification::Decided(intersection) = first
        .boolean_region(
            &second,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided shared-edge intersection");
    };
    assert!(intersection.is_empty());

    let Classification::Decided(difference) = first
        .boolean_region(&second, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided shared-edge difference");
    };
    assert_eq!(difference.material_contours().len(), 1);
    assert_region_location(&difference, p(1, 1), RegionPointLocation::Inside);
    assert_region_location(&difference, p(3, -1), RegionPointLocation::Outside);
    assert_region_location(&difference, p(3, 0), RegionPointLocation::Boundary);

    let Classification::Decided(xor) = first
        .boolean_region(&second, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided shared-edge xor");
    };
    assert_eq!(xor.material_contours().len(), 1);
    assert_region_location(&xor, p(3, 0), RegionPointLocation::Inside);
}

#[test]
fn region_boolean_boundary_contours_shared_edge_rectangles_are_regularized() {
    let first = region(vec![rectangle(0, 0, 4, 4)]);
    let second = region(vec![rectangle(2, -2, 6, 0)]);

    let Classification::Decided(union) = first
        .boolean_boundary_contours(&second, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided shared-edge union contours");
    };
    assert_eq!(union.len(), 1);

    let Classification::Decided(intersection) = first
        .boolean_boundary_contours(
            &second,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected decided shared-edge intersection contours");
    };
    assert!(intersection.is_empty());

    let Classification::Decided(difference) = first
        .boolean_boundary_contours(&second, BooleanOp::Difference, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided shared-edge difference contours");
    };
    assert_eq!(difference.len(), 1);

    let Classification::Decided(xor) = first
        .boolean_boundary_contours(&second, BooleanOp::Xor, FillRule::NonZero, &policy())
        .unwrap()
    else {
        panic!("expected decided shared-edge xor contours");
    };
    assert_eq!(xor.len(), 1);
}

#[test]
fn region_boolean_region_boundary_overlap_with_interior_containment_still_defers() {
    let outer = region(vec![rectangle(0, 0, 6, 6)]);
    let inner_touching_edge = region(vec![rectangle(2, 0, 4, 2)]);

    assert_eq!(
        outer
            .boolean_region(
                &inner_touching_edge,
                BooleanOp::Union,
                FillRule::NonZero,
                &policy(),
            )
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn region_boolean_boundary_pipeline_defers_shared_edges() {
    let first = region(vec![rectangle(0, 0, 4, 4)]);
    let second = region(vec![rectangle(2, -2, 6, 0)]);

    assert_eq!(
        first
            .boolean_boundary_loops(&second, BooleanOp::Union, &policy())
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn region_boolean_boundary_pipeline_rejects_point_touch_branch_vertices() {
    let first = region(vec![rectangle(0, 0, 2, 2)]);
    let second = region(vec![rectangle(2, 2, 4, 4)]);

    assert_eq!(
        first
            .boolean_boundary_loops(&second, BooleanOp::Union, &policy())
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}
