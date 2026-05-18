use hyperphysics::{
    AxisAlignedBox3, BoxPointClassification, PhysicsError, PhysicsShape3, Plane3,
    PlanePointClassification, Ray3, RayPlaneClassification, Real, Segment3,
    SegmentPlaneClassification, Triangle3, TrianglePointClassification, Vector3,
};
use proptest::prelude::*;

fn r(value: i32) -> Real {
    value.into()
}

fn v(x: i32, y: i32, z: i32) -> Vector3 {
    Vector3::new([r(x), r(y), r(z)])
}

#[test]
fn exact_box_classifies_inside_boundary_and_outside_points() {
    let aabb = AxisAlignedBox3::new(v(0, 0, 0), v(10, 10, 10)).unwrap();

    assert_eq!(
        aabb.classify_point(&v(5, 5, 5)).unwrap(),
        BoxPointClassification::Inside
    );
    assert_eq!(
        aabb.classify_point(&v(0, 5, 5)).unwrap(),
        BoxPointClassification::Boundary
    );
    assert_eq!(
        aabb.classify_point(&v(11, 5, 5)).unwrap(),
        BoxPointClassification::Outside
    );
}

#[test]
fn exact_box_disjoint_rejects_only_strict_separation() {
    let left = AxisAlignedBox3::new(v(0, 0, 0), v(1, 1, 1)).unwrap();
    let touching = AxisAlignedBox3::new(v(1, 0, 0), v(2, 1, 1)).unwrap();
    let separated = AxisAlignedBox3::new(v(2, 0, 0), v(3, 1, 1)).unwrap();

    assert!(!left.certified_disjoint(&touching).unwrap());
    assert!(left.certified_disjoint(&separated).unwrap());
}

#[test]
fn support_map_selects_extreme_corner_and_reports_zero_direction() {
    let aabb = AxisAlignedBox3::new(v(-1, -2, -3), v(4, 5, 6)).unwrap();
    let support = aabb.support_map(v(1, -1, 0)).unwrap();
    assert_eq!(support.support_point, Vector3::new([r(4), r(-2), r(-3)]));
    assert!(!support.zero_direction);

    let zero = aabb.support_map(v(0, 0, 0)).unwrap();
    assert_eq!(zero.support_point, Vector3::new([r(-1), r(-2), r(-3)]));
    assert!(zero.zero_direction);
}

#[test]
fn shape_classification_preserves_box_facts_and_rejects_invalid_bounds() {
    assert_eq!(
        AxisAlignedBox3::new(v(2, 0, 0), v(1, 1, 1)).unwrap_err(),
        PhysicsError::InvalidAxisAlignedBox
    );
    let shape =
        PhysicsShape3::axis_aligned_box(AxisAlignedBox3::new(v(0, 0, 0), v(1, 1, 1)).unwrap());
    let report = shape.classification_report();
    assert_eq!(report.family, "axis-aligned-box");
    assert!(report.convex);
    assert!(report.axis_aligned);
    assert!(report.exact_support_map);
}

#[test]
fn plane_classifies_points_rays_and_segments_exactly() {
    let plane = Plane3::new(v(0, 0, 0), v(0, 0, 1));

    assert_eq!(
        plane.classify_point(&v(0, 0, 5)).unwrap().classification,
        PlanePointClassification::Positive
    );
    assert_eq!(
        plane
            .classify_ray(&Ray3::new(v(0, 0, 5), v(0, 0, -1)))
            .unwrap()
            .classification,
        RayPlaneClassification::ForwardIntersection
    );
    assert_eq!(
        plane
            .classify_ray(&Ray3::new(v(0, 0, 5), v(1, 0, 0)))
            .unwrap()
            .classification,
        RayPlaneClassification::Parallel
    );
    assert_eq!(
        plane
            .classify_segment(&Segment3::new(v(0, 0, 5), v(0, 0, -5)))
            .unwrap()
            .classification,
        SegmentPlaneClassification::Crosses
    );
    assert_eq!(
        plane
            .classify_segment(&Segment3::new(v(0, 0, 5), v(1, 0, 5)))
            .unwrap()
            .classification,
        SegmentPlaneClassification::PositiveSide
    );
}

#[test]
fn triangle_classifies_coplanar_boundary_outside_and_off_plane_points() {
    let triangle = Triangle3::new([v(0, 0, 0), v(10, 0, 0), v(0, 10, 0)]);

    assert_eq!(
        triangle.classify_point(&v(2, 3, 0)).unwrap().classification,
        TrianglePointClassification::Inside
    );
    assert_eq!(
        triangle.classify_point(&v(5, 0, 0)).unwrap().classification,
        TrianglePointClassification::Boundary
    );
    assert_eq!(
        triangle.classify_point(&v(8, 8, 0)).unwrap().classification,
        TrianglePointClassification::Outside
    );
    assert_eq!(
        triangle.classify_point(&v(2, 3, 1)).unwrap().classification,
        TrianglePointClassification::OffPlane
    );

    let degenerate = Triangle3::new([v(0, 0, 0), v(1, 1, 1), v(2, 2, 2)]);
    assert_eq!(
        degenerate
            .classify_point(&v(0, 0, 0))
            .unwrap()
            .classification,
        TrianglePointClassification::DegenerateTriangle
    );
}

proptest! {
    #[test]
    fn generated_points_inside_unit_box_are_not_outside(x in 0_i32..=10, y in 0_i32..=10, z in 0_i32..=10) {
        let aabb = AxisAlignedBox3::new(v(0, 0, 0), v(10, 10, 10)).unwrap();
        let class = aabb.classify_point(&v(x, y, z)).unwrap();
        prop_assert_ne!(class, BoxPointClassification::Outside);
    }

    #[test]
    fn generated_vertical_ray_intersections_have_expected_parameter(height in 1_i32..1000) {
        let plane = Plane3::new(v(0, 0, 0), v(0, 0, 1));
        let ray = Ray3::new(v(0, 0, height), v(0, 0, -1));

        let report = plane.classify_ray(&ray).unwrap();

        prop_assert_eq!(report.classification, RayPlaneClassification::ForwardIntersection);
        prop_assert_eq!(report.parameter, Some(r(height)));
    }

    #[test]
    fn generated_lattice_points_inside_reference_triangle_are_not_outside(x in 0_i32..=10, y in 0_i32..=10) {
        let triangle = Triangle3::new([v(0, 0, 0), v(10, 0, 0), v(0, 10, 0)]);
        let report = triangle.classify_point(&v(x, y, 0)).unwrap();

        if x + y <= 10 {
            prop_assert_ne!(report.classification, TrianglePointClassification::Outside);
            prop_assert_ne!(report.classification, TrianglePointClassification::OffPlane);
        }
    }
}
