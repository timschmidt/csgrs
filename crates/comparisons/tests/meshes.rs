use exactcore_hyper_comparison::{plane3_relation, triangle3_relation};
use hyperlattice::HomogeneousPoint3;
use hypermesh::{
    Classification, ConvexPolygon, MeshCertainty, MeshContext, PairwiseIntersection, Plane, Point3,
    PredicatePolicy, Real, convex_triangle, intersect_polygons,
};
use proptest::prelude::*;
use std::cmp::Ordering;

const CONTEXT: MeshContext = MeshContext::new(PredicatePolicy::STRICT);

fn r(value: i64) -> Real {
    value.into()
}

fn p(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn homogeneous(point: &Point3) -> HomogeneousPoint3 {
    HomogeneousPoint3::new(
        point.x.clone(),
        point.y.clone(),
        point.z.clone(),
        Real::one(),
    )
}

fn triangle(a: Point3, b: Point3, c: Point3, polygon_index: isize) -> ConvexPolygon {
    let outcome = convex_triangle(&CONTEXT, &a, &b, &c, 0, polygon_index).unwrap();
    assert_eq!(outcome.certainty, MeshCertainty::Certified);
    outcome.into_value()
}

fn core_triangle_coordinates(first: [[i64; 3]; 3], second: [[i64; 3]; 3]) -> [i64; 18] {
    [
        first[0][0],
        first[0][1],
        first[0][2],
        first[1][0],
        first[1][1],
        first[1][2],
        first[2][0],
        first[2][1],
        first[2][2],
        second[0][0],
        second[0][1],
        second[0][2],
        second[1][0],
        second[1][1],
        second[1][2],
        second[2][0],
        second[2][1],
        second[2][2],
    ]
}

fn classification_sign(classification: Classification) -> i32 {
    match classification {
        Classification::Negative => -1,
        Classification::On => 0,
        Classification::Positive => 1,
    }
}

#[test]
fn plane_construction_expression_and_classification_match() {
    let a = p(1, -2, 3);
    let b = p(7, 1, 4);
    let c = p(-3, 6, 2);
    let plane = Plane::from_points(&a, &b, &c);
    assert!(plane.is_valid(&CONTEXT).unwrap().into_value());

    for query in [a.clone(), b.clone(), c.clone(), p(2, 3, 11), p(-5, 0, -7)] {
        let coordinates = [
            1,
            -2,
            3,
            7,
            1,
            4,
            -3,
            6,
            2,
            query.x.to_f64_lossy().unwrap() as i64,
            query.y.to_f64_lossy().unwrap() as i64,
            query.z.to_f64_lossy().unwrap() as i64,
            0,
            0,
            0,
            0,
            0,
            0,
        ];
        let classification = hypermesh::classify_point(&CONTEXT, &query, &plane)
            .unwrap()
            .into_value();
        assert_eq!(
            plane3_relation(1, &coordinates),
            classification_sign(classification)
        );
        assert_eq!(
            plane3_relation(0, &coordinates) == 1,
            classification == Classification::On
        );
        let expression_sign = match plane
            .expression_at_point(&query)
            .partial_cmp(&Real::zero())
            .unwrap()
        {
            Ordering::Less => -1,
            Ordering::Equal => 0,
            Ordering::Greater => 1,
        };
        assert_eq!(expression_sign, classification_sign(classification));
    }
}

#[test]
fn inverted_plane_matches_reversed_core_support_orientation() {
    let a = p(0, 0, 0);
    let b = p(6, 0, 0);
    let c = p(0, 6, 0);
    let query = p(1, 1, 5);
    let plane = Plane::from_points(&a, &b, &c);
    let forward = hypermesh::classify_point(&CONTEXT, &query, &plane)
        .unwrap()
        .into_value();
    let reverse = hypermesh::classify_point(&CONTEXT, &query, &plane.inverted())
        .unwrap()
        .into_value();
    assert_eq!(classification_sign(reverse), -classification_sign(forward));

    let forward_coordinates = [0, 0, 0, 6, 0, 0, 0, 6, 0, 1, 1, 5, 0, 0, 0, 0, 0, 0];
    let reverse_coordinates = [0, 0, 0, 0, 6, 0, 6, 0, 0, 1, 1, 5, 0, 0, 0, 0, 0, 0];
    assert_eq!(
        plane3_relation(1, &reverse_coordinates),
        -plane3_relation(1, &forward_coordinates)
    );
}

#[test]
fn convex_triangle_structure_and_point_topology_match() {
    let polygon = triangle(p(0, 0, 0), p(6, 0, 0), p(0, 6, 0), 0);
    assert_eq!(polygon.vertex_count(), 3);
    assert!(polygon.is_valid(&CONTEXT).unwrap().into_value());
    assert_eq!(
        polygon.vertices(&CONTEXT).unwrap().into_value(),
        vec![p(0, 0, 0), p(6, 0, 0), p(0, 6, 0),]
    );

    let cases = [
        ([1, 1, 0], true, true, false),
        ([3, 0, 0], true, false, true),
        ([0, 0, 0], true, false, true),
        ([5, 5, 0], false, false, false),
        ([1, 1, 2], false, false, false),
    ];
    for ([x, y, z], expected_contains, expected_inside, expected_edge) in cases {
        let query = p(x, y, z);
        let projective = homogeneous(&query);
        assert_eq!(
            polygon
                .contains_point(&CONTEXT, &projective)
                .unwrap()
                .into_value(),
            expected_contains
        );
        assert_eq!(
            polygon
                .contains_point_strictly(&CONTEXT, &projective)
                .unwrap()
                .into_value(),
            expected_inside
        );
        let coordinates = [0, 0, 0, 6, 0, 0, 0, 6, 0, x, y, z, 0, 0, 0, 0, 0, 0];
        assert_eq!(triangle3_relation(0, &coordinates) == 1, z == 0);
        if z == 0 {
            assert_eq!(
                triangle3_relation(1, &coordinates) == 1,
                expected_contains,
                "CORE contains at ({x}, {y}, {z})"
            );
            assert_eq!(
                triangle3_relation(2, &coordinates) == 1,
                expected_edge,
                "CORE edge at ({x}, {y}, {z})"
            );
            assert_eq!(
                triangle3_relation(3, &coordinates) == 1,
                expected_inside,
                "CORE inside at ({x}, {y}, {z})"
            );
        }
    }
}

#[test]
fn convex_triangle_inversion_preserves_closed_set_and_reverses_support() {
    let polygon = triangle(p(0, 0, 0), p(6, 0, 0), p(0, 6, 0), 0);
    let inverted = polygon.inverted();
    for query in [p(1, 1, 0), p(3, 0, 0), p(5, 5, 0), p(1, 1, 2)] {
        let projective = homogeneous(&query);
        assert_eq!(
            polygon
                .contains_point(&CONTEXT, &projective)
                .unwrap()
                .into_value(),
            inverted
                .contains_point(&CONTEXT, &projective)
                .unwrap()
                .into_value()
        );
    }
    let above = p(1, 1, 2);
    let forward = hypermesh::classify_point(&CONTEXT, &above, polygon.support_plane())
        .unwrap()
        .into_value();
    let reverse = hypermesh::classify_point(&CONTEXT, &above, inverted.support_plane())
        .unwrap()
        .into_value();
    assert_eq!(classification_sign(reverse), -classification_sign(forward));
}

#[test]
fn pairwise_triangle_intersection_topology_matches_for_contacts() {
    let base_points = [[0, 0, 0], [6, 0, 0], [0, 6, 0]];
    let base = triangle(p(0, 0, 0), p(6, 0, 0), p(0, 6, 0), 0);
    // CORE's coplanar clipping path returns pointers owned by temporary
    // Polygon3d values and can segfault. Keep the live differential call on
    // the non-coplanar path; the unsafe coplanar cases are recorded in the
    // coverage manifest instead of being invoked by a test process.
    let cases = [([[1, 1, -2], [1, 1, 2], [4, 1, 0]], "non-coplanar segment")];

    for (other_points, context) in cases {
        let other = triangle(
            p(other_points[0][0], other_points[0][1], other_points[0][2]),
            p(other_points[1][0], other_points[1][1], other_points[1][2]),
            p(other_points[2][0], other_points[2][1], other_points[2][2]),
            1,
        );
        let relation = intersect_polygons(&CONTEXT, &base, &other, 1)
            .unwrap()
            .into_value();
        assert!(
            !matches!(relation, PairwiseIntersection::Disjoint),
            "{context}: relation was {relation:?}"
        );
        assert_eq!(
            triangle3_relation(6, &core_triangle_coordinates(base_points, other_points)),
            1,
            "{context}"
        );
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(192))]

    #[test]
    fn integer_triangle_point_classification_matches(x in -4_i64..=10, y in -4_i64..=10, z in -2_i64..=2) {
        let polygon = triangle(p(0, 0, 0), p(6, 0, 0), p(0, 6, 0), 0);
        let query = p(x, y, z);
        let projective = homogeneous(&query);
        let hyper_contains = polygon.contains_point(&CONTEXT, &projective).unwrap().into_value();
        let hyper_inside = polygon.contains_point_strictly(&CONTEXT, &projective).unwrap().into_value();
        let coordinates = [0, 0, 0, 6, 0, 0, 0, 6, 0, x, y, z, 0, 0, 0, 0, 0, 0];
        prop_assert_eq!(triangle3_relation(0, &coordinates) == 1, z == 0);
        if z == 0 {
            // CORE treats any point on an edge's infinite supporting line as
            // contained, even beyond the finite edge endpoints.
            if hyper_contains || (x != 0 && y != 0 && x + y != 6) {
                prop_assert_eq!(triangle3_relation(1, &coordinates) == 1, hyper_contains);
            }
            prop_assert_eq!(triangle3_relation(3, &coordinates) == 1, hyper_inside);
        }
    }
}

#[test]
#[ignore = "known exactCorelib Triangle3d::contains omits its documented coplanarity precondition"]
fn off_plane_triangle_point_containment_matches() {
    let polygon = triangle(p(0, 0, 0), p(6, 0, 0), p(0, 6, 0), 0);
    let query = p(1, 1, 2);
    assert!(
        !polygon
            .contains_point(&CONTEXT, &homogeneous(&query))
            .unwrap()
            .into_value()
    );
    assert_eq!(
        triangle3_relation(1, &[0, 0, 0, 6, 0, 0, 0, 6, 0, 1, 1, 2, 0, 0, 0, 0, 0, 0]),
        0
    );
}

#[test]
#[ignore = "known exactCorelib Triangle3d::contains accepts extensions of finite edges"]
fn outside_point_on_edge_support_line_is_not_contained() {
    let polygon = triangle(p(0, 0, 0), p(6, 0, 0), p(0, 6, 0), 0);
    let query = p(-3, 9, 0);
    assert!(
        !polygon
            .contains_point(&CONTEXT, &homogeneous(&query))
            .unwrap()
            .into_value()
    );
    assert_eq!(
        triangle3_relation(1, &[0, 0, 0, 6, 0, 0, 0, 6, 0, -3, 9, 0, 0, 0, 0, 0, 0, 0]),
        0
    );
}

#[test]
#[ignore = "known exactCorelib Triangle3d do_intersect converts disjoint dimension -1 to true"]
fn disjoint_triangle_boolean_intersection_matches() {
    let base_points = [[0, 0, 0], [6, 0, 0], [0, 6, 0]];
    let other_points = [[10, 10, 0], [14, 10, 0], [10, 14, 0]];
    let base = triangle(p(0, 0, 0), p(6, 0, 0), p(0, 6, 0), 0);
    let other = triangle(p(10, 10, 0), p(14, 10, 0), p(10, 14, 0), 1);
    assert!(matches!(
        intersect_polygons(&CONTEXT, &base, &other, 1)
            .unwrap()
            .into_value(),
        PairwiseIntersection::Disjoint
    ));
    assert_eq!(
        triangle3_relation(6, &core_triangle_coordinates(base_points, other_points)),
        0
    );
}
