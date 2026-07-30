#![no_main]

mod support;

use std::collections::BTreeSet;

use hypermesh::bvh::bounds_overlap;
use hypermesh::{
    ApproxBounds, Classification, ExactBvh, ExactPointBvh, Plane, Point3, Real, classify_point,
    convex_triangle,
};
use libfuzzer_sys::fuzz_target;
use support::{CONTEXT, value};

fn r(value: i64) -> Real {
    Real::from(value)
}

fn p(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fuzz_target!(|data: [u8; 33]| {
    let count = usize::from(data[0] % 16) + 1;
    let polygons = (0..count)
        .map(|index| {
            let x = i64::from(data[index * 2 + 1] % 17) - 8;
            let y = i64::from(data[index * 2 + 2] % 17) - 8;
            value(convex_triangle(
                &CONTEXT,
                &p(x, y, 0),
                &p(x + 2, y, 0),
                &p(x, y + 2, 0),
                0,
                index as isize,
            ))
            .unwrap()
        })
        .collect::<Vec<_>>();
    let bvh = value(ExactBvh::build(&CONTEXT, &polygons)).unwrap();
    assert_eq!(bvh.len(), polygons.len());
    assert_eq!(bvh.is_empty(), polygons.is_empty());
    assert!(bvh.node_count() > 0);
    assert_eq!(bvh.primitives().len(), polygons.len());

    let query_min = p(-2, -2, -1);
    let query_max = p(2, 2, 1);
    let query = ApproxBounds::new(query_min, query_max);
    let mut actual = Vec::new();
    value(bvh.query_bounds(&CONTEXT, &query, |index| actual.push(index))).unwrap();
    let expected = bvh
        .primitives()
        .iter()
        .filter(|primitive| value(bounds_overlap(&CONTEXT, &primitive.bounds, &query)).unwrap())
        .map(|primitive| primitive.polygon_index)
        .collect::<Vec<_>>();
    assert_eq!(actual, expected);

    let mut pairs = BTreeSet::new();
    value(bvh.intersect_pairs(&CONTEXT, &bvh, |left, right| {
        pairs.insert((left, right));
    }))
    .unwrap();
    assert!(
        pairs
            .iter()
            .all(|(left, right)| *left < count && *right < count)
    );

    let points = polygons
        .iter()
        .map(|polygon| polygon.vertices(&CONTEXT).unwrap().into_value()[0].clone())
        .collect::<Vec<_>>();
    let point_bvh = value(ExactPointBvh::build(&CONTEXT, &points)).unwrap();
    assert_eq!(point_bvh.len(), points.len());
    assert_eq!(point_bvh.is_empty(), points.is_empty());
    assert!(point_bvh.node_count() > 0);

    let plane = Plane::axis_aligned(0, r(0));
    let mut positive = Vec::new();
    let positive_query =
        point_bvh.query_positive_halfspace(&CONTEXT, &points, &plane, |index| positive.push(index));
    value(positive_query).unwrap();
    positive.sort_unstable();
    let expected_positive = points
        .iter()
        .enumerate()
        .filter(|(_, point)| {
            value(classify_point(&CONTEXT, point, &plane)).unwrap() == Classification::Positive
        })
        .map(|(index, _)| index)
        .collect::<Vec<_>>();
    assert_eq!(positive, expected_positive);

    let a = p(0, 0, 0);
    let b = p(1, 0, 0);
    let c = p(0, 1, 0);
    let mut oriented_positive = BTreeSet::new();
    value(
        point_bvh.query_positive_oriented_plane(&CONTEXT, &points, &a, &b, &c, |index| {
            assert!(oriented_positive.insert(index));
        }),
    )
    .unwrap();
    let mut oriented_negative = BTreeSet::new();
    value(
        point_bvh.query_negative_oriented_plane(&CONTEXT, &points, &a, &b, &c, |index| {
            assert!(oriented_negative.insert(index));
        }),
    )
    .unwrap();
    assert!(oriented_positive.is_disjoint(&oriented_negative));
});
