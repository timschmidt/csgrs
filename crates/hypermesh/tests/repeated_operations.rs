//! Boolean outputs remain decidable inputs to further strict operations.
use hypermesh::{
    MeshCertainty, MeshContext, PairwiseIntersection, Point3, PredicatePolicy, Real,
    convex_triangle, intersect_polygons,
};

const STRICT: MeshContext = MeshContext::new(PredicatePolicy::STRICT);

fn q(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

#[test]
fn axis_plane_crossings_keep_the_plane_coordinate() {
    let sine = (Real::pi() / Real::from(18)).unwrap().sin();
    let host = convex_triangle(
        &STRICT,
        &Point3::new(sine.clone(), Real::zero(), Real::zero()),
        &Point3::new(&sine + &Real::from(2), Real::one(), Real::zero()),
        &Point3::new(sine.clone(), Real::from(2), Real::zero()),
        0,
        0,
    )
    .unwrap()
    .into_value();
    let half = q(1, 2);
    let wall = convex_triangle(
        &STRICT,
        &Point3::new(half.clone(), Real::from(-4), Real::from(-1)),
        &Point3::new(half.clone(), Real::from(4), Real::from(-1)),
        &Point3::new(half.clone(), Real::zero(), Real::from(1)),
        1,
        0,
    )
    .unwrap()
    .into_value();
    let outcome = intersect_polygons(&STRICT, &host, &wall, 1).unwrap();
    assert_eq!(outcome.certainty, MeshCertainty::Certified);
    let PairwiseIntersection::NonCoplanarSegment(segment) = outcome.value else {
        panic!(
            "the wall plane crosses the host triangle: {:?}",
            outcome.value
        );
    };
    for point in [&segment.v0, &segment.v1] {
        // Interpolating the symbolic endpoints would yield an expression equal
        // to 1/2 whose difference from the plane constant has no cheap proof.
        assert_eq!(point.x.exact_rational_ref(), half.exact_rational_ref());
    }
}
