//! Boolean outputs remain decidable inputs to further strict operations.
use hyperlattice::Matrix4;
use hypermesh::{
    BooleanOp, BooleanProgram, MeshCertainty, MeshContext, PairwiseIntersection, Point3,
    PredicatePolicy, Real, Triangle, TriangleMesh, convex_triangle, intersect_polygons,
};

const STRICT: MeshContext = MeshContext::new(PredicatePolicy::STRICT);

fn q(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn exact_box(min: [Real; 3], max: [Real; 3]) -> TriangleMesh {
    let corner = |x: usize, y: usize, z: usize| {
        let pick = |axis: usize, high: usize| {
            if high == 1 {
                max[axis].clone()
            } else {
                min[axis].clone()
            }
        };
        Point3::new(pick(0, x), pick(1, y), pick(2, z))
    };
    TriangleMesh::new(
        vec![
            corner(0, 0, 0),
            corner(1, 0, 0),
            corner(1, 1, 0),
            corner(0, 1, 0),
            corner(0, 0, 1),
            corner(1, 0, 1),
            corner(1, 1, 1),
            corner(0, 1, 1),
        ],
        [
            [4, 5, 6],
            [4, 6, 7],
            [0, 3, 2],
            [0, 2, 1],
            [1, 2, 6],
            [1, 6, 5],
            [0, 4, 7],
            [0, 7, 3],
            [3, 7, 6],
            [3, 6, 2],
            [0, 1, 5],
            [0, 5, 4],
        ]
        .into_iter()
        .map(|[a, b, c]| Triangle::new(a, b, c))
        .collect(),
    )
}

fn difference(left: &TriangleMesh, right: &TriangleMesh) -> TriangleMesh {
    let outcome = hypermesh::boolean(
        &STRICT,
        &[left.as_ref(), right.as_ref()],
        BooleanProgram::Operation(BooleanOp::Difference),
    )
    .unwrap();
    assert_eq!(outcome.certainty, MeshCertainty::Certified);
    outcome.value.into_triangle_meshes().unwrap().pop().unwrap()
}

/// A box rotated about z by the exact rational rotation with cosine 3/5.
fn rotated_box() -> TriangleMesh {
    let [c, s] = [q(3, 5), q(4, 5)];
    let zero = Real::zero;
    let rotation = Matrix4::from_row_major([
        c.clone(),
        -s.clone(),
        zero(),
        zero(),
        s,
        c,
        zero(),
        zero(),
        zero(),
        zero(),
        Real::one(),
        zero(),
        zero(),
        zero(),
        zero(),
        Real::one(),
    ]);
    exact_box([0, 0, 0].map(Real::from), [4, 4, 2].map(Real::from))
        .try_transformed(&STRICT, &rotation)
        .unwrap()
        .into_value()
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

#[test]
fn retained_arrangement_structure_does_not_change_later_outputs() {
    // A Boolean output carries the arrangement's proof that its triangles meet
    // only in shared vertices or edges, so a later Boolean skips testing it
    // against itself. Rebuilding the same rows drops that retained proof and
    // forces the full self-test; both paths must publish identical rows.
    let first_cut = difference(
        &rotated_box(),
        &exact_box(
            [Real::from(-10), Real::from(-10), Real::from(-1)],
            [Real::one(), Real::from(10), Real::from(3)],
        ),
    );
    let rebuilt = TriangleMesh::new(first_cut.positions.to_vec(), first_cut.triangles.to_vec());
    let cutter = exact_box(
        [q(3, 2), Real::from(-10), q(1, 2)],
        [Real::from(2), Real::from(10), Real::from(3)],
    );
    let retained = difference(&first_cut, &cutter);
    let recomputed = difference(&rebuilt, &cutter);
    assert_eq!(retained.positions, recomputed.positions);
    assert_eq!(retained.triangles, recomputed.triangles);
    let closed = retained.is_closed_manifold_geometry(&STRICT).unwrap();
    assert_eq!(closed.certainty, MeshCertainty::Certified);
    assert!(closed.value);
}
