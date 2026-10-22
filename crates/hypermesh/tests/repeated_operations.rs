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

fn operation(left: &TriangleMesh, right: &TriangleMesh, op: BooleanOp) -> TriangleMesh {
    let outcome = hypermesh::boolean(
        &STRICT,
        &[left.as_ref(), right.as_ref()],
        BooleanProgram::Operation(op),
    )
    .unwrap();
    assert_eq!(outcome.certainty, MeshCertainty::Certified);
    outcome.value.into_triangle_meshes().unwrap().pop().unwrap()
}

fn integer_box(min: [i64; 3], max: [i64; 3]) -> TriangleMesh {
    exact_box(min.map(Real::from), max.map(Real::from))
}

/// Exact enclosed volume, six times over: the sum of the triangles'
/// determinants. An independent check of the represented set's measure.
fn six_volume(mesh: &TriangleMesh) -> Real {
    mesh.triangles.iter().fold(Real::zero(), |sum, triangle| {
        let [a, b, c] = triangle.indices().map(|index| &mesh.positions[index]);
        let cross = [
            &b.y * &c.z - &b.z * &c.y,
            &b.z * &c.x - &b.x * &c.z,
            &b.x * &c.y - &b.y * &c.x,
        ];
        sum + &a.x * &cross[0] + &a.y * &cross[1] + &a.z * &cross[2]
    })
}

#[test]
fn retained_nonmanifold_outputs_match_rebuilt_ones_in_later_operations() {
    // Unions of boxes that touch along an edge, at a vertex, and along an
    // edge from both sides (a balanced four-sheet edge), each a certified
    // embedded output with non-manifold features. Later Booleans of the
    // retained output, and of the same rows rebuilt without retained
    // structure, must publish identical rows, including with the output as
    // both operands, and the volumes the boxes give independently.
    let block = integer_box([0, 0, 0], [2, 2, 2]);
    let touching = [
        ("edge", vec![integer_box([2, 2, 0], [4, 4, 2])], 16),
        ("vertex", vec![integer_box([2, 2, 2], [4, 4, 4])], 16),
        (
            "balanced edge",
            vec![
                integer_box([2, 2, 0], [4, 4, 2]),
                integer_box([-2, -2, 0], [0, 0, 2]),
            ],
            24,
        ),
    ];
    // Overlaps every touching feature: it contains the edge x = y = 2 for
    // 0 < z < 2, the vertex (2, 2, 2) and the edge x = y = 0.
    let cutter = integer_box([-1, -1, 1], [3, 3, 3]);
    for (name, others, volume) in touching {
        let mut body = block.clone();
        for other in &others {
            body = operation(&body, other, BooleanOp::Union);
        }
        assert_eq!(six_volume(&body), Real::from(6 * volume), "{name}");
        // The edge contacts have edges of four triangles. The vertex contact
        // keeps every edge manifold and shares one vertex row.
        let manifold = body.is_closed_manifold_geometry(&STRICT).unwrap();
        assert_eq!(manifold.certainty, MeshCertainty::Certified);
        assert_eq!(manifold.value, name == "vertex", "{name}");
        if name == "vertex" {
            let corner = Point3::new(Real::from(2), Real::from(2), Real::from(2));
            let rows = body
                .positions
                .iter()
                .filter(|point| **point == corner)
                .count();
            assert_eq!(rows, 1);
        }
        let rebuilt = TriangleMesh::new(body.positions.to_vec(), body.triangles.to_vec());
        // The cutter removes the part of each box inside it.
        let removed = std::iter::once(&block)
            .chain(&others)
            .fold(Real::zero(), |sum, part| {
                sum + six_volume(&operation(part, &cutter, BooleanOp::Intersection))
            });
        let total = Real::from(6 * volume);
        for (left, right, op, expected) in [
            (&body, &cutter, BooleanOp::Difference, &total - &removed),
            (&body, &cutter, BooleanOp::Intersection, removed.clone()),
            (
                &cutter,
                &body,
                BooleanOp::Difference,
                Real::from(6 * 32) - &removed,
            ),
            (&body, &body, BooleanOp::Union, total.clone()),
            (&body, &body, BooleanOp::Intersection, total.clone()),
            (&body, &body, BooleanOp::Difference, Real::zero()),
        ] {
            let role = |mesh: &TriangleMesh| {
                if std::ptr::eq(mesh, &body) {
                    "the union"
                } else {
                    "the cutter"
                }
            };
            let substitute = |mesh: &TriangleMesh| {
                if std::ptr::eq(mesh, &body) {
                    rebuilt.clone()
                } else {
                    mesh.clone()
                }
            };
            let retained = operation(left, right, op);
            let recomputed = operation(&substitute(left), &substitute(right), op);
            let label = format!("{name}: {op:?} of {} and {}", role(left), role(right));
            assert_eq!(retained.positions, recomputed.positions, "{label}");
            assert_eq!(retained.triangles, recomputed.triangles, "{label}");
            assert_eq!(six_volume(&retained), expected, "{label}");
        }
    }
}
