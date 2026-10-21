//! Boolean outputs keep only the vertices their exact set needs.
//!
//! Every requested boundary merges facets that lie in one plane class and
//! removes vertices interior to one class or on a straight crease between two
//! classes. Corners, crease ends and non-manifold contacts stay. These tests
//! check each case against independently derived exact sizes, the exact
//! enclosed volume, closure and orientation.

use std::collections::BTreeSet;

use hypermesh::{
    BooleanExpression, BooleanMeshBatch, BooleanMeshResult, BooleanOp, BooleanProgram,
    MeshCertainty, MeshContext, Point3, PredicatePolicy, Real, Triangle, TriangleMesh,
    boolean_mesh_closure_evidence,
};
use hyperreal::Rational;

const STRICT: MeshContext = MeshContext::new(PredicatePolicy::STRICT);

fn q(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn r(value: i64) -> Real {
    Real::from(value)
}

const BOX_TRIANGLES: [[usize; 3]; 12] = [
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
];

fn box_corners(min: &[Real; 3], max: &[Real; 3]) -> Vec<Point3> {
    (0..8)
        .map(|corner: usize| {
            let x = if matches!(corner % 4, 1 | 2) {
                &max[0]
            } else {
                &min[0]
            };
            let y = if matches!(corner % 4, 2 | 3) {
                &max[1]
            } else {
                &min[1]
            };
            let z = if corner >= 4 { &max[2] } else { &min[2] };
            Point3::new(x.clone(), y.clone(), z.clone())
        })
        .collect()
}

fn mesh(positions: Vec<Point3>, triangles: &[[usize; 3]]) -> TriangleMesh {
    TriangleMesh::new(
        positions,
        triangles
            .iter()
            .map(|&[a, b, c]| Triangle::new(a, b, c))
            .collect(),
    )
}

fn exact_box(min: [Real; 3], max: [Real; 3]) -> TriangleMesh {
    mesh(box_corners(&min, &max), &BOX_TRIANGLES)
}

fn integer_box(min: [i64; 3], max: [i64; 3]) -> TriangleMesh {
    exact_box(min.map(r), max.map(r))
}

fn batch(meshes: &[&TriangleMesh], program: BooleanProgram<'_>) -> BooleanMeshBatch {
    let views = meshes.iter().map(|mesh| mesh.as_ref()).collect::<Vec<_>>();
    let outcome = hypermesh::boolean(&STRICT, &views, program).unwrap();
    assert_eq!(outcome.certainty, MeshCertainty::Certified);
    outcome.value
}

fn operation(left: &TriangleMesh, right: &TriangleMesh, op: BooleanOp) -> TriangleMesh {
    batch(&[left, right], BooleanProgram::Operation(op))
        .into_triangle_meshes()
        .unwrap()
        .pop()
        .unwrap()
}

/// The regularized set of one operand, through the Boolean kernel.
fn regularized(mesh: &TriangleMesh) -> TriangleMesh {
    batch(
        &[mesh],
        BooleanProgram::Expressions {
            nodes: &[BooleanExpression::Operand(0)],
            roots: &[0],
        },
    )
    .into_triangle_meshes()
    .unwrap()
    .pop()
    .unwrap()
}

/// Six times the signed enclosed volume, as an exact rational.
fn six_volume(mesh: &TriangleMesh) -> Rational {
    let mut volume = Real::zero();
    for triangle in mesh.triangles.iter() {
        let [a, b, c] = triangle.indices().map(|index| &mesh.positions[index]);
        volume += &a.x * &(&b.y * &c.z - &b.z * &c.y)
            + &a.y * &(&b.z * &c.x - &b.x * &c.z)
            + &a.z * &(&b.x * &c.y - &b.y * &c.x);
    }
    volume.exact_rational().expect("rational fixture volume")
}

fn rational(value: i64) -> Rational {
    Rational::new(value)
}

fn as_result(mesh: &TriangleMesh) -> BooleanMeshResult {
    BooleanMeshResult {
        triangles: mesh
            .triangles
            .iter()
            .map(|triangle| triangle.indices().map(|index| index as u32))
            .collect(),
        sources: vec![Default::default(); mesh.triangles.len()],
        exterior_inside: false,
    }
}

/// Closed, consistently oriented and manifold, with the expected sizes, the
/// expected exact volume and every vertex used.
fn assert_closed_solid(mesh: &TriangleMesh, triangles: usize, vertices: usize, volume: Rational) {
    let evidence = boolean_mesh_closure_evidence(&as_result(mesh));
    assert!(evidence.is_closed(), "{evidence:?}");
    let closed = mesh.is_closed_manifold_geometry(&STRICT).unwrap();
    assert_eq!(closed.certainty, MeshCertainty::Certified);
    assert!(closed.value);
    assert_eq!(
        (mesh.triangles.len(), mesh.positions.len()),
        (triangles, vertices)
    );
    let used = mesh
        .triangles
        .iter()
        .flat_map(|triangle| triangle.indices())
        .collect::<BTreeSet<_>>();
    assert_eq!(used.len(), mesh.positions.len());
    assert_eq!(six_volume(mesh), &volume * &rational(6));
}

/// Order-independent oriented triangle rows, each rotated to its least vertex.
fn rows(mesh: &TriangleMesh) -> BTreeSet<[[String; 3]; 3]> {
    let point = |index: usize| {
        let point = &mesh.positions[index];
        [&point.x, &point.y, &point.z].map(|value| value.exact_rational().unwrap().to_string())
    };
    mesh.triangles
        .iter()
        .map(|triangle| {
            let [a, b, c] = triangle.indices().map(point);
            [
                [a.clone(), b.clone(), c.clone()],
                [b.clone(), c.clone(), a.clone()],
                [c, a, b],
            ]
            .into_iter()
            .min()
            .unwrap()
        })
        .collect()
}

fn has_point(mesh: &TriangleMesh, [x, y, z]: [Real; 3]) -> bool {
    mesh.positions
        .iter()
        .any(|point| point.x == x && point.y == y && point.z == z)
}

#[test]
fn a_vertex_interior_to_one_plane_is_removed() {
    // The top face of [0, 4] x [0, 4] x [0, 2] is a fan around its centre.
    let mut positions = box_corners(&[0, 0, 0].map(r), &[4, 4, 2].map(r));
    positions.push(Point3::new(r(2), r(2), r(2)));
    let mut triangles = BOX_TRIANGLES[2..].to_vec();
    triangles.extend([[4, 5, 8], [5, 6, 8], [6, 7, 8], [7, 4, 8]]);
    let output = regularized(&mesh(positions, &triangles));
    assert_closed_solid(&output, 12, 8, rational(32));
    assert!(!has_point(&output, [2, 2, 2].map(r)));
}

#[test]
fn a_vertex_on_a_straight_crease_between_two_planes_is_removed() {
    // The midpoint (2, 0, 2) of the top front edge splits both faces.
    let mut positions = box_corners(&[0, 0, 0].map(r), &[4, 4, 2].map(r));
    positions.push(Point3::new(r(2), r(0), r(2)));
    let mut triangles = BOX_TRIANGLES[2..10].to_vec();
    triangles.extend([
        [4, 8, 7],
        [8, 5, 6],
        [8, 6, 7],
        [0, 1, 5],
        [0, 5, 8],
        [0, 8, 4],
    ]);
    let output = regularized(&mesh(positions, &triangles));
    assert_closed_solid(&output, 12, 8, rational(32));
    assert!(!has_point(&output, [2, 0, 2].map(r)));
}

#[test]
fn a_through_hole_keeps_only_its_corners() {
    // The hole crosses the diagonal of the block's top and bottom faces, so
    // corefinement creates crossings on that diagonal and on the hole walls.
    let block = integer_box([0, 0, 0], [8, 8, 4]);
    let output = operation(
        &block,
        &integer_box([3, 2, -1], [5, 6, 5]),
        BooleanOp::Difference,
    );
    // Eight block and eight hole corners; the top and bottom are squares
    // with one hole (8 triangles each), plus four sides and four walls.
    assert_closed_solid(&output, 32, 16, rational(8 * 8 * 4 - 2 * 4 * 4));
    for point in output.positions.iter() {
        let corner = |value: &Real, values: [i64; 4]| values.iter().any(|&v| *value == r(v));
        assert!(corner(&point.x, [0, 8, 3, 5]), "{point:?}");
        assert!(corner(&point.y, [0, 8, 2, 6]), "{point:?}");
        assert!(corner(&point.z, [0, 4, 0, 4]), "{point:?}");
    }
}

#[test]
fn coplanar_neighbours_from_different_operands_merge() {
    let output = operation(
        &integer_box([0, 0, 0], [2, 2, 2]),
        &integer_box([2, 0, 0], [4, 2, 2]),
        BooleanOp::Union,
    );
    assert_closed_solid(&output, 12, 8, rational(16));
}

#[test]
fn successive_holes_stay_minimal() {
    // As in the successive-operations corpus: square holes at non-dyadic
    // offsets. Each needs eight vertices and twenty triangles.
    let steps = 6;
    let mut body = integer_box([0, 0, 0], [4 * steps + 4, 8, 4]);
    for k in 1..=steps {
        let x = q(3 * (4 * k - 1) + 1, 3);
        let hole = exact_box(
            [x.clone(), q(26, 7), r(-1)],
            [&x + &q(4, 3), q(31, 7), r(5)],
        );
        body = operation(&body, &hole, BooleanOp::Difference);
        // Volume: block minus k prisms of 4/3 x 5/7 x 4.
        let volume = &rational((4 * steps + 4) * 8 * 4) - &(&rational(80 * k) / &rational(21));
        assert_closed_solid(&body, (12 + 20 * k) as usize, (8 + 8 * k) as usize, volume);
    }
}

#[test]
fn repeating_a_cut_that_leaves_the_set_unchanged_is_a_fixed_point() {
    let block = integer_box([0, 0, 0], [12, 12, 12]);
    let z_bar = integer_box([4, 4, -2], [8, 8, 14]);
    let y_bar = integer_box([4, -2, 4], [8, 14, 8]);
    let once = operation(
        &operation(&block, &z_bar, BooleanOp::Difference),
        &y_bar,
        BooleanOp::Difference,
    );
    // Only integer corners of the cross-shaped tunnel and the block remain.
    for point in once.positions.iter() {
        for value in [&point.x, &point.y, &point.z] {
            assert!(value.exact_rational().unwrap().denominator().bits() <= 1);
        }
    }
    let mut repeated = once.clone();
    for cutter in [&z_bar, &y_bar, &z_bar] {
        repeated = operation(&repeated, cutter, BooleanOp::Difference);
        assert_eq!(rows(&repeated), rows(&once));
        assert_eq!(repeated.positions.len(), once.positions.len());
    }
}

#[test]
fn touching_contacts_and_non_manifold_edges_keep_their_vertices() {
    // The boxes share the edge segment x = y = 2, 1 <= z <= 2. Its endpoints
    // lie on straight creases of one operand but bound a non-manifold edge.
    let output = batch(
        &[
            &integer_box([0, 0, 0], [2, 2, 2]),
            &integer_box([2, 2, 1], [4, 4, 3]),
        ],
        BooleanProgram::Operation(BooleanOp::Union),
    );
    let result = &output.results[0];
    let evidence = boolean_mesh_closure_evidence(result);
    assert!(evidence.has_no_boundary(), "{evidence:?}");
    assert_eq!(evidence.non_manifold_edges, 1);
    let positions = output.vertices.clone();
    let mesh = TriangleMesh::new(
        positions.to_vec(),
        result
            .triangles
            .iter()
            .map(|&[a, b, c]| Triangle::new(a as usize, b as usize, c as usize))
            .collect(),
    );
    assert!(has_point(&mesh, [2, 2, 1].map(r)));
    assert!(has_point(&mesh, [2, 2, 2].map(r)));
    assert_eq!(six_volume(&mesh), rational(6 * 16));
    // Eight corners per box. Each contact endpoint splits two faces of the
    // other box into three triangles: 2 * (4 * 2 + 2 * 3) triangles.
    assert_eq!((result.triangles.len(), mesh.positions.len()), (28, 16));
}

#[test]
fn complement_and_operand_rows_reduce_identically() {
    // The complement selects the same boundary with reversed orientation.
    let block = integer_box([0, 0, 0], [8, 8, 4]);
    let hole = integer_box([3, 2, -1], [5, 6, 5]);
    let output = batch(
        &[&block, &hole],
        BooleanProgram::Expressions {
            nodes: &[
                BooleanExpression::Operand(0),
                BooleanExpression::Operand(1),
                BooleanExpression::Not(1),
                BooleanExpression::And([0, 2]),
                BooleanExpression::Not(3),
            ],
            roots: &[3, 4],
        },
    );
    let [difference, complement] = [&output.results[0], &output.results[1]];
    assert!(!difference.exterior_inside);
    assert!(complement.exterior_inside);
    assert_eq!(difference.triangles.len(), 32);
    let oriented = |result: &BooleanMeshResult, reverse: bool| {
        result
            .triangles
            .iter()
            .map(|&triangle| {
                let mut triangle = triangle;
                if reverse {
                    triangle.swap(1, 2);
                }
                let start = (0..3).min_by_key(|&index| triangle[index]).unwrap();
                [0, 1, 2].map(|offset| triangle[(start + offset) % 3])
            })
            .collect::<BTreeSet<_>>()
    };
    assert_eq!(oriented(difference, false), oriented(complement, true));
}

/// A hexagonal prism of circumradius 1 around `(cx, cy)` for `z` in
/// `[-1, 5]`, with `sqrt(3)` coordinates.
fn hexagonal_drill(cx: i64, cy: i64) -> TriangleMesh {
    let half_root = (Real::from(3).sqrt().unwrap() / r(2)).unwrap();
    let (half, one) = (q(1, 2), r(1));
    let ring = [
        (one.clone(), r(0)),
        (half.clone(), half_root.clone()),
        (-&half, half_root.clone()),
        (-&one, r(0)),
        (-&half, -&half_root),
        (half.clone(), -&half_root),
    ];
    let mut positions = Vec::new();
    for z in [-1, 5] {
        for (x, y) in &ring {
            positions.push(Point3::new(x + &r(cx), y + &r(cy), r(z)));
        }
    }
    let mut triangles = vec![
        [0, 2, 1],
        [0, 3, 2],
        [0, 4, 3],
        [0, 5, 4],
        [6, 7, 8],
        [6, 8, 9],
        [6, 9, 10],
        [6, 10, 11],
    ];
    for i in 0..6 {
        let next = (i + 1) % 6;
        triangles.extend([[i, next, next + 6], [i, next + 6, i + 6]]);
    }
    mesh(positions, &triangles)
}

#[test]
fn symbolic_reduction_is_independent_of_cached_approximations() {
    // The block's top diagonal crosses the hole, so reduction triangulates
    // polygons with sqrt(3) coordinates.
    let cold = || {
        operation(
            &integer_box([0, 0, 0], [8, 8, 4]),
            &hexagonal_drill(3, 4),
            BooleanOp::Difference,
        )
    };
    let block = integer_box([0, 0, 0], [8, 8, 4]);
    let drill = hexagonal_drill(3, 4);
    let first = operation(&block, &drill, BooleanOp::Difference);
    // Repeating on the same inputs reuses their cached approximations.
    let warm = operation(&block, &drill, BooleanOp::Difference);
    let fresh = cold();
    for other in [&warm, &fresh] {
        assert_eq!(first.triangles, other.triangles);
        assert_eq!(first.positions, other.positions);
    }
    // Eight block and twelve hole corners; the top and bottom are squares
    // with one hexagonal hole, plus four sides and six walls.
    let evidence = boolean_mesh_closure_evidence(&as_result(&first));
    assert!(evidence.is_closed(), "{evidence:?}");
    assert_eq!((first.triangles.len(), first.positions.len()), (40, 20));
}

fn max_coordinate_depth(mesh: &TriangleMesh) -> usize {
    mesh.positions
        .iter()
        .flat_map(|point| [&point.x, &point.y, &point.z])
        .map(|value| {
            let size = value.representation_size(1 << 16);
            assert!(size.complete);
            size.depth
        })
        .max()
        .unwrap_or(0)
}

#[test]
fn tilted_symbolic_drills_keep_constant_coordinate_size() {
    // Hexagonal drills tilted about x by the rotation with cosine 12/13:
    // every hole corner is a drill edge crossing a block face plane, and
    // every coordinate mixes sqrt(3). The merged faces keep the block's own
    // planes, so a later corner is not built on a plane through earlier
    // corners.
    let tilted = |cx: i64| {
        let drill = hexagonal_drill(0, 0);
        let [c, s] = [q(12, 13), q(5, 13)];
        let positions = drill
            .positions
            .iter()
            .map(|point| {
                let z = &point.z + &r(1);
                Point3::new(
                    &point.x + &r(cx),
                    &(&c * &point.y - &s * &z) + &r(4),
                    &(&s * &point.y + &c * &z) - &r(1),
                )
            })
            .collect::<Vec<_>>();
        TriangleMesh::new(positions, drill.triangles.to_vec())
    };
    // No drill corner lies on the block's diagonals, where the arrangement
    // would build it from the crossed edge instead.
    let mut body = integer_box([0, 0, 0], [20, 9, 4]);
    let mut depths = Vec::new();
    for k in 0..4 {
        body = operation(&body, &tilted(3 + 4 * k), BooleanOp::Difference);
        depths.push(max_coordinate_depth(&body));
        let evidence = boolean_mesh_closure_evidence(&as_result(&body));
        assert!(evidence.is_closed(), "{evidence:?}");
        let k = k as usize + 1;
        assert_eq!(
            (body.triangles.len(), body.positions.len()),
            (12 + 28 * k, 8 + 12 * k)
        );
    }
    assert!(
        depths.windows(2).all(|pair| pair[0] == pair[1]),
        "{depths:?}"
    );
}

#[test]
fn reduced_outputs_record_plane_provenance() {
    // Every output triangle names a source triangle in its own plane. The
    // reduced top face of the drilled block spans both source triangles of
    // the block's top, so its rows name one of them.
    let block = integer_box([0, 0, 0], [8, 8, 4]);
    let hole = integer_box([3, 2, -1], [5, 6, 5]);
    let output = batch(
        &[&block, &hole],
        BooleanProgram::Operation(BooleanOp::Difference),
    );
    let result = &output.results[0];
    for (triangle, source) in result.triangles.iter().zip(&result.sources) {
        let source_mesh = if source.mesh == 0 { &block } else { &hole };
        let local = source.triangle as usize - if source.mesh == 0 { 0 } else { 12 };
        let [a, b, c] = source_mesh.triangles[local]
            .indices()
            .map(|index| source_mesh.positions[index].clone());
        // Each output vertex lies in the plane of its named source triangle.
        for &vertex in triangle {
            let p = &output.vertices[vertex as usize];
            let ab = [&b.x - &a.x, &b.y - &a.y, &b.z - &a.z];
            let ac = [&c.x - &a.x, &c.y - &a.y, &c.z - &a.z];
            let ap = [&p.x - &a.x, &p.y - &a.y, &p.z - &a.z];
            let determinant = &ab[0] * &(&ac[1] * &ap[2] - &ac[2] * &ap[1])
                - &ab[1] * &(&ac[0] * &ap[2] - &ac[2] * &ap[0])
                + &ab[2] * &(&ac[0] * &ap[1] - &ac[1] * &ap[0]);
            assert_eq!(determinant.exact_rational(), Some(rational(0)));
        }
        assert!(matches!(source.orientation, -1 | 1));
    }
    assert_eq!(result.triangles.len(), 32);
}
