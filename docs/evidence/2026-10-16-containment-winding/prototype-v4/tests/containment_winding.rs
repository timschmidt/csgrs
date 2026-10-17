//! Strict interior of the regularized nonzero-winding set, with an independent
//! rectilinear oracle. Query coordinates are half-integers; box faces integers.
use csgrs::solid::SolidExt;
use csgrs::{GeometryCertainty, GeometryContext, Real, TriangleMesh, solid};
use hypermesh::{BooleanOp, BooleanProgram, MeshCertainty, Point3, Triangle};

#[derive(Clone, Copy)]
struct BoxInput {
    min: [i32; 3],
    max: [i32; 3],
    orientation: i32,
}

fn box_input(min: [i32; 3], max: [i32; 3], orientation: i32) -> BoxInput {
    assert!(orientation == 1 || orientation == -1);
    BoxInput {
        min,
        max,
        orientation,
    }
}

fn mesh(boxes: &[BoxInput]) -> TriangleMesh {
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    for input in boxes {
        let [a, b, c] = input.min;
        let [x, y, z] = input.max;
        let base = positions.len();
        positions.extend(
            [
                [a, b, c],
                [x, b, c],
                [x, y, c],
                [a, y, c],
                [a, b, z],
                [x, b, z],
                [x, y, z],
                [a, y, z],
            ]
            .map(|p| Point3::new(Real::from(p[0]), Real::from(p[1]), Real::from(p[2]))),
        );
        for mut face in [
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
        ] {
            if input.orientation < 0 {
                face.reverse();
            }
            triangles.push(Triangle::new(base + face[0], base + face[1], base + face[2]));
        }
    }
    TriangleMesh::new(positions, triangles)
}

fn point(twice: [i32; 3]) -> Point3 {
    let [x, y, z] = twice.map(|value| (Real::from(value) / Real::from(2)).unwrap());
    Point3::new(x, y, z)
}

fn interior(boxes: &[BoxInput], twice: [i32; 3]) -> bool {
    // A radius < 1/2 cannot cross another integer support plane from this
    // half-integer query. Nonzero winding in all eight adjacent open octants
    // therefore proves strict interior, including internal faces/edges/vertices.
    [-1, 1].into_iter().all(|dx| {
        [-1, 1].into_iter().all(|dy| {
            [-1, 1].into_iter().all(|dz| {
                let sample = [4 * twice[0] + dx, 4 * twice[1] + dy, 4 * twice[2] + dz];
                let winding: i32 = boxes
                    .iter()
                    .filter(|input| {
                        (0..3).all(|axis| {
                            8 * input.min[axis] < sample[axis]
                                && sample[axis] < 8 * input.max[axis]
                        })
                    })
                    .map(|input| input.orientation)
                    .sum();
                winding != 0
            })
        })
    })
}

fn check_queries(
    source: &TriangleMesh,
    boxes: &[BoxInput],
    points: &[[i32; 3]],
    transform: fn(Point3) -> Point3,
) {
    let original_positions = source.positions.clone();
    let original_triangles = source.triangles.clone();
    for context in [
        GeometryContext::STRICT,
        GeometryContext::APPROXIMATE_512,
        GeometryContext::STRICT,
    ] {
        assert_eq!(
            source
                .validate_closed_pwn(&context.mesh_context())
                .unwrap()
                .certainty,
            MeshCertainty::Certified
        );
        for &twice in points {
            let expected = interior(boxes, twice);
            let query = transform(point(twice));
            let native = source
                .contains_point(&context.mesh_context(), &query)
                .unwrap();
            assert_eq!(native.certainty, MeshCertainty::Certified);
            assert_eq!(
                native.value, expected,
                "native query {twice:?} under {context:?}"
            );
            let modeled =
                solid::contains_point_with_context(source, &query, &context).unwrap();
            assert_eq!(modeled.certainty, GeometryCertainty::Certified);
            assert_eq!(modeled.value, expected, "modeling query {twice:?}");
        }
    }
    assert_eq!(source.positions, original_positions);
    assert_eq!(source.triangles, original_triangles);
}

fn check(boxes: &[BoxInput], points: &[[i32; 3]]) {
    check_queries(&mesh(boxes), boxes, points, |p| p);
}

#[test]
fn nested_outward_shells_have_nonzero_interior_winding() {
    let boxes = [
        box_input([0, 0, 0], [3, 3, 3], 1),
        box_input([1, 1, 1], [2, 2, 2], 1),
    ];
    check(&boxes, &[[3, 3, 3], [1, 1, 1], [0, 3, 3], [7, 3, 3]]);
}

#[test]
fn opposite_coincident_shells_cancel_everywhere() {
    let boxes = [
        box_input([0, 0, 0], [2, 2, 2], 1),
        box_input([0, 0, 0], [2, 2, 2], -1),
    ];
    check(
        &boxes,
        &[[1, 1, 1], [0, 1, 1], [0, 0, 1], [0, 0, 0], [5, 1, 1]],
    );
}

#[test]
fn overlapping_outward_shells_do_not_create_a_parity_hole() {
    let boxes = [
        box_input([0, 0, 0], [2, 2, 2], 1),
        box_input([1, 0, 0], [3, 2, 2], 1),
    ];
    check(
        &boxes,
        &[
            [3, 1, 1],
            [1, 1, 1],
            [5, 1, 1],
            [2, 1, 1],
            [4, 1, 1],
            [3, 0, 1],
        ],
    );
}

#[test]
fn opposite_overlap_cancels_only_the_shared_volume() {
    let boxes = [
        box_input([0, 0, 0], [2, 2, 2], 1),
        box_input([1, 0, 0], [3, 2, 2], -1),
    ];
    check(
        &boxes,
        &[[3, 1, 1], [1, 1, 1], [5, 1, 1], [2, 1, 1], [4, 1, 1]],
    );
}

#[test]
fn nested_source_faces_edges_and_vertices_can_be_strictly_inside() {
    let boxes = [
        box_input([0, 0, 0], [3, 3, 3], 1),
        box_input([1, 1, 1], [2, 2, 2], 1),
    ];
    check(
        &boxes,
        &[
            [2, 3, 3],
            [2, 2, 3],
            [2, 2, 2],
            [4, 3, 3],
            [4, 4, 3],
            [4, 4, 4],
        ],
    );
}

#[test]
fn touching_shells_remove_their_internal_shared_face() {
    let boxes = [
        box_input([0, 0, 0], [1, 2, 2], 1),
        box_input([1, 0, 0], [2, 2, 2], 1),
    ];
    check(
        &boxes,
        &[
            [2, 1, 1],
            [2, 2, 2],
            [2, 0, 1],
            [2, 0, 0],
            [1, 1, 1],
            [3, 1, 1],
        ],
    );
}

#[test]
fn reversed_inner_shell_retains_a_cavity_and_its_boundary() {
    let boxes = [
        box_input([0, 0, 0], [3, 3, 3], 1),
        box_input([1, 1, 1], [2, 2, 2], -1),
    ];
    check(
        &boxes,
        &[
            [3, 3, 3],
            [1, 1, 1],
            [2, 3, 3],
            [2, 2, 3],
            [2, 2, 2],
            [0, 3, 3],
        ],
    );
}

#[test]
fn a_tangent_vertex_contact_does_not_create_interior() {
    // The legacy ray (1,3/8,5/16) touches [16,6,5] without crossing the
    // positive box. The negative box makes the origin lie inside combined
    // bounds but is entirely behind that ray. Neither box contains the origin.
    let boxes = [
        box_input([16, 6, 4], [17, 7, 5], 1),
        box_input([-2, -2, -2], [-1, -1, -1], 1),
    ];
    check(&boxes, &[[0, 0, 0], [33, 13, 9], [-3, -3, -3], [31, 12, 10]]);
}

#[test]
fn winding_multiplicity_and_global_reversal_preserve_the_represented_set() {
    for sign in [1, -1] {
        let boxes = [
            box_input([0, 0, 0], [2, 2, 2], sign),
            box_input([0, 0, 0], [2, 2, 2], sign),
            box_input([4, 0, 0], [6, 2, 2], -sign),
        ];
        check(
            &boxes,
            &[[1, 1, 1], [9, 1, 1], [6, 1, 1], [0, 1, 1], [12, 1, 1]],
        );
    }
}

fn affine(p: Point3) -> Point3 {
    // Determinant 6: an invertible rational affine map.
    Point3::new(
        &p.x + &p.y + (Real::one() / Real::from(3)).unwrap(),
        Real::from(2) * &p.y - &p.z,
        Real::from(3) * &p.z + Real::from(2),
    )
}

#[test]
fn nonzero_membership_commutes_with_an_invertible_affine_map() {
    let boxes = [
        box_input([0, 0, 0], [3, 3, 3], 1),
        box_input([1, 1, 1], [2, 2, 2], 1),
    ];
    let source = mesh(&boxes);
    let transformed = TriangleMesh::new(
        source.positions.iter().cloned().map(affine).collect(),
        source.triangles.to_vec(),
    );
    check_queries(
        &transformed,
        &boxes,
        &[
            [3, 3, 3],
            [2, 3, 3],
            [2, 2, 3],
            [2, 2, 2],
            [0, 3, 3],
            [7, 3, 3],
        ],
        affine,
    );
}

#[test]
fn unary_regularization_and_containment_share_nonzero_set_meaning() {
    for boxes in [
        vec![
            box_input([0, 0, 0], [3, 3, 3], 1),
            box_input([1, 1, 1], [2, 2, 2], 1),
        ],
        vec![
            box_input([0, 0, 0], [3, 3, 3], 1),
            box_input([0, 0, 0], [3, 3, 3], -1),
        ],
    ] {
        let source = mesh(&boxes);
        let context = GeometryContext::STRICT;
        let batch = hypermesh::boolean(
            &context.mesh_context(),
            &[source.as_ref()],
            BooleanProgram::Operation(BooleanOp::Union),
        )
        .unwrap();
        assert_eq!(batch.certainty, MeshCertainty::Certified);
        let regularized = batch.value.into_triangle_meshes().unwrap().remove(0);
        for twice in [[3, 3, 3], [2, 3, 3], [0, 3, 3], [7, 3, 3]] {
            let expected = interior(&boxes, twice);
            for input in [&source, &regularized] {
                let outcome =
                    solid::contains_point_with_context(input, &point(twice), &context)
                        .unwrap();
                assert_eq!(outcome.certainty, GeometryCertainty::Certified);
                assert_eq!(outcome.value, expected, "query {twice:?}");
            }
        }
    }
}

#[test]
fn a_cancelled_flat_source_has_empty_regularized_interior() {
    let source = TriangleMesh::new(
        vec![point([0, 0, 0]), point([2, 0, 0]), point([0, 2, 0])],
        vec![Triangle::new(0, 1, 2), Triangle::new(2, 1, 0)],
    );
    for context in [
        GeometryContext::STRICT,
        GeometryContext::APPROXIMATE_512,
        GeometryContext::STRICT,
    ] {
        assert_eq!(
            source
                .validate_closed_pwn(&context.mesh_context())
                .unwrap()
                .certainty,
            MeshCertainty::Certified
        );
        for query in [
            point([0, 0, 0]),
            point([1, 0, 0]),
            point([1, 1, 0]),
            point([0, 0, 1]),
        ] {
            let outcome =
                solid::contains_point_with_context(&source, &query, &context).unwrap();
            assert_eq!(outcome.certainty, GeometryCertainty::Certified);
            assert!(!outcome.value);
        }
    }
}


fn flat_source(copies: usize, z: i32) -> TriangleMesh {
    let positions = vec![point([-4, -4, 2*z]), point([4, -4, 2*z]), point([0, 4, 2*z])];
    let mut triangles = Vec::new();
    for _ in 0..copies {
        triangles.extend([Triangle::new(0, 1, 2), Triangle::new(2, 1, 0)]);
    }
    TriangleMesh::new(positions, triangles)
}

fn regularize(context: &GeometryContext, sources: &[&TriangleMesh], op: BooleanOp) -> TriangleMesh {
    let refs: Vec<_> = sources.iter().map(|source| source.as_ref()).collect();
    let result = hypermesh::boolean(&context.mesh_context(), &refs, BooleanProgram::Operation(op)).unwrap();
    assert_eq!(result.certainty, MeshCertainty::Certified);
    let mut meshes = result.value.into_triangle_meshes().unwrap();
    assert_eq!(meshes.len(), 1);
    meshes.remove(0)
}

#[test]
fn cancelled_flat_sheets_regularize_to_canonical_empty() {
    for copies in [1, 2, 3] {
        let source = flat_source(copies, 0);
        for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512, GeometryContext::STRICT] {
            assert_eq!(source.validate_closed_pwn(&context.mesh_context()).unwrap().certainty, MeshCertainty::Certified);
            let output = regularize(&context, &[&source], BooleanOp::Union);
            assert!(output.positions.is_empty());
            assert!(output.triangles.is_empty());
            assert!(!solid::contains_point_with_context(&output, &point([0, 0, 0]), &context).unwrap().value);
            let continued = output.try_union_with_context(&source, &context).unwrap();
            assert_eq!(continued.certainty, GeometryCertainty::Certified);
        }
    }
}

fn assert_unit_box_boundary(output: &TriangleMesh) {
    assert_eq!(output.validate_closed_pwn(&GeometryContext::STRICT.mesh_context()).unwrap().certainty, MeshCertainty::Certified);
    let mut six_volume = Real::zero();
    for triangle in output.triangles.iter() {
        let [a, b, c] = triangle.indices().map(|i| &output.positions[i]);
        fn coords(p: &Point3) -> [&Real; 3] { [&p.x, &p.y, &p.z] }
        assert!([a,b,c].into_iter().all(|p| coords(p).into_iter().all(|v| *v >= Real::from(-1) && *v <= Real::one())));
        assert!((0..3).any(|axis| [-1,1].into_iter().any(|edge| [a,b,c].into_iter().all(|p| *coords(p)[axis] == Real::from(edge)))));
        six_volume = six_volume + a.x.clone() * (b.y.clone()*c.z.clone()-b.z.clone()*c.y.clone())
            + a.y.clone() * (b.z.clone()*c.x.clone()-b.x.clone()*c.z.clone())
            + a.z.clone() * (b.x.clone()*c.y.clone()-b.y.clone()*c.x.clone());
    }
    assert_eq!(six_volume, Real::from(48));
}

#[test]
fn cancelled_flat_sheets_compose_with_a_closed_volume() {
    let boxes = [box_input([-1,-1,-1],[1,1,1],1)];
    let cube = mesh(&boxes);
    let points: Vec<_> = (-3..=3).flat_map(|x| (-3..=3).flat_map(move |y| (-3..=3).map(move |z| [x,y,z]))).collect();
    for copies in [1,2] {
        for z in [0,1,3] {
            let flat = flat_source(copies,z);
            for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
                for op in [BooleanOp::Union, BooleanOp::Intersection, BooleanOp::Difference, BooleanOp::SymmetricDifference] {
                    for reverse in [false,true] {
                        let sources = if reverse { [&flat,&cube] } else { [&cube,&flat] };
                        let output = regularize(&context, &sources, op);
                        let empty = op == BooleanOp::Intersection || (op == BooleanOp::Difference && reverse);
                        if empty {
                            assert!(output.positions.is_empty());
                            assert!(output.triangles.is_empty());
                        } else {
                            assert_unit_box_boundary(&output);
                            check_queries(&output,&boxes,&points,|p|p);
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn symbolic_translation_retains_certified_boundary_and_interior_queries() {
    fn translate(p: Point3) -> Point3 { Point3::new(p.x + Real::pi(), p.y, p.z) }
    let boxes = [box_input([0,0,0],[2,2,2],1)];
    let original = mesh(&boxes);
    let source = TriangleMesh::new(original.positions.iter().cloned().map(translate).collect(),original.triangles.to_vec());
    check_queries(&source,&boxes,&[[1,1,1],[0,1,1],[0,0,1],[0,0,0],[5,1,1]],translate);
}
