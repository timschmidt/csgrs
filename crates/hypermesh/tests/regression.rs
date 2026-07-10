use std::collections::{BTreeMap, BTreeSet};
use std::str::FromStr;

use hypermesh::{
    Aabb, BooleanOp, BooleanResult, Classification, EmberConfig, ExactBvh, HypermeshResult,
    InputMesh, MeshRef, OutputVertex, Plane, Point3, Real, Triangle, TriangleSoup,
    boolean_difference, boolean_intersection, boolean_operation, boolean_union,
    certify_output_polygon_closure, classify_point, prepare_input,
    triangulate_and_resolve_certified,
};
use proptest::prelude::*;

fn r(value: i32) -> Real {
    value.into()
}

fn ratio(numerator: i32, denominator: i32) -> Real {
    if numerator % denominator == 0 {
        r(numerator / denominator)
    } else {
        Real::from_str(&format!("{numerator}/{denominator}")).unwrap()
    }
}

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn pr(x: Real, y: Real, z: Real) -> Point3 {
    Point3::new(x, y, z)
}

fn standard_cube_triangles() -> Vec<Triangle> {
    vec![
        Triangle::new(4, 5, 6),
        Triangle::new(4, 6, 7),
        Triangle::new(0, 3, 2),
        Triangle::new(0, 2, 1),
        Triangle::new(1, 2, 6),
        Triangle::new(1, 6, 5),
        Triangle::new(0, 4, 7),
        Triangle::new(0, 7, 3),
        Triangle::new(3, 7, 6),
        Triangle::new(3, 6, 2),
        Triangle::new(0, 1, 5),
        Triangle::new(0, 5, 4),
    ]
}

fn box_mesh(min: [i32; 3], max: [i32; 3]) -> InputMesh {
    InputMesh::new(
        vec![
            p(min[0], min[1], min[2]),
            p(max[0], min[1], min[2]),
            p(max[0], max[1], min[2]),
            p(min[0], max[1], min[2]),
            p(min[0], min[1], max[2]),
            p(max[0], min[1], max[2]),
            p(max[0], max[1], max[2]),
            p(min[0], max[1], max[2]),
        ],
        standard_cube_triangles(),
    )
}

fn rational_cube(center: [Real; 3], half_extent: Real) -> InputMesh {
    let min = [
        &center[0] - &half_extent,
        &center[1] - &half_extent,
        &center[2] - &half_extent,
    ];
    let max = [
        &center[0] + &half_extent,
        &center[1] + &half_extent,
        &center[2] + &half_extent,
    ];
    InputMesh::new(
        vec![
            pr(min[0].clone(), min[1].clone(), min[2].clone()),
            pr(max[0].clone(), min[1].clone(), min[2].clone()),
            pr(max[0].clone(), max[1].clone(), min[2].clone()),
            pr(min[0].clone(), max[1].clone(), min[2].clone()),
            pr(min[0].clone(), min[1].clone(), max[2].clone()),
            pr(max[0].clone(), min[1].clone(), max[2].clone()),
            pr(max[0].clone(), max[1].clone(), max[2].clone()),
            pr(min[0].clone(), max[1].clone(), max[2].clone()),
        ],
        standard_cube_triangles(),
    )
}

fn tetrahedron(points: [[i32; 3]; 4]) -> InputMesh {
    InputMesh::new(
        points
            .into_iter()
            .map(|[x, y, z]| p(x, y, z))
            .collect::<Vec<_>>(),
        vec![
            Triangle::new(0, 2, 1),
            Triangle::new(0, 1, 3),
            Triangle::new(1, 2, 3),
            Triangle::new(2, 0, 3),
        ],
    )
}

fn tetra_from_face_and_apex(a: Point3, b: Point3, c: Point3, apex: Point3) -> InputMesh {
    InputMesh::new(
        vec![a, b, c, apex],
        vec![
            Triangle::new(0, 2, 1),
            Triangle::new(0, 1, 3),
            Triangle::new(0, 3, 2),
            Triangle::new(1, 2, 3),
        ],
    )
}

fn octahedron(center: [Real; 3], radius: Real) -> InputMesh {
    let cx = center[0].clone();
    let cy = center[1].clone();
    let cz = center[2].clone();
    InputMesh::new(
        vec![
            pr(&cx + &radius, cy.clone(), cz.clone()),
            pr(&cx - &radius, cy.clone(), cz.clone()),
            pr(cx.clone(), &cy + &radius, cz.clone()),
            pr(cx.clone(), &cy - &radius, cz.clone()),
            pr(cx.clone(), cy.clone(), &cz + &radius),
            pr(cx, cy, &cz - &radius),
        ],
        vec![
            Triangle::new(0, 2, 4),
            Triangle::new(2, 1, 4),
            Triangle::new(1, 3, 4),
            Triangle::new(3, 0, 4),
            Triangle::new(2, 0, 5),
            Triangle::new(1, 2, 5),
            Triangle::new(3, 1, 5),
            Triangle::new(0, 3, 5),
        ],
    )
}

fn combine_meshes(meshes: &[InputMesh]) -> InputMesh {
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    for mesh in meshes {
        let offset = positions.len();
        positions.extend(mesh.positions.iter().cloned());
        triangles.extend(mesh.triangles.iter().map(|triangle| {
            Triangle::new(
                triangle.v0 + offset,
                triangle.v1 + offset,
                triangle.v2 + offset,
            )
        }));
    }
    InputMesh::new(positions, triangles)
}

fn config() -> EmberConfig {
    EmberConfig { max_depth: 10 }
}

fn run_op(a: &InputMesh, b: &InputMesh, op: BooleanOp) -> HypermeshResult<TriangleSoup> {
    let refs = [a.as_ref(), b.as_ref()];
    let result = boolean_operation(&refs, op, config())?;
    assert_output_polygons_closed(&result);
    triangulate_and_resolve_certified(&result)
}

fn run_certified_op(a: &InputMesh, b: &InputMesh, op: BooleanOp) -> HypermeshResult<TriangleSoup> {
    let refs = [a.as_ref(), b.as_ref()];
    let result = boolean_operation(&refs, op, config())?;
    assert_output_polygons_closed(&result);
    triangulate_and_resolve_certified(&result)
}

fn run_op_refs(meshes: &[MeshRef<'_>], op: BooleanOp) -> HypermeshResult<TriangleSoup> {
    let result = boolean_operation(meshes, op, config())?;
    assert_output_polygons_closed(&result);
    triangulate_and_resolve_certified(&result)
}

fn vertex_key(vertex: &OutputVertex) -> [String; 3] {
    [
        vertex.x.to_string(),
        vertex.y.to_string(),
        vertex.z.to_string(),
    ]
}

fn point_key(point: &Point3) -> [String; 3] {
    [
        point.x.to_string(),
        point.y.to_string(),
        point.z.to_string(),
    ]
}

fn sorted_triangle_key(soup: &TriangleSoup, triangle: [usize; 3]) -> [[String; 3]; 3] {
    let mut keys = [
        vertex_key(&soup.vertices[triangle[0]]),
        vertex_key(&soup.vertices[triangle[1]]),
        vertex_key(&soup.vertices[triangle[2]]),
    ];
    keys.sort();
    keys
}

fn undirected_edge(a: [String; 3], b: [String; 3]) -> [[String; 3]; 2] {
    if a <= b { [a, b] } else { [b, a] }
}

fn assert_closed_triangle_soup(soup: &TriangleSoup) {
    let mut edge_uses: BTreeMap<[[String; 3]; 2], usize> = BTreeMap::new();
    let mut faces = BTreeSet::new();

    for triangle in &soup.triangles {
        let a = vertex_key(&soup.vertices[triangle[0]]);
        let b = vertex_key(&soup.vertices[triangle[1]]);
        let c = vertex_key(&soup.vertices[triangle[2]]);
        assert_ne!(a, b, "degenerate triangle has repeated first edge");
        assert_ne!(b, c, "degenerate triangle has repeated second edge");
        assert_ne!(a, c, "degenerate triangle has repeated third edge");
        assert!(
            faces.insert(sorted_triangle_key(soup, *triangle)),
            "duplicate triangle {:?}",
            triangle
        );

        *edge_uses
            .entry(undirected_edge(a.clone(), b.clone()))
            .or_default() += 1;
        *edge_uses
            .entry(undirected_edge(b.clone(), c.clone()))
            .or_default() += 1;
        *edge_uses.entry(undirected_edge(c, a)).or_default() += 1;
    }

    let bad_edges = edge_uses
        .iter()
        .filter(|(_, uses)| **uses != 2)
        .collect::<Vec<_>>();
    assert!(
        bad_edges.is_empty(),
        "expected all edges to have two incident triangles; bad edges: {:?}",
        bad_edges.iter().take(8).collect::<Vec<_>>()
    );
}

fn assert_no_boundary_edges(soup: &TriangleSoup) {
    let closure = hypermesh::triangle_soup_closure_report(soup);
    assert_eq!(
        closure.boundary_edges, 0,
        "expected no boundary edges; closure report: {closure:?}",
    );
    assert_eq!(
        closure.unbalanced_edges, 0,
        "expected signed edge cancellation; closure report: {closure:?}",
    );
}

fn assert_output_polygons_closed(result: &BooleanResult) {
    let closure = certify_output_polygon_closure(result).unwrap();
    assert_eq!(
        closure.boundary_edges, 0,
        "expected classified polygon output to be closed before cleanup; closure report: {closure:?}",
    );
    assert_eq!(
        closure.unbalanced_edges, 0,
        "expected classified polygon output to have signed edge cancellation; closure report: {closure:?}",
    );
}

fn assert_bounds(soup: &TriangleSoup, min: [Real; 3], max: [Real; 3]) -> HypermeshResult<()> {
    let bounds = Aabb::new(
        pr(min[0].clone(), min[1].clone(), min[2].clone()),
        pr(max[0].clone(), max[1].clone(), max[2].clone()),
    );
    for vertex in &soup.vertices {
        let point = pr(vertex.x.clone(), vertex.y.clone(), vertex.z.clone());
        assert!(
            bounds.contains_point(&point)?,
            "vertex {:?} outside bounds {:?}..{:?}",
            vertex_key(vertex),
            point_key(&bounds.min),
            point_key(&bounds.max)
        );
    }
    Ok(())
}

fn signed_volume_numerator(soup: &TriangleSoup) -> Real {
    let mut volume = Real::zero();
    for triangle in &soup.triangles {
        let v0 = &soup.vertices[triangle[0]];
        let v1 = &soup.vertices[triangle[1]];
        let v2 = &soup.vertices[triangle[2]];
        volume += &v0.x * &((&v1.y * &v2.z) - (&v1.z * &v2.y))
            + &v0.y * &((&v1.z * &v2.x) - (&v1.x * &v2.z))
            + &v0.z * &((&v1.x * &v2.y) - (&v1.y * &v2.x));
    }
    volume.abs()
}

fn assert_volume_numerator(soup: &TriangleSoup, expected: Real) {
    assert_eq!(signed_volume_numerator(soup), expected);
}

fn assert_same_shape(left: &TriangleSoup, right: &TriangleSoup) {
    let left_faces = left
        .triangles
        .iter()
        .map(|triangle| sorted_triangle_key(left, *triangle))
        .collect::<BTreeSet<_>>();
    let right_faces = right
        .triangles
        .iter()
        .map(|triangle| sorted_triangle_key(right, *triangle))
        .collect::<BTreeSet<_>>();
    assert_eq!(left_faces, right_faces);
}

fn passthrough(mesh: &InputMesh) -> HypermeshResult<TriangleSoup> {
    let result = boolean_operation(
        &[mesh.as_ref()],
        BooleanOp::Union,
        EmberConfig { max_depth: 0 },
    )?;
    triangulate_and_resolve_certified(&result)
}

fn point_strictly_inside_convex_mesh(point: &Point3, mesh: &InputMesh) -> HypermeshResult<bool> {
    let mut saw_boundary = false;

    for triangle in &mesh.triangles {
        let [i0, i1, i2] = triangle.indices();
        let plane = Plane::from_points(
            &mesh.positions[i0],
            &mesh.positions[i1],
            &mesh.positions[i2],
        );
        match classify_point(point, &plane)? {
            Classification::Positive => return Ok(false),
            Classification::On => saw_boundary = true,
            Classification::Negative => {}
        }
    }

    Ok(!saw_boundary)
}

fn assert_no_strictly_contained_source_vertices(
    left: &InputMesh,
    right: &InputMesh,
) -> HypermeshResult<()> {
    for point in &left.positions {
        assert!(
            !point_strictly_inside_convex_mesh(point, right)?,
            "left source vertex {:?} lies strictly inside right mesh",
            point_key(point),
        );
    }

    for point in &right.positions {
        assert!(
            !point_strictly_inside_convex_mesh(point, left)?,
            "right source vertex {:?} lies strictly inside left mesh",
            point_key(point),
        );
    }

    Ok(())
}

#[test]
fn cube_boolean_outputs_are_closed_and_exact_volume() {
    let cube_a = box_mesh([-1, -1, -1], [1, 1, 1]);
    let cube_b = rational_cube([ratio(1, 2), ratio(1, 2), ratio(1, 2)], r(1));

    let union = boolean_union(cube_a.as_ref(), cube_b.as_ref(), config()).unwrap();
    assert_output_polygons_closed(&union);
    let union_soup = triangulate_and_resolve_certified(&union).unwrap();
    assert_closed_triangle_soup(&union_soup);
    assert_bounds(
        &union_soup,
        [r(-1), r(-1), r(-1)],
        [ratio(3, 2), ratio(3, 2), ratio(3, 2)],
    )
    .unwrap();
    assert_volume_numerator(&union_soup, ratio(303, 4));

    let intersection = boolean_intersection(cube_a.as_ref(), cube_b.as_ref(), config()).unwrap();
    assert_output_polygons_closed(&intersection);
    let intersection_soup = triangulate_and_resolve_certified(&intersection).unwrap();
    assert_closed_triangle_soup(&intersection_soup);
    assert_bounds(
        &intersection_soup,
        [ratio(-1, 2), ratio(-1, 2), ratio(-1, 2)],
        [r(1), r(1), r(1)],
    )
    .unwrap();
    assert_volume_numerator(&intersection_soup, ratio(81, 4));

    let difference = boolean_difference(cube_a.as_ref(), cube_b.as_ref(), config()).unwrap();
    assert_output_polygons_closed(&difference);
    let difference_soup = triangulate_and_resolve_certified(&difference).unwrap();
    assert_closed_triangle_soup(&difference_soup);
    assert_bounds(&difference_soup, [r(-1), r(-1), r(-1)], [r(1), r(1), r(1)]).unwrap();
    assert_volume_numerator(&difference_soup, ratio(111, 4));
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 2,
        failure_persistence: None,
        ..ProptestConfig::default()
    })]

    #[test]
    fn integer_box_booleans_match_exact_volume_oracle(
        a_min_x in -4i32..5,
        a_min_y in -4i32..5,
        a_min_z in -4i32..5,
        a_extent_x in 1i32..5,
        a_extent_y in 1i32..5,
        a_extent_z in 1i32..5,
        b_min_x in -4i32..5,
        b_min_y in -4i32..5,
        b_min_z in -4i32..5,
        b_extent_x in 1i32..5,
        b_extent_y in 1i32..5,
        b_extent_z in 1i32..5,
    ) {
        let a_min = [a_min_x, a_min_y, a_min_z];
        let a_max = [
            a_min_x + a_extent_x,
            a_min_y + a_extent_y,
            a_min_z + a_extent_z,
        ];
        let b_min = [b_min_x, b_min_y, b_min_z];
        let b_max = [
            b_min_x + b_extent_x,
            b_min_y + b_extent_y,
            b_min_z + b_extent_z,
        ];
        let volume = |min: [i32; 3], max: [i32; 3]| {
            (max[0] - min[0]) * (max[1] - min[1]) * (max[2] - min[2])
        };
        let overlap_min = [
            a_min[0].max(b_min[0]),
            a_min[1].max(b_min[1]),
            a_min[2].max(b_min[2]),
        ];
        let overlap_max = [
            a_max[0].min(b_max[0]),
            a_max[1].min(b_max[1]),
            a_max[2].min(b_max[2]),
        ];
        let overlap_volume = (overlap_max[0] - overlap_min[0]).max(0)
            * (overlap_max[1] - overlap_min[1]).max(0)
            * (overlap_max[2] - overlap_min[2]).max(0);
        let a_volume = volume(a_min, a_max);
        let b_volume = volume(b_min, b_max);
        let a = box_mesh(a_min, a_max);
        let b = box_mesh(b_min, b_max);

        for op in [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::SymmetricDifference,
        ] {
            let expected_volume = match op {
                BooleanOp::Union => a_volume + b_volume - overlap_volume,
                BooleanOp::Intersection => overlap_volume,
                BooleanOp::Difference => a_volume - overlap_volume,
                BooleanOp::SymmetricDifference => a_volume + b_volume - 2 * overlap_volume,
            };
            let result = run_certified_op(&a, &b, op).map_err(|err| {
                TestCaseError::fail(format!(
                    "{op:?} failed for {a_min:?}..{a_max:?} and {b_min:?}..{b_max:?}: {err:?}"
                ))
            })?;

            assert_closed_triangle_soup(&result);
            prop_assert_eq!(
                signed_volume_numerator(&result),
                r(6 * expected_volume),
                "{:?} volume mismatch for {:?}..{:?} and {:?}..{:?}",
                op,
                a_min,
                a_max,
                b_min,
                b_max,
            );
        }
    }
}

#[test]
fn ordered_axis_aligned_boxes_use_same_basis_cell_decomposition_with_certified_output() {
    let cube_a = box_mesh([-1, -1, -1], [1, 1, 1]);
    let cube_b = rational_cube([ratio(1, 2), ratio(1, 2), ratio(1, 2)], r(1));

    let union = run_certified_op(&cube_a, &cube_b, BooleanOp::Union).unwrap();
    assert_closed_triangle_soup(&union);
    assert_bounds(
        &union,
        [r(-1), r(-1), r(-1)],
        [ratio(3, 2), ratio(3, 2), ratio(3, 2)],
    )
    .unwrap();
    assert_volume_numerator(&union, ratio(303, 4));

    let intersection = run_certified_op(&cube_a, &cube_b, BooleanOp::Intersection).unwrap();
    assert_closed_triangle_soup(&intersection);
    assert_bounds(
        &intersection,
        [ratio(-1, 2), ratio(-1, 2), ratio(-1, 2)],
        [r(1), r(1), r(1)],
    )
    .unwrap();
    assert_volume_numerator(&intersection, ratio(81, 4));

    let difference = run_certified_op(&cube_a, &cube_b, BooleanOp::Difference).unwrap();
    assert_closed_triangle_soup(&difference);
    assert_bounds(&difference, [r(-1), r(-1), r(-1)], [r(1), r(1), r(1)]).unwrap();
    assert_volume_numerator(&difference, ratio(111, 4));
}

#[test]
fn roundtrip_preserves_triangle_vertices_exactly() {
    let mesh = octahedron([r(0), r(0), r(0)], r(2));
    let soup = prepare_input(&[mesh.as_ref()]).unwrap();

    assert_eq!(soup.polygons.len(), mesh.triangles.len());
    for (poly_index, polygon) in soup.polygons.iter().enumerate() {
        let mut actual = polygon
            .vertices()
            .unwrap()
            .into_iter()
            .map(|point| point_key(&point))
            .collect::<Vec<_>>();
        actual.sort();

        let triangle = mesh.triangles[poly_index];
        let mut expected = triangle
            .indices()
            .into_iter()
            .map(|index| point_key(&mesh.positions[index]))
            .collect::<Vec<_>>();
        expected.sort();

        assert_eq!(actual, expected, "polygon {poly_index}");
    }

    let result = passthrough(&mesh).unwrap();
    assert_closed_triangle_soup(&result);
    assert_volume_numerator(&result, r(64));
}

#[test]
fn bvh_candidates_match_bruteforce_bounds_for_complex_fixture() {
    let a = octahedron([r(0), r(0), r(0)], r(3));
    let b = octahedron([r(1), r(1), r(1)], r(3));
    let soup = prepare_input(&[a.as_ref(), b.as_ref()]).unwrap();
    let polygons = soup.polygons;
    let left = polygons
        .iter()
        .filter(|polygon| polygon.mesh_index == 0)
        .cloned()
        .collect::<Vec<_>>();
    let right = polygons
        .iter()
        .filter(|polygon| polygon.mesh_index == 1)
        .cloned()
        .collect::<Vec<_>>();
    let left_bvh = ExactBvh::build(&left).unwrap();
    let right_bvh = ExactBvh::build(&right).unwrap();

    let mut bvh_pairs = BTreeSet::new();
    left_bvh
        .intersect_pairs(&right_bvh, |left_index, right_index| {
            bvh_pairs.insert((left_index, right_index));
        })
        .unwrap();

    let mut brute_pairs = BTreeSet::new();
    for (left_index, left_polygon) in left.iter().enumerate() {
        for (right_index, right_polygon) in right.iter().enumerate() {
            if hypermesh::bvh::bounds_overlap(
                left_polygon.approx_bounds.as_ref().unwrap(),
                right_polygon.approx_bounds.as_ref().unwrap(),
            )
            .unwrap()
            {
                brute_pairs.insert((left_index, right_index));
            }
        }
    }

    assert_eq!(bvh_pairs, brute_pairs);
}

#[test]
fn generated_sphere_booleans_are_closed() {
    let a = octahedron([r(0), r(0), r(0)], r(3));
    let b = octahedron([r(1), r(1), r(1)], r(3));
    let refs = [a.as_ref(), b.as_ref()];

    for op in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::SymmetricDifference,
    ] {
        let boolean = boolean_operation(&refs, op, config())
            .unwrap_or_else(|err| panic!("{op:?} failed: {err:?}"));
        assert_output_polygons_closed(&boolean);
        let result = triangulate_and_resolve_certified(&boolean)
            .unwrap_or_else(|err| panic!("{op:?} certified output failed: {err:?}"));
        if !result.triangles.is_empty() {
            assert_no_boundary_edges(&result);
            if op != BooleanOp::SymmetricDifference {
                assert_closed_triangle_soup(&result);
            }
        }
    }
}

#[test]
#[ignore = "benchmark-style smoke test"]
fn perf_smoke_for_cube_and_generated_sphere_booleans() {
    let cube_a = box_mesh([-1, -1, -1], [1, 1, 1]);
    let cube_b = rational_cube([ratio(1, 2), ratio(1, 2), ratio(1, 2)], r(1));
    assert!(
        !run_op(&cube_a, &cube_b, BooleanOp::Union)
            .unwrap()
            .triangles
            .is_empty()
    );

    let sphere_a = octahedron([r(0), r(0), r(0)], r(3));
    let sphere_b = octahedron([r(1), r(1), r(1)], r(3));
    assert!(
        !run_op(&sphere_a, &sphere_b, BooleanOp::Union)
            .unwrap()
            .triangles
            .is_empty()
    );
    assert!(
        !run_op(&sphere_a, &sphere_b, BooleanOp::Intersection)
            .unwrap()
            .triangles
            .is_empty()
    );
    assert!(
        !run_op(&sphere_a, &sphere_b, BooleanOp::Difference)
            .unwrap()
            .triangles
            .is_empty()
    );
}

#[test]
fn hypermesh_nested_closed_tetrahedra_booleans_have_expected_shape() {
    let outer = tetrahedron([[0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]]);
    let inner = tetrahedron([[1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]]);
    let outer_soup = passthrough(&outer).unwrap();
    let inner_soup = passthrough(&inner).unwrap();

    let union = run_op(&outer, &inner, BooleanOp::Union).unwrap();
    assert_closed_triangle_soup(&union);
    assert_volume_numerator(&union, signed_volume_numerator(&outer_soup));

    let intersection = run_op(&outer, &inner, BooleanOp::Intersection).unwrap();
    assert_closed_triangle_soup(&intersection);
    assert_volume_numerator(&intersection, signed_volume_numerator(&inner_soup));

    let difference = run_op(&outer, &inner, BooleanOp::Difference).unwrap();
    assert_closed_triangle_soup(&difference);
    assert!(difference.triangles.len() >= outer_soup.triangles.len());
}

#[test]
fn nested_closed_tetrahedra_use_strict_containment_with_certified_output() {
    let outer = tetrahedron([[0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]]);
    let inner = tetrahedron([[1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]]);
    let outer_soup = passthrough(&outer).unwrap();
    let inner_soup = passthrough(&inner).unwrap();

    let union = run_certified_op(&outer, &inner, BooleanOp::Union).unwrap();
    assert_no_boundary_edges(&union);
    assert_volume_numerator(&union, signed_volume_numerator(&outer_soup));

    let intersection = run_certified_op(&outer, &inner, BooleanOp::Intersection).unwrap();
    assert_no_boundary_edges(&intersection);
    assert_volume_numerator(&intersection, signed_volume_numerator(&inner_soup));

    let difference = run_certified_op(&inner, &outer, BooleanOp::Difference).unwrap();
    assert!(difference.triangles.is_empty());
}

#[test]
fn hypermesh_disconnected_container_routes_containment_correctly() {
    let outer = tetrahedron([[0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]]);
    let disjoint_shell = tetrahedron([[20, 0, 0], [21, 0, 0], [20, 1, 0], [20, 0, 1]]);
    let container = combine_meshes(&[outer.clone(), disjoint_shell.clone()]);
    let contained = tetrahedron([[1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]]);
    let container_soup = passthrough(&container).unwrap();
    let contained_soup = passthrough(&contained).unwrap();

    let union = run_op(&container, &contained, BooleanOp::Union).unwrap();
    assert_closed_triangle_soup(&union);
    assert_volume_numerator(&union, signed_volume_numerator(&container_soup));

    let intersection = run_op(&container, &contained, BooleanOp::Intersection).unwrap();
    assert_closed_triangle_soup(&intersection);
    assert_volume_numerator(&intersection, signed_volume_numerator(&contained_soup));

    let difference = run_op(&contained, &container, BooleanOp::Difference).unwrap();
    assert!(difference.triangles.is_empty());
}

#[test]
fn hypermesh_boundary_touching_boxes_regularize_named_booleans() {
    let left = box_mesh([0, 0, 0], [1, 1, 1]);
    let right = box_mesh([1, 0, 0], [2, 1, 1]);

    let union = run_op(&left, &right, BooleanOp::Union).unwrap();
    assert_closed_triangle_soup(&union);
    assert_bounds(&union, [r(0), r(0), r(0)], [r(2), r(1), r(1)]).unwrap();
    assert_volume_numerator(&union, r(12));

    let intersection = run_op(&left, &right, BooleanOp::Intersection).unwrap();
    assert!(intersection.triangles.is_empty());

    let difference = run_op(&left, &right, BooleanOp::Difference).unwrap();
    assert_closed_triangle_soup(&difference);
    assert_volume_numerator(&difference, r(6));
}

#[test]
fn boundary_touching_boxes_regularize_intersection_and_difference_with_certified_output() {
    let left = box_mesh([0, 0, 0], [1, 1, 1]);
    let right = box_mesh([1, 0, 0], [2, 1, 1]);
    let left_soup = passthrough(&left).unwrap();

    let intersection = run_certified_op(&left, &right, BooleanOp::Intersection).unwrap();
    assert!(intersection.triangles.is_empty());

    let difference = run_certified_op(&left, &right, BooleanOp::Difference).unwrap();
    assert_no_boundary_edges(&difference);
    assert_volume_numerator(&difference, signed_volume_numerator(&left_soup));
}

#[test]
fn disjoint_boxes_use_general_leaf_path() -> HypermeshResult<()> {
    let left = box_mesh([0, 0, 0], [1, 1, 1]);
    let right = box_mesh([3, 0, 0], [4, 1, 1]);
    let refs = [left.as_ref(), right.as_ref()];
    let left_soup = passthrough(&left).unwrap();
    let config = EmberConfig {
        // Keep this at depth zero so the test exercises the root certified
        // leaf classifier without subdivision.
        max_depth: 0,
        ..config()
    };

    let union_result = boolean_operation(&refs, BooleanOp::Union, config)?;
    let union = triangulate_and_resolve_certified(&union_result)?;
    assert_no_boundary_edges(&union);

    let intersection_result = boolean_operation(&refs, BooleanOp::Intersection, config)?;
    let intersection = triangulate_and_resolve_certified(&intersection_result)?;
    assert!(intersection.triangles.is_empty());

    let difference_result = boolean_operation(&refs, BooleanOp::Difference, config)?;
    let difference = triangulate_and_resolve_certified(&difference_result)?;
    assert_same_shape(&difference, &left_soup);

    Ok(())
}

#[test]
fn same_surface_solids_use_general_leaf_path_in_one_leaf() -> HypermeshResult<()> {
    let left = tetrahedron([[0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]]);
    let same_surface = InputMesh {
        positions: vec![p(4, 0, 0), p(0, 0, 0), p(0, 4, 0), p(0, 0, 4)],
        triangles: vec![
            Triangle::new(1, 2, 0),
            Triangle::new(1, 0, 3),
            Triangle::new(0, 2, 3),
            Triangle::new(2, 1, 3),
        ],
    };
    let refs = [left.as_ref(), same_surface.as_ref()];
    let config = EmberConfig { max_depth: 0 };

    let union_result = boolean_operation(&refs, BooleanOp::Union, config)?;
    assert_output_polygons_closed(&union_result);
    let union = triangulate_and_resolve_certified(&union_result)?;
    assert_no_boundary_edges(&union);

    let difference_result = boolean_operation(&refs, BooleanOp::Difference, config)?;
    assert_output_polygons_closed(&difference_result);
    let difference = triangulate_and_resolve_certified(&difference_result)?;
    assert!(difference.triangles.is_empty());

    Ok(())
}

#[test]
fn partial_face_boundary_touch_uses_general_leaf_path() -> HypermeshResult<()> {
    let left = tetrahedron([[0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]]);
    let right = tetrahedron([[2, 2, 2], [4, 1, 1], [1, 4, 1], [3, 3, 3]]);
    let refs = [left.as_ref(), right.as_ref()];
    let left_soup = passthrough(&left).unwrap();
    let config = EmberConfig { max_depth: 0 };

    let intersection_result = boolean_operation(&refs, BooleanOp::Intersection, config)?;
    assert_output_polygons_closed(&intersection_result);
    let intersection = triangulate_and_resolve_certified(&intersection_result)?;
    assert!(intersection.triangles.is_empty());

    let difference_result = boolean_operation(&refs, BooleanOp::Difference, config)?;
    assert_output_polygons_closed(&difference_result);
    let difference = triangulate_and_resolve_certified(&difference_result)?;
    assert_no_boundary_edges(&difference);
    assert_volume_numerator(&difference, signed_volume_numerator(&left_soup));

    Ok(())
}

#[test]
fn nested_closed_tetrahedra_use_general_leaf_path() -> HypermeshResult<()> {
    let outer = tetrahedron([[0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]]);
    let inner = tetrahedron([[1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]]);
    let refs = [outer.as_ref(), inner.as_ref()];
    let reverse_refs = [inner.as_ref(), outer.as_ref()];
    let outer_soup = passthrough(&outer).unwrap();
    let inner_soup = passthrough(&inner).unwrap();
    let config = EmberConfig { max_depth: 0 };

    let union_result = boolean_operation(&refs, BooleanOp::Union, config)?;
    assert_output_polygons_closed(&union_result);
    let union = triangulate_and_resolve_certified(&union_result)?;
    assert_no_boundary_edges(&union);
    assert_volume_numerator(&union, signed_volume_numerator(&outer_soup));

    let intersection_result = boolean_operation(&refs, BooleanOp::Intersection, config)?;
    assert_output_polygons_closed(&intersection_result);
    let intersection = triangulate_and_resolve_certified(&intersection_result)?;
    assert_no_boundary_edges(&intersection);
    assert_volume_numerator(&intersection, signed_volume_numerator(&inner_soup));

    let difference_result = boolean_operation(&refs, BooleanOp::Difference, config)?;
    assert_output_polygons_closed(&difference_result);
    let difference = triangulate_and_resolve_certified(&difference_result)?;
    assert_no_boundary_edges(&difference);
    assert!(difference.triangles.len() >= outer_soup.triangles.len());

    let reverse_difference_result =
        boolean_operation(&reverse_refs, BooleanOp::Difference, config)?;
    assert_output_polygons_closed(&reverse_difference_result);
    let reverse_difference = triangulate_and_resolve_certified(&reverse_difference_result)?;
    assert!(reverse_difference.triangles.is_empty());

    let xor_result = boolean_operation(&refs, BooleanOp::SymmetricDifference, config)?;
    assert_output_polygons_closed(&xor_result);
    let xor = triangulate_and_resolve_certified(&xor_result)?;
    assert_no_boundary_edges(&xor);
    assert_volume_numerator(
        &xor,
        signed_volume_numerator(&outer_soup) - signed_volume_numerator(&inner_soup),
    );

    Ok(())
}

#[test]
fn disconnected_container_uses_general_leaf_path() -> HypermeshResult<()> {
    let outer = tetrahedron([[0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]]);
    let disjoint_shell = tetrahedron([[20, 0, 0], [21, 0, 0], [20, 1, 0], [20, 0, 1]]);
    let container = combine_meshes(&[outer.clone(), disjoint_shell.clone()]);
    let contained = tetrahedron([[1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]]);
    let refs = [container.as_ref(), contained.as_ref()];
    let reverse_refs = [contained.as_ref(), container.as_ref()];
    let container_soup = passthrough(&container).unwrap();
    let contained_soup = passthrough(&contained).unwrap();
    let config = EmberConfig { max_depth: 0 };

    let union_result = boolean_operation(&refs, BooleanOp::Union, config)?;
    assert_output_polygons_closed(&union_result);
    let union = triangulate_and_resolve_certified(&union_result)?;
    assert_no_boundary_edges(&union);
    assert_volume_numerator(&union, signed_volume_numerator(&container_soup));

    let intersection_result = boolean_operation(&refs, BooleanOp::Intersection, config)?;
    assert_output_polygons_closed(&intersection_result);
    let intersection = triangulate_and_resolve_certified(&intersection_result)?;
    assert_no_boundary_edges(&intersection);
    assert_volume_numerator(&intersection, signed_volume_numerator(&contained_soup));

    let difference_result = boolean_operation(&reverse_refs, BooleanOp::Difference, config)?;
    assert_output_polygons_closed(&difference_result);
    let difference = triangulate_and_resolve_certified(&difference_result)?;
    assert!(difference.triangles.is_empty());

    let forward_difference_result = boolean_operation(&refs, BooleanOp::Difference, config)?;
    assert_output_polygons_closed(&forward_difference_result);
    let forward_difference = triangulate_and_resolve_certified(&forward_difference_result)?;
    assert_no_boundary_edges(&forward_difference);
    assert_volume_numerator(
        &forward_difference,
        signed_volume_numerator(&container_soup) - signed_volume_numerator(&contained_soup),
    );

    let xor_result = boolean_operation(&refs, BooleanOp::SymmetricDifference, config)?;
    assert_output_polygons_closed(&xor_result);
    let xor = triangulate_and_resolve_certified(&xor_result)?;
    assert_no_boundary_edges(&xor);
    assert_volume_numerator(
        &xor,
        signed_volume_numerator(&container_soup) - signed_volume_numerator(&contained_soup),
    );

    Ok(())
}

#[test]
fn crossing_octahedra_use_general_leaf_path() -> HypermeshResult<()> {
    let left = octahedron([r(0), r(0), r(0)], r(3));
    let right = octahedron([r(1), r(1), r(1)], r(3));
    let refs = [left.as_ref(), right.as_ref()];
    let config = EmberConfig { max_depth: 0 };

    for op in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::SymmetricDifference,
    ] {
        let boolean = boolean_operation(&refs, op, config)?;
        assert_output_polygons_closed(&boolean);
        let result = triangulate_and_resolve_certified(&boolean)?;
        assert!(
            !result.triangles.is_empty(),
            "{op:?} should produce non-empty output",
        );
        assert_no_boundary_edges(&result);
        if op != BooleanOp::SymmetricDifference {
            assert_closed_triangle_soup(&result);
        }
    }

    Ok(())
}

#[test]
fn crossing_octahedra_use_general_path() -> HypermeshResult<()> {
    let left = octahedron([r(0), r(0), r(0)], r(3));
    let right = octahedron([r(1), r(1), r(1)], r(3));
    let union = run_op(&left, &right, BooleanOp::Union)?;
    assert!(!union.triangles.is_empty());
    assert_no_boundary_edges(&union);
    assert_closed_triangle_soup(&union);

    let reverse_union = run_op(&right, &left, BooleanOp::Union)?;
    assert_same_shape(&reverse_union, &union);

    let intersection = run_op(&left, &right, BooleanOp::Intersection)?;
    assert!(!intersection.triangles.is_empty());
    assert_no_boundary_edges(&intersection);
    assert_closed_triangle_soup(&intersection);

    let difference = run_op(&left, &right, BooleanOp::Difference)?;
    assert!(!difference.triangles.is_empty());
    assert_no_boundary_edges(&difference);
    assert_closed_triangle_soup(&difference);

    let reverse_difference = run_op(&right, &left, BooleanOp::Difference)?;
    assert!(!reverse_difference.triangles.is_empty());
    assert_no_boundary_edges(&reverse_difference);
    assert_closed_triangle_soup(&reverse_difference);

    let xor = run_op(&left, &right, BooleanOp::SymmetricDifference)?;
    assert!(!xor.triangles.is_empty());
    assert_no_boundary_edges(&xor);

    let reverse_xor = run_op(&right, &left, BooleanOp::SymmetricDifference)?;
    assert_same_shape(&reverse_xor, &xor);

    Ok(())
}

#[test]
fn crossing_octahedra_have_no_strictly_contained_source_vertices() -> HypermeshResult<()> {
    let left = octahedron([r(0), r(0), r(0)], r(3));
    let right = octahedron([r(1), r(1), r(1)], r(3));
    assert_no_strictly_contained_source_vertices(&left, &right)
}

#[test]
fn affine_boxes_use_general_leaf_path() -> HypermeshResult<()> {
    let (left, right) = affine_box_pair();
    let refs = [left.as_ref(), right.as_ref()];
    let config = EmberConfig { max_depth: 0 };

    for op in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::SymmetricDifference,
    ] {
        let result = triangulate_and_resolve_certified(&boolean_operation(&refs, op, config)?)?;
        assert!(
            !result.triangles.is_empty(),
            "{op:?} should produce non-empty output",
        );
        assert_no_boundary_edges(&result);
        if op != BooleanOp::SymmetricDifference {
            assert_closed_triangle_soup(&result);
        }
        let expected_volume_numerator = match op {
            BooleanOp::Union => r(576),
            BooleanOp::Intersection | BooleanOp::Difference => r(192),
            BooleanOp::SymmetricDifference => r(384),
        };
        assert_volume_numerator(&result, expected_volume_numerator);
    }

    Ok(())
}

#[test]
fn affine_boxes_use_general_path() -> HypermeshResult<()> {
    let (left, right) = affine_box_pair();
    let union = run_op(&left, &right, BooleanOp::Union)?;
    assert_no_boundary_edges(&union);
    assert_closed_triangle_soup(&union);
    assert_volume_numerator(&union, r(576));
    assert_bounds(&union, [r(0), r(0), r(0)], [r(8), r(4), r(4)])?;

    let reverse_union = run_op(&right, &left, BooleanOp::Union)?;
    assert_no_boundary_edges(&reverse_union);
    assert_closed_triangle_soup(&reverse_union);
    assert_volume_numerator(&reverse_union, r(576));
    assert_bounds(&reverse_union, [r(0), r(0), r(0)], [r(8), r(4), r(4)])?;

    let intersection = run_op(&left, &right, BooleanOp::Intersection)?;
    assert_no_boundary_edges(&intersection);
    assert_closed_triangle_soup(&intersection);
    assert_volume_numerator(&intersection, r(192));
    assert_bounds(&intersection, [r(2), r(0), r(0)], [r(6), r(4), r(4)])?;

    let difference = run_op(&left, &right, BooleanOp::Difference)?;
    assert_no_boundary_edges(&difference);
    assert_closed_triangle_soup(&difference);
    assert_volume_numerator(&difference, r(192));
    assert_bounds(&difference, [r(0), r(0), r(0)], [r(6), r(4), r(4)])?;

    let reverse_difference = run_op(&right, &left, BooleanOp::Difference)?;
    assert_no_boundary_edges(&reverse_difference);
    assert_closed_triangle_soup(&reverse_difference);
    assert_volume_numerator(&reverse_difference, r(192));
    assert_bounds(&reverse_difference, [r(2), r(0), r(0)], [r(8), r(4), r(4)])?;

    let xor = run_op(&left, &right, BooleanOp::SymmetricDifference)?;
    assert_no_boundary_edges(&xor);
    assert_volume_numerator(&xor, r(384));
    assert_bounds(&xor, [r(0), r(0), r(0)], [r(8), r(4), r(4)])?;

    let reverse_xor = run_op(&right, &left, BooleanOp::SymmetricDifference)?;
    assert_no_boundary_edges(&reverse_xor);
    assert_volume_numerator(&reverse_xor, r(384));
    assert_bounds(&reverse_xor, [r(0), r(0), r(0)], [r(8), r(4), r(4)])?;

    Ok(())
}

#[test]
fn boundary_touching_boxes_union_returns_closed_result_before_triangulation() -> HypermeshResult<()>
{
    let left = box_mesh([0, 0, 0], [1, 1, 1]);
    let right = box_mesh([1, 0, 0], [2, 1, 1]);
    let refs = [left.as_ref(), right.as_ref()];

    let union_result = boolean_operation(&refs, BooleanOp::Union, config())?;
    assert_output_polygons_closed(&union_result);
    assert!(!union_result.output().polygons.is_empty());

    Ok(())
}

#[test]
fn boundary_touching_boxes_union_triangulates_with_certified_output() -> HypermeshResult<()> {
    let left = box_mesh([0, 0, 0], [1, 1, 1]);
    let right = box_mesh([1, 0, 0], [2, 1, 1]);
    let refs = [left.as_ref(), right.as_ref()];

    let union_result = boolean_operation(&refs, BooleanOp::Union, config())?;
    assert_output_polygons_closed(&union_result);
    let union = triangulate_and_resolve_certified(&union_result)?;
    assert_no_boundary_edges(&union);
    assert_volume_numerator(&union, r(12));

    Ok(())
}

#[test]
fn boundary_touching_boxes_intersection_use_general_path() -> HypermeshResult<()> {
    let left = box_mesh([0, 0, 0], [1, 1, 1]);
    let right = box_mesh([1, 0, 0], [2, 1, 1]);
    let refs = [left.as_ref(), right.as_ref()];

    let intersection_result = boolean_operation(&refs, BooleanOp::Intersection, config())?;
    assert_output_polygons_closed(&intersection_result);
    let intersection = triangulate_and_resolve_certified(&intersection_result)?;
    assert!(intersection.triangles.is_empty());

    Ok(())
}

#[test]
fn boundary_touching_boxes_difference_use_general_path() -> HypermeshResult<()> {
    let left = box_mesh([0, 0, 0], [1, 1, 1]);
    let right = box_mesh([1, 0, 0], [2, 1, 1]);
    let refs = [left.as_ref(), right.as_ref()];

    let difference_result = boolean_operation(&refs, BooleanOp::Difference, config())?;
    assert_output_polygons_closed(&difference_result);
    let difference = triangulate_and_resolve_certified(&difference_result)?;
    assert_no_boundary_edges(&difference);
    assert_volume_numerator(&difference, r(6));

    Ok(())
}

#[test]
fn boundary_touching_boxes_reverse_difference_use_general_path() -> HypermeshResult<()> {
    let left = box_mesh([0, 0, 0], [1, 1, 1]);
    let right = box_mesh([1, 0, 0], [2, 1, 1]);
    let refs = [right.as_ref(), left.as_ref()];

    let difference_result = boolean_operation(&refs, BooleanOp::Difference, config())?;
    assert_output_polygons_closed(&difference_result);
    let difference = triangulate_and_resolve_certified(&difference_result)?;
    assert_no_boundary_edges(&difference);
    assert_bounds(&difference, [r(1), r(0), r(0)], [r(2), r(1), r(1)])?;
    assert_volume_numerator(&difference, r(6));

    Ok(())
}

#[test]
fn boundary_touching_boxes_xor_use_general_path() -> HypermeshResult<()> {
    let left = box_mesh([0, 0, 0], [1, 1, 1]);
    let right = box_mesh([1, 0, 0], [2, 1, 1]);
    let refs = [left.as_ref(), right.as_ref()];

    let xor_result = boolean_operation(&refs, BooleanOp::SymmetricDifference, config())?;
    assert_output_polygons_closed(&xor_result);
    let xor = triangulate_and_resolve_certified(&xor_result)?;
    assert_no_boundary_edges(&xor);
    assert_bounds(&xor, [r(0), r(0), r(0)], [r(2), r(1), r(1)]).unwrap();
    assert_volume_numerator(&xor, r(12));

    Ok(())
}

#[test]
fn edge_touching_boxes_use_general_path() -> HypermeshResult<()> {
    let left = box_mesh([0, 0, 0], [1, 1, 1]);
    let right = box_mesh([1, 1, 0], [2, 2, 1]);
    let refs = [left.as_ref(), right.as_ref()];
    let reverse_refs = [right.as_ref(), left.as_ref()];
    let left_soup = passthrough(&left).unwrap();
    let right_soup = passthrough(&right).unwrap();
    let config = config();

    let union_result = boolean_operation(&refs, BooleanOp::Union, config)?;
    let union = triangulate_and_resolve_certified(&union_result)?;
    assert_no_boundary_edges(&union);
    assert_bounds(&union, [r(0), r(0), r(0)], [r(2), r(2), r(1)])?;
    assert_volume_numerator(&union, r(12));

    let intersection_result = boolean_operation(&refs, BooleanOp::Intersection, config)?;
    let intersection = triangulate_and_resolve_certified(&intersection_result)?;
    assert!(intersection.triangles.is_empty());

    let difference_result = boolean_operation(&refs, BooleanOp::Difference, config)?;
    let difference = triangulate_and_resolve_certified(&difference_result)?;
    assert_no_boundary_edges(&difference);
    assert_closed_triangle_soup(&difference);
    assert_bounds(&difference, [r(0), r(0), r(0)], [r(1), r(1), r(1)])?;
    assert_volume_numerator(&difference, signed_volume_numerator(&left_soup));

    let reverse_difference_result =
        boolean_operation(&reverse_refs, BooleanOp::Difference, config)?;
    let reverse_difference = triangulate_and_resolve_certified(&reverse_difference_result)?;
    assert_no_boundary_edges(&reverse_difference);
    assert_closed_triangle_soup(&reverse_difference);
    assert_bounds(&reverse_difference, [r(1), r(1), r(0)], [r(2), r(2), r(1)])?;
    assert_volume_numerator(&reverse_difference, signed_volume_numerator(&right_soup));

    let xor_result = boolean_operation(&refs, BooleanOp::SymmetricDifference, config)?;
    let xor = triangulate_and_resolve_certified(&xor_result)?;
    assert_no_boundary_edges(&xor);
    assert_volume_numerator(&xor, r(12));
    assert_bounds(&xor, [r(0), r(0), r(0)], [r(2), r(2), r(1)])?;

    let reverse_union_result = boolean_operation(&reverse_refs, BooleanOp::Union, config)?;
    let reverse_union = triangulate_and_resolve_certified(&reverse_union_result)?;
    assert_no_boundary_edges(&reverse_union);
    assert_volume_numerator(&reverse_union, r(12));
    assert_bounds(&reverse_union, [r(0), r(0), r(0)], [r(2), r(2), r(1)])?;

    let reverse_xor_result =
        boolean_operation(&reverse_refs, BooleanOp::SymmetricDifference, config)?;
    let reverse_xor = triangulate_and_resolve_certified(&reverse_xor_result)?;
    assert_no_boundary_edges(&reverse_xor);
    assert_volume_numerator(&reverse_xor, r(12));
    assert_bounds(&reverse_xor, [r(0), r(0), r(0)], [r(2), r(2), r(1)])?;

    Ok(())
}

#[test]
fn edge_touching_boxes_use_general_leaf_path() -> HypermeshResult<()> {
    let left = box_mesh([0, 0, 0], [1, 1, 1]);
    let right = box_mesh([1, 1, 0], [2, 2, 1]);
    let refs = [left.as_ref(), right.as_ref()];
    let reverse_refs = [right.as_ref(), left.as_ref()];
    let left_soup = passthrough(&left).unwrap();
    let right_soup = passthrough(&right).unwrap();
    let config = EmberConfig { max_depth: 0 };

    let union =
        triangulate_and_resolve_certified(&boolean_operation(&refs, BooleanOp::Union, config)?)?;
    assert_no_boundary_edges(&union);
    assert_bounds(&union, [r(0), r(0), r(0)], [r(2), r(2), r(1)])?;
    assert_volume_numerator(&union, r(12));

    let intersection = triangulate_and_resolve_certified(&boolean_operation(
        &refs,
        BooleanOp::Intersection,
        config,
    )?)?;
    assert!(intersection.triangles.is_empty());

    let difference = triangulate_and_resolve_certified(&boolean_operation(
        &refs,
        BooleanOp::Difference,
        config,
    )?)?;
    assert_same_shape(&difference, &left_soup);

    let reverse_difference = triangulate_and_resolve_certified(&boolean_operation(
        &reverse_refs,
        BooleanOp::Difference,
        config,
    )?)?;
    assert_same_shape(&reverse_difference, &right_soup);

    let xor = triangulate_and_resolve_certified(&boolean_operation(
        &refs,
        BooleanOp::SymmetricDifference,
        config,
    )?)?;
    assert_no_boundary_edges(&xor);
    assert_same_shape(&xor, &union);

    Ok(())
}

#[test]
fn boundary_touching_boxes_union_use_general_leaf_path() -> HypermeshResult<()> {
    let left = box_mesh([0, 0, 0], [1, 1, 1]);
    let right = box_mesh([1, 0, 0], [2, 1, 1]);
    let refs = [left.as_ref(), right.as_ref()];
    let config = EmberConfig { max_depth: 0 };

    let union =
        triangulate_and_resolve_certified(&boolean_operation(&refs, BooleanOp::Union, config)?)?;
    assert_no_boundary_edges(&union);
    assert_bounds(&union, [r(0), r(0), r(0)], [r(2), r(1), r(1)])?;
    assert_volume_numerator(&union, r(12));

    Ok(())
}

#[test]
fn boundary_touching_boxes_intersection_use_general_leaf_path() -> HypermeshResult<()> {
    let left = box_mesh([0, 0, 0], [1, 1, 1]);
    let right = box_mesh([1, 0, 0], [2, 1, 1]);
    let refs = [left.as_ref(), right.as_ref()];
    let config = EmberConfig { max_depth: 0 };

    let intersection = triangulate_and_resolve_certified(&boolean_operation(
        &refs,
        BooleanOp::Intersection,
        config,
    )?)?;
    assert!(intersection.triangles.is_empty());

    Ok(())
}

#[test]
fn boundary_touching_boxes_difference_use_general_leaf_path() -> HypermeshResult<()> {
    let left = box_mesh([0, 0, 0], [1, 1, 1]);
    let right = box_mesh([1, 0, 0], [2, 1, 1]);
    let refs = [left.as_ref(), right.as_ref()];
    let config = EmberConfig { max_depth: 0 };

    let difference = triangulate_and_resolve_certified(&boolean_operation(
        &refs,
        BooleanOp::Difference,
        config,
    )?)?;
    assert_no_boundary_edges(&difference);
    assert_volume_numerator(&difference, r(6));

    Ok(())
}

#[test]
fn boundary_touching_boxes_reverse_difference_use_general_leaf_path() -> HypermeshResult<()> {
    let left = box_mesh([0, 0, 0], [1, 1, 1]);
    let right = box_mesh([1, 0, 0], [2, 1, 1]);
    let refs = [right.as_ref(), left.as_ref()];
    let right_soup = passthrough(&right).unwrap();
    let config = EmberConfig { max_depth: 0 };

    let difference = triangulate_and_resolve_certified(&boolean_operation(
        &refs,
        BooleanOp::Difference,
        config,
    )?)?;
    assert_same_shape(&difference, &right_soup);

    Ok(())
}

#[test]
fn boundary_touching_boxes_xor_use_general_leaf_path() -> HypermeshResult<()> {
    let left = box_mesh([0, 0, 0], [1, 1, 1]);
    let right = box_mesh([1, 0, 0], [2, 1, 1]);
    let refs = [left.as_ref(), right.as_ref()];
    let config = EmberConfig { max_depth: 0 };

    let xor = triangulate_and_resolve_certified(&boolean_operation(
        &refs,
        BooleanOp::SymmetricDifference,
        config,
    )?)?;
    assert_no_boundary_edges(&xor);
    assert_bounds(&xor, [r(0), r(0), r(0)], [r(2), r(1), r(1)]).unwrap();
    assert_volume_numerator(&xor, r(12));

    Ok(())
}

#[test]
fn vertex_touching_boxes_use_general_path() -> HypermeshResult<()> {
    let left = box_mesh([0, 0, 0], [1, 1, 1]);
    let right = box_mesh([1, 1, 1], [2, 2, 2]);
    let refs = [left.as_ref(), right.as_ref()];
    let reverse_refs = [right.as_ref(), left.as_ref()];
    let left_soup = passthrough(&left).unwrap();
    let right_soup = passthrough(&right).unwrap();
    let config = config();

    let union_result = boolean_operation(&refs, BooleanOp::Union, config)?;
    let union = triangulate_and_resolve_certified(&union_result)?;
    assert_no_boundary_edges(&union);
    assert_bounds(&union, [r(0), r(0), r(0)], [r(2), r(2), r(2)])?;
    assert_volume_numerator(&union, r(12));

    let intersection_result = boolean_operation(&refs, BooleanOp::Intersection, config)?;
    let intersection = triangulate_and_resolve_certified(&intersection_result)?;
    assert!(intersection.triangles.is_empty());

    let difference_result = boolean_operation(&refs, BooleanOp::Difference, config)?;
    let difference = triangulate_and_resolve_certified(&difference_result)?;
    assert_no_boundary_edges(&difference);
    assert_closed_triangle_soup(&difference);
    assert_bounds(&difference, [r(0), r(0), r(0)], [r(1), r(1), r(1)])?;
    assert_volume_numerator(&difference, signed_volume_numerator(&left_soup));

    let reverse_difference_result =
        boolean_operation(&reverse_refs, BooleanOp::Difference, config)?;
    let reverse_difference = triangulate_and_resolve_certified(&reverse_difference_result)?;
    assert_no_boundary_edges(&reverse_difference);
    assert_closed_triangle_soup(&reverse_difference);
    assert_bounds(&reverse_difference, [r(1), r(1), r(1)], [r(2), r(2), r(2)])?;
    assert_volume_numerator(&reverse_difference, signed_volume_numerator(&right_soup));

    let xor_result = boolean_operation(&refs, BooleanOp::SymmetricDifference, config)?;
    let xor = triangulate_and_resolve_certified(&xor_result)?;
    assert_no_boundary_edges(&xor);
    assert_volume_numerator(&xor, r(12));
    assert_bounds(&xor, [r(0), r(0), r(0)], [r(2), r(2), r(2)])?;

    let reverse_union_result = boolean_operation(&reverse_refs, BooleanOp::Union, config)?;
    let reverse_union = triangulate_and_resolve_certified(&reverse_union_result)?;
    assert_no_boundary_edges(&reverse_union);
    assert_volume_numerator(&reverse_union, r(12));
    assert_bounds(&reverse_union, [r(0), r(0), r(0)], [r(2), r(2), r(2)])?;

    let reverse_xor_result =
        boolean_operation(&reverse_refs, BooleanOp::SymmetricDifference, config)?;
    let reverse_xor = triangulate_and_resolve_certified(&reverse_xor_result)?;
    assert_no_boundary_edges(&reverse_xor);
    assert_volume_numerator(&reverse_xor, r(12));
    assert_bounds(&reverse_xor, [r(0), r(0), r(0)], [r(2), r(2), r(2)])?;

    Ok(())
}

#[test]
fn vertex_touching_boxes_use_general_leaf_path() -> HypermeshResult<()> {
    let left = box_mesh([0, 0, 0], [1, 1, 1]);
    let right = box_mesh([1, 1, 1], [2, 2, 2]);
    let refs = [left.as_ref(), right.as_ref()];
    let reverse_refs = [right.as_ref(), left.as_ref()];
    let left_soup = passthrough(&left).unwrap();
    let right_soup = passthrough(&right).unwrap();
    let config = EmberConfig { max_depth: 0 };

    let union =
        triangulate_and_resolve_certified(&boolean_operation(&refs, BooleanOp::Union, config)?)?;
    assert_no_boundary_edges(&union);
    assert_bounds(&union, [r(0), r(0), r(0)], [r(2), r(2), r(2)])?;
    assert_volume_numerator(&union, r(12));

    let intersection = triangulate_and_resolve_certified(&boolean_operation(
        &refs,
        BooleanOp::Intersection,
        config,
    )?)?;
    assert!(intersection.triangles.is_empty());

    let difference = triangulate_and_resolve_certified(&boolean_operation(
        &refs,
        BooleanOp::Difference,
        config,
    )?)?;
    assert_same_shape(&difference, &left_soup);

    let reverse_difference = triangulate_and_resolve_certified(&boolean_operation(
        &reverse_refs,
        BooleanOp::Difference,
        config,
    )?)?;
    assert_same_shape(&reverse_difference, &right_soup);

    let xor = triangulate_and_resolve_certified(&boolean_operation(
        &refs,
        BooleanOp::SymmetricDifference,
        config,
    )?)?;
    assert_no_boundary_edges(&xor);
    assert_same_shape(&xor, &union);

    Ok(())
}

#[test]
fn hypermesh_identical_and_same_surface_solids_regularize() {
    let left = tetrahedron([[0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]]);
    let identical = left.clone();
    let same_surface = InputMesh {
        positions: vec![p(4, 0, 0), p(0, 0, 0), p(0, 4, 0), p(0, 0, 4)],
        triangles: vec![
            Triangle::new(1, 2, 0),
            Triangle::new(1, 0, 3),
            Triangle::new(0, 2, 3),
            Triangle::new(2, 1, 3),
        ],
    };
    let reversed_same_surface = InputMesh {
        positions: same_surface.positions.clone(),
        triangles: vec![
            Triangle::new(1, 0, 2),
            Triangle::new(1, 3, 0),
            Triangle::new(0, 3, 2),
            Triangle::new(2, 3, 1),
        ],
    };
    let left_soup = passthrough(&left).unwrap();

    for right in [&identical, &same_surface, &reversed_same_surface] {
        let union = run_op(&left, right, BooleanOp::Union).unwrap();
        assert_closed_triangle_soup(&union);
        assert_volume_numerator(&union, signed_volume_numerator(&left_soup));

        let difference = run_op(&left, right, BooleanOp::Difference).unwrap();
        assert!(difference.triangles.is_empty());
    }
}

#[test]
fn disconnected_container_uses_strict_containment_with_certified_output() {
    let outer = tetrahedron([[0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]]);
    let disjoint_shell = tetrahedron([[20, 0, 0], [21, 0, 0], [20, 1, 0], [20, 0, 1]]);
    let container = combine_meshes(&[outer.clone(), disjoint_shell.clone()]);
    let contained = tetrahedron([[1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]]);
    let container_soup = passthrough(&container).unwrap();
    let contained_soup = passthrough(&contained).unwrap();

    let union = run_certified_op(&container, &contained, BooleanOp::Union).unwrap();
    assert_no_boundary_edges(&union);
    assert_volume_numerator(&union, signed_volume_numerator(&container_soup));

    let intersection = run_certified_op(&container, &contained, BooleanOp::Intersection).unwrap();
    assert_no_boundary_edges(&intersection);
    assert_volume_numerator(&intersection, signed_volume_numerator(&contained_soup));

    let difference = run_certified_op(&contained, &container, BooleanOp::Difference).unwrap();
    assert!(difference.triangles.is_empty());
}

#[test]
fn hypermesh_affine_box_booleans_are_closed() {
    let (left, right) = affine_box_pair();

    for op in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
    ] {
        let result = run_op(&left, &right, op).unwrap();
        assert!(
            !result.triangles.is_empty(),
            "{op:?} should produce non-empty output"
        );
        assert_closed_triangle_soup(&result);
    }
}

#[test]
fn affine_box_booleans_use_exact_cell_decomposition_with_certified_output() {
    let (left, right) = affine_box_pair();

    for op in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
    ] {
        let result = run_certified_op(&left, &right, op).unwrap();
        assert!(
            !result.triangles.is_empty(),
            "{op:?} should produce non-empty output"
        );
        assert_closed_triangle_soup(&result);
    }
}

fn affine_box_pair() -> (InputMesh, InputMesh) {
    let map = |u: i32, v: i32, w: i32| [2 * u + v, 2 * v, 2 * w];
    let affine_box = |min: [i32; 3], max: [i32; 3]| {
        let corners = [
            map(min[0], min[1], min[2]),
            map(max[0], min[1], min[2]),
            map(max[0], max[1], min[2]),
            map(min[0], max[1], min[2]),
            map(min[0], min[1], max[2]),
            map(max[0], min[1], max[2]),
            map(max[0], max[1], max[2]),
            map(min[0], max[1], max[2]),
        ];
        InputMesh::new(
            corners
                .into_iter()
                .map(|[x, y, z]| p(x, y, z))
                .collect::<Vec<_>>(),
            standard_cube_triangles(),
        )
    };
    (
        affine_box([0, 0, 0], [2, 2, 2]),
        affine_box([1, 0, 0], [3, 2, 2]),
    )
}

#[test]
fn same_surface_solids_are_exact_equivalence_with_certified_output() {
    let left = tetrahedron([[0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]]);
    let same_surface = InputMesh {
        positions: vec![p(4, 0, 0), p(0, 0, 0), p(0, 4, 0), p(0, 0, 4)],
        triangles: vec![
            Triangle::new(1, 2, 0),
            Triangle::new(1, 0, 3),
            Triangle::new(0, 2, 3),
            Triangle::new(2, 1, 3),
        ],
    };
    let left_soup = passthrough(&left).unwrap();

    let union = run_certified_op(&left, &same_surface, BooleanOp::Union).unwrap();
    assert_no_boundary_edges(&union);
    assert_volume_numerator(&union, signed_volume_numerator(&left_soup));

    let difference = run_certified_op(&left, &same_surface, BooleanOp::Difference).unwrap();
    assert!(difference.triangles.is_empty());
}

#[test]
fn same_surface_solids_use_general_path() -> HypermeshResult<()> {
    let left = tetrahedron([[0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]]);
    let same_surface = InputMesh {
        positions: vec![p(4, 0, 0), p(0, 0, 0), p(0, 4, 0), p(0, 0, 4)],
        triangles: vec![
            Triangle::new(1, 2, 0),
            Triangle::new(1, 0, 3),
            Triangle::new(0, 2, 3),
            Triangle::new(2, 1, 3),
        ],
    };
    let refs = [left.as_ref(), same_surface.as_ref()];
    let config = config();

    let union_result = boolean_operation(&refs, BooleanOp::Union, config)?;
    assert_output_polygons_closed(&union_result);
    let union = triangulate_and_resolve_certified(&union_result)?;
    assert_no_boundary_edges(&union);

    let difference_result = boolean_operation(&refs, BooleanOp::Difference, config)?;
    assert_output_polygons_closed(&difference_result);
    let difference = triangulate_and_resolve_certified(&difference_result)?;
    assert!(difference.triangles.is_empty());

    Ok(())
}

#[test]
fn hypermesh_partial_face_boundary_touch_regularizes_empty_intersection() {
    let left = tetrahedron([[0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]]);
    let right = tetrahedron([[2, 2, 2], [4, 1, 1], [1, 4, 1], [3, 3, 3]]);
    let left_soup = passthrough(&left).unwrap();

    let intersection = run_op(&left, &right, BooleanOp::Intersection).unwrap();
    assert!(intersection.triangles.is_empty());

    let difference = run_op(&left, &right, BooleanOp::Difference).unwrap();
    assert_no_boundary_edges(&difference);
    assert_volume_numerator(&difference, signed_volume_numerator(&left_soup));
}

#[test]
fn partial_face_boundary_touch_uses_contact_proof_with_certified_output() {
    let left = tetrahedron([[0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]]);
    let right = tetrahedron([[2, 2, 2], [4, 1, 1], [1, 4, 1], [3, 3, 3]]);
    let left_soup = passthrough(&left).unwrap();

    let intersection = run_certified_op(&left, &right, BooleanOp::Intersection).unwrap();
    assert!(intersection.triangles.is_empty());

    let difference = run_certified_op(&left, &right, BooleanOp::Difference).unwrap();
    assert_no_boundary_edges(&difference);
    assert_volume_numerator(&difference, signed_volume_numerator(&left_soup));
}

#[test]
fn hypermesh_borrowed_multi_mesh_union_uses_slice_api() {
    let left_a = tetrahedron([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]]);
    let left_b = tetrahedron([[10, 0, 0], [11, 0, 0], [10, 1, 0], [10, 0, 1]]);
    let right = tetrahedron([[5, 0, 0], [6, 0, 0], [5, 1, 0], [5, 0, 1]]);
    let refs = [left_a.as_ref(), left_b.as_ref(), right.as_ref()];

    let union = run_op_refs(&refs, BooleanOp::Union).unwrap();
    assert_closed_triangle_soup(&union);
    assert_volume_numerator(&union, r(3));
}

#[test]
fn projected_reference_escape_case_uses_general_path() -> HypermeshResult<()> {
    let left = tetra_from_face_and_apex(p(1, 1, 1), p(1, 5, 1), p(1, 3, 5), p(0, 3, 2));
    let right = tetra_from_face_and_apex(p(4, 1, 1), p(4, 5, 1), p(4, 3, 5), p(5, 3, 2));
    let left_soup = passthrough(&left).unwrap();
    let right_soup = passthrough(&right).unwrap();

    let union = run_op(&left, &right, BooleanOp::Union)?;
    assert_no_boundary_edges(&union);
    assert_closed_triangle_soup(&union);
    assert_volume_numerator(
        &union,
        signed_volume_numerator(&left_soup) + signed_volume_numerator(&right_soup),
    );
    assert_bounds(&union, [r(0), r(1), r(1)], [r(5), r(5), r(5)])?;

    let intersection = run_op(&left, &right, BooleanOp::Intersection)?;
    assert!(intersection.triangles.is_empty());

    let difference = run_op(&left, &right, BooleanOp::Difference)?;
    assert_no_boundary_edges(&difference);
    assert_closed_triangle_soup(&difference);
    assert_volume_numerator(&difference, signed_volume_numerator(&left_soup));
    assert_bounds(&difference, [r(0), r(1), r(1)], [r(1), r(5), r(5)])?;

    let reverse_difference = run_op(&right, &left, BooleanOp::Difference)?;
    assert_no_boundary_edges(&reverse_difference);
    assert_closed_triangle_soup(&reverse_difference);
    assert_volume_numerator(&reverse_difference, signed_volume_numerator(&right_soup));
    assert_bounds(&reverse_difference, [r(4), r(1), r(1)], [r(5), r(5), r(5)])?;

    let xor = run_op(&left, &right, BooleanOp::SymmetricDifference)?;
    assert_no_boundary_edges(&xor);
    assert_closed_triangle_soup(&xor);
    assert_volume_numerator(
        &xor,
        signed_volume_numerator(&left_soup) + signed_volume_numerator(&right_soup),
    );
    assert_bounds(&xor, [r(0), r(1), r(1)], [r(5), r(5), r(5)])?;

    Ok(())
}
