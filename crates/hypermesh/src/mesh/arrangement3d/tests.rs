use super::*;
use crate::mesh::arrangement3d::cell_complex::{
    ExactRegionOwnershipStatus, arrangement_cell_complex_labeling_policy,
};
use crate::mesh::arrangement3d::loop_triangulation::projected_loop_orientation;
use crate::mesh::boolean::ExactBooleanOperation;
use crate::mesh::validation::ExactMeshValidationPolicy;
use hyperlimit::{RingPointLocation, classify_point_ring_even_odd, projected_polygon_area2_value};

fn p3(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

fn q(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / &Real::from(denominator)).expect("nonzero denominator")
}

fn rational_p3(x: [i64; 2], y: [i64; 2], z: [i64; 2]) -> Point3 {
    Point3::new(q(x[0], x[1]), q(y[0], y[1]), q(z[0], z[1]))
}

fn test_face_cell(face: usize, points: Vec<Point3>) -> ArrangementFaceCell {
    ArrangementFaceCell {
        carrier: ArrangementFaceCarrier {
            side: MeshSide::Left,
            face,
            triangle: [0, 1, 2],
        },
        boundary: points
            .iter()
            .enumerate()
            .map(|(vertex, _)| ArrangementFaceCellNode::FacePlane {
                arrangement: 0,
                vertex,
            })
            .collect(),
        boundary_points: points,
        opposite: None,
    }
}

#[test]
fn arrangement_vertex_merge_index_buckets_exact_rational_points() {
    let mut vertices = Vec::new();
    let mut index = ArrangementVertexMergeIndex::default();
    let mut blockers = Vec::new();
    let point = rational_p3([1, 2], [-3, 4], [5, 6]);

    push_arrangement_vertex(
        &mut vertices,
        &mut index,
        point.clone(),
        ArrangementVertexProvenance::SourceVertex {
            side: MeshSide::Left,
            vertex: 0,
        },
        &mut blockers,
    );
    push_arrangement_vertex(
        &mut vertices,
        &mut index,
        point,
        ArrangementVertexProvenance::SourceVertex {
            side: MeshSide::Right,
            vertex: 1,
        },
        &mut blockers,
    );
    push_arrangement_vertex(
        &mut vertices,
        &mut index,
        rational_p3([2, 3], [-3, 4], [5, 6]),
        ArrangementVertexProvenance::GraphIntersection { graph_vertex: 2 },
        &mut blockers,
    );

    assert!(blockers.is_empty(), "{blockers:?}");
    assert_eq!(vertices.len(), 2);
    assert_eq!(vertices[0].provenance.len(), 2);
    assert_eq!(vertices[1].provenance.len(), 1);
    assert_eq!(index.point_key_buckets.len(), 2);
    assert!(index.unkeyed_vertices.is_empty());
}

#[test]
fn arrangement_point_uniqueness_index_buckets_exact_rational_points() {
    let mut points = Vec::new();
    let mut index = ArrangementPointUniquenessIndex::default();
    let point = rational_p3([1, 2], [-3, 4], [5, 6]);

    index.push_unique(&mut points, point.clone());
    index.push_unique(&mut points, point);
    index.push_unique(&mut points, rational_p3([2, 3], [-3, 4], [5, 6]));

    assert_eq!(points.len(), 2);
    assert_eq!(index.point_key_buckets.len(), 2);
    assert!(index.unkeyed_points.is_empty());
}

#[test]
fn arrangement_boundary_point_index_buckets_exact_rational_points() {
    let boundary_point = |side, vertex, point| ArrangementFaceCellBoundaryPoint {
        node: ArrangementFaceCellNode::Source { side, vertex },
        point,
    };
    let point = rational_p3([1, 2], [-3, 4], [5, 6]);
    let mut points = vec![
        boundary_point(MeshSide::Right, 7, point.clone()),
        boundary_point(MeshSide::Left, 1, rational_p3([2, 3], [-3, 4], [5, 6])),
    ];
    let mut index = ArrangementBoundaryPointUniquenessIndex::from_points(&points);

    index.push_unique(
        &mut points,
        boundary_point(MeshSide::Left, 0, point.clone()),
    );
    index.push_unique(
        &mut points,
        boundary_point(MeshSide::Right, 2, rational_p3([3, 4], [-3, 4], [5, 6])),
    );

    assert_eq!(points.len(), 3);
    assert_eq!(
        points[0].node,
        ArrangementFaceCellNode::Source {
            side: MeshSide::Left,
            vertex: 0
        }
    );
    assert_eq!(index.point_key_buckets.len(), 3);
    assert!(index.unkeyed_points.is_empty());
}

#[test]
fn arrangement_edge_user_index_buckets_exact_rational_edges() {
    let point_a = rational_p3([1, 2], [2, 3], [3, 4]);
    let point_b = rational_p3([-5, 6], [7, 8], [-9, 10]);
    let boundary_point = |side, vertex, point| ArrangementFaceCellBoundaryPoint {
        node: ArrangementFaceCellNode::Source { side, vertex },
        point,
    };
    let left_start = boundary_point(MeshSide::Left, 0, point_a.clone());
    let left_end = boundary_point(MeshSide::Left, 1, point_b.clone());
    let left_edge = ArrangementFaceCellBoundaryEdge {
        nodes: [left_start.node, left_end.node],
        points: Some([left_start.point, left_end.point]),
    };
    let right_start = boundary_point(MeshSide::Right, 4, point_b);
    let right_end = boundary_point(MeshSide::Right, 5, point_a);
    let right_edge = ArrangementFaceCellBoundaryEdge {
        nodes: [right_start.node, right_end.node],
        points: Some([right_start.point, right_end.point]),
    };
    let mut index = ArrangementEdgeUserIndex::default();

    index.push(left_edge, 0);
    index.push(right_edge, 1);

    assert_eq!(index.edge_users.len(), 1);
    assert_eq!(index.edge_users[0].1, vec![0, 1]);
    assert_eq!(index.point_key_buckets.len(), 1);
    assert_eq!(index.node_key_buckets.len(), 2);
    assert!(index.unkeyed_edges.is_empty());
}

fn tetrahedron_i64(a: [i64; 3], b: [i64; 3], c: [i64; 3], d: [i64; 3]) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2],
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap()
}

fn two_tetrahedra_i64(tetrahedra: &[[[i64; 3]; 4]]) -> ExactMesh {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for tetrahedron in tetrahedra {
        let start = vertices.len() / 3;
        for point in tetrahedron {
            vertices.extend(point);
        }
        triangles.extend([
            start,
            start + 2,
            start + 1,
            start,
            start + 1,
            start + 3,
            start + 1,
            start + 2,
            start + 3,
            start + 2,
            start,
            start + 3,
        ]);
    }
    ExactMesh::from_i64_triangles(&vertices, &triangles).unwrap()
}

#[test]
fn arrangement_from_retained_graph_matches_mesh_construction() {
    let left = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let right = tetrahedron_i64([1, 1, 1], [5, 1, 1], [1, 5, 1], [1, 1, 5]);
    let graph = crate::mesh::graph::build_unvalidated_intersection_graph(&left, &right).unwrap();

    let from_meshes = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();
    let from_graph = ExactArrangement::from_intersection_graph_with_policy(
        graph,
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();

    assert_eq!(from_graph, from_meshes);
    from_graph.validate().unwrap();
}

fn tetrahedron_with_reversed_inner_i64(outer: [[i64; 3]; 4], inner: [[i64; 3]; 4]) -> ExactMesh {
    let mut vertices = Vec::new();
    for point in outer.iter().chain(inner.iter()) {
        vertices.extend(point);
    }
    let outer_start = 0usize;
    let inner_start = 4usize;
    let shell_triangles = [[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]];
    let mut triangles = Vec::new();
    for tri in shell_triangles {
        triangles.extend([
            outer_start + tri[0],
            outer_start + tri[1],
            outer_start + tri[2],
        ]);
    }
    for tri in shell_triangles {
        triangles.extend([
            inner_start + tri[0],
            inner_start + tri[2],
            inner_start + tri[1],
        ]);
    }
    ExactMesh::from_i64_triangles(&vertices, &triangles).unwrap()
}

fn open_triangle_i64(a: [i64; 3], b: [i64; 3], c: [i64; 3]) -> ExactMesh {
    ExactMesh::from_i64_triangles_with_policy(
        &[a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2]],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
}

#[test]
fn shell_replay_triangulates_concave_face_cell_with_exact_earcut() {
    let points = vec![
        p3(0, 0, 0),
        p3(4, 0, 0),
        p3(4, 1, 0),
        p3(1, 1, 0),
        p3(1, 4, 0),
        p3(0, 4, 0),
    ];
    let cell = ArrangementFaceCell {
        carrier: ArrangementFaceCarrier {
            side: MeshSide::Left,
            face: 0,
            triangle: [0, 1, 2],
        },
        boundary: points
            .iter()
            .enumerate()
            .map(|(vertex, _)| ArrangementFaceCellNode::FacePlane {
                arrangement: 0,
                vertex,
            })
            .collect(),
        boundary_points: points.clone(),
        opposite: None,
    };
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();

    triangulate_exact_loop_group(
        std::slice::from_ref(&cell.boundary_points),
        &mut vertices,
        &mut triangles,
    )
    .unwrap();

    assert_eq!(vertices.len(), points.len());
    assert_eq!(triangles.len(), points.len() - 2);
    let expected_orientation = projected_loop_orientation(&points, CoplanarProjection::Xy).unwrap();
    let mut triangle_area_sum = Real::from(0);
    for triangle in &triangles {
        let triangle_points = [
            vertices[triangle.0[0]].clone(),
            vertices[triangle.0[1]].clone(),
            vertices[triangle.0[2]].clone(),
        ];
        assert_eq!(
            projected_loop_orientation(&triangle_points, CoplanarProjection::Xy).unwrap(),
            expected_orientation
        );
        triangle_area_sum +=
            &projected_polygon_area2_value(&triangle_points, CoplanarProjection::Xy);
    }
    assert_eq!(
        compare_reals(
            &triangle_area_sum,
            &projected_polygon_area2_value(&points, CoplanarProjection::Xy),
        )
        .value(),
        Some(Ordering::Equal)
    );
}

#[test]
fn shell_replay_triangulates_grouped_hole_carrier_loops() {
    let loops = [
        (0, vec![p3(0, 0, 1), p3(4, 0, 1), p3(4, 4, 1), p3(0, 4, 1)]),
        (10, vec![p3(1, 3, 1), p3(3, 3, 1), p3(3, 1, 1), p3(1, 1, 1)]),
        (1, vec![p3(0, 4, 0), p3(4, 4, 0), p3(4, 0, 0), p3(0, 0, 0)]),
        (11, vec![p3(1, 1, 0), p3(3, 1, 0), p3(3, 3, 0), p3(1, 3, 0)]),
        (2, vec![p3(0, 0, 0), p3(4, 0, 0), p3(4, 0, 1), p3(0, 0, 1)]),
        (3, vec![p3(4, 0, 0), p3(4, 4, 0), p3(4, 4, 1), p3(4, 0, 1)]),
        (4, vec![p3(4, 4, 0), p3(0, 4, 0), p3(0, 4, 1), p3(4, 4, 1)]),
        (5, vec![p3(0, 4, 0), p3(0, 0, 0), p3(0, 0, 1), p3(0, 4, 1)]),
        (6, vec![p3(1, 1, 0), p3(1, 1, 1), p3(3, 1, 1), p3(3, 1, 0)]),
        (7, vec![p3(3, 1, 0), p3(3, 1, 1), p3(3, 3, 1), p3(3, 3, 0)]),
        (8, vec![p3(3, 3, 0), p3(3, 3, 1), p3(1, 3, 1), p3(1, 3, 0)]),
        (9, vec![p3(1, 3, 0), p3(1, 3, 1), p3(1, 1, 1), p3(1, 1, 0)]),
    ];
    let face_cells = loops
        .into_iter()
        .map(|(face, points)| test_face_cell(face, points))
        .collect::<Vec<_>>();
    let shell = ArrangementRegion {
        face_cells: (0..face_cells.len()).collect(),
        adjacent_face_cells: Vec::new(),
        edge_incidences: Vec::new(),
        oriented_sides: Vec::new(),
        boundary_edges: 0,
        non_manifold_edges: 0,
        source_sides: vec![MeshSide::Left],
        closed: true,
        manifold: true,
    };

    let mesh = shell_region_mesh(&shell, &face_cells).unwrap();

    assert!(mesh.facts().mesh.closed_manifold, "{:?}", mesh.facts().mesh);
    assert_ne!(
        exact_mesh_orientation(&mesh),
        ClosedMeshOrientation::Unknown
    );
    assert!(
        mesh.vertices().iter().all(|point| point3_equal(
            point,
            &Point3::new(Real::from(2), Real::from(2), Real::from(1))
        )
        .value()
            == Some(false)),
        "shell replay must preserve annular cap holes instead of fan-filling them"
    );
}

#[test]
fn disjoint_tetrahedra_build_complete_arrangement_cells() {
    let left = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
    let right = tetrahedron_i64([3, 0, 0], [4, 0, 0], [3, 1, 0], [3, 0, 1]);

    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();

    assert!(
        arrangement.blockers.is_empty(),
        "{:?}",
        arrangement.blockers
    );
    assert_eq!(arrangement.face_cells.len(), 8);
    assert_eq!(arrangement.vertices.len(), 8);
    assert_eq!(
        arrangement
            .shells_or_regions
            .as_ref()
            .map(|regions| regions.len()),
        Some(2)
    );
    let regions = arrangement.shells_or_regions.as_ref().unwrap();
    assert!(regions.iter().all(|region| region.closed));
    assert!(regions.iter().all(|region| region.manifold));
    assert!(regions.iter().all(|region| region.face_cells.len() == 4));
    assert!(
        regions
            .iter()
            .all(|region| region.adjacent_face_cells.len() == 6)
    );
    assert!(
        regions
            .iter()
            .all(|region| region.edge_incidences.len() == 6)
    );
    assert!(
        regions.iter().all(
            |region| region
                .edge_incidences
                .iter()
                .all(|incidence| incidence.face_cells.len() == 2
                    && !incidence.boundary
                    && !incidence.non_manifold)
        )
    );
    assert!(
        regions
            .iter()
            .all(|region| region.oriented_sides.len() == 4)
    );
    assert!(regions.iter().all(|region| {
        region
            .oriented_sides
            .iter()
            .all(|side| side.boundary.len() == 3)
    }));
    let topology_report = arrangement.topology_assembly_report_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );
    topology_report.validate().unwrap();
    assert_eq!(topology_report.arrangement_regions, 2);
    assert_eq!(topology_report.arrangement_region_face_cells, 8);
    assert_eq!(topology_report.arrangement_region_adjacencies, 12);
    assert_eq!(topology_report.arrangement_region_edge_incidences, 12);
    assert_eq!(topology_report.arrangement_region_oriented_sides, 8);
    assert_eq!(topology_report.arrangement_region_boundary_edges, 0);
    assert_eq!(topology_report.arrangement_region_non_manifold_edges, 0);
    let mut stale_region_report = topology_report.clone();
    stale_region_report.arrangement_region_face_cells -= 1;
    assert_eq!(
        stale_region_report.validate(),
        Err(ExactArrangementBlocker::UnresolvedRegionClassification)
    );
    let mut stale_region = arrangement.clone();
    stale_region.shells_or_regions.as_mut().unwrap()[0]
        .face_cells
        .push(0);
    assert_eq!(
        stale_region.validate(),
        Err(ExactArrangementBlocker::NonManifoldCellComplex)
    );
    let volume_regions = arrangement
        .volume_regions
        .as_ref()
        .expect("closed shell arrangement should expose volume regions");
    assert_eq!(volume_regions.len(), 3);
    assert!(volume_regions[0].exterior);
    assert_eq!(volume_regions[0].boundary_shells, vec![0, 1]);
    assert_eq!(volume_regions[1].boundary_shells, vec![0]);
    assert_eq!(volume_regions[2].boundary_shells, vec![1]);
    let volume_adjacencies = arrangement
        .volume_adjacencies
        .as_ref()
        .expect("closed shell arrangement should expose volume adjacencies");
    assert_eq!(volume_adjacencies.len(), 2);
    assert!(
        volume_adjacencies
            .iter()
            .all(|adjacency| adjacency.exterior_volume == 0
                && adjacency.separating_face_cells.len() == 4
                && adjacency.oriented_face_sides.len() == 4
                && adjacency.oriented_face_sides.iter().all(|side| {
                    side.exterior_volume == adjacency.exterior_volume
                        && side.interior_volume == adjacency.interior_volume
                        && adjacency.separating_face_cells.contains(&side.face_cell)
                }))
    );
    assert_eq!(
        arrangement.freshness_against_sources_with_policy(
            &left,
            &right,
            ExactRegularizationPolicy::REGULARIZED_SOLID
        ),
        ExactArrangementFreshness::Current
    );
}

#[test]
fn volume_graph_validation_rejects_missing_shell_adjacency() {
    let left = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
    let right = tetrahedron_i64([3, 0, 0], [4, 0, 0], [3, 1, 0], [3, 0, 1]);
    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();
    let shell_regions = arrangement.shells_or_regions.as_ref().unwrap();
    let volume_regions = arrangement.volume_regions.as_ref().unwrap();
    let mut stale_adjacencies = arrangement.volume_adjacencies.clone().unwrap();
    stale_adjacencies.pop();
    let mut blockers = Vec::new();

    validate_arrangement_volume_graph(
        shell_regions,
        &arrangement.face_cells,
        Some(volume_regions),
        Some(&stale_adjacencies),
        &mut blockers,
    );

    assert_eq!(
        blockers,
        vec![ExactArrangementBlocker::NonManifoldCellComplex]
    );
}

#[test]
fn arrangement_validate_rejects_missing_volume_adjacency() {
    let left = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
    let right = tetrahedron_i64([3, 0, 0], [4, 0, 0], [3, 1, 0], [3, 0, 1]);
    let mut arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();
    arrangement.validate().unwrap();
    arrangement.volume_adjacencies.as_mut().unwrap().pop();

    assert_eq!(
        arrangement.validate(),
        Err(ExactArrangementBlocker::NonManifoldCellComplex)
    );
    assert_eq!(
        arrangement.freshness_against_sources_with_policy(
            &left,
            &right,
            ExactRegularizationPolicy::REGULARIZED_SOLID
        ),
        ExactArrangementFreshness::StaleArrangement
    );
}

#[test]
fn arrangement_validate_rejects_missing_unblocked_topology() {
    let left = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
    let right = tetrahedron_i64([3, 0, 0], [4, 0, 0], [3, 1, 0], [3, 0, 1]);
    let mut arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();
    arrangement.validate().unwrap();
    arrangement.topology = None;
    arrangement.blockers.clear();

    assert_eq!(
        arrangement.validate(),
        Err(ExactArrangementBlocker::UnresolvedIntersection)
    );
}

#[test]
fn volume_graph_validation_rejects_relabelled_source_sides() {
    let left = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
    let right = tetrahedron_i64([3, 0, 0], [4, 0, 0], [3, 1, 0], [3, 0, 1]);
    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();
    let shell_regions = arrangement.shells_or_regions.as_ref().unwrap();
    let mut stale_volume_regions = arrangement.volume_regions.clone().unwrap();
    stale_volume_regions[1].source_sides = vec![MeshSide::Right];
    let volume_adjacencies = arrangement.volume_adjacencies.as_ref().unwrap();
    let mut blockers = Vec::new();

    validate_arrangement_volume_graph(
        shell_regions,
        &arrangement.face_cells,
        Some(&stale_volume_regions),
        Some(volume_adjacencies),
        &mut blockers,
    );

    assert_eq!(
        blockers,
        vec![ExactArrangementBlocker::NonManifoldCellComplex]
    );
}

#[test]
fn volume_graph_validation_rejects_extra_boundary_shell() {
    let left = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
    let right = tetrahedron_i64([3, 0, 0], [4, 0, 0], [3, 1, 0], [3, 0, 1]);
    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();
    let shell_regions = arrangement.shells_or_regions.as_ref().unwrap();
    let mut stale_volume_regions = arrangement.volume_regions.clone().unwrap();
    stale_volume_regions[1].boundary_shells.push(1);
    let volume_adjacencies = arrangement.volume_adjacencies.as_ref().unwrap();
    let mut blockers = Vec::new();

    validate_arrangement_volume_graph(
        shell_regions,
        &arrangement.face_cells,
        Some(&stale_volume_regions),
        Some(volume_adjacencies),
        &mut blockers,
    );

    assert_eq!(
        blockers,
        vec![ExactArrangementBlocker::NonManifoldCellComplex]
    );
}

#[test]
fn volume_graph_validation_rejects_duplicate_separating_face() {
    let left = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
    let right = tetrahedron_i64([3, 0, 0], [4, 0, 0], [3, 1, 0], [3, 0, 1]);
    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();
    let shell_regions = arrangement.shells_or_regions.as_ref().unwrap();
    let volume_regions = arrangement.volume_regions.as_ref().unwrap();
    let mut stale_adjacencies = arrangement.volume_adjacencies.clone().unwrap();
    let duplicate = stale_adjacencies[0].separating_face_cells[0];
    stale_adjacencies[0].separating_face_cells.push(duplicate);
    let mut blockers = Vec::new();

    validate_arrangement_volume_graph(
        shell_regions,
        &arrangement.face_cells,
        Some(volume_regions),
        Some(&stale_adjacencies),
        &mut blockers,
    );

    assert_eq!(
        blockers,
        vec![ExactArrangementBlocker::NonManifoldCellComplex]
    );
}

#[test]
fn volume_graph_validation_rejects_duplicate_boundary_shell() {
    let left = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
    let right = tetrahedron_i64([3, 0, 0], [4, 0, 0], [3, 1, 0], [3, 0, 1]);
    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();
    let shell_regions = arrangement.shells_or_regions.as_ref().unwrap();
    let mut stale_volume_regions = arrangement.volume_regions.clone().unwrap();
    stale_volume_regions[1].boundary_shells.push(0);
    let volume_adjacencies = arrangement.volume_adjacencies.as_ref().unwrap();
    let mut blockers = Vec::new();

    validate_arrangement_volume_graph(
        shell_regions,
        &arrangement.face_cells,
        Some(&stale_volume_regions),
        Some(volume_adjacencies),
        &mut blockers,
    );

    assert_eq!(
        blockers,
        vec![ExactArrangementBlocker::NonManifoldCellComplex]
    );
}

#[test]
fn label_regions_rejects_relabelled_volume_source_sides() {
    let left = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
    let right = tetrahedron_i64([3, 0, 0], [4, 0, 0], [3, 1, 0], [3, 0, 1]);
    let mut arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();
    arrangement.volume_regions.as_mut().unwrap()[1].source_sides = vec![MeshSide::Right];

    assert_eq!(
        arrangement
            .label_regions(ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap_err(),
        ExactArrangementBlocker::NonManifoldCellComplex
    );
}

#[test]
fn label_regions_rejects_stale_volume_boundary_shells() {
    let left = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
    let right = tetrahedron_i64([3, 0, 0], [4, 0, 0], [3, 1, 0], [3, 0, 1]);
    let mut arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();
    arrangement.volume_regions.as_mut().unwrap()[1]
        .boundary_shells
        .push(1);

    assert_eq!(
        arrangement
            .label_regions(ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap_err(),
        ExactArrangementBlocker::NonManifoldCellComplex
    );
}

#[test]
fn region_edge_users_merge_exact_geometric_edge_coincidence() {
    let cell = |side, vertices: [usize; 3], points: [Point3; 3]| ArrangementFaceCell {
        carrier: ArrangementFaceCarrier {
            side,
            face: 0,
            triangle: vertices,
        },
        boundary: vertices
            .iter()
            .map(|vertex| ArrangementFaceCellNode::Source {
                side,
                vertex: *vertex,
            })
            .collect(),
        boundary_points: points.to_vec(),
        opposite: None,
    };
    let left = cell(
        MeshSide::Left,
        [0, 1, 2],
        [p3(0, 0, 0), p3(1, 0, 0), p3(0, 1, 0)],
    );
    let right = cell(
        MeshSide::Right,
        [3, 4, 5],
        [p3(1, 0, 0), p3(0, 0, 0), p3(1, 1, 0)],
    );

    let edge_users = arrangement_edge_users(&[left, right], &mut Vec::new());
    let shared = edge_users
        .iter()
        .find(|(_, users)| users.as_slice() == [0, 1])
        .expect("exact coincident geometric edge should share one incidence");

    assert_eq!(shared.1, vec![0, 1]);
}

#[test]
fn region_edge_users_split_collinear_geometric_subedges() {
    let cell = |side, vertices: [usize; 3], points: [Point3; 3]| ArrangementFaceCell {
        carrier: ArrangementFaceCarrier {
            side,
            face: 0,
            triangle: vertices,
        },
        boundary: vertices
            .iter()
            .map(|vertex| ArrangementFaceCellNode::Source {
                side,
                vertex: *vertex,
            })
            .collect(),
        boundary_points: points.to_vec(),
        opposite: None,
    };
    let long = cell(
        MeshSide::Left,
        [0, 1, 2],
        [p3(0, 0, 0), p3(2, 0, 0), p3(0, 1, 0)],
    );
    let first_half = cell(
        MeshSide::Right,
        [3, 4, 5],
        [p3(1, 0, 0), p3(0, 0, 0), p3(1, -1, 0)],
    );
    let second_half = cell(
        MeshSide::Right,
        [6, 7, 8],
        [p3(2, 0, 0), p3(1, 0, 0), p3(2, -1, 0)],
    );

    let mut blockers = Vec::new();
    let edge_users = arrangement_edge_users(&[long, first_half, second_half], &mut blockers);
    let shared_subedges = edge_users
        .iter()
        .filter(|(_, users)| users.len() == 2 && users.contains(&0))
        .map(|(_, users)| users.clone())
        .collect::<Vec<_>>();

    assert!(blockers.is_empty(), "{blockers:?}");
    assert_eq!(shared_subedges, vec![vec![0, 1], vec![0, 2]]);
}

#[test]
fn shell_containment_classifier_uses_consistent_exact_witnesses() {
    let container = tetrahedron_i64([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);

    assert_eq!(
        classify_shell_witnesses_against_container(&[p3(1, 1, 1), p3(2, 1, 1)], &container),
        ShellContainmentRelation::Inside
    );
    assert_eq!(
        classify_shell_witnesses_against_container(&[p3(20, 20, 20), p3(21, 20, 20)], &container),
        ShellContainmentRelation::Outside
    );
    assert_eq!(
        classify_shell_witnesses_against_container(&[p3(0, 0, 0), p3(1, 1, 1)], &container),
        ShellContainmentRelation::Inside
    );
    assert_eq!(
        classify_shell_witnesses_against_container(&[p3(0, 0, 0)], &container),
        ShellContainmentRelation::Boundary
    );
    assert_eq!(
        classify_shell_witnesses_against_container(&[p3(1, 1, 1), p3(20, 20, 20)], &container),
        ShellContainmentRelation::Boundary
    );
}

#[test]
fn shell_containment_classifier_requires_convex_certified_container() {
    let container = two_tetrahedra_i64(&[
        [[0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]],
        [[20, 0, 0], [21, 0, 0], [20, 1, 0], [20, 0, 1]],
    ]);

    assert_eq!(
        classify_shell_witnesses_against_container(&[p3(1, 1, 1)], &container),
        ShellContainmentRelation::Unknown
    );
}

#[test]
fn shell_region_witnesses_include_exact_face_interior_points() {
    let left = tetrahedron_i64([0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]);
    let right = tetrahedron_i64([10, 0, 0], [16, 0, 0], [10, 6, 0], [10, 0, 6]);
    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();
    let shell = &arrangement.shells_or_regions.as_ref().unwrap()[0];
    let witnesses = shell_region_witnesses(shell, &arrangement.face_cells, &left, &right);
    let boundary_points = shell
        .face_cells
        .iter()
        .flat_map(|&cell| arrangement.face_cells[cell].boundary_points.iter())
        .collect::<Vec<_>>();

    assert!(witnesses.len() > boundary_points.len() / 3);
    assert!(witnesses.iter().any(|witness| {
        boundary_points
            .iter()
            .all(|boundary| point3_equal(witness, boundary).value() == Some(false))
    }));
}

#[test]
fn split_face_region_uses_strict_loop_interior_witness() {
    let mesh = open_triangle_i64([0, 0, 0], [10, 0, 0], [0, 10, 0]);
    let boundary_points = [
        p3(0, 0, 0),
        p3(6, 0, 0),
        p3(6, 2, 0),
        p3(2, 2, 0),
        p3(2, 6, 0),
        p3(0, 6, 0),
    ];
    let region = FaceRegionBoundary {
        side: MeshSide::Left,
        face: 0,
        triangle: [0, 1, 2],
        boundary: boundary_points
            .iter()
            .cloned()
            .map(|point| FaceSplitBoundaryNode::FaceInterior { point })
            .collect(),
    };
    let projected = boundary_points
        .iter()
        .map(|point| project_point3(point, CoplanarProjection::Xy))
        .collect::<Vec<_>>();
    let boundary_average = representative_from_boundary_nodes(&region.boundary).unwrap();
    assert_eq!(
        classify_point_ring_even_odd(
            &projected,
            &project_point3(&boundary_average, CoplanarProjection::Xy)
        )
        .value(),
        Some(RingPointLocation::Outside)
    );
    let mut blockers = Vec::new();

    let representative = face_region_interior_representative(&region, &mesh, &mesh, &mut blockers)
        .expect("concave split face should have a strict exact witness");

    assert!(blockers.is_empty(), "{blockers:?}");
    assert_eq!(
        classify_point_ring_even_odd(
            &projected,
            &project_point3(&representative, CoplanarProjection::Xy)
        )
        .value(),
        Some(RingPointLocation::Inside)
    );
}

#[test]
fn nested_tetrahedra_build_nested_volume_regions() {
    let left = tetrahedron_i64([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let right = tetrahedron_i64([1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]);

    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();

    assert!(
        arrangement.blockers.is_empty(),
        "{:?}",
        arrangement.blockers
    );
    let volume_regions = arrangement
        .volume_regions
        .as_ref()
        .expect("nested closed shells should expose volume regions");
    assert_eq!(volume_regions.len(), 3);
    assert!(volume_regions[0].exterior);
    assert_eq!(volume_regions[0].source_sides, Vec::<MeshSide>::new());
    assert_eq!(volume_regions[1].source_sides, vec![MeshSide::Left]);
    assert_eq!(
        volume_regions[2].source_sides,
        vec![MeshSide::Left, MeshSide::Right]
    );
    let volume_adjacencies = arrangement
        .volume_adjacencies
        .as_ref()
        .expect("nested closed shells should expose volume adjacencies");
    assert_eq!(volume_adjacencies.len(), 2);
    assert_eq!(volume_adjacencies[0].exterior_volume, 0);
    assert_eq!(volume_adjacencies[0].interior_volume, 1);
    assert_eq!(volume_adjacencies[1].exterior_volume, 1);
    assert_eq!(volume_adjacencies[1].interior_volume, 2);

    let union = arrangement
        .clone()
        .label_regions(ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap()
        .select_with_policy(
            ExactBooleanOperation::Union,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .unwrap();
    assert_eq!(union.selected_volume_regions, vec![1, 2]);
    assert_eq!(union.selected_faces.len(), 4);
    assert!(
        union
            .selected_face_orientations
            .iter()
            .all(|orientation| !orientation.reverse && orientation.from_volume_adjacency)
    );
    let intersection = arrangement
        .clone()
        .label_regions(ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap()
        .select_with_policy(
            ExactBooleanOperation::Intersection,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .unwrap();
    assert_eq!(intersection.selected_volume_regions, vec![2]);
    assert_eq!(intersection.selected_faces.len(), 4);
    assert!(
        intersection
            .selected_face_orientations
            .iter()
            .all(|orientation| !orientation.reverse && orientation.from_volume_adjacency)
    );
    let difference = arrangement
        .label_regions(ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap()
        .select_with_policy(
            ExactBooleanOperation::Difference,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .unwrap();
    assert_eq!(difference.selected_volume_regions, vec![1]);
    assert_eq!(difference.selected_faces.len(), 8);
    assert_eq!(
        difference
            .selected_face_orientations
            .iter()
            .filter(|orientation| orientation.reverse)
            .count(),
        4
    );
    assert!(
        difference
            .selected_face_orientations
            .iter()
            .all(|orientation| orientation.from_volume_adjacency)
    );
}

#[test]
fn region_ownership_report_certifies_volume_resolved_nested_solids() {
    let left = tetrahedron_i64([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let right = tetrahedron_i64([1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]);
    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();

    let topology_report = arrangement.topology_assembly_report_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );
    topology_report.validate().unwrap();
    assert_eq!(
        topology_report.status,
        ExactTopologyAssemblyStatus::Complete
    );
    assert_eq!(topology_report.volume_regions, 3);
    assert_eq!(topology_report.volume_adjacencies, 2);
    assert_eq!(topology_report.volume_adjacency_face_sides, 8);
    assert_eq!(topology_report.volume_adjacency_separating_faces, 8);
    let mut stale_volume_topology = topology_report.clone();
    stale_volume_topology.volume_adjacency_separating_faces = 0;
    assert_eq!(
        stale_volume_topology.validate(),
        Err(ExactArrangementBlocker::UnresolvedRegionClassification)
    );

    let report = arrangement
        .region_ownership_report_with_policy(
            &left,
            &right,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .unwrap();

    assert_eq!(report.status, ExactRegionOwnershipStatus::VolumeResolved);
    assert!(report.is_resolved());
    assert_eq!(report.freshness, ExactLabeledCellComplexFreshness::Current);
    assert!(report.blockers.is_empty(), "{:?}", report.blockers);
    let (face_cell_boundary_nodes, face_cell_boundary_points) =
        arrangement_face_cell_boundary_counts(&arrangement.face_cells);
    assert_eq!(report.face_cell_boundary_nodes, face_cell_boundary_nodes);
    assert_eq!(report.face_cell_boundary_points, face_cell_boundary_points);
    let mut stale_ownership_face_boundary = report.clone();
    stale_ownership_face_boundary.face_cell_boundary_nodes += 1;
    assert_eq!(
        stale_ownership_face_boundary.validate(),
        Err(ExactArrangementBlocker::UnresolvedRegionClassification)
    );
    assert_eq!(report.volume_regions, 3);
    assert_eq!(report.exterior_volume_regions, 1);
    assert_eq!(report.left_owned_volumes, 2);
    assert_eq!(report.right_owned_volumes, 1);
    assert_eq!(report.shared_owned_volumes, 1);
    assert_eq!(report.unowned_bounded_volumes, 0);
    assert_eq!(report.volume_adjacencies, 2);
    assert_eq!(report.volume_adjacency_face_sides, 8);
    assert_eq!(report.volume_adjacency_separating_faces, 8);
    let mut stale_volume_proof = report.clone();
    stale_volume_proof.volume_adjacency_face_sides = 0;
    assert_eq!(
        stale_volume_proof.validate(),
        Err(ExactArrangementBlocker::UnresolvedRegionClassification)
    );
}

#[test]
fn region_ownership_report_retains_blocked_open_shell_reason() {
    let left = open_triangle_i64([0, 0, 0], [2, 0, 0], [0, 2, 0]);
    let right = open_triangle_i64([4, 0, 0], [6, 0, 0], [4, 2, 0]);
    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();

    let report = arrangement
        .region_ownership_report_with_policy(
            &left,
            &right,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .unwrap();

    assert_eq!(report.status, ExactRegionOwnershipStatus::Blocked);
    assert!(!report.is_resolved());
    assert_eq!(report.freshness, ExactLabeledCellComplexFreshness::Current);
    assert!(
        report
            .blockers
            .contains(&ExactArrangementBlocker::NonManifoldCellComplex),
        "{:?}",
        report.blockers
    );
    assert_eq!(report.face_cells, 2);
    assert_eq!(report.left_boundary_faces, 1);
    assert_eq!(report.right_boundary_faces, 1);
    assert_eq!(report.volume_regions, 0);
}

#[test]
fn coincident_closed_shell_builds_mixed_source_volume_region() {
    let left = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let right = left.clone();

    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();

    assert!(
        arrangement.blockers.is_empty(),
        "{:?}",
        arrangement.blockers
    );
    let volume_regions = arrangement
        .volume_regions
        .as_ref()
        .expect("coincident closed shells should expose volume regions");
    assert_eq!(volume_regions.len(), 2);
    assert!(volume_regions[0].exterior);
    assert_eq!(
        volume_regions[1].source_sides,
        vec![MeshSide::Left, MeshSide::Right]
    );

    let union = arrangement
        .clone()
        .label_regions(ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap()
        .select_with_policy(
            ExactBooleanOperation::Union,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .unwrap();
    assert_eq!(union.selected_volume_regions, vec![1]);
    assert_eq!(union.selected_faces.len(), 4);
    let simplified_union = union
        .simplify_exact_with_policy(ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap();
    assert_eq!(simplified_union.faces.len(), 4);
    assert_eq!(simplified_union.duplicate_cells_removed, 0);
    assert_eq!(simplified_union.triangulate().unwrap().triangles().len(), 4);

    let intersection = arrangement
        .clone()
        .label_regions(ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap()
        .select_with_policy(
            ExactBooleanOperation::Intersection,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .unwrap();
    assert_eq!(intersection.selected_volume_regions, vec![1]);
    assert_eq!(intersection.selected_faces.len(), 4);
    let simplified_intersection = intersection
        .simplify_exact_with_policy(ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap();
    assert_eq!(simplified_intersection.faces.len(), 4);
    assert_eq!(simplified_intersection.duplicate_cells_removed, 0);
    assert_eq!(
        simplified_intersection
            .triangulate()
            .unwrap()
            .triangles()
            .len(),
        4
    );

    let difference = arrangement
        .label_regions(ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap()
        .select_with_policy(
            ExactBooleanOperation::Difference,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .unwrap();
    assert!(difference.selected_volume_regions.is_empty());
    assert!(difference.selected_faces.is_empty());
}

#[test]
fn nested_tetrahedron_with_two_inner_shells_builds_volume_tree() {
    let left = tetrahedron_i64([0, 0, 0], [20, 0, 0], [0, 20, 0], [0, 0, 20]);
    let right = two_tetrahedra_i64(&[
        [[1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]],
        [[5, 1, 1], [6, 1, 1], [5, 2, 1], [5, 1, 2]],
    ]);

    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();

    assert!(
        arrangement.blockers.is_empty(),
        "{:?}",
        arrangement.blockers
    );
    let volume_regions = arrangement
        .volume_regions
        .as_ref()
        .expect("nested closed shells should expose volume regions");
    assert_eq!(volume_regions.len(), 4);
    assert!(volume_regions[0].exterior);
    assert_eq!(volume_regions[0].source_sides, Vec::<MeshSide>::new());
    let left_volume = volume_regions
        .iter()
        .find(|region| region.source_sides == [MeshSide::Left])
        .expect("outer shell interior should be left-owned");
    assert_eq!(left_volume.boundary_shells.len(), 3);
    assert_eq!(
        volume_regions
            .iter()
            .filter(|region| region.source_sides == [MeshSide::Left, MeshSide::Right])
            .count(),
        2
    );
    let volume_adjacencies = arrangement
        .volume_adjacencies
        .as_ref()
        .expect("nested closed shells should expose volume adjacencies");
    assert_eq!(volume_adjacencies.len(), 3);
    assert_eq!(
        volume_adjacencies
            .iter()
            .filter(|adjacency| adjacency.exterior_volume == left_volume.index)
            .count(),
        2
    );

    let difference = arrangement
        .label_regions(ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap()
        .select_with_policy(
            ExactBooleanOperation::Difference,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .unwrap();
    assert_eq!(difference.selected_volume_regions, vec![left_volume.index]);
}

#[test]
fn same_source_reversed_nested_shell_builds_cavity_volume() {
    let left = tetrahedron_with_reversed_inner_i64(
        [[0, 0, 0], [20, 0, 0], [0, 20, 0], [0, 0, 20]],
        [[1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]],
    );
    let right = tetrahedron_i64([30, 0, 0], [31, 0, 0], [30, 1, 0], [30, 0, 1]);

    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();

    assert!(
        arrangement.blockers.is_empty(),
        "{:?}",
        arrangement.blockers
    );
    let volume_regions = arrangement
        .volume_regions
        .as_ref()
        .expect("closed shells should expose volume regions");
    assert_eq!(volume_regions.len(), 4);
    assert_eq!(
        volume_regions
            .iter()
            .filter(|region| region.exterior && region.source_sides.is_empty())
            .count(),
        1
    );
    let cavity = volume_regions
        .iter()
        .find(|region| !region.exterior && region.source_sides.is_empty())
        .expect("oppositely oriented nested left shell should bound an empty cavity");
    let left_volume = volume_regions
        .iter()
        .find(|region| region.source_sides == [MeshSide::Left])
        .expect("between outer shell and cavity should remain left-owned");
    assert!(left_volume.boundary_shells.len() >= 2);

    let union = arrangement
        .clone()
        .label_regions(ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap()
        .select_with_policy(
            ExactBooleanOperation::Union,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .unwrap();
    assert!(!union.selected_volume_regions.contains(&cavity.index));
    assert!(union.selected_volume_regions.contains(&left_volume.index));
    let difference = arrangement
        .label_regions(ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap()
        .select_with_policy(
            ExactBooleanOperation::Difference,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .unwrap();
    assert_eq!(difference.selected_volume_regions, vec![left_volume.index]);
}

#[test]
fn same_source_same_orientation_nested_shell_reports_nonmanifold_volume() {
    let left = two_tetrahedra_i64(&[
        [[0, 0, 0], [20, 0, 0], [0, 20, 0], [0, 0, 20]],
        [[1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]],
    ]);
    let right = tetrahedron_i64([30, 0, 0], [31, 0, 0], [30, 1, 0], [30, 0, 1]);

    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();

    assert!(
        arrangement
            .blockers
            .contains(&ExactArrangementBlocker::NonManifoldCellComplex),
        "{:?}",
        arrangement.blockers
    );
    assert!(
        arrangement.volume_regions.is_some(),
        "blocker volume graph should still be retained"
    );
    assert_eq!(
        arrangement
            .label_regions(ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap_err(),
        ExactArrangementBlocker::NonManifoldCellComplex
    );
}

#[test]
fn arrangement_pipeline_labels_selects_and_simplifies() {
    let left = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
    let right = tetrahedron_i64([3, 0, 0], [4, 0, 0], [3, 1, 0], [3, 0, 1]);
    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();

    let labeled = arrangement
        .label_regions(ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap();
    assert!(
        labeled
            .validate_against_sources(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .is_ok()
    );
    let selected = labeled
        .select_with_policy(
            ExactBooleanOperation::Union,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .unwrap();
    assert_eq!(selected.selected_faces.len(), 8);
    assert_eq!(selected.volume_regions.len(), 3);
    assert_eq!(selected.selected_volume_regions, vec![1, 2]);
    assert!(
        selected
            .validate_against_sources(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .is_ok()
    );

    let intersection = arrangement
        .label_regions(ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap()
        .select_with_policy(
            ExactBooleanOperation::Intersection,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .unwrap();
    assert!(intersection.selected_volume_regions.is_empty());
    let difference = arrangement
        .label_regions(ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap()
        .select_with_policy(
            ExactBooleanOperation::Difference,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .unwrap();
    assert_eq!(difference.selected_volume_regions, vec![1]);

    let simplified = selected
        .simplify_exact_with_policy(ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap();
    assert_eq!(simplified.faces.len(), 8);
    assert_eq!(simplified.duplicate_cells_removed, 0);
    assert!(
        simplified
            .validate_against_sources(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .is_ok()
    );
    let mesh = simplified.triangulate().unwrap();
    assert_eq!(mesh.vertices().len(), 8);
    assert_eq!(mesh.triangles().len(), 8);
}

#[test]
fn regularized_solid_arrangement_blocks_open_shell_regions() {
    let left = open_triangle_i64([0, 0, 0], [2, 0, 0], [0, 2, 0]);
    let right = open_triangle_i64([4, 0, 0], [6, 0, 0], [4, 2, 0]);

    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();

    assert!(
        arrangement
            .blockers
            .contains(&ExactArrangementBlocker::NonManifoldCellComplex),
        "{:?}",
        arrangement.blockers
    );
    let regions = arrangement
        .shells_or_regions
        .as_ref()
        .expect("arrangement should retain region blockers");
    assert_eq!(regions.len(), 2);
    assert!(regions.iter().all(|region| region.boundary_edges == 3));
    assert!(
        regions
            .iter()
            .all(|region| region.edge_incidences.len() == 3)
    );
    assert!(regions.iter().all(|region| {
        region
            .edge_incidences
            .iter()
            .all(|incidence| incidence.boundary && incidence.face_cells.len() == 1)
    }));
    assert!(
        regions
            .iter()
            .all(|region| region.oriented_sides.len() == 1)
    );
    assert!(regions.iter().all(|region| region.non_manifold_edges == 0));
    assert!(regions.iter().all(|region| !region.closed));
    assert!(regions.iter().all(|region| region.manifold));
    let topology_report = arrangement.topology_assembly_report_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );
    topology_report.validate().unwrap();
    assert_eq!(topology_report.arrangement_regions, 2);
    assert_eq!(topology_report.arrangement_region_face_cells, 2);
    assert_eq!(topology_report.arrangement_region_edge_incidences, 6);
    assert_eq!(topology_report.arrangement_region_oriented_sides, 2);
    assert_eq!(topology_report.arrangement_region_boundary_edges, 6);
    assert_eq!(topology_report.arrangement_region_non_manifold_edges, 0);
    assert!(arrangement.volume_regions.is_none());
    assert!(arrangement.volume_adjacencies.is_none());
    assert!(
        regions
            .iter()
            .any(|region| region.source_sides == vec![MeshSide::Left])
    );
    assert!(
        regions
            .iter()
            .any(|region| region.source_sides == vec![MeshSide::Right])
    );
}

#[test]
fn coplanar_overlapping_triangles_retain_carrier_plane_overlay() {
    let left = open_triangle_i64([0, 0, 0], [4, 0, 0], [0, 4, 0]);
    let right = open_triangle_i64([1, 1, 0], [5, 1, 0], [1, 5, 0]);

    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::RETAIN_ARTIFACTS,
    )
    .unwrap();
    assert!(
        arrangement.blockers.is_empty(),
        "{:?}",
        arrangement.blockers
    );

    assert_eq!(arrangement.carrier_plane_overlays.len(), 1);
    let overlay = &arrangement.carrier_plane_overlays[0].overlay;
    assert!(overlay.blockers.is_empty(), "{:?}", overlay.blockers);
    assert!(!overlay.arrangement.faces.is_empty());
    assert!(
        overlay
            .faces
            .iter()
            .any(|face| face.in_left && face.in_right)
    );
    assert!(
        arrangement.face_cells.iter().any(|cell| cell
            .boundary
            .iter()
            .any(|node| matches!(node, ArrangementFaceCellNode::CarrierPlane { .. }))),
        "coplanar overlay cells should be lifted into 3D face cells"
    );
    assert!(
        arrangement
            .vertices
            .iter()
            .any(|vertex| vertex.provenance.iter().any(|provenance| matches!(
                provenance,
                ArrangementVertexProvenance::CarrierPlaneVertex { .. }
            )))
    );
    assert!(arrangement.edges.iter().any(|edge| {
        edge.provenance
            .iter()
            .any(|provenance| matches!(provenance, ArrangementEdgeProvenance::CarrierPlane { .. }))
    }));
    assert!(arrangement.face_cells.len() > 2);
}

#[test]
fn selected_regions_materialize_open_coplanar_overlap_without_winding_blocker() {
    let left = open_triangle_i64([0, 0, 0], [4, 0, 0], [0, 4, 0]);
    let right = open_triangle_i64([1, 1, 0], [5, 1, 0], [1, 5, 0]);

    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::RETAIN_ARTIFACTS,
    )
    .unwrap();
    assert!(
        arrangement.blockers.is_empty(),
        "{:?}",
        arrangement.blockers
    );

    let operation = ExactBooleanOperation::SelectedRegions(
        crate::mesh::boolean::region::ExactRegionSelection::KeepLeft,
    );
    let labeling_policy = arrangement_cell_complex_labeling_policy(
        &arrangement,
        Some(operation),
        ExactRegularizationPolicy::RETAIN_ARTIFACTS,
    );
    let selected = arrangement
        .label_regions(labeling_policy)
        .unwrap()
        .select_with_policy(operation, ExactRegularizationPolicy::RETAIN_ARTIFACTS)
        .unwrap();
    assert!(selected.blockers.is_empty(), "{:?}", selected.blockers);
    assert!(
        selected
            .selected_faces
            .iter()
            .all(|face| selected.faces[*face].cell.carrier.side == MeshSide::Left)
    );

    let simplified = selected
        .simplify_exact_with_policy(ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap();
    assert!(simplified.blockers.is_empty(), "{:?}", simplified.blockers);
    let mesh = simplified.triangulate().unwrap();
    assert!(!mesh.triangles().is_empty());
    assert!(
        mesh.facts().mesh.boundary_edges > 0,
        "{:?}",
        mesh.facts().mesh
    );
}

#[test]
fn blocking_policy_reports_open_coplanar_overlap_winding_blockers() {
    let left = open_triangle_i64([0, 0, 0], [4, 0, 0], [0, 4, 0]);
    let right = open_triangle_i64([1, 1, 0], [5, 1, 0], [1, 5, 0]);

    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();

    assert!(
        arrangement
            .blockers
            .contains(&ExactArrangementBlocker::UnresolvedRegionClassification),
        "{:?}",
        arrangement.blockers
    );
}

#[test]
fn retained_artifact_policy_keeps_open_sheet_complex_without_regularization_blockers() {
    let left = open_triangle_i64([0, 0, 0], [4, 0, 0], [0, 4, 0]);
    let right = open_triangle_i64([1, -1, -1], [1, 3, 1], [1, 3, -1]);

    let retained = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::RETAIN_ARTIFACTS,
    )
    .unwrap();

    assert!(retained.blockers.is_empty(), "{:?}", retained.blockers);
    let regions = retained
        .shells_or_regions
        .as_ref()
        .expect("retained arrangement should keep sheet blockers");
    assert_eq!(regions.len(), 1);
    assert!(!regions[0].closed);
    assert!(!regions[0].manifold);
    assert_eq!(
        regions[0].source_sides,
        vec![MeshSide::Left, MeshSide::Right]
    );
    assert!(regions[0].boundary_edges > 0);
    assert!(regions[0].non_manifold_edges > 0);

    let regularized = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();
    assert!(
        regularized
            .blockers
            .contains(&ExactArrangementBlocker::UnregularizedOpenSheetComplex),
        "{:?}",
        regularized.blockers
    );
    assert!(
        regularized
            .blockers
            .contains(&ExactArrangementBlocker::UnregularizedCoincidentSheetComplex),
        "{:?}",
        regularized.blockers
    );
}

#[test]
fn crossing_triangles_build_face_plane_arrangement_cells() {
    let left = open_triangle_i64([0, 0, 0], [4, 0, 0], [0, 4, 0]);
    let right = open_triangle_i64([1, -1, -1], [1, 3, 1], [1, 3, -1]);

    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::RETAIN_ARTIFACTS,
    )
    .unwrap();

    assert!(!arrangement.face_plane_arrangements.is_empty());
    assert!(
        arrangement
            .face_plane_arrangements
            .iter()
            .all(|face_arrangement| !face_arrangement.arrangement.faces.is_empty())
    );
    assert!(
        arrangement
            .face_cells
            .iter()
            .any(|cell| cell.boundary.iter().any(|node| matches!(
                node,
                ArrangementFaceCellNode::FacePlane { .. } | ArrangementFaceCellNode::Graph { .. }
            ))),
        "non-coplanar split cells should be lifted into 3D face cells"
    );
    assert!(
        arrangement
            .vertices
            .iter()
            .any(|vertex| vertex.provenance.iter().any(|provenance| matches!(
                provenance,
                ArrangementVertexProvenance::FacePlaneVertex { .. }
            )))
    );
    assert!(arrangement.edges.iter().any(|edge| {
        edge.provenance
            .iter()
            .any(|provenance| matches!(provenance, ArrangementEdgeProvenance::FacePlane { .. }))
    }));
    assert!(arrangement.face_cells.len() > 2);
}

#[test]
fn lower_dimensional_policy_controls_coplanar_touch_artifacts() {
    let left = open_triangle_i64([0, 0, 0], [2, 0, 0], [0, 2, 0]);
    let right = open_triangle_i64([2, 0, 0], [4, 0, 0], [2, 2, 0]);

    let dropped = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();
    assert!(dropped.lower_dimensional_artifacts.is_empty());

    let retained = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::RETAIN_ARTIFACTS,
    )
    .unwrap();
    assert!(
        retained
            .lower_dimensional_artifacts
            .iter()
            .any(|artifact| matches!(
                artifact,
                ArrangementLowerDimensionalArtifact::PointContact { .. }
            )),
        "{:?}",
        retained.lower_dimensional_artifacts
    );
    assert_eq!(
        retained.freshness_against_sources_with_policy(
            &left,
            &right,
            ExactRegularizationPolicy::RETAIN_ARTIFACTS
        ),
        ExactArrangementFreshness::Current
    );
    let topology_report = retained.topology_assembly_report_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::RETAIN_ARTIFACTS,
    );
    topology_report.validate().unwrap();
    assert_eq!(
        topology_report.lower_dimensional_artifacts,
        retained.lower_dimensional_artifacts.len()
    );
    assert!(
        topology_report.lower_dimensional_point_contacts > 0,
        "{topology_report:?}"
    );
    assert_eq!(topology_report.lower_dimensional_edge_endpoints, 0);
    let ownership_report = retained
        .region_ownership_report_with_policy(
            &left,
            &right,
            ExactRegularizationPolicy::RETAIN_ARTIFACTS,
        )
        .unwrap();
    ownership_report.validate().unwrap();
    assert_eq!(
        ownership_report.lower_dimensional_artifacts,
        topology_report.lower_dimensional_artifacts
    );
    assert_eq!(
        ownership_report.lower_dimensional_point_contacts,
        topology_report.lower_dimensional_point_contacts
    );
    let mut stale_topology_shape = topology_report.clone();
    stale_topology_shape.lower_dimensional_point_contacts = 0;
    assert_eq!(
        stale_topology_shape.validate(),
        Err(ExactArrangementBlocker::UnresolvedRegionClassification)
    );

    let mut duplicated = retained.clone();
    duplicated
        .lower_dimensional_artifacts
        .push(retained.lower_dimensional_artifacts[0].clone());
    assert_eq!(
        duplicated.validate(),
        Err(ExactArrangementBlocker::NonManifoldCellComplex)
    );

    let mut stale_face_pair = retained.clone();
    match &mut stale_face_pair.lower_dimensional_artifacts[0] {
        ArrangementLowerDimensionalArtifact::PointContact { left_face, .. }
        | ArrangementLowerDimensionalArtifact::EdgeContact { left_face, .. } => {
            *left_face = usize::MAX;
        }
    }
    assert_eq!(
        stale_face_pair.validate(),
        Err(ExactArrangementBlocker::NonManifoldCellComplex)
    );

    let mut off_source_face = retained.clone();
    let point_artifact = off_source_face
        .lower_dimensional_artifacts
        .iter_mut()
        .find(|artifact| {
            matches!(
                artifact,
                ArrangementLowerDimensionalArtifact::PointContact { .. }
            )
        })
        .unwrap();
    match point_artifact {
        ArrangementLowerDimensionalArtifact::PointContact { point, .. } => {
            *point = p3(2, 0, 1);
        }
        ArrangementLowerDimensionalArtifact::EdgeContact { .. } => unreachable!(),
    }
    assert_eq!(
        off_source_face.freshness_against_sources_with_policy(
            &left,
            &right,
            ExactRegularizationPolicy::RETAIN_ARTIFACTS
        ),
        ExactArrangementFreshness::StaleArrangement
    );
}

#[test]
fn lower_dimensional_policy_retains_noncoplanar_point_touch() {
    let left = open_triangle_i64([0, 0, 0], [2, 0, 0], [0, 2, 0]);
    let right = open_triangle_i64([0, 0, 0], [0, -2, 0], [0, 0, 2]);

    let retained = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::RETAIN_ARTIFACTS,
    )
    .unwrap();

    assert!(
        retained
            .lower_dimensional_artifacts
            .iter()
            .any(|artifact| matches!(
                artifact,
                ArrangementLowerDimensionalArtifact::PointContact { .. }
            )),
        "{:?}",
        retained.lower_dimensional_artifacts
    );
    assert!(
        retained.face_plane_arrangements.is_empty(),
        "point-only contact should not create positive-area face-plane cells"
    );
}

#[test]
fn lower_dimensional_policy_ignores_endpoint_on_plane_outside_triangle() {
    let left = open_triangle_i64([0, 3, 3], [1, 1, 1], [1, 3, 4]);
    let right = open_triangle_i64([0, 0, 0], [0, 4, 0], [0, 0, 4]);
    let outside_endpoint = p3(0, 3, 3);

    let graph = crate::mesh::graph::build_unvalidated_intersection_graph(&left, &right).unwrap();
    assert!(
        graph
            .face_pairs
            .iter()
            .flat_map(|pair| pair.events.iter())
            .any(|event| matches!(
                event,
                crate::mesh::graph::IntersectionEvent::SegmentPlane {
                    relation: hyperlimit::SegmentPlaneRelation::EndpointOnPlane,
                    point: Some(point),
                    ..
                } if point3_equal(point, &outside_endpoint).value() == Some(true)
            )),
        "{graph:?}"
    );

    let retained = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::RETAIN_ARTIFACTS,
    )
    .unwrap();

    assert!(
        retained
            .lower_dimensional_artifacts
            .iter()
            .all(|artifact| !matches!(
                artifact,
                ArrangementLowerDimensionalArtifact::PointContact { point, .. }
                    if point3_equal(point, &outside_endpoint).value() == Some(true)
            )),
        "{:?}",
        retained.lower_dimensional_artifacts
    );
}

#[test]
fn lower_dimensional_policy_retains_noncoplanar_edge_touch() {
    let left = open_triangle_i64([0, 0, 0], [2, 0, 0], [0, 2, 0]);
    let right = open_triangle_i64([0, 0, 0], [2, 0, 0], [0, 0, 2]);

    let dropped = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();
    assert!(dropped.lower_dimensional_artifacts.is_empty());

    let retained = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::RETAIN_ARTIFACTS,
    )
    .unwrap();

    assert!(
        retained
            .lower_dimensional_artifacts
            .iter()
            .any(|artifact| matches!(
                artifact,
                ArrangementLowerDimensionalArtifact::EdgeContact { .. }
            )),
        "{:?}",
        retained.lower_dimensional_artifacts
    );
    assert_eq!(
        retained.freshness_against_sources_with_policy(
            &left,
            &right,
            ExactRegularizationPolicy::RETAIN_ARTIFACTS
        ),
        ExactArrangementFreshness::Current
    );
    let topology_report = retained.topology_assembly_report_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::RETAIN_ARTIFACTS,
    );
    topology_report.validate().unwrap();
    assert!(
        topology_report.lower_dimensional_edge_contacts > 0,
        "{topology_report:?}"
    );
    assert_eq!(
        topology_report.lower_dimensional_edge_endpoints,
        topology_report.lower_dimensional_edge_contacts * 2
    );
    let ownership_report = retained
        .region_ownership_report_with_policy(
            &left,
            &right,
            ExactRegularizationPolicy::RETAIN_ARTIFACTS,
        )
        .unwrap();
    ownership_report.validate().unwrap();
    assert_eq!(
        ownership_report.lower_dimensional_edge_contacts,
        topology_report.lower_dimensional_edge_contacts
    );
    let mut stale_ownership_shape = ownership_report;
    stale_ownership_shape.lower_dimensional_edge_endpoints = 0;
    assert_eq!(
        stale_ownership_shape.validate(),
        Err(ExactArrangementBlocker::UnresolvedRegionClassification)
    );
}

#[test]
fn topology_assembly_report_certifies_current_arrangement_bridge() {
    let left = tetrahedron_i64([0, 0, 0], [2, 0, 0], [0, 2, 0], [0, 0, 2]);
    let right = tetrahedron_i64([1, 0, 0], [3, 0, 0], [1, 2, 0], [1, 0, 2]);
    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();

    let report = arrangement.topology_assembly_report_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );

    assert_eq!(report.status, ExactTopologyAssemblyStatus::Complete);
    assert!(report.status.is_complete());
    assert!(report.is_complete());
    assert_eq!(report.freshness, ExactArrangementFreshness::Current);
    assert!(report.blockers.is_empty(), "{:?}", report.blockers);
    assert_eq!(report.graph_face_pairs, arrangement.graph.face_pairs.len());
    assert_eq!(report.graph_events, arrangement.graph.event_count());
    assert_eq!(report.arrangement_vertices, arrangement.vertices.len());
    assert_eq!(report.arrangement_edges, arrangement.edges.len());
    assert_eq!(report.arrangement_face_cells, arrangement.face_cells.len());
    let (face_cell_boundary_nodes, face_cell_boundary_points) =
        arrangement_face_cell_boundary_counts(&arrangement.face_cells);
    assert_eq!(
        report.arrangement_face_cell_boundary_nodes,
        face_cell_boundary_nodes
    );
    assert_eq!(
        report.arrangement_face_cell_boundary_points,
        face_cell_boundary_points
    );
    let mut stale_face_boundary_report = report.clone();
    stale_face_boundary_report.arrangement_face_cell_boundary_points += 1;
    assert_eq!(
        stale_face_boundary_report.validate(),
        Err(ExactArrangementBlocker::UnresolvedRegionClassification)
    );
    let mut stale_face_boundary_arrangement = arrangement.clone();
    stale_face_boundary_arrangement.face_cells[0]
        .boundary_points
        .pop();
    assert_eq!(
        stale_face_boundary_arrangement.validate(),
        Err(ExactArrangementBlocker::NonManifoldCellComplex)
    );
    assert_eq!(
        report.split_graph_vertices,
        arrangement.topology.as_ref().unwrap().graph_vertices.len()
    );
    assert!(report.split_edge_chains > 0);
    assert!(report.split_graph_vertex_references >= report.split_edge_chains);
    let mut stale_split_report = report.clone();
    stale_split_report.split_graph_vertex_references = stale_split_report.split_edge_chains - 1;
    assert_eq!(
        stale_split_report.validate(),
        Err(ExactArrangementBlocker::UnresolvedRegionClassification)
    );
    assert_eq!(
        report.region_boundaries,
        arrangement.region_plan.as_ref().unwrap().regions.len()
    );
    assert!(report.region_boundary_nodes >= report.region_boundaries * 3);
    let mut stale_region_boundary_report = report.clone();
    stale_region_boundary_report.region_boundary_nodes = report.region_boundaries * 3 - 1;
    assert_eq!(
        stale_region_boundary_report.validate(),
        Err(ExactArrangementBlocker::UnresolvedRegionClassification)
    );
    assert_eq!(report.volume_regions, 0);
    assert_eq!(report.volume_adjacencies, 0);
    assert_eq!(report.volume_adjacency_face_sides, 0);
    assert_eq!(report.volume_adjacency_separating_faces, 0);
}

#[test]
fn topology_assembly_report_retains_blocked_bridge_reason() {
    let left = open_triangle_i64([0, 0, 0], [2, 0, 0], [0, 2, 0]);
    let right = open_triangle_i64([0, 0, 0], [2, 0, 0], [0, 0, 2]);
    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();

    let report = arrangement.topology_assembly_report_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );

    assert_eq!(report.freshness, ExactArrangementFreshness::Current);
    assert!(!report.is_complete());
    assert_eq!(
        report.status,
        ExactTopologyAssemblyStatus::ArrangementBlocked
    );
    assert!(
        report
            .blockers
            .contains(&ExactArrangementBlocker::NonManifoldCellComplex),
        "{:?}",
        report.blockers
    );
    assert_eq!(report.graph_face_pairs, arrangement.graph.face_pairs.len());
    assert_eq!(report.lower_dimensional_artifacts, 0);
}

#[test]
fn lower_dimensional_policy_retains_noncoplanar_partial_edge_touch() {
    let left = open_triangle_i64([-1, 0, 0], [3, 0, 0], [-1, 2, 0]);
    let right = open_triangle_i64([0, 0, 0], [2, 0, 0], [0, 0, 2]);

    let retained = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::RETAIN_ARTIFACTS,
    )
    .unwrap();

    let expected_start = p3(0, 0, 0);
    let expected_end = p3(2, 0, 0);
    assert!(
        retained
            .lower_dimensional_artifacts
            .iter()
            .any(|artifact| matches!(
                artifact,
                ArrangementLowerDimensionalArtifact::EdgeContact { endpoints, .. }
                    if point3_equal(&endpoints[0], &expected_start).value() == Some(true)
                        && point3_equal(&endpoints[1], &expected_end).value() == Some(true)
            )),
        "{:?}",
        retained.lower_dimensional_artifacts
    );
    assert_eq!(
        retained.freshness_against_sources_with_policy(
            &left,
            &right,
            ExactRegularizationPolicy::RETAIN_ARTIFACTS
        ),
        ExactArrangementFreshness::Current
    );

    let mut stale_edge_source = retained.clone();
    let edge_artifact = stale_edge_source
        .lower_dimensional_artifacts
        .iter_mut()
        .find(|artifact| {
            matches!(
                artifact,
                ArrangementLowerDimensionalArtifact::EdgeContact { .. }
            )
        })
        .unwrap();
    match edge_artifact {
        ArrangementLowerDimensionalArtifact::EdgeContact { endpoints, .. } => {
            endpoints[1] = p3(3, 0, 0);
        }
        ArrangementLowerDimensionalArtifact::PointContact { .. } => unreachable!(),
    }
    assert_eq!(
        stale_edge_source.freshness_against_sources_with_policy(
            &left,
            &right,
            ExactRegularizationPolicy::RETAIN_ARTIFACTS
        ),
        ExactArrangementFreshness::StaleArrangement
    );
}
