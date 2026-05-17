#![no_main]

use hypermesh::exact::{
    ExactMesh, ExactPoint3, ExactReal, build_intersection_graph, classify_coplanar_triangles,
    classify_face_regions_against_opposite_planes, classify_mesh_face_pair, classify_mesh_face_pairs,
    classify_triangle_triangle, intersect_segment_with_face_plane,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let mut pos = Vec::new();
    let mut idx = Vec::new();

    for chunk in data.chunks_exact(8).take(96) {
        pos.push(i64::from_le_bytes(chunk.try_into().unwrap()));
    }

    for chunk in data.chunks_exact(2).skip(96).take(192) {
        idx.push(u16::from_le_bytes(chunk.try_into().unwrap()) as usize);
    }

    if let Ok(mesh) = ExactMesh::from_i64_triangles(&pos, &idx) {
        let _ = mesh.bounds().candidate_face_pairs(mesh.bounds());
        if !mesh.triangles().is_empty() {
            let _ = classify_mesh_face_pair(&mesh, 0, &mesh, 0);
            let _ = classify_mesh_face_pairs(&mesh, &mesh);
            if let Ok(graph) = build_intersection_graph(&mesh, &mesh) {
                let _ = graph.edge_split_plan();
                let _ = graph.graph_vertex_plan();
                let topology_plan = graph.split_topology_plan();
                let _ = topology_plan.validate();
                let face_plan = graph.face_split_plan();
                let _ = face_plan.validate_against_topology(&topology_plan);
                if let Ok(geometry_plan) = graph.face_split_geometry_plan(&mesh, &mesh) {
                    let _ = geometry_plan.validate_boundary_incidence(&mesh, &mesh);
                    let region_plan = geometry_plan.region_plan(&mesh, &mesh);
                    let _ = region_plan.validate(&mesh, &mesh);
                    let _ = classify_face_regions_against_opposite_planes(
                        &region_plan,
                        &mesh,
                        &mesh,
                    );
                }
            }
        }
    }

    if pos.len() >= 15 {
        let points = pos
            .chunks_exact(3)
            .take(5)
            .map(|coords| {
                ExactPoint3::new(
                    ExactReal::from(coords[0]),
                    ExactReal::from(coords[1]),
                    ExactReal::from(coords[2]),
                )
                .to_hyperlimit_point()
            })
            .collect::<Vec<_>>();
        let _ = intersect_segment_with_face_plane(&points, [0, 1, 2], [3, 4]);
    }

    if pos.len() >= 18 {
        let points = pos
            .chunks_exact(3)
            .take(6)
            .map(|coords| {
                ExactPoint3::new(
                    ExactReal::from(coords[0]),
                    ExactReal::from(coords[1]),
                    ExactReal::from(coords[2]),
                )
                .to_hyperlimit_point()
            })
            .collect::<Vec<_>>();
        let _ = classify_triangle_triangle(&points, [0, 1, 2], [3, 4, 5]);
        let _ = classify_coplanar_triangles(&points, [0, 1, 2], [3, 4, 5]);
    }
});
