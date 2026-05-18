#![no_main]

use hypermesh::exact::{
    ExactMesh, ExactPoint3, build_intersection_graph,
    checked_classify_face_regions_against_opposite_planes, classify_coplanar_triangles,
    classify_mesh_face_pair, classify_mesh_face_pairs,
    classify_mesh_triangle_against_retained_face_plane, classify_triangle_triangle,
    intersect_segment_with_face_plane, intersect_segment_with_retained_face_plane,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let mut pos = Vec::new();
    let mut idx = Vec::new();

    for chunk in data.chunks_exact(8).take(48) {
        pos.push(f64::from_le_bytes(chunk.try_into().unwrap()));
    }

    for chunk in data.chunks_exact(2).skip(48).take(96) {
        idx.push(u16::from_le_bytes(chunk.try_into().unwrap()) as usize);
    }

    if let Ok(mesh) = ExactMesh::from_f64_triangles(&pos, &idx) {
        assert_eq!(mesh.facts().faces.len(), mesh.triangles().len());
        for face in &mesh.facts().faces {
            let _ = (&face.plane.normal, &face.plane.offset);
        }
        let _ = mesh.bounds().candidate_face_pairs(mesh.bounds());
        if !mesh.triangles().is_empty() {
            if mesh.vertices().len() >= 2 {
                let p0 = mesh.vertices()[0].to_hyperlimit_point();
                let p1 = mesh.vertices()[1].to_hyperlimit_point();
                let _ = intersect_segment_with_retained_face_plane(
                    &mesh.facts().faces[0].plane,
                    &p0,
                    &p1,
                );
            }
            let _ = classify_mesh_face_pair(&mesh, 0, &mesh, 0);
            let _ = classify_mesh_triangle_against_retained_face_plane(&mesh, 0, &mesh, 0);
            let _ = classify_mesh_face_pairs(&mesh, &mesh);
            if let Ok(graph) = build_intersection_graph(&mesh, &mesh) {
                let _ = graph.edge_split_plan();
                let _ = graph.graph_vertex_plan();
                let topology_plan = graph.split_topology_plan();
                let _ = topology_plan.validate();
                let _ = graph.checked_graph_vertex_plan();
                let _ = graph.checked_split_topology_plan();
                let _ = graph.checked_face_split_plan();
                let face_plan = graph.face_split_plan();
                let _ = face_plan.validate_against_topology(&topology_plan);
                if let Ok(geometry_plan) = graph.face_split_geometry_plan(&mesh, &mesh) {
                    let _ = geometry_plan.validate_boundary_incidence(&mesh, &mesh);
                    let region_plan = geometry_plan.region_plan(&mesh, &mesh);
                    let _ = region_plan.validate(&mesh, &mesh);
                    let _ = checked_classify_face_regions_against_opposite_planes(
                        &region_plan,
                        &mesh,
                        &mesh,
                    );
                    #[cfg(feature = "exact-triangulation")]
                    {
                        if let Ok(triangulations) =
                            hypermesh::exact::checked_triangulate_face_regions_with_earcut(
                                &region_plan,
                                &mesh,
                                &mesh,
                            )
                        {
                            for triangulation in &triangulations {
                                let _ = triangulation.validate();
                            }
                            let _ =
                                hypermesh::exact::ExactBooleanAssemblyPlan::from_region_triangulations(
                                    &triangulations,
                                    hypermesh::exact::ExactRegionSelection::KeepAll,
                                );
                        }
                    }
                }
            }
        }
    }

    if pos.len() >= 15
        && pos.iter().take(15).all(|value| value.is_finite())
    {
        let points = pos
            .chunks_exact(3)
            .take(5)
            .filter_map(|coords| {
                ExactPoint3::from_f64_lossy([coords[0], coords[1], coords[2]], 0)
                    .ok()
                    .map(|point| point.to_hyperlimit_point())
            })
            .collect::<Vec<_>>();
        if points.len() == 5 {
            let _ = intersect_segment_with_face_plane(&points, [0, 1, 2], [3, 4]);
        }
    }

    if pos.len() >= 18 && pos.iter().take(18).all(|value| value.is_finite()) {
        let points = pos
            .chunks_exact(3)
            .take(6)
            .filter_map(|coords| {
                ExactPoint3::from_f64_lossy([coords[0], coords[1], coords[2]], 0)
                    .ok()
                    .map(|point| point.to_hyperlimit_point())
            })
            .collect::<Vec<_>>();
        if points.len() == 6 {
            let _ = classify_triangle_triangle(&points, [0, 1, 2], [3, 4, 5]);
            let _ = classify_coplanar_triangles(&points, [0, 1, 2], [3, 4, 5]);
        }
    }
});
