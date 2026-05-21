use criterion::{Criterion, criterion_group, criterion_main};
use hyperbrep::{
    BrepCoedge, BrepConstructionKind, BrepConstructionManifest, BrepEdge, BrepEdgeId,
    BrepEdgeOrientation, BrepExportFormat, BrepExportManifest, BrepExportScalarPolicy, BrepFace,
    BrepFaceId, BrepFaceTessellationManifest, BrepFeatureId, BrepImportedSurfaceFamily, BrepLoop,
    BrepLoopId, BrepLossyFloatImportReport, BrepMeshHandoffReport, BrepShell,
    BrepShellTessellationReport, BrepSourceVersion, BrepSurface, BrepSurfaceId, BrepSurfaceSource,
    BrepVertex, BrepVertexId,
};
use hyperlimit::{Plane3, Point3};
use hyperreal::Real;

fn r(value: i32) -> Real {
    Real::from(value)
}

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn paired_strip(edge_count: usize) -> BrepShell {
    let vertices = (0..=edge_count)
        .map(|i| BrepVertex::new(BrepVertexId(i as u64), p(i as i32, 0, 0)))
        .collect::<Vec<_>>();
    let edges = (0..edge_count)
        .map(|i| {
            BrepEdge::new(
                BrepEdgeId(i as u64),
                BrepVertexId(i as u64),
                BrepVertexId((i + 1) as u64),
            )
        })
        .collect::<Vec<_>>();
    let forward = (0..edge_count)
        .map(|i| BrepCoedge::new(BrepEdgeId(i as u64), BrepEdgeOrientation::Forward))
        .collect::<Vec<_>>();
    let reversed = (0..edge_count)
        .map(|i| BrepCoedge::new(BrepEdgeId(i as u64), BrepEdgeOrientation::Reversed))
        .collect::<Vec<_>>();
    BrepShell {
        vertices,
        edges,
        surfaces: vec![BrepSurface::plane(
            BrepSurfaceId(0),
            Plane3::new(p(0, 0, 1), r(0)),
            BrepSurfaceSource::ExactConstruction,
        )],
        faces: vec![
            BrepFace::new(
                BrepFaceId(0),
                BrepSurfaceId(0),
                BrepLoop::new(BrepLoopId(0), forward),
            ),
            BrepFace::new(
                BrepFaceId(1),
                BrepSurfaceId(0),
                BrepLoop::new(BrepLoopId(1), reversed),
            ),
        ],
    }
}

fn planar_ring(edge_count: usize) -> BrepShell {
    let vertices = (0..edge_count)
        .map(|i| BrepVertex::new(BrepVertexId(i as u64), p(i as i32, (i % 13) as i32, 0)))
        .collect::<Vec<_>>();
    let edges = (0..edge_count)
        .map(|i| {
            BrepEdge::new(
                BrepEdgeId(i as u64),
                BrepVertexId(i as u64),
                BrepVertexId(((i + 1) % edge_count) as u64),
            )
        })
        .collect::<Vec<_>>();
    let coedges = (0..edge_count)
        .map(|i| BrepCoedge::new(BrepEdgeId(i as u64), BrepEdgeOrientation::Forward))
        .collect::<Vec<_>>();
    BrepShell {
        vertices,
        edges,
        surfaces: vec![BrepSurface::plane(
            BrepSurfaceId(0),
            Plane3::new(p(0, 0, 1), r(0)),
            BrepSurfaceSource::ExactConstruction,
        )],
        faces: vec![BrepFace::new(
            BrepFaceId(0),
            BrepSurfaceId(0),
            BrepLoop::new(BrepLoopId(0), coedges),
        )],
    }
}

fn bench_shell_audit(c: &mut Criterion) {
    let shell = paired_strip(1024);
    c.bench_function("hyperbrep paired strip shell audit", |b| {
        b.iter(|| shell.audit_closure())
    });
    c.bench_function("hyperbrep topology validation report", |b| {
        b.iter(|| shell.validate_topology())
    });
    c.bench_function("hyperbrep shell validation report", |b| {
        b.iter(|| shell.shell_validation_report())
    });
    let trim_shell = planar_ring(1024);
    c.bench_function("hyperbrep exact shell bounds report", |b| {
        b.iter(|| trim_shell.shell_bounds_report())
    });
    c.bench_function("hyperbrep trim-loop topology report", |b| {
        b.iter(|| trim_shell.trim_set_report(BrepFaceId(0)))
    });
    c.bench_function("hyperbrep exact face bounds report", |b| {
        b.iter(|| trim_shell.face_bounds_report(BrepFaceId(0)))
    });
    c.bench_function("hyperbrep exact planar face area report", |b| {
        b.iter(|| trim_shell.face_area_report(BrepFaceId(0)))
    });
    c.bench_function("hyperbrep face aabb preflight report", |b| {
        b.iter(|| trim_shell.face_aabb_preflight(BrepFaceId(0), BrepFaceId(0)))
    });
    let query_plane = Plane3::new(p(0, 0, 1), r(0));
    c.bench_function("hyperbrep face plane preflight report", |b| {
        b.iter(|| trim_shell.face_plane_preflight(BrepFaceId(0), &query_plane))
    });
    let segment_start = p(0, 0, -1);
    let segment_end = p(0, 0, 1);
    c.bench_function("hyperbrep segment face-plane preflight report", |b| {
        b.iter(|| {
            trim_shell.segment_face_plane_preflight(BrepFaceId(0), &segment_start, &segment_end)
        })
    });
    let query_point = p(0, 0, 0);
    c.bench_function("hyperbrep point face-plane preflight report", |b| {
        b.iter(|| trim_shell.point_face_plane_preflight(BrepFaceId(0), &query_point))
    });
    c.bench_function("hyperbrep face validation report", |b| {
        b.iter(|| trim_shell.face_validation_report(BrepFaceId(0), None))
    });
    c.bench_function("hyperbrep geometry validation report", |b| {
        b.iter(|| trim_shell.geometry_validation_report(BrepFaceId(0)))
    });
    c.bench_function("hyperbrep shell volume report", |b| {
        b.iter(|| shell.shell_volume_report())
    });
    c.bench_function("hyperbrep solid readiness report", |b| {
        b.iter(|| trim_shell.solid_readiness_report(None))
    });
    c.bench_function("hyperbrep surface inventory", |b| {
        b.iter(|| hyperbrep::BrepSurfaceInventoryReport::from_surfaces(&shell.surfaces))
    });
    let manifests = shell
        .faces
        .iter()
        .map(|face| {
            let boundary_edges = face.loops().map(|face_loop| face_loop.coedges.len()).sum();
            BrepFaceTessellationManifest::exact_planar(face.id, 1022, 1024, boundary_edges, 0)
        })
        .collect::<Vec<_>>();
    c.bench_function("hyperbrep shell tessellation readiness report", |b| {
        b.iter(|| BrepShellTessellationReport::from_shell_manifests(&shell, &manifests))
    });
    c.bench_function("hyperbrep derived mesh handoff report", |b| {
        b.iter(|| BrepMeshHandoffReport::from_shell_manifests(&shell, &manifests))
    });
    let construction = BrepConstructionManifest::exact(
        BrepFeatureId::new("bench:strip").unwrap(),
        BrepConstructionKind::PlanarFace,
        vec![BrepSourceVersion::new("source:strip", 1024).unwrap()],
        &shell,
    );
    c.bench_function("hyperbrep construction provenance report", |b| {
        b.iter(|| construction.report(&shell))
    });
    c.bench_function("hyperbrep derived mesh handoff with provenance", |b| {
        b.iter(|| {
            BrepMeshHandoffReport::from_shell_manifests_with_construction(
                &shell,
                &manifests,
                Some(&construction),
            )
        })
    });
    let coordinates = (0..3072).map(|i| (i as f64) * 0.25).collect::<Vec<_>>();
    let imported_surfaces = vec![BrepImportedSurfaceFamily::Plane; 128];
    c.bench_function("hyperbrep lossy f64 import audit", |b| {
        b.iter(|| {
            BrepLossyFloatImportReport::inspect_f64(
                "bench:f64",
                &coordinates,
                &imported_surfaces,
                true,
                true,
            )
        })
    });
    let mesh_handoff = BrepMeshHandoffReport::from_shell_manifests(&shell, &manifests);
    let export = BrepExportManifest {
        format: BrepExportFormat::Obj,
        scalar_policy: BrepExportScalarPolicy::F64,
        source_object_ids: vec!["shell:bench-strip".into()],
        exported_primitives: mesh_handoff.triangle_count,
        exported_coordinates: mesh_handoff.lifted_vertex_count * 3,
        finite_exported_coordinates: mesh_handoff.lifted_vertex_count * 3,
        labels_preserved: true,
        exact_replay_declared: true,
    };
    c.bench_function("hyperbrep export certificate report", |b| {
        b.iter(|| export.report(Some(&mesh_handoff)))
    });
    let plane_surface = BrepSurface::plane(
        BrepSurfaceId(99),
        Plane3::new(p(0, 0, 1), r(-512)),
        BrepSurfaceSource::ExactConstruction,
    );
    let prepared_surface = plane_surface.prepare();
    let query_points = (0..1024)
        .map(|i| p((i % 17) as i32, (i % 31) as i32, i as i32))
        .collect::<Vec<_>>();
    c.bench_function("hyperbrep prepared plane surface point reports", |b| {
        b.iter(|| {
            query_points
                .iter()
                .map(|point| prepared_surface.classify_point(point))
                .collect::<Vec<_>>()
        })
    });
}

criterion_group!(benches, bench_shell_audit);
criterion_main!(benches);
