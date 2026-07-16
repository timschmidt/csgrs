use criterion::{Criterion, criterion_group, criterion_main};
use hyperbrep::{
    BrepCoedge, BrepConstructionKind, BrepConstructionManifest, BrepEdge, BrepEdgeId,
    BrepEdgeOrientation, BrepExportFormat, BrepExportManifest, BrepExportScalarPolicy, BrepFace,
    BrepFaceId, BrepFaceTessellationManifest, BrepFeatureId, BrepImportedSurfaceFamily, BrepLoop,
    BrepLoopId, BrepLossyFloatImportReport, BrepMeshHandoffReport, BrepNurbsCurve3, BrepPcurve,
    BrepPlanarExtrusionConstruction, BrepPlanarFaceRegion, BrepPlanarRegionConstruction,
    BrepPlanarTrimLoop, BrepRationalBezier3, BrepShell, BrepShellTessellationReport,
    BrepSourceVersion, BrepSurface, BrepSurfaceId, BrepSurfaceSource, BrepTopologyFingerprint,
    BrepVertex, BrepVertexId,
};
use hypercurve::{Contour2, Curve2, CurvePath2, CurvePolicy, LineSeg2, Region2, Segment2};
use hyperlimit::{Plane3, Point2, Point3};
use hyperreal::Real;

fn r(value: i32) -> Real {
    Real::from(value)
}

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn uv(x: i32, y: i32) -> hypercurve::Point2 {
    hypercurve::Point2::new(r(x), r(y))
}

fn line2(start: hypercurve::Point2, end: hypercurve::Point2) -> Segment2 {
    Segment2::Line(LineSeg2::try_new(start, end).unwrap())
}

fn rectangle_region(width: i32, height: i32) -> Region2 {
    Region2::from_material_contours(vec![rectangle_contour(0, 0, width, height)])
}

fn rectangle_contour(min_x: i32, min_y: i32, max_x: i32, max_y: i32) -> Contour2 {
    Contour2::try_new(vec![
        line2(uv(min_x, min_y), uv(max_x, min_y)),
        line2(uv(max_x, min_y), uv(max_x, max_y)),
        line2(uv(max_x, max_y), uv(min_x, max_y)),
        line2(uv(min_x, max_y), uv(min_x, min_y)),
    ])
    .unwrap()
}

fn curve_path(points: &[(i32, i32)]) -> CurvePath2 {
    CurvePath2::try_new(
        points
            .windows(2)
            .map(|pair| {
                Curve2::from(
                    LineSeg2::try_new(uv(pair[0].0, pair[0].1), uv(pair[1].0, pair[1].1)).unwrap(),
                )
            })
            .collect(),
    )
    .unwrap()
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

fn planar_many_loop_face(loop_count: usize) -> BrepShell {
    let mut vertices = Vec::with_capacity(loop_count * 4);
    let mut edges = Vec::with_capacity(loop_count * 4);
    let mut loops = Vec::with_capacity(loop_count);
    for loop_index in 0..loop_count {
        let first_vertex = loop_index * 4;
        let first_edge = loop_index * 4;
        let x = (loop_index * 3) as i32;
        for (offset, point) in [p(x, 0, 0), p(x + 1, 0, 0), p(x + 1, 1, 0), p(x, 1, 0)]
            .into_iter()
            .enumerate()
        {
            vertices.push(BrepVertex::new(
                BrepVertexId((first_vertex + offset) as u64),
                point,
            ));
            edges.push(BrepEdge::new(
                BrepEdgeId((first_edge + offset) as u64),
                BrepVertexId((first_vertex + offset) as u64),
                BrepVertexId((first_vertex + (offset + 1) % 4) as u64),
            ));
        }
        loops.push(BrepLoop::new(
            BrepLoopId(loop_index as u64),
            (0..4)
                .map(|offset| {
                    BrepCoedge::new(
                        BrepEdgeId((first_edge + offset) as u64),
                        BrepEdgeOrientation::Forward,
                    )
                })
                .collect(),
        ));
    }
    let outer = loops.remove(0);
    BrepShell {
        vertices,
        edges,
        surfaces: vec![BrepSurface::plane(
            BrepSurfaceId(0),
            Plane3::new(p(0, 0, 1), r(0)),
            BrepSurfaceSource::ExactConstruction,
        )],
        faces: vec![BrepFace::with_inner(
            BrepFaceId(0),
            BrepSurfaceId(0),
            outer,
            loops,
        )],
    }
}

fn cube_shell() -> BrepShell {
    use BrepEdgeOrientation::{Forward as F, Reversed as R};
    BrepShell {
        vertices: vec![
            BrepVertex::new(BrepVertexId(0), p(0, 0, 0)),
            BrepVertex::new(BrepVertexId(1), p(1, 0, 0)),
            BrepVertex::new(BrepVertexId(2), p(1, 1, 0)),
            BrepVertex::new(BrepVertexId(3), p(0, 1, 0)),
            BrepVertex::new(BrepVertexId(4), p(0, 0, 1)),
            BrepVertex::new(BrepVertexId(5), p(1, 0, 1)),
            BrepVertex::new(BrepVertexId(6), p(1, 1, 1)),
            BrepVertex::new(BrepVertexId(7), p(0, 1, 1)),
        ],
        edges: vec![
            BrepEdge::new(BrepEdgeId(0), BrepVertexId(0), BrepVertexId(1)),
            BrepEdge::new(BrepEdgeId(1), BrepVertexId(1), BrepVertexId(2)),
            BrepEdge::new(BrepEdgeId(2), BrepVertexId(2), BrepVertexId(3)),
            BrepEdge::new(BrepEdgeId(3), BrepVertexId(3), BrepVertexId(0)),
            BrepEdge::new(BrepEdgeId(4), BrepVertexId(4), BrepVertexId(5)),
            BrepEdge::new(BrepEdgeId(5), BrepVertexId(5), BrepVertexId(6)),
            BrepEdge::new(BrepEdgeId(6), BrepVertexId(6), BrepVertexId(7)),
            BrepEdge::new(BrepEdgeId(7), BrepVertexId(7), BrepVertexId(4)),
            BrepEdge::new(BrepEdgeId(8), BrepVertexId(0), BrepVertexId(4)),
            BrepEdge::new(BrepEdgeId(9), BrepVertexId(1), BrepVertexId(5)),
            BrepEdge::new(BrepEdgeId(10), BrepVertexId(2), BrepVertexId(6)),
            BrepEdge::new(BrepEdgeId(11), BrepVertexId(3), BrepVertexId(7)),
        ],
        surfaces: vec![
            BrepSurface::plane(
                BrepSurfaceId(0),
                Plane3::new(p(0, 0, -1), r(0)),
                BrepSurfaceSource::ExactConstruction,
            ),
            BrepSurface::plane(
                BrepSurfaceId(1),
                Plane3::new(p(0, 0, 1), r(-1)),
                BrepSurfaceSource::ExactConstruction,
            ),
            BrepSurface::plane(
                BrepSurfaceId(2),
                Plane3::new(p(0, -1, 0), r(0)),
                BrepSurfaceSource::ExactConstruction,
            ),
            BrepSurface::plane(
                BrepSurfaceId(3),
                Plane3::new(p(0, 1, 0), r(-1)),
                BrepSurfaceSource::ExactConstruction,
            ),
            BrepSurface::plane(
                BrepSurfaceId(4),
                Plane3::new(p(-1, 0, 0), r(0)),
                BrepSurfaceSource::ExactConstruction,
            ),
            BrepSurface::plane(
                BrepSurfaceId(5),
                Plane3::new(p(1, 0, 0), r(-1)),
                BrepSurfaceSource::ExactConstruction,
            ),
        ],
        faces: vec![
            BrepFace::new(
                BrepFaceId(0),
                BrepSurfaceId(0),
                BrepLoop::new(
                    BrepLoopId(0),
                    vec![
                        BrepCoedge::new(BrepEdgeId(0), R),
                        BrepCoedge::new(BrepEdgeId(3), R),
                        BrepCoedge::new(BrepEdgeId(2), R),
                        BrepCoedge::new(BrepEdgeId(1), R),
                    ],
                ),
            ),
            BrepFace::new(
                BrepFaceId(1),
                BrepSurfaceId(1),
                BrepLoop::new(
                    BrepLoopId(1),
                    vec![
                        BrepCoedge::new(BrepEdgeId(4), F),
                        BrepCoedge::new(BrepEdgeId(5), F),
                        BrepCoedge::new(BrepEdgeId(6), F),
                        BrepCoedge::new(BrepEdgeId(7), F),
                    ],
                ),
            ),
            BrepFace::new(
                BrepFaceId(2),
                BrepSurfaceId(2),
                BrepLoop::new(
                    BrepLoopId(2),
                    vec![
                        BrepCoedge::new(BrepEdgeId(0), F),
                        BrepCoedge::new(BrepEdgeId(9), F),
                        BrepCoedge::new(BrepEdgeId(4), R),
                        BrepCoedge::new(BrepEdgeId(8), R),
                    ],
                ),
            ),
            BrepFace::new(
                BrepFaceId(3),
                BrepSurfaceId(3),
                BrepLoop::new(
                    BrepLoopId(3),
                    vec![
                        BrepCoedge::new(BrepEdgeId(2), F),
                        BrepCoedge::new(BrepEdgeId(11), F),
                        BrepCoedge::new(BrepEdgeId(6), R),
                        BrepCoedge::new(BrepEdgeId(10), R),
                    ],
                ),
            ),
            BrepFace::new(
                BrepFaceId(4),
                BrepSurfaceId(4),
                BrepLoop::new(
                    BrepLoopId(4),
                    vec![
                        BrepCoedge::new(BrepEdgeId(3), F),
                        BrepCoedge::new(BrepEdgeId(8), F),
                        BrepCoedge::new(BrepEdgeId(7), R),
                        BrepCoedge::new(BrepEdgeId(11), R),
                    ],
                ),
            ),
            BrepFace::new(
                BrepFaceId(5),
                BrepSurfaceId(5),
                BrepLoop::new(
                    BrepLoopId(5),
                    vec![
                        BrepCoedge::new(BrepEdgeId(1), F),
                        BrepCoedge::new(BrepEdgeId(10), F),
                        BrepCoedge::new(BrepEdgeId(5), R),
                        BrepCoedge::new(BrepEdgeId(9), R),
                    ],
                ),
            ),
        ],
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
    c.bench_function("hyperbrep edge agreement report", |b| {
        b.iter(|| shell.edge_agreement_report())
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
    let many_loop_shell = planar_many_loop_face(128);
    c.bench_function("hyperbrep 128-loop trim-set report", |b| {
        b.iter(|| many_loop_shell.trim_set_report(BrepFaceId(0)))
    });
    c.bench_function("hyperbrep exact face bounds report", |b| {
        b.iter(|| trim_shell.face_bounds_report(BrepFaceId(0)))
    });
    c.bench_function("hyperbrep exact face uv bounds report", |b| {
        b.iter(|| trim_shell.face_uv_bounds_report(BrepFaceId(0)))
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
    let prepared_query = trim_shell.prepare_face_query(BrepFaceId(0));
    let query_points = (0..1024).map(|i| p(i % 17, i % 31, 0)).collect::<Vec<_>>();
    let query_segments = query_points
        .iter()
        .take(128)
        .map(|point| (point, &segment_end))
        .collect::<Vec<_>>();
    c.bench_function("hyperbrep prepared face query batch report", |b| {
        b.iter(|| prepared_query.batch_report(&query_points, &query_segments))
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
    c.bench_function("hyperbrep exact retained surface handoff", |b| {
        b.iter(|| shell.exact_surface_handoff())
    });
    c.bench_function("hyperbrep exact retained solid handoff", |b| {
        b.iter(|| shell.exact_solid_handoff(None))
    });
    let cube = cube_shell();
    c.bench_function("hyperbrep exact triangle mesh handoff report", |b| {
        b.iter(|| cube.exact_triangle_mesh_handoff_report())
    });
    c.bench_function("hyperbrep physics shape handoff report", |b| {
        b.iter(|| cube.physics_shape_handoff_report())
    });
    c.bench_function("hyperbrep physics mass handoff report", |b| {
        b.iter(|| cube.physics_mass_handoff_report(r(1)))
    });
    let voxel_frame = hypervoxel::GridFrame::new(
        [r(0), r(0), r(0)],
        [r(1), r(1), r(1)],
        2,
        hypervoxel::LengthUnit::Unitless,
        Some(hypervoxel::GridSource::new("bench:cube", 1)),
    )
    .unwrap();
    let voxel_source = hypervoxel::GridSource::new("bench:cube", 1);
    c.bench_function("hyperbrep voxel handoff report", |b| {
        b.iter(|| cube.voxel_handoff_report(voxel_frame.clone(), Some(voxel_source.clone())))
    });
    let voxel_handoff = cube.voxel_handoff_report(voxel_frame.clone(), Some(voxel_source.clone()));
    let prepared_voxel_solid = voxel_handoff.prepared_triangle_solid.clone().unwrap();
    c.bench_function("hyperbrep prepared voxel solid materialization", |b| {
        b.iter(|| {
            hypervoxel::voxelize_prepared_exact_triangle_solid_mesh(
                voxel_frame.clone(),
                &prepared_voxel_solid,
                hypervoxel::MaterialRegionId(1),
                hypervoxel::VoxelizationPolicy::conservative_cover(),
            )
            .unwrap()
        })
    });
    let package_manifest = hyperbrep::BrepHandoffPackageManifest::basic()
        .with_physics_density(r(1))
        .with_voxel(hyperbrep::BrepVoxelPackageRequest {
            frame: voxel_frame,
            expected_source: Some(voxel_source),
            require_triangle_voxelization: true,
        });
    c.bench_function("hyperbrep consolidated handoff package", |b| {
        b.iter(|| cube.handoff_package_report(None, package_manifest.clone()))
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
    c.bench_function("hyperbrep generated exact planar mesh handoff", |b| {
        b.iter(|| shell.exact_planar_mesh_handoff_report())
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
    c.bench_function("hyperbrep retained shell fingerprint", |b| {
        b.iter(|| BrepTopologyFingerprint::from_shell(&shell))
    });
    let source_region = rectangle_region(64, 32);
    let source_surface = BrepSurface::plane(
        BrepSurfaceId(0),
        Plane3::new(p(0, 0, 1), r(0)),
        BrepSurfaceSource::ExactConstruction,
    );
    c.bench_function("hyperbrep planar region face construction", |b| {
        b.iter(|| {
            BrepPlanarRegionConstruction::from_region_on_surface(
                &source_region,
                source_surface.clone(),
                BrepFeatureId::new("bench:region-face").unwrap(),
                vec![BrepSourceVersion::new("region:bench", 1).unwrap()],
            )
        })
    });
    c.bench_function("hyperbrep planar region extrusion construction", |b| {
        b.iter(|| {
            BrepPlanarExtrusionConstruction::vertical_prism_from_region(
                &source_region,
                r(0),
                r(8),
                BrepFeatureId::new("bench:region-extrusion").unwrap(),
                vec![BrepSourceVersion::new("region:bench", 2).unwrap()],
            )
        })
    });
    let holed_region = Region2::new(
        vec![rectangle_contour(0, 0, 64, 32)],
        vec![rectangle_contour(8, 8, 24, 24)],
    );
    c.bench_function("hyperbrep holed region extrusion construction", |b| {
        b.iter(|| {
            BrepPlanarExtrusionConstruction::vertical_prism_from_region(
                &holed_region,
                r(0),
                r(8),
                BrepFeatureId::new("bench:holed-region-extrusion").unwrap(),
                vec![BrepSourceVersion::new("region:bench", 3).unwrap()],
            )
        })
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
    let surface_query_points = (0..1024).map(|i| p(i % 17, i % 31, i)).collect::<Vec<_>>();
    let surface_uvs = (0..1024)
        .map(|i| Point2::new(r(i % 17), r(i % 31)))
        .collect::<Vec<_>>();
    c.bench_function("hyperbrep planar surface frame uv evaluation", |b| {
        b.iter(|| {
            surface_uvs
                .iter()
                .cloned()
                .map(|uv| plane_surface.evaluate_frame_uv(uv))
                .collect::<Vec<_>>()
        })
    });
    c.bench_function("hyperbrep prepared plane surface point reports", |b| {
        b.iter(|| {
            surface_query_points
                .iter()
                .map(|point| prepared_surface.classify_point(point))
                .collect::<Vec<_>>()
        })
    });

    let pcurve_surface = BrepSurfaceId::new(103);
    let trim = BrepPlanarTrimLoop::new(pcurve_surface, rectangle_contour(0, 0, 100, 100));
    let rotated_trim = BrepPlanarTrimLoop::new(
        pcurve_surface,
        Contour2::try_new(vec![
            line2(uv(100, 100), uv(0, 100)),
            line2(uv(0, 100), uv(0, 0)),
            line2(uv(0, 0), uv(100, 0)),
            line2(uv(100, 0), uv(100, 100)),
        ])
        .unwrap(),
    );
    c.bench_function("hyperbrep planar trim image equality", |b| {
        b.iter(|| trim.image_equality_report(&rotated_trim).unwrap())
    });

    let face_region = BrepPlanarFaceRegion::try_new(
        pcurve_surface,
        vec![trim],
        vec![BrepPlanarTrimLoop::new(
            pcurve_surface,
            rectangle_contour(40, 40, 60, 60),
        )],
    )
    .unwrap();
    let query = uv(10, 10);
    let curve_policy = CurvePolicy::certified();
    c.bench_function("hyperbrep planar face point query", |b| {
        b.iter(|| {
            face_region
                .classify_uv_point(pcurve_surface, &query, &curve_policy)
                .unwrap()
        })
    });
    let prepared_face = face_region.prepare_topology_queries(&curve_policy);
    c.bench_function("hyperbrep prepared planar face point query", |b| {
        b.iter(|| {
            prepared_face
                .classify_uv_point(pcurve_surface, &query, &curve_policy)
                .unwrap()
        })
    });
    let pcurve = BrepPcurve::new(pcurve_surface, curve_path(&[(0, 100), (0, 0), (100, 0)]));
    c.bench_function("hyperbrep prepared planar face edge-use query", |b| {
        b.iter(|| prepared_face.edge_use_report(&pcurve).unwrap())
    });

    let spatial_controls = vec![p(0, 0, 0), p(1, 2, 0), p(2, 0, 2)];
    let spatial_weights = vec![r(1), r(2), r(1)];
    let spatial_bezier =
        BrepRationalBezier3::try_new(spatial_controls.clone(), spatial_weights.clone()).unwrap();
    let spatial_nurbs = BrepNurbsCurve3::try_new(
        2,
        spatial_controls,
        spatial_weights,
        vec![r(0), r(0), r(0), r(1), r(1), r(1)],
    )
    .unwrap();
    let half = (r(1) / r(2)).unwrap();
    c.bench_function("hyperbrep spatial rational Bezier point", |b| {
        b.iter(|| spatial_bezier.point_at(&half).unwrap())
    });
    c.bench_function("hyperbrep spatial NURBS point", |b| {
        b.iter(|| spatial_nurbs.point_at(&half).unwrap())
    });
}

criterion_group!(benches, bench_shell_audit);
criterion_main!(benches);
