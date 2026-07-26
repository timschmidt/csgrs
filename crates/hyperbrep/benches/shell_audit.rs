use criterion::{Criterion, criterion_group, criterion_main};
use hyperbrep::{
    BrepCoedge, BrepEdge, BrepEdgeId, BrepEdgeOrientation, BrepFace, BrepFaceId, BrepLoop,
    BrepLoopId, BrepNurbsCurve3, BrepPcurve, BrepPlanarExtrusionConstruction, BrepPlanarFaceRegion,
    BrepPlanarRegionConstruction, BrepPlanarTrimLoop, BrepRationalBezier3, BrepShell, BrepSurface,
    BrepSurfaceId, BrepVertex, BrepVertexId,
};
use hypercurve::{Contour2, Curve2, CurvePath2, CurvePolicy, CurveRegion2, LineSeg2, Segment2};
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

fn curve_region(material: Vec<Contour2>, holes: Vec<Contour2>) -> CurveRegion2 {
    CurveRegion2::try_from_native_contours(material, holes, &CurvePolicy::certified()).unwrap()
}

fn rectangle_region(width: i32, height: i32) -> CurveRegion2 {
    curve_region(vec![rectangle_contour(0, 0, width, height)], Vec::new())
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
            BrepSurface::plane(BrepSurfaceId(0), Plane3::new(p(0, 0, -1), r(0))),
            BrepSurface::plane(BrepSurfaceId(1), Plane3::new(p(0, 0, 1), r(-1))),
            BrepSurface::plane(BrepSurfaceId(2), Plane3::new(p(0, -1, 0), r(0))),
            BrepSurface::plane(BrepSurfaceId(3), Plane3::new(p(0, 1, 0), r(-1))),
            BrepSurface::plane(BrepSurfaceId(4), Plane3::new(p(-1, 0, 0), r(0))),
            BrepSurface::plane(BrepSurfaceId(5), Plane3::new(p(1, 0, 0), r(-1))),
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
        b.iter(|| shell.closure_report())
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
        b.iter(|| trim_shell.face_validation_report(BrepFaceId(0)))
    });
    c.bench_function("hyperbrep geometry validation report", |b| {
        b.iter(|| trim_shell.geometry_validation_report(BrepFaceId(0)))
    });
    c.bench_function("hyperbrep shell volume report", |b| {
        b.iter(|| shell.shell_volume_report())
    });
    c.bench_function("hyperbrep solid readiness report", |b| {
        b.iter(|| trim_shell.solid_readiness_report())
    });
    c.bench_function("hyperbrep exact retained surface handoff", |b| {
        b.iter(|| shell.exact_surface_handoff())
    });
    c.bench_function("hyperbrep exact retained solid handoff", |b| {
        b.iter(|| shell.exact_solid_handoff())
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
    )
    .unwrap();
    c.bench_function("hyperbrep prepare voxel geometry", |b| {
        b.iter(|| cube.prepare_voxel_geometry().unwrap())
    });
    let prepared_voxel_solid = cube.prepare_voxel_geometry().unwrap().triangle_solid;
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
    c.bench_function("hyperbrep surface inventory", |b| {
        b.iter(|| hyperbrep::BrepSurfaceInventoryReport::from_surfaces(&shell.surfaces))
    });
    let source_region = rectangle_region(64, 32);
    let source_surface = BrepSurface::plane(BrepSurfaceId(0), Plane3::new(p(0, 0, 1), r(0)));
    c.bench_function("hyperbrep planar region face construction", |b| {
        b.iter(|| {
            BrepPlanarRegionConstruction::from_region_on_surface(
                &source_region,
                source_surface.clone(),
            )
        })
    });
    c.bench_function("hyperbrep planar region extrusion construction", |b| {
        b.iter(|| {
            BrepPlanarExtrusionConstruction::vertical_prism_from_region(&source_region, r(0), r(8))
        })
    });
    let holed_region = curve_region(
        vec![rectangle_contour(0, 0, 64, 32)],
        vec![rectangle_contour(8, 8, 24, 24)],
    );
    c.bench_function("hyperbrep holed region extrusion construction", |b| {
        b.iter(|| {
            BrepPlanarExtrusionConstruction::vertical_prism_from_region(&holed_region, r(0), r(8))
        })
    });
    let plane_surface = BrepSurface::plane(BrepSurfaceId(99), Plane3::new(p(0, 0, 1), r(-512)));
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
    let batch_queries = vec![query.clone(); 64];
    c.bench_function("hyperbrep batched planar face point query 64", |b| {
        b.iter(|| {
            BrepPlanarFaceRegion::classify_uv_points(
                &face_region,
                pcurve_surface,
                &batch_queries,
                &curve_policy,
            )
            .unwrap()
        })
    });
    let pcurve = BrepPcurve::new(pcurve_surface, curve_path(&[(0, 100), (0, 0), (100, 0)]));
    c.bench_function("hyperbrep planar face edge-use query", |b| {
        b.iter(|| face_region.edge_use_report(&pcurve).unwrap())
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
