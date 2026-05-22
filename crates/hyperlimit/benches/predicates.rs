#![allow(dead_code, unused_variables)]

mod benchmark_report;
mod dispatch_trace;

use criterion::{BenchmarkId, Criterion, black_box};
use dispatch_trace::{begin_dispatch_trace_run, trace_dispatch_cases, write_dispatch_trace_report};
use hyperlimit::{
    CoplanarProjection, CoplanarTriangleRelation, ExactGeometrySession, LineSide, Plane3,
    PlaneSide, Point2, Point3, PredicateOutcome, PreparedIncircle2, PreparedInsphere3,
    PreparedLine2, PreparedOrientedPlane3, Segment3Intersection, SegmentPlaneRelation, Sign,
    SupportDopRelation, TriangleDegeneracy, affine_independent_d, certified_ball_sign,
    classify_aabb3_sphere_intersection, classify_circle_line2, classify_circle_segment2,
    classify_coplanar_triangles, classify_halfspace_feasibility3, classify_homogeneous_point_plane,
    classify_point_convex_planes3, classify_point_convex_polygon2, classify_point_line,
    classify_point_oriented_plane, classify_point_plane, classify_ray_triangle3_intersection,
    classify_segment_triangle3_intersection, classify_segment3_intersection,
    classify_sphere3_intersection, classify_triangle3_degeneracy,
    compare_point_line3_distance_squared, compare_point_plane_distance_squared,
    compare_point_segment3_distance_squared, incircle2d, insphere_d, insphere3d,
    intersect_segment_with_oriented_plane, intersect_three_planes, intersect_two_planes, orient_d,
    orient2d, orient3d, projected_line_parameter3, projected_segment_parameter3,
    support_dop3_from_points,
};

const BATCH: usize = 512;

type Orient2Case = (Point2, Point2, Point2);
type Orient3Case = (Point3, Point3, Point3, Point3);
type Incircle2Case = (Point2, Point2, Point2, Point2);
type Insphere3Case = (Point3, Point3, Point3, Point3, Point3);
type Segment3Case = (Point3, Point3, Point3, Point3);
type SegmentTriangle3Case = (Point3, Point3, Point3, Point3, Point3);

#[derive(Clone, Copy)]
enum Workload {
    Easy,
    NearDegenerate,
}

impl Workload {
    const ALL: [Self; 2] = [Self::Easy, Self::NearDegenerate];

    const fn name(self) -> &'static str {
        match self {
            Self::Easy => "easy",
            Self::NearDegenerate => "near_degenerate",
        }
    }
}

fn bench_predicates(c: &mut Criterion) {
    bench_representation(c, "hyperreal", hyperreal_real);
    bench_certified_filters(c);
    bench_exact_rational_kernels(c);
    bench_shared_scale_views(c);
    bench_transformed_predicates(c);
    bench_versioned_prepared(c);
    bench_hypermesh_port_helpers(c);

    // Parallel batch APIs require `Sync` real storage. `hyperreal::Real`
    // currently keeps local refinement caches behind `RefCell`, so exact
    // hyperreal benchmark rows stay sequential until the real layer exposes a
    // thread-safe sharing mode.
}

fn bench_hypermesh_port_helpers(c: &mut Criterion) {
    let mut group = c.benchmark_group("hypermesh_port_helpers");
    let points = vec![
        rational_point3(0, 1, 0, 1, 0, 1),
        rational_point3(4, 1, 0, 1, 0, 1),
        rational_point3(0, 1, 4, 1, 0, 1),
        rational_point3(1, 1, 1, 1, 0, 1),
        rational_point3(5, 1, 1, 1, 0, 1),
        rational_point3(1, 1, 5, 1, 0, 1),
    ];
    let segment_start = rational_point3(0, 1, 0, 1, -1, 1);
    let segment_end = rational_point3(0, 1, 0, 1, 1, 1);

    // These rows track the construction and coplanar helpers lifted from
    // hypermesh into hyperlimit. The benchmark keeps Yap's exact object model
    // visible at the reusable predicate layer: topology is classified by
    // retained exact predicates, and split parameters remain exact
    // determinant-ratio constructions.
    group.bench_function("triangle3_degeneracy/projected_orientations", |bench| {
        bench.iter(|| {
            let report = classify_triangle3_degeneracy(
                black_box(&points[0]),
                black_box(&points[1]),
                black_box(&points[2]),
            );
            black_box(match report.degeneracy {
                TriangleDegeneracy::NonDegenerate => 1_i64,
                TriangleDegeneracy::Degenerate => 0,
                TriangleDegeneracy::Unknown => -1,
            })
        });
    });
    group.bench_function("segment_plane/determinant_ratio", |bench| {
        bench.iter(|| {
            let event = intersect_segment_with_oriented_plane(
                black_box(&points[0]),
                black_box(&points[1]),
                black_box(&points[2]),
                black_box(&segment_start),
                black_box(&segment_end),
            );
            black_box(match event.relation {
                SegmentPlaneRelation::ProperCrossing => 3_i64,
                SegmentPlaneRelation::EndpointOnPlane => 2,
                SegmentPlaneRelation::Coplanar => 1,
                SegmentPlaneRelation::Disjoint => 0,
                SegmentPlaneRelation::Unknown => -1,
                SegmentPlaneRelation::ConstructionFailed => -2,
            })
        });
    });
    group.bench_function("coplanar_triangles/projected_overlap", |bench| {
        bench.iter(|| {
            let classification =
                classify_coplanar_triangles(black_box(&points), [0, 1, 2], [3, 4, 5]);
            black_box(match classification.relation {
                CoplanarTriangleRelation::Disjoint => 0_i64,
                CoplanarTriangleRelation::Touching => 1,
                CoplanarTriangleRelation::Overlapping => 2,
                CoplanarTriangleRelation::Unknown => -1,
            })
        });
    });
    group.bench_function("projected_parameters/exact_ratio", |bench| {
        let midpoint = rational_point3(2, 1, 0, 1, 0, 1);
        bench.iter(|| {
            let segment_t = projected_segment_parameter3(
                black_box(&midpoint),
                black_box(&points[0]),
                black_box(&points[1]),
                CoplanarProjection::Xy,
            );
            let line_t = projected_line_parameter3(
                black_box(&segment_start),
                black_box(&segment_end),
                black_box(&points[0]),
                black_box(&points[1]),
                CoplanarProjection::Xy,
            );
            black_box((segment_t, line_t))
        });
    });
    group.bench_function("support_dop3/build_and_aabb_project", |bench| {
        let axes = vec![
            rational_point3(1, 1, 0, 1, 0, 1),
            rational_point3(0, 1, 1, 1, 0, 1),
            rational_point3(0, 1, 0, 1, 1, 1),
            rational_point3(1, 1, 1, 1, 1, 1),
            rational_point3(1, 1, -1, 1, 0, 1),
            rational_point3(0, 1, 1, 1, -1, 1),
        ];
        let cloud = vec![
            rational_point3(0, 1, 0, 1, 0, 1),
            rational_point3(4, 1, 0, 1, 0, 1),
            rational_point3(0, 1, 4, 1, 0, 1),
            rational_point3(0, 1, 0, 1, 4, 1),
            rational_point3(1, 2, 3, 2, 5, 2),
            rational_point3(-1, 2, 7, 2, 1, 2),
        ];
        let query_min = rational_point3(1, 1, 1, 1, 1, 1);
        let query_max = rational_point3(2, 1, 2, 1, 2, 1);
        bench.iter(|| {
            let dop = support_dop3_from_points(black_box(&axes), black_box(&cloud))
                .value()
                .expect("benchmark support DOP should decide");
            black_box(
                match dop
                    .classify_aabb3(black_box(&query_min), black_box(&query_max))
                    .value()
                {
                    Some(SupportDopRelation::Degenerate) => -2,
                    Some(SupportDopRelation::Separated) => 0_i64,
                    Some(SupportDopRelation::BoundaryTouch) => 1,
                    Some(SupportDopRelation::ConservativeOverlap) => 2,
                    None => -1,
                },
            )
        });
    });
    group.finish();
}

fn bench_certified_filters(c: &mut Criterion) {
    let mut group = c.benchmark_group("certified_filters");
    let balls = certified_ball_cases();
    group.bench_function("ball_sign/rational", |bench| {
        bench.iter(|| {
            let mut score = 0_i64;
            for (center, radius) in &balls {
                score += maybe_sign_score(black_box(certified_ball_sign(
                    black_box(center),
                    black_box(radius),
                )));
            }
            black_box(score)
        });
    });
    group.finish();
}

fn bench_versioned_prepared(c: &mut Criterion) {
    let mut group = c.benchmark_group("versioned_prepared");
    let session = ExactGeometrySession::default();
    let a = rational_point2(0, 1, 0, 1);
    let b = rational_point2(4, 1, 0, 1);
    let line = session.versioned_prepared(session.prepare_line2(&a, &b));

    // This row keeps construction-version cache diagnostics visible beside
    // predicate rows. It measures metadata checks only; stale prepared facts are
    // scheduling data, not topology certificates, following Yap's object-cache
    // boundary in "Towards Exact Geometric Computation," Computational
    // Geometry 7.1-2 (1997).
    group.bench_function("line2/freshness_current", |bench| {
        bench.iter(|| black_box(line.freshness_for(black_box(session)).is_current()))
    });

    let mut stale_session = session;
    stale_session.advance_version();
    group.bench_function("line2/freshness_stale", |bench| {
        bench.iter(|| black_box(line.freshness_for(black_box(stale_session)).is_current()))
    });
    group.finish();
}

fn bench_representation(c: &mut Criterion, label: &'static str, real: fn(f64) -> hyperreal::Real) {
    bench_orient2d(c, label, real);
    bench_line_side(c, label, real);
    bench_fixed_line_side(c, label, real);
    bench_orient3d(c, label, real);
    bench_explicit_plane(c, label, real);
    bench_oriented_plane(c, label, real);
    bench_incircle2d(c, label, real);
    bench_insphere3d(c, label, real);
}

fn bench_exact_rational_kernels(c: &mut Criterion) {
    let mut group = c.benchmark_group("exact_rational_kernels");

    let orient2 = exact_rational_orient2d_cases();
    group.bench_function("orient2d/common_denominator", |b| {
        b.iter(|| {
            let mut score = 0_i64;
            for (a, b, c) in &orient2 {
                score += sign_score(black_box(orient2d(
                    black_box(a),
                    black_box(b),
                    black_box(c),
                )));
            }
            black_box(score)
        });
    });

    // Larger near-degenerate exact rationals keep the benchmark surface aligned
    // with Yap's warning that exact geometric computation must preserve the
    // algebraic decision, not a nearby primitive-float surrogate; see Yap,
    // "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
    // (1997).
    let orient2_large = large_rational_near_degenerate_orient2d_cases();
    group.bench_function("orient2d/larger_rational_near_degenerate", |b| {
        b.iter(|| {
            let mut score = 0_i64;
            for (a, b, c) in &orient2_large {
                score += sign_score(black_box(orient2d(
                    black_box(a),
                    black_box(b),
                    black_box(c),
                )));
            }
            black_box(score)
        });
    });

    let orient3 = exact_rational_orient3d_cases();
    group.bench_function("orient3d/common_denominator", |b| {
        b.iter(|| {
            let mut score = 0_i64;
            for (a, b, c, d) in &orient3 {
                score += sign_score(black_box(orient3d(
                    black_box(a),
                    black_box(b),
                    black_box(c),
                    black_box(d),
                )));
            }
            black_box(score)
        });
    });

    let incircle = exact_rational_incircle2d_cases();
    group.bench_function("incircle2d/common_denominator", |b| {
        b.iter(|| {
            let mut score = 0_i64;
            for (a, b, c, d) in &incircle {
                score += sign_score(black_box(incircle2d(
                    black_box(a),
                    black_box(b),
                    black_box(c),
                    black_box(d),
                )));
            }
            black_box(score)
        });
    });

    let incircle_large = large_rational_near_degenerate_incircle2d_cases();
    group.bench_function("incircle2d/larger_rational_near_degenerate", |b| {
        b.iter(|| {
            let mut score = 0_i64;
            for (a, b, c, d) in &incircle_large {
                score += sign_score(black_box(incircle2d(
                    black_box(a),
                    black_box(b),
                    black_box(c),
                    black_box(d),
                )));
            }
            black_box(score)
        });
    });

    let insphere = exact_rational_insphere3d_cases();
    group.bench_function("insphere3d/common_denominator", |b| {
        b.iter(|| {
            let mut score = 0_i64;
            for (a, b, c, d, e) in &insphere {
                score += sign_score(black_box(insphere3d(
                    black_box(a),
                    black_box(b),
                    black_box(c),
                    black_box(d),
                    black_box(e),
                )));
            }
            black_box(score)
        });
    });

    let plane_triples = exact_rational_coordinate_plane_cases();
    group.bench_function("homogeneous/three_plane_coordinate_triples", |b| {
        b.iter(|| {
            let mut score = 0_i64;
            for (x_plane, y_plane, z_plane) in &plane_triples {
                let point = black_box(intersect_three_planes(
                    black_box(x_plane),
                    black_box(y_plane),
                    black_box(z_plane),
                ));
                score += i64::from(point.coordinate_facts().all_exact_rational);
                score += bool_score(classify_homogeneous_point_plane(&point, x_plane));
                score += bool_score(classify_homogeneous_point_plane(&point, y_plane));
                score += bool_score(classify_homogeneous_point_plane(&point, z_plane));
            }
            black_box(score)
        });
    });
    group.bench_function("homogeneous/two_plane_line_then_plane", |b| {
        b.iter(|| {
            let mut score = 0_i64;
            for (x_plane, y_plane, z_plane) in &plane_triples {
                let line = black_box(intersect_two_planes(black_box(x_plane), black_box(y_plane)));
                let point = black_box(line.intersect_plane(black_box(z_plane)));
                score += i64::from(line.coordinate_facts().all_exact_rational);
                score += bool_score(classify_homogeneous_point_plane(&point, z_plane));
            }
            black_box(score)
        });
    });

    // This mixed relation row keeps the exact 3D segment classifier visible for
    // mesh-kernel-style edge/topology workloads. The implementation follows
    // Yap's exact-decision contract and de Berg et al.'s orientation/interval
    // decomposition without replacing skew or coplanar cases by primitive-float
    // tolerances.
    let segment3 = exact_rational_segment3_cases();
    group.bench_function("segment3_intersection/mixed_exact_rational", |b| {
        b.iter(|| {
            let mut score = 0_i64;
            for (a, b, c, d) in &segment3 {
                score += segment3_intersection_score(black_box(classify_segment3_intersection(
                    black_box(a),
                    black_box(b),
                    black_box(c),
                    black_box(d),
                )));
            }
            black_box(score)
        });
    });

    let distance3 = exact_rational_point_feature_distance_cases();
    let threshold = rational_real(25, 121);
    group.bench_function("distance3/point_feature_scaled_thresholds", |b| {
        b.iter(|| {
            let mut score = 0_i64;
            for (point, a, b, plane) in &distance3 {
                score += ordering_score(black_box(compare_point_line3_distance_squared(
                    black_box(point),
                    black_box(a),
                    black_box(b),
                    black_box(&threshold),
                )));
                score += ordering_score(black_box(compare_point_segment3_distance_squared(
                    black_box(point),
                    black_box(a),
                    black_box(b),
                    black_box(&threshold),
                )));
                score += ordering_score(black_box(compare_point_plane_distance_squared(
                    black_box(point),
                    black_box(plane),
                    black_box(&threshold),
                )));
                score += i64::from(
                    black_box(classify_sphere3_intersection(
                        black_box(a),
                        black_box(&threshold),
                        black_box(b),
                        black_box(&threshold),
                    ))
                    .value()
                    .is_some_and(|relation| relation.intersects()),
                );
                score += i64::from(
                    black_box(classify_aabb3_sphere_intersection(
                        black_box(a),
                        black_box(b),
                        black_box(point),
                        black_box(&threshold),
                    ))
                    .value()
                    .is_some_and(|relation| relation.intersects()),
                );
            }
            black_box(score)
        });
    });

    let triangle_hits = exact_rational_segment_triangle3_cases();
    group.bench_function("triangle3/segment_and_ray_intersections", |b| {
        b.iter(|| {
            let mut score = 0_i64;
            for (p, q, a, b, c) in &triangle_hits {
                let direction = Point3::new(&q.x - &p.x, &q.y - &p.y, &q.z - &p.z);
                score += i64::from(
                    classify_segment_triangle3_intersection(
                        black_box(p),
                        black_box(q),
                        black_box(a),
                        black_box(b),
                        black_box(c),
                    )
                    .value()
                    .is_some_and(|relation| relation.intersects()),
                );
                score += i64::from(
                    classify_ray_triangle3_intersection(
                        black_box(p),
                        black_box(&direction),
                        black_box(a),
                        black_box(b),
                        black_box(c),
                    )
                    .value()
                    .is_some_and(|relation| relation.intersects()),
                );
            }
            black_box(score)
        });
    });

    let circle2 = exact_rational_circle2_line_segment_cases();
    group.bench_function("circle2/line_and_segment_relations", |b| {
        b.iter(|| {
            let mut score = 0_i64;
            for (center, radius_squared, a, b) in &circle2 {
                score += i64::from(
                    classify_circle_line2(
                        black_box(center),
                        black_box(radius_squared),
                        black_box(a),
                        black_box(b),
                    )
                    .value()
                    .is_some(),
                );
                score += i64::from(
                    classify_circle_segment2(
                        black_box(center),
                        black_box(radius_squared),
                        black_box(a),
                        black_box(b),
                    )
                    .value()
                    .is_some(),
                );
            }
            black_box(score)
        });
    });

    let polygon = exact_rational_convex_polygon2();
    let planes = exact_rational_convex_box_planes3();
    let orient2_points = exact_rational_orient2d_cases();
    let orient3_points = exact_rational_orient3d_cases();
    group.bench_function("convex/point_halfspace_composition", |b| {
        b.iter(|| {
            let mut score = 0_i64;
            for (_, _, point) in &orient2_points {
                score += i64::from(
                    classify_point_convex_polygon2(black_box(&polygon), black_box(point))
                        .value()
                        .is_some_and(|location| location.is_inside_or_boundary()),
                );
            }
            for (_, _, _, point) in &orient3_points {
                score += i64::from(
                    classify_point_convex_planes3(black_box(&planes), black_box(point))
                        .value()
                        .is_some_and(|location| location.is_inside_or_boundary()),
                );
            }
            black_box(score)
        });
    });

    group.bench_function("convex/halfspace_feasibility3_active_sets", |b| {
        let feasible = exact_rational_shifted_box_planes3();
        let infeasible = vec![
            Plane3::new(rational_point3(1, 1, 0, 1, 0, 1), rational_real(1, 1)),
            Plane3::new(rational_point3(-1, 1, 0, 1, 0, 1), rational_real(0, 1)),
        ];
        b.iter(|| {
            let feasible_status = classify_halfspace_feasibility3(black_box(&feasible))
                .value()
                .map(|report| report.is_feasible())
                .unwrap_or(false);
            let infeasible_status = classify_halfspace_feasibility3(black_box(&infeasible))
                .value()
                .map(|report| report.infeasibility_certificate.is_some())
                .unwrap_or(true);
            black_box((feasible_status, infeasible_status))
        });
    });

    // These rows keep the D-dimensional predicate boundary visible while it is
    // still intentionally generic. They measure exact determinant ownership in
    // `hyperlimit` before `hypertri` or mesh crates add prepared common-scale
    // schedules, following Yap's object/predicate layering in "Towards Exact
    // Geometric Computation," *Computational Geometry* 7.1-2 (1997).
    let orient_d_cases = exact_rational_orient_d_cases();
    group.bench_function("orient_d/4d_common_denominator", |b| {
        b.iter(|| {
            let mut score = 0_i64;
            for case in &orient_d_cases {
                score += sign_score(black_box(orient_d(black_box(case))));
            }
            black_box(score)
        });
    });

    let insphere_d_cases = exact_rational_insphere_d_cases();
    group.bench_function("insphere_d/4d_common_denominator", |b| {
        b.iter(|| {
            let mut score = 0_i64;
            for (simplex, query) in &insphere_d_cases {
                score += sign_score(black_box(insphere_d(black_box(simplex), black_box(query))));
            }
            black_box(score)
        });
    });

    group.bench_function("affine_independent_d/4d_common_denominator", |b| {
        b.iter(|| {
            let mut score = 0_i64;
            for case in &orient_d_cases {
                score += bool_score(black_box(affine_independent_d(black_box(case))));
            }
            black_box(score)
        });
    });

    group.finish();
}

fn bench_shared_scale_views(c: &mut Criterion) {
    let mut group = c.benchmark_group("shared_scale_views");
    let point2 = rational_point2(1, 7, -2, 7);
    group.bench_function("point2/common_denominator", |bench| {
        bench.iter(|| black_box(point2.shared_scale_view()))
    });

    let point3 = rational_point3(1, 11, -2, 11, 3, 11);
    group.bench_function("point3/common_denominator", |bench| {
        bench.iter(|| black_box(point3.shared_scale_view()))
    });

    // Shared-scale predicate rows use nonzero rational coordinates so each
    // point can prove a point-local common denominator. This gives Criterion
    // and dispatch traces a direct view of Yap's "preserve object structure
    // before scalar expansion" route; see Yap, "Towards Exact Geometric
    // Computation," *Computational Geometry* 7.1-2 (1997).
    let incircle_cases = shared_scale_incircle2d_cases();
    group.bench_function("incircle2d/common_denominator_predicate", |bench| {
        bench.iter(|| {
            let mut score = 0_i64;
            for (a, b, c, d) in &incircle_cases {
                score += sign_score(black_box(incircle2d(
                    black_box(a),
                    black_box(b),
                    black_box(c),
                    black_box(d),
                )));
            }
            black_box(score)
        });
    });
    if let Some((a, b, c, _)) = incircle_cases.first() {
        let prepared = PreparedIncircle2::new(a, b, c);
        group.bench_function("incircle2d/common_denominator_prepared", |bench| {
            bench.iter(|| {
                let mut score = 0_i64;
                for (_, _, _, d) in &incircle_cases {
                    score += sign_score(black_box(prepared.test_point(black_box(d))));
                }
                black_box(score)
            });
        });
    }

    let insphere_cases = shared_scale_insphere3d_cases();
    group.bench_function("insphere3d/common_denominator_predicate", |bench| {
        bench.iter(|| {
            let mut score = 0_i64;
            for (a, b, c, d, e) in &insphere_cases {
                score += sign_score(black_box(insphere3d(
                    black_box(a),
                    black_box(b),
                    black_box(c),
                    black_box(d),
                    black_box(e),
                )));
            }
            black_box(score)
        });
    });
    if let Some((a, b, c, d, _)) = insphere_cases.first() {
        let prepared = PreparedInsphere3::new(a, b, c, d);
        group.bench_function("insphere3d/common_denominator_prepared", |bench| {
            bench.iter(|| {
                let mut score = 0_i64;
                for (_, _, _, _, e) in &insphere_cases {
                    score += sign_score(black_box(prepared.test_point(black_box(e))));
                }
                black_box(score)
            });
        });
    }
    group.finish();
}

fn bench_transformed_predicates(c: &mut Criterion) {
    let mut group = c.benchmark_group("transformed_predicates");

    // Yap treats exact geometric computation as an object-level discipline, not
    // only a scalar one. These rows keep transformed exact-rational workloads
    // visible as first-class benchmark cases so future affine/common-scale
    // carriers can be measured before scalar expansion; see Yap, "Towards
    // Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997).
    let orient_cases = transformed_orient2d_cases();
    trace_dispatch_cases(
        "transformed_predicates/orient2d/exact_rational_affine",
        &orient_cases,
        |(a, b, c)| {
            black_box(sign_score(orient2d(a, b, c)));
        },
    );
    group.bench_function("orient2d/exact_rational_affine", |bench| {
        bench.iter(|| {
            let mut score = 0_i64;
            for (a, b, c) in &orient_cases {
                score += sign_score(black_box(orient2d(
                    black_box(a),
                    black_box(b),
                    black_box(c),
                )));
            }
            black_box(score)
        });
    });

    let line_cases = transformed_line_cases();
    if let Some((a, b, _)) = line_cases.first() {
        let prepared = PreparedLine2::new(a, b);
        trace_dispatch_cases(
            "transformed_predicates/classify_point_line/prepared_exact_rational_affine",
            &line_cases,
            |(_, _, point)| {
                black_box(line_score(prepared.classify_point(point)));
            },
        );
        group.bench_function(
            "classify_point_line/prepared_exact_rational_affine",
            |bench| {
                bench.iter(|| {
                    let mut score = 0_i64;
                    for (_, _, point) in &line_cases {
                        score += line_score(black_box(prepared.classify_point(black_box(point))));
                    }
                    black_box(score)
                });
            },
        );
    }

    // The in-circle determinant is especially sensitive to expansion strategy.
    // Keeping both cold and prepared transformed rows beside the shared-scale
    // rows lets us check whether future lifted-circle coefficient reuse follows
    // Shewchuk's determinant orientation of the predicate while preserving
    // exact signs; see Shewchuk, "Adaptive Precision Floating-Point Arithmetic
    // and Fast Robust Geometric Predicates," *Discrete & Computational
    // Geometry* 18.3 (1997).
    let incircle_cases = transformed_incircle2d_cases();
    trace_dispatch_cases(
        "transformed_predicates/incircle2d/exact_rational_affine",
        &incircle_cases,
        |(a, b, c, d)| {
            black_box(sign_score(incircle2d(a, b, c, d)));
        },
    );
    group.bench_function("incircle2d/exact_rational_affine", |bench| {
        bench.iter(|| {
            let mut score = 0_i64;
            for (a, b, c, d) in &incircle_cases {
                score += sign_score(black_box(incircle2d(
                    black_box(a),
                    black_box(b),
                    black_box(c),
                    black_box(d),
                )));
            }
            black_box(score)
        });
    });
    if let Some((a, b, c, _)) = incircle_cases.first() {
        let prepared = PreparedIncircle2::new(a, b, c);
        trace_dispatch_cases(
            "transformed_predicates/incircle2d/prepared_exact_rational_affine",
            &incircle_cases,
            |(_, _, _, d)| {
                black_box(sign_score(prepared.test_point(d)));
            },
        );
        group.bench_function("incircle2d/prepared_exact_rational_affine", |bench| {
            bench.iter(|| {
                let mut score = 0_i64;
                for (_, _, _, d) in &incircle_cases {
                    score += sign_score(black_box(prepared.test_point(black_box(d))));
                }
                black_box(score)
            });
        });
    }

    group.finish();
}

fn bench_orient2d(c: &mut Criterion, label: &'static str, real: fn(f64) -> hyperreal::Real) {
    let mut group = c.benchmark_group("orient2d");
    for workload in Workload::ALL {
        let cases = orient2d_cases(workload, real);
        trace_dispatch_cases(
            format!("orient2d/{label}/{}", workload.name()),
            &cases,
            |(a, b, d)| {
                black_box(sign_score(orient2d(a, b, d)));
            },
        );
        group.bench_with_input(
            BenchmarkId::new(label, workload.name()),
            &cases,
            |b, cases| {
                b.iter(|| {
                    let mut score = 0_i64;
                    for (a, b, d) in cases {
                        score += sign_score(black_box(orient2d(
                            black_box(a),
                            black_box(b),
                            black_box(d),
                        )));
                    }
                    black_box(score)
                });
            },
        );
    }
    group.finish();
}

fn bench_line_side(c: &mut Criterion, label: &'static str, real: fn(f64) -> hyperreal::Real) {
    let mut group = c.benchmark_group("classify_point_line");
    for workload in Workload::ALL {
        let cases = orient2d_cases(workload, real);
        trace_dispatch_cases(
            format!("classify_point_line/{label}/{}", workload.name()),
            &cases,
            |(a, b, d)| {
                black_box(line_score(classify_point_line(a, b, d)));
            },
        );
        group.bench_with_input(
            BenchmarkId::new(label, workload.name()),
            &cases,
            |b, cases| {
                b.iter(|| {
                    let mut score = 0_i64;
                    for (a, b, d) in cases {
                        score += line_score(black_box(classify_point_line(
                            black_box(a),
                            black_box(b),
                            black_box(d),
                        )));
                    }
                    black_box(score)
                });
            },
        );
    }
    group.finish();
}

fn bench_fixed_line_side(c: &mut Criterion, label: &'static str, real: fn(f64) -> hyperreal::Real) {
    let mut group = c.benchmark_group("classify_point_line_fixed");
    for workload in Workload::ALL {
        let cases = fixed_line_cases(workload, real);
        trace_dispatch_cases(
            format!("classify_point_line_fixed/{label}/{}", workload.name()),
            &cases,
            |(a, b, point)| {
                black_box(line_score(classify_point_line(a, b, point)));
            },
        );
        group.bench_with_input(
            BenchmarkId::new(label, workload.name()),
            &cases,
            |b, cases| {
                b.iter(|| {
                    let mut score = 0_i64;
                    for (a, b, point) in cases {
                        score += line_score(black_box(classify_point_line(
                            black_box(a),
                            black_box(b),
                            black_box(point),
                        )));
                    }
                    black_box(score)
                });
            },
        );
        if let Some((a, b, _)) = cases.first() {
            let prepared = PreparedLine2::new(a, b);
            trace_dispatch_cases(
                format!(
                    "classify_point_line_fixed/{label}_prepared/{}",
                    workload.name()
                ),
                &cases,
                |(_, _, point)| {
                    black_box(line_score(prepared.classify_point(point)));
                },
            );
            group.bench_with_input(
                BenchmarkId::new(format!("{label}_prepared"), workload.name()),
                &cases,
                |b, cases| {
                    b.iter(|| {
                        let mut score = 0_i64;
                        for (_, _, point) in cases {
                            score +=
                                line_score(black_box(prepared.classify_point(black_box(point))));
                        }
                        black_box(score)
                    });
                },
            );
        }
    }
    group.finish();
}

fn bench_orient3d(c: &mut Criterion, label: &'static str, real: fn(f64) -> hyperreal::Real) {
    let mut group = c.benchmark_group("orient3d");
    for workload in Workload::ALL {
        let cases = orient3d_cases(workload, real);
        trace_dispatch_cases(
            format!("orient3d/{label}/{}", workload.name()),
            &cases,
            |(a, b, c, d)| {
                black_box(sign_score(orient3d(a, b, c, d)));
            },
        );
        group.bench_with_input(
            BenchmarkId::new(label, workload.name()),
            &cases,
            |b, cases| {
                b.iter(|| {
                    let mut score = 0_i64;
                    for (a, b, c, d) in cases {
                        score += sign_score(black_box(orient3d(
                            black_box(a),
                            black_box(b),
                            black_box(c),
                            black_box(d),
                        )));
                    }
                    black_box(score)
                });
            },
        );
    }
    group.finish();
}

fn bench_explicit_plane(c: &mut Criterion, label: &'static str, real: fn(f64) -> hyperreal::Real) {
    let mut group = c.benchmark_group("classify_point_plane");
    for workload in Workload::ALL {
        let cases = explicit_plane_cases(workload, real);
        trace_dispatch_cases(
            format!("classify_point_plane/{label}/{}", workload.name()),
            &cases,
            |(point, plane)| {
                black_box(plane_score(classify_point_plane(point, plane)));
            },
        );
        group.bench_with_input(
            BenchmarkId::new(label, workload.name()),
            &cases,
            |b, cases| {
                b.iter(|| {
                    let mut score = 0_i64;
                    for (point, plane) in cases {
                        score += plane_score(black_box(classify_point_plane(
                            black_box(point),
                            black_box(plane),
                        )));
                    }
                    black_box(score)
                });
            },
        );
        if let Some((_, plane)) = cases.first() {
            let prepared = plane.prepare();
            trace_dispatch_cases(
                format!("classify_point_plane/{label}_prepared/{}", workload.name()),
                &cases,
                |(point, _)| {
                    black_box(plane_score(prepared.classify_point(point)));
                },
            );
            group.bench_with_input(
                BenchmarkId::new(format!("{label}_prepared"), workload.name()),
                &cases,
                |b, cases| {
                    b.iter(|| {
                        let mut score = 0_i64;
                        for (point, _) in cases {
                            score +=
                                plane_score(black_box(prepared.classify_point(black_box(point))));
                        }
                        black_box(score)
                    });
                },
            );
        }
    }
    group.finish();
}

fn bench_oriented_plane(c: &mut Criterion, label: &'static str, real: fn(f64) -> hyperreal::Real) {
    let mut group = c.benchmark_group("classify_point_oriented_plane");
    for workload in Workload::ALL {
        let cases = oriented_plane_cases(workload, real);
        trace_dispatch_cases(
            format!("classify_point_oriented_plane/{label}/{}", workload.name()),
            &cases,
            |(a, b, c, point)| {
                black_box(plane_score(classify_point_oriented_plane(a, b, c, point)));
            },
        );
        group.bench_with_input(
            BenchmarkId::new(label, workload.name()),
            &cases,
            |b, cases| {
                b.iter(|| {
                    let mut score = 0_i64;
                    for (a, b, c, point) in cases {
                        score += plane_score(black_box(classify_point_oriented_plane(
                            black_box(a),
                            black_box(b),
                            black_box(c),
                            black_box(point),
                        )));
                    }
                    black_box(score)
                });
            },
        );
        if let Some((a, b, c, _)) = cases.first() {
            let prepared = PreparedOrientedPlane3::new(a, b, c);
            trace_dispatch_cases(
                format!(
                    "classify_point_oriented_plane/{label}_prepared/{}",
                    workload.name()
                ),
                &cases,
                |(_, _, _, point)| {
                    black_box(plane_score(prepared.classify_point(point)));
                },
            );
            group.bench_with_input(
                BenchmarkId::new(format!("{label}_prepared"), workload.name()),
                &cases,
                |b, cases| {
                    b.iter(|| {
                        let mut score = 0_i64;
                        for (_, _, _, point) in cases {
                            score +=
                                plane_score(black_box(prepared.classify_point(black_box(point))));
                        }
                        black_box(score)
                    });
                },
            );
        }
    }
    group.finish();
}

fn bench_incircle2d(c: &mut Criterion, label: &'static str, real: fn(f64) -> hyperreal::Real) {
    let mut group = c.benchmark_group("incircle2d");
    for workload in Workload::ALL {
        let cases = incircle2d_cases(workload, real);
        trace_dispatch_cases(
            format!("incircle2d/{label}/{}", workload.name()),
            &cases,
            |(a, b, c, d)| {
                black_box(sign_score(incircle2d(a, b, c, d)));
            },
        );
        group.bench_with_input(
            BenchmarkId::new(label, workload.name()),
            &cases,
            |b, cases| {
                b.iter(|| {
                    let mut score = 0_i64;
                    for (a, b, c, d) in cases {
                        score += sign_score(black_box(incircle2d(
                            black_box(a),
                            black_box(b),
                            black_box(c),
                            black_box(d),
                        )));
                    }
                    black_box(score)
                });
            },
        );
        if let Some((a, b, c, _)) = cases.first() {
            let prepared = PreparedIncircle2::new(a, b, c);
            trace_dispatch_cases(
                format!("incircle2d/{label}_prepared/{}", workload.name()),
                &cases,
                |(_, _, _, d)| {
                    black_box(sign_score(prepared.test_point(d)));
                },
            );
            group.bench_with_input(
                BenchmarkId::new(format!("{label}_prepared"), workload.name()),
                &cases,
                |b, cases| {
                    b.iter(|| {
                        let mut score = 0_i64;
                        for (_, _, _, d) in cases {
                            score += sign_score(black_box(prepared.test_point(black_box(d))));
                        }
                        black_box(score)
                    });
                },
            );
        }
    }
    group.finish();
}

fn bench_insphere3d(c: &mut Criterion, label: &'static str, real: fn(f64) -> hyperreal::Real) {
    let mut group = c.benchmark_group("insphere3d");
    for workload in Workload::ALL {
        let cases = insphere3d_cases(workload, real);
        trace_dispatch_cases(
            format!("insphere3d/{label}/{}", workload.name()),
            &cases,
            |(a, b, c, d, e)| {
                black_box(sign_score(insphere3d(a, b, c, d, e)));
            },
        );
        group.bench_with_input(
            BenchmarkId::new(label, workload.name()),
            &cases,
            |b, cases| {
                b.iter(|| {
                    let mut score = 0_i64;
                    for (a, b, c, d, e) in cases {
                        score += sign_score(black_box(insphere3d(
                            black_box(a),
                            black_box(b),
                            black_box(c),
                            black_box(d),
                            black_box(e),
                        )));
                    }
                    black_box(score)
                });
            },
        );
        if let Some((a, b, c, d, _)) = cases.first() {
            let prepared = PreparedInsphere3::new(a, b, c, d);
            trace_dispatch_cases(
                format!("insphere3d/{label}_prepared/{}", workload.name()),
                &cases,
                |(_, _, _, _, e)| {
                    black_box(sign_score(prepared.test_point(e)));
                },
            );
            group.bench_with_input(
                BenchmarkId::new(format!("{label}_prepared"), workload.name()),
                &cases,
                |b, cases| {
                    b.iter(|| {
                        let mut score = 0_i64;
                        for (_, _, _, _, e) in cases {
                            score += sign_score(black_box(prepared.test_point(black_box(e))));
                        }
                        black_box(score)
                    });
                },
            );
        }
    }
    group.finish();
}

fn orient2d_cases(
    workload: Workload,
    real: fn(f64) -> hyperreal::Real,
) -> Vec<(Point2, Point2, Point2)> {
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let t = unit(i);
        let u = unit(i.wrapping_mul(17).wrapping_add(3));
        let (a, b, c) = match workload {
            Workload::Easy => (
                point2(-0.75 + 0.2 * t, -0.35 + 0.1 * u, real),
                point2(0.85 - 0.15 * u, -0.25 + 0.2 * t, real),
                point2(-0.15 + 0.9 * u, 0.8 - 0.4 * t, real),
            ),
            Workload::NearDegenerate => {
                let x = -0.9 + 1.8 * t;
                let eps = alternating_eps(i, 1.0e-13);
                (
                    point2(-1.0, -1.0, real),
                    point2(1.0, 1.0, real),
                    point2(x, x + eps, real),
                )
            }
        };
        cases.push((a, b, c));
    }
    cases
}

fn fixed_line_cases(
    workload: Workload,
    real: fn(f64) -> hyperreal::Real,
) -> Vec<(Point2, Point2, Point2)> {
    let a = point2(-1.0, -1.0, real);
    let b = point2(1.0, 1.0, real);
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let x = -0.9 + 1.8 * unit(i);
        let y = match workload {
            Workload::Easy => -0.5 + 1.1 * unit(i.wrapping_mul(17).wrapping_add(3)),
            Workload::NearDegenerate => x + alternating_eps(i, 1.0e-13),
        };
        cases.push((a.clone(), b.clone(), point2(x, y, real)));
    }
    cases
}

fn orient3d_cases(workload: Workload, real: fn(f64) -> hyperreal::Real) -> Vec<Orient3Case> {
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let t = unit(i);
        let u = unit(i.wrapping_mul(13).wrapping_add(5));
        let v = unit(i.wrapping_mul(29).wrapping_add(11));
        let (a, b, c, d) = match workload {
            Workload::Easy => (
                point3(-0.4 + 0.2 * t, -0.7, -0.2 + 0.1 * u, real),
                point3(0.8, -0.25 + 0.2 * u, 0.1, real),
                point3(-0.2, 0.75, 0.25 + 0.1 * v, real),
                point3(-0.1 + 0.5 * v, -0.05 + 0.3 * t, 0.95, real),
            ),
            Workload::NearDegenerate => {
                let x = -0.8 + 1.6 * t;
                let y = -0.8 + 1.6 * u;
                let eps = alternating_eps(i, 1.0e-13);
                (
                    point3(0.0, 0.0, 0.0, real),
                    point3(1.0, 0.0, 0.0, real),
                    point3(0.0, 1.0, 0.0, real),
                    point3(x, y, eps, real),
                )
            }
        };
        cases.push((a, b, c, d));
    }
    cases
}

fn explicit_plane_cases(
    workload: Workload,
    real: fn(f64) -> hyperreal::Real,
) -> Vec<(Point3, Plane3)> {
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let t = unit(i);
        let u = unit(i.wrapping_mul(19).wrapping_add(7));
        let z = match workload {
            Workload::Easy => -0.8 + 1.6 * unit(i.wrapping_mul(31).wrapping_add(2)),
            Workload::NearDegenerate => {
                let on_plane = 0.05 - 0.8 * t + 0.55 * u;
                on_plane + alternating_eps(i, 1.0e-13)
            }
        };
        cases.push((
            point3(t, u, z, real),
            Plane3::new(point3(0.8, -0.55, 1.0, real), real(-0.05)),
        ));
    }
    cases
}

fn oriented_plane_cases(workload: Workload, real: fn(f64) -> hyperreal::Real) -> Vec<Orient3Case> {
    let a = point3(-0.85, -0.7, -0.25, real);
    let b = point3(0.9, -0.35, 0.35, real);
    let c = point3(-0.35, 0.85, 0.05, real);
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let t = -0.9 + 1.8 * unit(i);
        let u = -0.9 + 1.8 * unit(i.wrapping_mul(23).wrapping_add(17));
        let point = match workload {
            Workload::Easy => point3(t, u, 0.5 + 0.4 * unit(i.wrapping_add(9)), real),
            Workload::NearDegenerate => point3(
                t,
                u,
                0.38 * t + 0.24 * u + alternating_eps(i, 1.0e-13),
                real,
            ),
        };
        cases.push((a.clone(), b.clone(), c.clone(), point));
    }
    cases
}

fn incircle2d_cases(workload: Workload, real: fn(f64) -> hyperreal::Real) -> Vec<Incircle2Case> {
    let a = point2(0.82, 0.0, real);
    let b = point2(0.0, 0.82, real);
    let c = point2(-0.82, 0.0, real);
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let theta = std::f64::consts::TAU * unit(i);
        let r = match workload {
            Workload::Easy => 0.35 + 0.45 * unit(i.wrapping_mul(11).wrapping_add(1)),
            Workload::NearDegenerate => 0.82 + alternating_eps(i, 1.0e-12),
        };
        let d = point2(r * theta.cos(), r * theta.sin(), real);
        cases.push((a.clone(), b.clone(), c.clone(), d));
    }
    cases
}

fn insphere3d_cases(workload: Workload, real: fn(f64) -> hyperreal::Real) -> Vec<Insphere3Case> {
    let a = point3(0.82, 0.0, 0.0, real);
    let b = point3(-0.82, 0.0, 0.0, real);
    let c = point3(0.0, 0.82, 0.0, real);
    let d = point3(0.0, 0.0, 0.82, real);
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let theta = std::f64::consts::TAU * unit(i);
        let z = -0.6 + 1.2 * unit(i.wrapping_mul(37).wrapping_add(13));
        let r = match workload {
            Workload::Easy => 0.25 + 0.35 * unit(i.wrapping_mul(7).wrapping_add(1)),
            Workload::NearDegenerate => {
                (0.82_f64.powi(2) - z * z).max(0.0).sqrt() + alternating_eps(i, 1.0e-12)
            }
        };
        let e = point3(r * theta.cos(), r * theta.sin(), z, real);
        cases.push((a.clone(), b.clone(), c.clone(), d.clone(), e));
    }
    cases
}

fn exact_rational_orient2d_cases() -> Vec<Orient2Case> {
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let j = i as i64;
        cases.push((
            rational_point2(j - 40, 7, j % 17 - 8, 7),
            rational_point2(j % 31 + 2, 7, 19 - j % 23, 7),
            rational_point2(j % 13 - 6, 7, j % 29 - 14, 7),
        ));
    }
    cases
}

fn exact_rational_orient3d_cases() -> Vec<Orient3Case> {
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let j = i as i64;
        cases.push((
            rational_point3(j % 11 - 5, 9, j % 13 - 6, 9, j % 17 - 8, 9),
            rational_point3(j % 19 + 1, 9, j % 23 - 11, 9, 3 - j % 7, 9),
            rational_point3(j % 29 - 14, 9, j % 5 + 2, 9, j % 31 - 15, 9),
            rational_point3(j % 37 - 18, 9, j % 41 - 20, 9, j % 43 - 21, 9),
        ));
    }
    cases
}

fn exact_rational_incircle2d_cases() -> Vec<Incircle2Case> {
    let a = rational_point2(0, 11, 0, 11);
    let b = rational_point2(8, 11, 0, 11);
    let c = rational_point2(0, 11, 8, 11);
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let j = i as i64;
        cases.push((
            a.clone(),
            b.clone(),
            c.clone(),
            rational_point2(j % 17 + 1, 11, j % 19 + 1, 11),
        ));
    }
    cases
}

fn exact_rational_insphere3d_cases() -> Vec<Insphere3Case> {
    let a = rational_point3(0, 13, 0, 13, 0, 13);
    let b = rational_point3(9, 13, 0, 13, 0, 13);
    let c = rational_point3(0, 13, 9, 13, 0, 13);
    let d = rational_point3(0, 13, 0, 13, 9, 13);
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let j = i as i64;
        cases.push((
            a.clone(),
            b.clone(),
            c.clone(),
            d.clone(),
            rational_point3(j % 17 + 1, 13, j % 19 + 1, 13, j % 23 + 1, 13),
        ));
    }
    cases
}

fn exact_rational_coordinate_plane_cases() -> Vec<(Plane3, Plane3, Plane3)> {
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let j = i as i64;
        cases.push((
            coordinate_plane(0, rational_real(j % 31 - 15, 17)),
            coordinate_plane(1, rational_real(j % 37 - 18, 17)),
            coordinate_plane(2, rational_real(j % 41 - 20, 17)),
        ));
    }
    cases
}

fn exact_rational_segment3_cases() -> Vec<Segment3Case> {
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let j = i as i64;
        match i % 6 {
            0 => cases.push((
                rational_point3(0, 11, 0, 11, 0, 11),
                rational_point3(8, 11, 0, 11, 0, 11),
                rational_point3(4, 11, -2, 11, 0, 11),
                rational_point3(4, 11, 2, 11, 0, 11),
            )),
            1 => cases.push((
                rational_point3(0, 11, 0, 11, 0, 11),
                rational_point3(8, 11, 0, 11, 0, 11),
                rational_point3(9, 11, 1, 11, 0, 11),
                rational_point3(12, 11, 1, 11, 0, 11),
            )),
            2 => cases.push((
                rational_point3(0, 11, 0, 11, 0, 11),
                rational_point3(8, 11, 0, 11, 0, 11),
                rational_point3(4, 11, -2, 11, 1, 11),
                rational_point3(4, 11, 2, 11, 1, 11),
            )),
            3 => cases.push((
                rational_point3(0, 11, 0, 11, 0, 11),
                rational_point3(8, 11, 8, 11, 8, 11),
                rational_point3(4, 11, 4, 11, 4, 11),
                rational_point3(10, 11, 10, 11, 10, 11),
            )),
            4 => cases.push((
                rational_point3(j % 5, 11, 0, 11, 0, 11),
                rational_point3(j % 5 + 8, 11, 0, 11, 0, 11),
                rational_point3(j % 5 + 8, 11, 0, 11, 0, 11),
                rational_point3(j % 5 + 8, 11, 2, 11, 0, 11),
            )),
            _ => cases.push((
                rational_point3(1, 11, 1, 11, 1, 11),
                rational_point3(7, 11, 7, 11, 7, 11),
                rational_point3(7, 11, 7, 11, 7, 11),
                rational_point3(1, 11, 1, 11, 1, 11),
            )),
        }
    }
    cases
}

fn exact_rational_point_feature_distance_cases() -> Vec<(Point3, Point3, Point3, Plane3)> {
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let j = i as i64;
        let point = rational_point3(j % 13 - 6, 11, j % 17 - 8, 11, j % 19 - 9, 11);
        let a = rational_point3(-4, 11, j % 5 - 2, 11, 0, 11);
        let b = rational_point3(8, 11, j % 7 - 3, 11, j % 3 + 1, 11);
        let plane = Plane3::new(
            rational_point3(1, 11, -2, 11, 3, 11),
            rational_real(j % 23 - 11, 11),
        );
        cases.push((point, a, b, plane));
    }
    cases
}

fn exact_rational_segment_triangle3_cases() -> Vec<SegmentTriangle3Case> {
    let a = rational_point3(0, 11, 0, 11, 0, 11);
    let b = rational_point3(40, 11, 0, 11, 0, 11);
    let c = rational_point3(0, 11, 40, 11, 0, 11);
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let j = i as i64;
        let x = j % 31;
        let y = (j * 7) % 31;
        cases.push((
            rational_point3(x, 11, y, 11, -5, 11),
            rational_point3(x, 11, y, 11, 5, 11),
            a.clone(),
            b.clone(),
            c.clone(),
        ));
    }
    cases
}

fn exact_rational_circle2_line_segment_cases() -> Vec<(Point2, hyperreal::Real, Point2, Point2)> {
    let center = rational_point2(0, 11, 0, 11);
    let radius_squared = rational_real(25, 121);
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let j = i as i64;
        cases.push((
            center.clone(),
            radius_squared.clone(),
            rational_point2(-10, 11, j % 15 - 7, 11),
            rational_point2(10, 11, j % 15 - 7, 11),
        ));
    }
    cases
}

fn exact_rational_convex_polygon2() -> Vec<Point2> {
    vec![
        rational_point2(0, 11, 0, 11),
        rational_point2(40, 11, 0, 11),
        rational_point2(40, 11, 40, 11),
        rational_point2(0, 11, 40, 11),
    ]
}

fn exact_rational_convex_box_planes3() -> Vec<Plane3> {
    vec![
        Plane3::new(rational_point3(-1, 11, 0, 11, 0, 11), rational_real(0, 11)),
        Plane3::new(
            rational_point3(1, 11, 0, 11, 0, 11),
            rational_real(-40, 121),
        ),
        Plane3::new(rational_point3(0, 11, -1, 11, 0, 11), rational_real(0, 11)),
        Plane3::new(
            rational_point3(0, 11, 1, 11, 0, 11),
            rational_real(-40, 121),
        ),
        Plane3::new(rational_point3(0, 11, 0, 11, -1, 11), rational_real(0, 11)),
        Plane3::new(
            rational_point3(0, 11, 0, 11, 1, 11),
            rational_real(-40, 121),
        ),
    ]
}

fn exact_rational_shifted_box_planes3() -> Vec<Plane3> {
    vec![
        Plane3::new(rational_point3(-1, 1, 0, 1, 0, 1), rational_real(2, 1)),
        Plane3::new(rational_point3(1, 1, 0, 1, 0, 1), rational_real(-5, 1)),
        Plane3::new(rational_point3(0, 1, -1, 1, 0, 1), rational_real(3, 1)),
        Plane3::new(rational_point3(0, 1, 1, 1, 0, 1), rational_real(-6, 1)),
        Plane3::new(rational_point3(0, 1, 0, 1, -1, 1), rational_real(1, 1)),
        Plane3::new(rational_point3(0, 1, 0, 1, 1, 1), rational_real(-4, 1)),
    ]
}

fn exact_rational_orient_d_cases() -> Vec<Vec<Vec<hyperreal::Real>>> {
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let j = i as i64;
        cases.push(vec![
            rational_point_d(&[0, 0, 0, 0], 17),
            rational_point_d(&[9, 0, 0, 0], 17),
            rational_point_d(&[0, 9, 0, 0], 17),
            rational_point_d(&[0, 0, 9, 0], 17),
            rational_point_d(&[j % 5, j % 7, j % 11, 9 + j % 3], 17),
        ]);
    }
    cases
}

fn exact_rational_insphere_d_cases() -> Vec<(Vec<Vec<hyperreal::Real>>, Vec<hyperreal::Real>)> {
    let simplex = vec![
        rational_point_d(&[0, 0, 0, 0], 19),
        rational_point_d(&[8, 0, 0, 0], 19),
        rational_point_d(&[0, 8, 0, 0], 19),
        rational_point_d(&[0, 0, 8, 0], 19),
        rational_point_d(&[0, 0, 0, 8], 19),
    ];
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let j = i as i64;
        cases.push((
            simplex.clone(),
            rational_point_d(&[j % 13 + 1, j % 17 + 1, j % 19 + 1, j % 23 + 1], 19),
        ));
    }
    cases
}

fn large_rational_near_degenerate_orient2d_cases() -> Vec<Orient2Case> {
    let den = 1_000_003_u64;
    let den_sq = den * den;
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let x_num = i as i64 % 1_003 - 501;
        let y_num = x_num * den as i64 + signed_jitter(i as i64);
        cases.push((
            rational_point2(-1, 1, -1, 1),
            rational_point2(1, 1, 1, 1),
            rational_point2(x_num, den, y_num, den_sq),
        ));
    }
    cases
}

fn large_rational_near_degenerate_incircle2d_cases() -> Vec<Incircle2Case> {
    let den = 1_000_003_u64;
    let a = rational_point2(1, 1, 0, 1);
    let b = rational_point2(0, 1, 1, 1);
    let c = rational_point2(-1, 1, 0, 1);
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let t_num = i as i64 % 1_003 - 501;
        let circle_den = den * den + (t_num * t_num) as u64;
        let x_num = 2 * t_num * den as i64;
        let y_num = den as i128 * den as i128 - t_num as i128 * t_num as i128;
        let y_den = circle_den * den;
        let y_final_num = y_num * den as i128 + signed_jitter(i as i64) as i128;
        cases.push((
            a.clone(),
            b.clone(),
            c.clone(),
            Point2::new(
                rational_real(x_num, den),
                rational_real(
                    y_final_num
                        .try_into()
                        .expect("benchmark numerator fits in i64"),
                    y_den,
                ),
            ),
        ));
    }
    cases
}

fn shared_scale_incircle2d_cases() -> Vec<Incircle2Case> {
    let a = rational_point2(1, 7, 1, 7);
    let b = rational_point2(4, 7, 1, 7);
    let c = rational_point2(1, 7, 4, 7);
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let j = i as i64;
        cases.push((
            a.clone(),
            b.clone(),
            c.clone(),
            rational_point2(j % 3 + 2, 7, (j / 3) % 3 + 2, 7),
        ));
    }
    cases
}

fn shared_scale_insphere3d_cases() -> Vec<Insphere3Case> {
    let a = rational_point3(1, 7, 1, 7, 1, 7);
    let b = rational_point3(4, 7, 1, 7, 1, 7);
    let c = rational_point3(1, 7, 4, 7, 1, 7);
    let d = rational_point3(1, 7, 1, 7, 4, 7);
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let j = i as i64;
        cases.push((
            a.clone(),
            b.clone(),
            c.clone(),
            d.clone(),
            rational_point3(j % 3 + 2, 7, (j / 3) % 3 + 2, 7, (j / 9) % 3 + 2, 7),
        ));
    }
    cases
}

fn transformed_orient2d_cases() -> Vec<Orient2Case> {
    let mut cases = Vec::with_capacity(BATCH);
    let scale = rational_real(5, 3);
    let tx = rational_real(-17, 11);
    let ty = rational_real(23, 13);
    for (a, b, c) in exact_rational_orient2d_cases() {
        cases.push((
            transform_point2(&a, &scale, &tx, &ty),
            transform_point2(&b, &scale, &tx, &ty),
            transform_point2(&c, &scale, &tx, &ty),
        ));
    }
    cases
}

fn transformed_line_cases() -> Vec<Orient2Case> {
    let base_a = rational_point2(-1, 5, -1, 5);
    let base_b = rational_point2(7, 5, 7, 5);
    let scale = rational_real(7, 4);
    let tx = rational_real(31, 17);
    let ty = rational_real(-29, 19);
    let a = transform_point2(&base_a, &scale, &tx, &ty);
    let b = transform_point2(&base_b, &scale, &tx, &ty);
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let j = i as i64;
        let point = rational_point2(j % 23 - 11, 23, j % 23 - 11 + signed_jitter(j), 23);
        cases.push((
            a.clone(),
            b.clone(),
            transform_point2(&point, &scale, &tx, &ty),
        ));
    }
    cases
}

fn transformed_incircle2d_cases() -> Vec<Incircle2Case> {
    let scale = rational_real(9, 5);
    let tx = rational_real(-41, 29);
    let ty = rational_real(37, 31);
    let mut cases = Vec::with_capacity(BATCH);
    for (a, b, c, d) in exact_rational_incircle2d_cases() {
        cases.push((
            transform_point2(&a, &scale, &tx, &ty),
            transform_point2(&b, &scale, &tx, &ty),
            transform_point2(&c, &scale, &tx, &ty),
            transform_point2(&d, &scale, &tx, &ty),
        ));
    }
    cases
}

fn transform_point2(
    point: &Point2,
    scale: &hyperreal::Real,
    tx: &hyperreal::Real,
    ty: &hyperreal::Real,
) -> Point2 {
    let scaled_x = &point.x * scale;
    let scaled_y = &point.y * scale;
    Point2::new(&scaled_x + tx, &scaled_y + ty)
}

fn signed_jitter(index: i64) -> i64 {
    if index % 2 == 0 { 1 } else { -1 }
}

fn point2(x: f64, y: f64, real: fn(f64) -> hyperreal::Real) -> Point2 {
    Point2::new(real(x), real(y))
}

fn point3(x: f64, y: f64, z: f64, real: fn(f64) -> hyperreal::Real) -> Point3 {
    Point3::new(real(x), real(y), real(z))
}

fn hyperreal_real(value: f64) -> hyperreal::Real {
    hyperreal::Real::try_from(value).expect("finite benchmark real")
}

fn rational_point2(x_num: i64, x_den: u64, y_num: i64, y_den: u64) -> Point2 {
    Point2::new(rational_real(x_num, x_den), rational_real(y_num, y_den))
}

fn rational_point3(
    x_num: i64,
    x_den: u64,
    y_num: i64,
    y_den: u64,
    z_num: i64,
    z_den: u64,
) -> Point3 {
    Point3::new(
        rational_real(x_num, x_den),
        rational_real(y_num, y_den),
        rational_real(z_num, z_den),
    )
}

fn rational_point_d(numerators: &[i64], denominator: u64) -> Vec<hyperreal::Real> {
    numerators
        .iter()
        .map(|&numerator| rational_real(numerator, denominator))
        .collect()
}

fn rational_real(numerator: i64, denominator: u64) -> hyperreal::Real {
    hyperreal::Real::new(hyperreal::Rational::fraction(numerator, denominator).unwrap())
}

fn coordinate_plane(axis: usize, coordinate: hyperreal::Real) -> Plane3 {
    let normal = match axis {
        0 => Point3::new(1.into(), 0.into(), 0.into()),
        1 => Point3::new(0.into(), 1.into(), 0.into()),
        2 => Point3::new(0.into(), 0.into(), 1.into()),
        _ => unreachable!("benchmark helper only builds 3D coordinate planes"),
    };
    Plane3::new(normal, -coordinate)
}

fn certified_ball_cases() -> Vec<(hyperreal::Real, hyperreal::Real)> {
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let magnitude = (i % 17 + 3) as i64;
        let radius = (i % 5 + 1) as i64;
        let sign = if i.is_multiple_of(2) { 1 } else { -1 };
        cases.push((
            rational_real(sign * magnitude, 7),
            rational_real(radius, 14),
        ));
    }
    cases
}

fn unit(index: usize) -> f64 {
    let mut x = index as u64;
    x = x.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
    ((x >> 11) as f64) * (1.0 / ((1_u64 << 53) as f64))
}

fn alternating_eps(index: usize, magnitude: f64) -> f64 {
    if index.is_multiple_of(2) {
        magnitude
    } else {
        -magnitude
    }
}

fn sign_score(outcome: PredicateOutcome<Sign>) -> i64 {
    match outcome {
        PredicateOutcome::Decided { value, .. } => match value {
            Sign::Negative => -1,
            Sign::Zero => 0,
            Sign::Positive => 1,
        },
        PredicateOutcome::Unknown { .. } => 7,
    }
}

fn bool_score(outcome: PredicateOutcome<bool>) -> i64 {
    match outcome {
        PredicateOutcome::Decided { value, .. } => i64::from(value),
        PredicateOutcome::Unknown { .. } => 7,
    }
}

fn maybe_sign_score(outcome: Option<PredicateOutcome<Sign>>) -> i64 {
    outcome.map_or(11, sign_score)
}

fn line_score(outcome: PredicateOutcome<LineSide>) -> i64 {
    match outcome {
        PredicateOutcome::Decided { value, .. } => match value {
            LineSide::Right => -1,
            LineSide::On => 0,
            LineSide::Left => 1,
        },
        PredicateOutcome::Unknown { .. } => 7,
    }
}

fn plane_score(outcome: PredicateOutcome<PlaneSide>) -> i64 {
    match outcome {
        PredicateOutcome::Decided { value, .. } => match value {
            PlaneSide::Below => -1,
            PlaneSide::On => 0,
            PlaneSide::Above => 1,
        },
        PredicateOutcome::Unknown { .. } => 7,
    }
}

fn ordering_score(outcome: PredicateOutcome<std::cmp::Ordering>) -> i64 {
    match outcome {
        PredicateOutcome::Decided { value, .. } => match value {
            std::cmp::Ordering::Less => -1,
            std::cmp::Ordering::Equal => 0,
            std::cmp::Ordering::Greater => 1,
        },
        PredicateOutcome::Unknown { .. } => 7,
    }
}

fn segment3_intersection_score(outcome: PredicateOutcome<Segment3Intersection>) -> i64 {
    match outcome {
        PredicateOutcome::Decided { value, .. } => match value {
            Segment3Intersection::SkewDisjoint => -3,
            Segment3Intersection::CoplanarDisjoint => -2,
            Segment3Intersection::Proper => 1,
            Segment3Intersection::EndpointTouch => 2,
            Segment3Intersection::CollinearOverlap => 3,
            Segment3Intersection::Identical => 4,
        },
        PredicateOutcome::Unknown { .. } => 7,
    }
}

fn main() {
    let trace_only = std::env::args()
        .any(|arg| arg == "--write-dispatch-trace-md" || arg == "--dispatch-trace-only");
    if trace_only {
        begin_dispatch_trace_run();
    }

    let mut criterion = if trace_only {
        Criterion::default().with_filter("$^")
    } else {
        Criterion::default().configure_from_args()
    };
    bench_predicates(&mut criterion);
    if trace_only {
        write_dispatch_trace_report();
    } else {
        criterion.final_summary();

        match benchmark_report::write_benchmarks_md() {
            Ok(summary) => eprintln!(
                "updated {} from {} Criterion benchmark results",
                summary.path.display(),
                summary.rows
            ),
            Err(error) => eprintln!("failed to update benchmarks.md: {error}"),
        }
    }
}
