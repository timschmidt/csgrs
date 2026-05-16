#![allow(dead_code, unused_variables)]

mod benchmark_report;
mod dispatch_trace;

use criterion::{BenchmarkId, Criterion, black_box};
use dispatch_trace::{begin_dispatch_trace_run, trace_dispatch_cases, write_dispatch_trace_report};
use hyperlimit::{
    LineSide, Plane3, PlaneSide, Point2, Point3, PredicateOutcome, PreparedIncircle2,
    PreparedInsphere3, PreparedLine2, PreparedOrientedPlane3, Sign, classify_point_line,
    classify_point_oriented_plane, classify_point_plane, incircle2d, insphere3d, orient2d,
    orient3d,
};

const BATCH: usize = 512;

type Orient2Case = (Point2, Point2, Point2);
type Orient3Case = (Point3, Point3, Point3, Point3);
type Incircle2Case = (Point2, Point2, Point2, Point2);
type Insphere3Case = (Point3, Point3, Point3, Point3, Point3);

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
    bench_exact_rational_kernels(c);
    bench_shared_scale_views(c);

    // Parallel batch APIs require `Sync` real storage. `hyperreal::Real`
    // currently keeps local refinement caches behind `RefCell`, so exact
    // hyperreal benchmark rows stay sequential until the real layer exposes a
    // thread-safe sharing mode.
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

fn rational_real(numerator: i64, denominator: u64) -> hyperreal::Real {
    hyperreal::Real::new(hyperreal::Rational::fraction(numerator, denominator).unwrap())
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
