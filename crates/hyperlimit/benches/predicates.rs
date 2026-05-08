mod benchmark_report;
mod dispatch_trace;

use criterion::{BenchmarkId, Criterion, black_box};
#[cfg(feature = "parallel")]
use dispatch_trace::trace_dispatch_row;
use dispatch_trace::{begin_dispatch_trace_run, trace_dispatch_cases, write_dispatch_trace_report};
use predicated::{
    BorrowedPredicateScalar, LineSide, Plane3, PlaneSide, Point2, Point3, PredicateOutcome, Sign,
    classify_point_line, classify_point_oriented_plane, classify_point_plane, incircle2d,
    insphere3d, orient2d, orient3d,
};

const BATCH: usize = 512;

type Orient3Case<S> = (Point3<S>, Point3<S>, Point3<S>, Point3<S>);
type Incircle2Case<S> = (Point2<S>, Point2<S>, Point2<S>, Point2<S>);
type Insphere3Case<S> = (Point3<S>, Point3<S>, Point3<S>, Point3<S>, Point3<S>);

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
    bench_representation(c, "f64", f64_scalar);
    bench_representation(c, "f32", f32_scalar);

    #[cfg(feature = "hyperreal")]
    bench_representation(c, "hyperreal", hyperreal_scalar);

    #[cfg(feature = "realistic-blas")]
    bench_representation(c, "realistic_blas", realistic_scalar);

    #[cfg(feature = "interval")]
    bench_interval_representation(c);

    #[cfg(feature = "parallel")]
    bench_parallel_batches(c);
}

fn bench_representation<S>(c: &mut Criterion, label: &'static str, scalar: fn(f64) -> S)
where
    S: BorrowedPredicateScalar + 'static,
{
    bench_orient2d(c, label, scalar);
    bench_line_side(c, label, scalar);
    bench_orient3d(c, label, scalar);
    bench_explicit_plane(c, label, scalar);
    bench_oriented_plane(c, label, scalar);
    bench_incircle2d(c, label, scalar);
    bench_insphere3d(c, label, scalar);
}

fn bench_orient2d<S>(c: &mut Criterion, label: &'static str, scalar: fn(f64) -> S)
where
    S: BorrowedPredicateScalar + 'static,
{
    let mut group = c.benchmark_group("orient2d");
    for workload in Workload::ALL {
        let cases = orient2d_cases(workload, scalar);
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

fn bench_line_side<S>(c: &mut Criterion, label: &'static str, scalar: fn(f64) -> S)
where
    S: BorrowedPredicateScalar + 'static,
{
    let mut group = c.benchmark_group("classify_point_line");
    for workload in Workload::ALL {
        let cases = orient2d_cases(workload, scalar);
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

fn bench_orient3d<S>(c: &mut Criterion, label: &'static str, scalar: fn(f64) -> S)
where
    S: BorrowedPredicateScalar + 'static,
{
    let mut group = c.benchmark_group("orient3d");
    for workload in Workload::ALL {
        let cases = orient3d_cases(workload, scalar);
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

fn bench_explicit_plane<S>(c: &mut Criterion, label: &'static str, scalar: fn(f64) -> S)
where
    S: BorrowedPredicateScalar + 'static,
{
    let mut group = c.benchmark_group("classify_point_plane");
    for workload in Workload::ALL {
        let cases = explicit_plane_cases(workload, scalar);
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
    }
    group.finish();
}

fn bench_oriented_plane<S>(c: &mut Criterion, label: &'static str, scalar: fn(f64) -> S)
where
    S: BorrowedPredicateScalar + 'static,
{
    let mut group = c.benchmark_group("classify_point_oriented_plane");
    for workload in Workload::ALL {
        let cases = oriented_plane_cases(workload, scalar);
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
    }
    group.finish();
}

fn bench_incircle2d<S>(c: &mut Criterion, label: &'static str, scalar: fn(f64) -> S)
where
    S: BorrowedPredicateScalar + 'static,
{
    let mut group = c.benchmark_group("incircle2d");
    for workload in Workload::ALL {
        let cases = incircle2d_cases(workload, scalar);
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
    }
    group.finish();
}

fn bench_insphere3d<S>(c: &mut Criterion, label: &'static str, scalar: fn(f64) -> S)
where
    S: BorrowedPredicateScalar + 'static,
{
    let mut group = c.benchmark_group("insphere3d");
    for workload in Workload::ALL {
        let cases = insphere3d_cases(workload, scalar);
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
    }
    group.finish();
}

#[cfg(feature = "parallel")]
fn bench_parallel_batches(c: &mut Criterion) {
    let mut orient3 = c.benchmark_group("batch_orient3d");
    for workload in Workload::ALL {
        let cases = orient3d_cases(workload, f64_scalar);
        trace_dispatch_row(
            format!("batch_orient3d/f64_sequential/{}", workload.name()),
            || {
                let outcomes = predicated::orient3d_batch(black_box(&cases));
                black_box(outcomes.into_iter().map(sign_score).sum::<i64>());
            },
        );
        trace_dispatch_row(
            format!("batch_orient3d/f64_parallel/{}", workload.name()),
            || {
                let outcomes = predicated::orient3d_batch_parallel(black_box(&cases));
                black_box(outcomes.into_iter().map(sign_score).sum::<i64>());
            },
        );
        orient3.bench_with_input(
            BenchmarkId::new("f64_sequential", workload.name()),
            &cases,
            |b, cases| {
                b.iter(|| {
                    let outcomes = predicated::orient3d_batch(black_box(cases));
                    black_box(outcomes.into_iter().map(sign_score).sum::<i64>())
                });
            },
        );
        orient3.bench_with_input(
            BenchmarkId::new("f64_parallel", workload.name()),
            &cases,
            |b, cases| {
                b.iter(|| {
                    let outcomes = predicated::orient3d_batch_parallel(black_box(cases));
                    black_box(outcomes.into_iter().map(sign_score).sum::<i64>())
                });
            },
        );
    }
    orient3.finish();

    let mut incircle = c.benchmark_group("batch_incircle2d");
    for workload in Workload::ALL {
        let cases = incircle2d_cases(workload, f64_scalar);
        trace_dispatch_row(
            format!("batch_incircle2d/f64_sequential/{}", workload.name()),
            || {
                let outcomes = predicated::incircle2d_batch(black_box(&cases));
                black_box(outcomes.into_iter().map(sign_score).sum::<i64>());
            },
        );
        trace_dispatch_row(
            format!("batch_incircle2d/f64_parallel/{}", workload.name()),
            || {
                let outcomes = predicated::incircle2d_batch_parallel(black_box(&cases));
                black_box(outcomes.into_iter().map(sign_score).sum::<i64>());
            },
        );
        incircle.bench_with_input(
            BenchmarkId::new("f64_sequential", workload.name()),
            &cases,
            |b, cases| {
                b.iter(|| {
                    let outcomes = predicated::incircle2d_batch(black_box(cases));
                    black_box(outcomes.into_iter().map(sign_score).sum::<i64>())
                });
            },
        );
        incircle.bench_with_input(
            BenchmarkId::new("f64_parallel", workload.name()),
            &cases,
            |b, cases| {
                b.iter(|| {
                    let outcomes = predicated::incircle2d_batch_parallel(black_box(cases));
                    black_box(outcomes.into_iter().map(sign_score).sum::<i64>())
                });
            },
        );
    }
    incircle.finish();

    let mut insphere = c.benchmark_group("batch_insphere3d");
    for workload in Workload::ALL {
        let cases = insphere3d_cases(workload, f64_scalar);
        trace_dispatch_row(
            format!("batch_insphere3d/f64_sequential/{}", workload.name()),
            || {
                let outcomes = predicated::insphere3d_batch(black_box(&cases));
                black_box(outcomes.into_iter().map(sign_score).sum::<i64>());
            },
        );
        trace_dispatch_row(
            format!("batch_insphere3d/f64_parallel/{}", workload.name()),
            || {
                let outcomes = predicated::insphere3d_batch_parallel(black_box(&cases));
                black_box(outcomes.into_iter().map(sign_score).sum::<i64>());
            },
        );
        insphere.bench_with_input(
            BenchmarkId::new("f64_sequential", workload.name()),
            &cases,
            |b, cases| {
                b.iter(|| {
                    let outcomes = predicated::insphere3d_batch(black_box(cases));
                    black_box(outcomes.into_iter().map(sign_score).sum::<i64>())
                });
            },
        );
        insphere.bench_with_input(
            BenchmarkId::new("f64_parallel", workload.name()),
            &cases,
            |b, cases| {
                b.iter(|| {
                    let outcomes = predicated::insphere3d_batch_parallel(black_box(cases));
                    black_box(outcomes.into_iter().map(sign_score).sum::<i64>())
                });
            },
        );
    }
    insphere.finish();
}

#[cfg(feature = "interval")]
fn bench_interval_representation(c: &mut Criterion) {
    bench_representation(c, "interval_singleton", interval_singleton);

    let mut orient = c.benchmark_group("orient2d");
    let cases = interval_orient2d_cells();
    trace_dispatch_cases("orient2d/interval_cells/strict", &cases, |(a, b, d)| {
        black_box(sign_score(predicated::orient::orient2d_with_policy(
            a,
            b,
            d,
            predicated::PredicatePolicy::STRICT,
        )));
    });
    orient.bench_with_input(
        BenchmarkId::new("interval_cells", "strict"),
        &cases,
        |b, cases| {
            b.iter(|| {
                let mut score = 0_i64;
                for (a, b, d) in cases {
                    score += sign_score(black_box(predicated::orient::orient2d_with_policy(
                        black_box(a),
                        black_box(b),
                        black_box(d),
                        predicated::PredicatePolicy::STRICT,
                    )));
                }
                black_box(score)
            });
        },
    );
    orient.finish();

    let mut incircle = c.benchmark_group("incircle2d");
    let cases = interval_incircle2d_cells();
    trace_dispatch_cases(
        "incircle2d/interval_cells/strict",
        &cases,
        |(a, b, c, d)| {
            black_box(sign_score(predicated::orient::incircle2d_with_policy(
                a,
                b,
                c,
                d,
                predicated::PredicatePolicy::STRICT,
            )));
        },
    );
    incircle.bench_with_input(
        BenchmarkId::new("interval_cells", "strict"),
        &cases,
        |b, cases| {
            b.iter(|| {
                let mut score = 0_i64;
                for (a, b, c, d) in cases {
                    score += sign_score(black_box(predicated::orient::incircle2d_with_policy(
                        black_box(a),
                        black_box(b),
                        black_box(c),
                        black_box(d),
                        predicated::PredicatePolicy::STRICT,
                    )));
                }
                black_box(score)
            });
        },
    );
    incircle.finish();
}

fn orient2d_cases<S>(
    workload: Workload,
    scalar: fn(f64) -> S,
) -> Vec<(Point2<S>, Point2<S>, Point2<S>)> {
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let t = unit(i);
        let u = unit(i.wrapping_mul(17).wrapping_add(3));
        let (a, b, c) = match workload {
            Workload::Easy => (
                point2(-0.75 + 0.2 * t, -0.35 + 0.1 * u, scalar),
                point2(0.85 - 0.15 * u, -0.25 + 0.2 * t, scalar),
                point2(-0.15 + 0.9 * u, 0.8 - 0.4 * t, scalar),
            ),
            Workload::NearDegenerate => {
                let x = -0.9 + 1.8 * t;
                let eps = alternating_eps(i, 1.0e-13);
                (
                    point2(-1.0, -1.0, scalar),
                    point2(1.0, 1.0, scalar),
                    point2(x, x + eps, scalar),
                )
            }
        };
        cases.push((a, b, c));
    }
    cases
}

fn orient3d_cases<S>(workload: Workload, scalar: fn(f64) -> S) -> Vec<Orient3Case<S>> {
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let t = unit(i);
        let u = unit(i.wrapping_mul(13).wrapping_add(5));
        let v = unit(i.wrapping_mul(29).wrapping_add(11));
        let (a, b, c, d) = match workload {
            Workload::Easy => (
                point3(-0.4 + 0.2 * t, -0.7, -0.2 + 0.1 * u, scalar),
                point3(0.8, -0.25 + 0.2 * u, 0.1, scalar),
                point3(-0.2, 0.75, 0.25 + 0.1 * v, scalar),
                point3(-0.1 + 0.5 * v, -0.05 + 0.3 * t, 0.95, scalar),
            ),
            Workload::NearDegenerate => {
                let x = -0.8 + 1.6 * t;
                let y = -0.8 + 1.6 * u;
                let eps = alternating_eps(i, 1.0e-13);
                (
                    point3(0.0, 0.0, 0.0, scalar),
                    point3(1.0, 0.0, 0.0, scalar),
                    point3(0.0, 1.0, 0.0, scalar),
                    point3(x, y, eps, scalar),
                )
            }
        };
        cases.push((a, b, c, d));
    }
    cases
}

fn explicit_plane_cases<S>(
    workload: Workload,
    scalar: fn(f64) -> S,
) -> Vec<(Point3<S>, Plane3<S>)> {
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
            point3(t, u, z, scalar),
            Plane3::new(point3(0.8, -0.55, 1.0, scalar), scalar(-0.05)),
        ));
    }
    cases
}

fn oriented_plane_cases<S>(workload: Workload, scalar: fn(f64) -> S) -> Vec<Orient3Case<S>>
where
    S: Clone,
{
    let a = point3(-0.85, -0.7, -0.25, scalar);
    let b = point3(0.9, -0.35, 0.35, scalar);
    let c = point3(-0.35, 0.85, 0.05, scalar);
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let t = -0.9 + 1.8 * unit(i);
        let u = -0.9 + 1.8 * unit(i.wrapping_mul(23).wrapping_add(17));
        let point = match workload {
            Workload::Easy => point3(t, u, 0.5 + 0.4 * unit(i.wrapping_add(9)), scalar),
            Workload::NearDegenerate => point3(
                t,
                u,
                0.38 * t + 0.24 * u + alternating_eps(i, 1.0e-13),
                scalar,
            ),
        };
        cases.push((a.clone(), b.clone(), c.clone(), point));
    }
    cases
}

fn incircle2d_cases<S>(workload: Workload, scalar: fn(f64) -> S) -> Vec<Incircle2Case<S>>
where
    S: Clone,
{
    let a = point2(0.82, 0.0, scalar);
    let b = point2(0.0, 0.82, scalar);
    let c = point2(-0.82, 0.0, scalar);
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let theta = std::f64::consts::TAU * unit(i);
        let r = match workload {
            Workload::Easy => 0.35 + 0.45 * unit(i.wrapping_mul(11).wrapping_add(1)),
            Workload::NearDegenerate => 0.82 + alternating_eps(i, 1.0e-12),
        };
        let d = point2(r * theta.cos(), r * theta.sin(), scalar);
        cases.push((a.clone(), b.clone(), c.clone(), d));
    }
    cases
}

fn insphere3d_cases<S>(workload: Workload, scalar: fn(f64) -> S) -> Vec<Insphere3Case<S>>
where
    S: Clone,
{
    let a = point3(0.82, 0.0, 0.0, scalar);
    let b = point3(-0.82, 0.0, 0.0, scalar);
    let c = point3(0.0, 0.82, 0.0, scalar);
    let d = point3(0.0, 0.0, 0.82, scalar);
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
        let e = point3(r * theta.cos(), r * theta.sin(), z, scalar);
        cases.push((a.clone(), b.clone(), c.clone(), d.clone(), e));
    }
    cases
}

#[cfg(feature = "interval")]
fn interval_orient2d_cells() -> Vec<(
    Point2<inari::Interval>,
    Point2<inari::Interval>,
    Point2<inari::Interval>,
)> {
    let a = interval_point2(-0.85, -0.55);
    let b = interval_point2(0.9, 0.45);
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let x = -1.1 + 2.2 * unit(i);
        let y = -1.1 + 2.2 * unit(i.wrapping_mul(17).wrapping_add(3));
        let h = 1.0e-3;
        cases.push((
            a,
            b,
            Point2::new(interval(x - h, x + h), interval(y - h, y + h)),
        ));
    }
    cases
}

#[cfg(feature = "interval")]
fn interval_incircle2d_cells() -> Vec<(
    Point2<inari::Interval>,
    Point2<inari::Interval>,
    Point2<inari::Interval>,
    Point2<inari::Interval>,
)> {
    let a = interval_point2(0.82, 0.0);
    let b = interval_point2(0.0, 0.82);
    let c = interval_point2(-0.82, 0.0);
    let mut cases = Vec::with_capacity(BATCH);
    for i in 0..BATCH {
        let theta = 6.283_185_307_179_586 * unit(i);
        let r = 0.82 + alternating_eps(i, 1.0e-3);
        let h = 1.0e-4;
        let x = r * theta.cos();
        let y = r * theta.sin();
        cases.push((
            a,
            b,
            c,
            Point2::new(interval(x - h, x + h), interval(y - h, y + h)),
        ));
    }
    cases
}

fn point2<S>(x: f64, y: f64, scalar: fn(f64) -> S) -> Point2<S> {
    Point2::new(scalar(x), scalar(y))
}

fn point3<S>(x: f64, y: f64, z: f64, scalar: fn(f64) -> S) -> Point3<S> {
    Point3::new(scalar(x), scalar(y), scalar(z))
}

fn f64_scalar(value: f64) -> f64 {
    value
}

fn f32_scalar(value: f64) -> f32 {
    value as f32
}

#[cfg(feature = "hyperreal")]
fn hyperreal_scalar(value: f64) -> hyperreal::Real {
    hyperreal::Real::try_from(value).expect("finite benchmark scalar")
}

#[cfg(feature = "realistic-blas")]
fn realistic_scalar(value: f64) -> realistic_blas::Scalar<realistic_blas::DefaultBackend> {
    realistic_blas::Scalar::try_from(value).expect("finite benchmark scalar")
}

#[cfg(feature = "interval")]
fn interval_singleton(value: f64) -> inari::Interval {
    interval(value, value)
}

#[cfg(feature = "interval")]
fn interval_point2(x: f64, y: f64) -> Point2<inari::Interval> {
    Point2::new(interval(x, x), interval(y, y))
}

#[cfg(feature = "interval")]
fn interval(inf: f64, sup: f64) -> inari::Interval {
    inari::Interval::try_from((inf, sup)).expect("valid benchmark interval")
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
