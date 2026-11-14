use super::*;
use crate::{ArchiveLimits, ArenaModelBuilder, ImportedModel};
use hyperlattice::Matrix4;
use std::sync::Arc;

fn graph() -> ExpressionBudget {
    ExpressionBudget::new(1_000_000, 100_000_000)
}
fn arithmetic() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000)
}
fn d(lo: i64, hi: i64, exponent: i32) -> DyadicInterval {
    DyadicInterval::new(lo.into(), hi.into(), exponent).unwrap()
}
fn patch() -> Surface {
    Surface::rational_bezier(
        vec![
            vec![
                Point3::new(0.into(), 0.into(), 0.into()),
                Point3::new(1.into(), 0.into(), 0.into()),
            ],
            vec![
                Point3::new(0.into(), 2.into(), 0.into()),
                Point3::new(1.into(), 2.into(), 1.into()),
            ],
        ],
        vec![vec![1.into(); 2]; 2],
    )
    .unwrap()
}
fn native(source: &Surface) -> &RationalBezierSurface {
    let SurfaceGeometry::RationalBezier(surface) = &source.data.geometry else {
        panic!("tensor fixture")
    };
    surface
}

#[test]
fn cold_warm_failed_and_cancelled_preparation_preserve_native_cache_identity() {
    let source = patch();
    assert!(native(&source).homogeneous_controls.get().is_none());
    let cold = prepare_rational_bezier_surface_ranges(&source, &mut graph()).unwrap();
    assert!(native(&source).homogeneous_controls.get().is_none());
    assert!(Arc::ptr_eq(&cold.source.data, &source.data));
    // Fail after native coefficient conversion, while emitting the graph.
    assert!(matches!(
        prepare_rational_bezier_surface_ranges(
            &source,
            &mut ExpressionBudget::new(1_000_000, 1153)
        ),
        Err(SurfaceRangeError::Expression(
            ExpressionError::WorkExhausted
        ))
    ));
    assert!(native(&source).homogeneous_controls.get().is_none());
    let token = hypersolve::work_budget::CancellationToken::new();
    token.cancel();
    let mut cancelled = graph();
    cancelled.cancellation = Some(token);
    assert!(matches!(
        prepare_rational_bezier_surface_ranges(&source, &mut cancelled),
        Err(SurfaceRangeError::Expression(ExpressionError::Cancelled))
    ));
    assert!(native(&source).homogeneous_controls.get().is_none());
    source
        .evaluate_real(&Point2::new(0.into(), 0.into()))
        .unwrap();
    let cache = native(&source).homogeneous_controls.get().unwrap() as *const _;
    let warm = prepare_rational_bezier_surface_ranges(&source, &mut graph()).unwrap();
    assert_eq!(
        native(&source).homogeneous_controls.get().unwrap() as *const _,
        cache
    );
    assert_eq!(
        cold.program().program().definition(),
        warm.program().program().definition()
    );
    let domain = BoxN::new([d(0, 1, 0), d(0, 1, 0)]).unwrap();
    let first = cold
        .enclose(&domain, -64, &mut graph(), &mut arithmetic())
        .unwrap();
    let second = warm
        .enclose(&domain, -64, &mut graph(), &mut arithmetic())
        .unwrap();
    assert!(Arc::ptr_eq(&first.source.data, &source.data));
    assert_eq!(first.evidence().values(), second.evidence().values());
    assert_eq!(
        native(&source).homogeneous_controls.get().unwrap() as *const _,
        cache
    );
}

#[test]
fn affine_transform_reversal_and_exact_reconstruction_feed_the_next_range_query() {
    // Anisotropic affine map (x,y,z)->(2x+3,-y+4,3z-1).
    let transform = Matrix4::new([
        [2.into(), 0.into(), 0.into(), 3.into()],
        [0.into(), (-1).into(), 0.into(), 4.into()],
        [0.into(), 0.into(), 3.into(), (-1).into()],
        [0.into(), 0.into(), 0.into(), 1.into()],
    ]);
    let changed = patch()
        .transformed(&super::super::TransformMetric::new(&transform), true)
        .unwrap();
    let rebuilt = Surface::from_exact_data(changed.exact_data()).unwrap();
    for source in [&changed, &rebuilt] {
        assert!(native(source).homogeneous_controls.get().is_none());
        let prepared = prepare_rational_bezier_surface_ranges(source, &mut graph()).unwrap();
        let report = prepared
            .enclose(
                &BoxN::new([d(1, 1, -2), d(1, 1, -1)]).unwrap(),
                -64,
                &mut graph(),
                &mut arithmetic(),
            )
            .unwrap();
        // u reflection: (5-2u,4-2v,3(1-u)v-1).
        assert_eq!(
            report.coordinates(),
            [&d(9, 9, -1), &d(3, 3, 0), &d(1, 1, -3)]
        );
        assert_eq!(
            report.partials()[0],
            [&d(-2, -2, 0), &d(0, 0, 0), &d(-3, -3, -1)]
        );
        assert_eq!(
            report.partials()[1],
            [&d(0, 0, 0), &d(-2, -2, 0), &d(9, 9, -2)]
        );
        assert!(native(source).homogeneous_controls.get().is_none());
        assert!(Arc::ptr_eq(&report.source.data, &source.data));
    }
}

#[test]
fn fresh_process_native_archive_replay_rebuilds_surface_and_range_authority() {
    use std::io::{Read, Write};
    const KEY: &str = "HYPERBREP_INTERVAL_SURFACE_REPLAY_CHILD";
    if std::env::var_os(KEY).is_none() {
        let half = (Real::one() / Real::from(2)).unwrap();
        let (left, _) = patch().split_u_at(&half).unwrap();
        let (lower, _) = left.split_v_at(&half).unwrap();
        let mut builder = ArenaModelBuilder::new();
        builder.surface(lower).unwrap();
        let archive = builder.finish().unwrap().to_json().unwrap();
        let mut child=std::process::Command::new(std::env::current_exe().unwrap())
            .args(["--exact","geometry::interval_ranges::tests::fresh_process_native_archive_replay_rebuilds_surface_and_range_authority","--nocapture"])
            .env(KEY,"1").stdin(std::process::Stdio::piped()).stdout(std::process::Stdio::piped()).stderr(std::process::Stdio::piped()).spawn().unwrap();
        child
            .stdin
            .take()
            .unwrap()
            .write_all(archive.as_bytes())
            .unwrap();
        let output = child.wait_with_output().unwrap();
        assert!(
            output.status.success(),
            "{}\n{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
        return;
    }
    let mut archive = String::new();
    std::io::stdin().read_to_string(&mut archive).unwrap();
    let model = ImportedModel::from_json(&archive, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(model.to_json().unwrap(), archive);
    let source = model.surfaces().next().unwrap().1;
    assert!(native(source).homogeneous_controls.get().is_none());
    let prepared = prepare_rational_bezier_surface_ranges(source, &mut graph()).unwrap();
    let report = prepared
        .enclose(
            &BoxN::new([d(0, 1, 0), d(0, 1, 0)]).unwrap(),
            -64,
            &mut graph(),
            &mut arithmetic(),
        )
        .unwrap();
    assert_eq!(
        report.coordinates(),
        [&d(0, 1, -1), &d(0, 1, 0), &d(0, 1, -2)]
    );
    assert_eq!(
        report.partials()[0],
        [&d(1, 1, -1), &d(0, 0, 0), &d(0, 1, -2)]
    );
    assert_eq!(
        report.partials()[1],
        [&d(0, 0, 0), &d(1, 1, 0), &d(0, 1, -2)]
    );
    assert!(native(source).homogeneous_controls.get().is_none());
    assert!(Arc::ptr_eq(&report.source.data, &source.data));
    let projected = report
        .native_bounds(&mut graph(), &mut arithmetic())
        .unwrap();
    assert_eq!(
        projected.bounds().unwrap().maxs,
        Point3::new(
            (Real::one() / Real::from(2)).unwrap(),
            1.into(),
            (Real::one() / Real::from(4)).unwrap()
        )
    );
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn native_dispatch_and_certainty_survive_preparation_in_fresh_processes() {
    use hyperreal::dispatch_trace;
    const KEY: &str = "HYPERBREP_INTERVAL_SURFACE_TRACE_CHILD";
    let Ok(mode) = std::env::var(KEY) else {
        for lifecycle in ["cold", "warm"] {
            for action in ["baseline", "prepare", "fail", "cancel"] {
                let output=std::process::Command::new(std::env::current_exe().unwrap())
                    .args(["--exact","geometry::interval_ranges::tests::native_dispatch_and_certainty_survive_preparation_in_fresh_processes","--nocapture"])
                    .env(KEY,format!("{lifecycle}-{action}")).output().unwrap();
                assert!(
                    output.status.success(),
                    "{}\n{}",
                    String::from_utf8_lossy(&output.stdout),
                    String::from_utf8_lossy(&output.stderr)
                );
                let stdout = String::from_utf8(output.stdout).unwrap();
                let snapshots = stdout
                    .lines()
                    .filter(|line| line.starts_with("NATIVE_TRACE "))
                    .collect::<Vec<_>>()
                    .join("\n");
                assert_eq!(snapshots.lines().count(), 2);
                // Preparation can warm a shared scalar's write-once arithmetic
                // facts without populating the native homogeneous-control cache.
                // Pin every count for each actual cache lifecycle, including
                // the one first-observation difference; omit no trace fields.
                let expected = match (lifecycle, action) {
                    ("cold", "baseline" | "cancel") => include_str!("trace-cold-baseline.txt"),
                    ("cold", "prepare" | "fail") => include_str!("trace-cold-prepare.txt"),
                    ("warm", "baseline" | "cancel") => include_str!("trace-warm-baseline.txt"),
                    ("warm", "prepare" | "fail") => include_str!("trace-warm-prepare.txt"),
                    _ => unreachable!("enumerated lifecycle"),
                };
                assert_eq!(snapshots, expected.trim_end(), "{lifecycle}-{action}");
            }
        }
        return;
    };
    for policy in [
        hyperlimit::PredicatePolicy::STRICT,
        hyperlimit::PredicatePolicy::APPROXIMATE_512,
    ] {
        let source = patch();
        let half = (Real::one() / Real::from(2)).unwrap();
        let parameter = Point2::new(half.clone(), half);
        let uv = hypercurve::Point2::new(parameter.x.clone(), parameter.y.clone()).into();
        if mode.starts_with("warm-") {
            source.evaluate_real(&parameter).unwrap();
        }
        let prepared = if mode.ends_with("-prepare") {
            Some(prepare_rational_bezier_surface_ranges(&source, &mut graph()).unwrap())
        } else if mode.ends_with("-fail") {
            assert!(matches!(
                prepare_rational_bezier_surface_ranges(
                    &source,
                    &mut ExpressionBudget::new(1_000_000, 1153)
                ),
                Err(SurfaceRangeError::Expression(
                    ExpressionError::WorkExhausted
                ))
            ));
            None
        } else if mode.ends_with("-cancel") {
            let token = hypersolve::work_budget::CancellationToken::new();
            token.cancel();
            let mut cancelled = graph();
            cancelled.cancellation = Some(token);
            assert!(matches!(
                prepare_rational_bezier_surface_ranges(&source, &mut cancelled),
                Err(SurfaceRangeError::Expression(ExpressionError::Cancelled))
            ));
            None
        } else {
            None
        };
        assert_eq!(
            native(&source).homogeneous_controls.get().is_some(),
            mode.starts_with("warm-")
        );
        dispatch_trace::reset();
        let outcome = dispatch_trace::with_recording(|| {
            hypercurve::evaluate_under(policy, || {
                (
                    source.evaluate_real(&parameter).unwrap(),
                    source.partials_at(&uv).unwrap(),
                )
            })
        });
        let snapshot = dispatch_trace::take_trace();
        let certainty = outcome.certainty();
        assert_eq!(certainty, hypercurve::CurveCertainty::Certified);
        let (point, partials) = outcome.certified().unwrap();
        let u = Point3::from(partials.u().try_coordinates().unwrap().clone());
        let v = Point3::from(partials.v().try_coordinates().unwrap().clone());
        let values = [point.x, point.y, point.z, u.x, u.y, u.z, v.x, v.y, v.z]
            .map(|value| value.exact_rational_normal_form().unwrap());
        let half = hyperreal::Rational::from(1) / hyperreal::Rational::from(2);
        assert_eq!(
            values,
            [
                half.clone(),
                1.into(),
                hyperreal::Rational::from(1) / hyperreal::Rational::from(4),
                1.into(),
                0.into(),
                half.clone(),
                0.into(),
                2.into(),
                half
            ]
        );
        println!("NATIVE_TRACE {policy:?} {certainty:?} {values:?} {snapshot:?}");
        drop(prepared);
    }
}
