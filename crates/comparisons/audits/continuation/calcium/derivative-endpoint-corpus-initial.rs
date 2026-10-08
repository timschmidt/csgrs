use hypercurve::{
    BezierArrangementFragment2, BezierArrangementGraph2, BezierParameter2, BezierSplitFragment2,
    BezierSubcurve2, Classification, CurveContext, Point2, RationalBezier2,
};
use hyperreal::Real;
use std::{hint::black_box, time::Instant};

fn q(n: usize, d: usize) -> Real {
    (Real::from(n as i64) / Real::from(d as i64)).unwrap()
}
fn run(counters: Option<fn(bool) -> [usize; 4]>) {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 6);
    let variant = &args[1];
    let kind: usize = args[2].parse().unwrap();
    let degree: usize = args[3].parse().unwrap();
    let lifecycle = &args[4];
    let iterations: usize = args[5].parse().unwrap();
    assert!(["baseline", "candidate"].contains(&variant.as_str()));
    assert!(kind < 2 && [1, 3, 8, 24].contains(&degree) && iterations > 0);
    assert!(["retained_graph", "fresh_graph"].contains(&lifecycle.as_str()));
    let scale = if kind == 0 { Real::one() } else { Real::pi() };
    // Four counterclockwise square sides with small inward control-point bends.
    // Positive weights keep each side in its nonintersecting convex hull.
    let templates: Vec<Vec<Point2>> = (0..4)
        .map(|side| {
            (0..=degree)
                .map(|i| {
                    let u = q(i, degree);
                    let bend = if i == 0 || i == degree {
                        Real::zero()
                    } else {
                        q((7 * i) % 11 + 1, 1000 * degree)
                    };
                    let (x, y) = match side {
                        0 => (u, bend),
                        1 => (Real::one() - bend, u),
                        2 => (Real::one() - u, Real::one() - bend),
                        3 => (bend, Real::one() - u),
                        _ => unreachable!(),
                    };
                    Point2::new(x * &scale, y * &scale)
                })
                .collect()
        })
        .collect();
    let weights: Vec<_> = (0..=degree)
        .map(|i| {
            if kind == 0 {
                Real::one()
            } else {
                q(degree + i, degree)
            }
        })
        .collect();
    let build = || {
        BezierArrangementGraph2::new(
            templates
                .iter()
                .enumerate()
                .map(|(i, points)| {
                    BezierArrangementFragment2::new(
                        i,
                        0,
                        BezierSplitFragment2::Materialized {
                            start: BezierParameter2::exact(Real::zero()),
                            end: BezierParameter2::exact(Real::one()),
                            curve: BezierSubcurve2::Rational(
                                RationalBezier2::try_new(points.clone(), weights.clone()).unwrap(),
                            ),
                        },
                    )
                })
                .collect(),
        )
        .unwrap()
    };
    let graph = build();
    let query = |graph: &BezierArrangementGraph2| {
        let Classification::Decided(result) = graph
            .traverse_with_tangent_order(&CurveContext::STRICT)
            .unwrap()
        else {
            panic!("square traversal was not certified")
        };
        assert_eq!(result.closed_count(), 1);
        assert_eq!(result.chains().len(), 1);
        assert_eq!(result.chains()[0].fragment_indices(), &[0, 1, 2, 3]);
        black_box(result);
    };
    query(&graph);
    for _ in 0..8 {
        if lifecycle == "fresh_graph" {
            query(&build())
        } else {
            query(&graph)
        }
    }
    let before = counters.map_or([0; 4], |f| f(true));
    let start = Instant::now();
    for _ in 0..iterations {
        if lifecycle == "fresh_graph" {
            query(&build())
        } else {
            query(&graph)
        }
    }
    let elapsed_ns = start.elapsed().as_nanos();
    let after = counters.map_or([0; 4], |f| f(false));
    let requests = after[0] - before[0];
    let requested_bytes = after[1] - before[1];
    let live_delta = after[2] as i128 - before[2] as i128;
    let peak_delta = after[3].saturating_sub(before[2]);
    let mode = if counters.is_some() {
        "allocation"
    } else {
        "cpu"
    };
    println!(
        "{{\"mode\":\"{mode}\",\"variant\":\"{variant}\",\"kind\":{kind},\"degree\":{degree},\"lifecycle\":\"{lifecycle}\",\"iterations\":{iterations},\"elapsed_ns\":{elapsed_ns},\"requests\":{requests},\"requested_bytes\":{requested_bytes},\"live_delta\":{live_delta},\"peak_delta\":{peak_delta}}}"
    );
}
