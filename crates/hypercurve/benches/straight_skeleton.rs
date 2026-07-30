use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    Classification, Contour2, Curve2, CurvePath2, CurvePolicy, LineSeg2, Point2, Real, Segment2,
    StraightSkeletonArc2, StraightSkeletonArcGeometry2, StraightSkeletonArcKind2,
    StraightSkeletonNode2, StraightSkeletonStage2,
};

fn r(value: i32) -> Real {
    value.into()
}

fn contour(points: &[(i32, i32)]) -> Contour2 {
    let points = points
        .iter()
        .map(|&(x, y)| Point2::new(r(x), r(y)))
        .collect::<Vec<_>>();
    Contour2::try_new(
        (0..points.len())
            .map(|index| {
                Segment2::Line(
                    LineSeg2::try_new(
                        points[index].clone(),
                        points[(index + 1) % points.len()].clone(),
                    )
                    .expect("benchmark edge is nonzero"),
                )
            })
            .collect(),
    )
    .expect("benchmark contour is connected")
}

fn orthogonal_comb(teeth: i32) -> Contour2 {
    let width = 4 * teeth + 2;
    let height = 4 * teeth + 4;
    let mut points = vec![(0, 0), (width, 0), (width, height)];
    for tooth in (0..teeth).rev() {
        let right = 4 * tooth + 4;
        let left = 4 * tooth + 2;
        let depth = tooth + 1;
        points.extend([
            (right, height),
            (right, depth),
            (left, depth),
            (left, height),
        ]);
    }
    points.push((0, height));
    contour(&points)
}

fn as_curve_path(source: &Contour2) -> CurvePath2 {
    CurvePath2::try_new(
        source
            .segments()
            .iter()
            .cloned()
            .map(|segment| match segment {
                Segment2::Line(line) => Curve2::from(line),
                Segment2::Arc(arc) => Curve2::from(arc),
            })
            .collect(),
    )
    .expect("benchmark path is connected")
}

fn measure(name: &str, iterations: u32, mut workload: impl FnMut() -> usize) {
    if let Ok(group) = std::env::var("HYPERCURVE_STRAIGHT_SKELETON_GROUP")
        && !name.contains(&group)
    {
        return;
    }
    let iterations = std::env::var("HYPERCURVE_STRAIGHT_SKELETON_ITERATIONS")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(iterations);
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..iterations {
        checksum = checksum.wrapping_add(black_box(workload()));
    }
    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );
}

fn main() {
    println!(
        "straight_skeleton/carrier_bytes: node={}, arc={}, arc_kind={}, arc_geometry={}",
        std::mem::size_of::<StraightSkeletonNode2>(),
        std::mem::size_of::<StraightSkeletonArc2>(),
        std::mem::size_of::<StraightSkeletonArcKind2>(),
        std::mem::size_of::<StraightSkeletonArcGeometry2>(),
    );
    let policy = CurvePolicy::STRICT;
    let concave = contour(&[
        (0, 0),
        (30, 0),
        (30, 24),
        (20, 24),
        (20, 7),
        (17, 11),
        (17, 24),
        (0, 24),
    ]);
    let concave_path = as_curve_path(&concave);

    measure("straight_skeleton/concave/trajectories", 1_000, || {
        let Classification::Decided(trajectories) = concave
            .straight_skeleton_vertex_trajectories(black_box(&policy))
            .expect("trajectory query is exact")
        else {
            panic!("trajectory query became uncertain")
        };
        trajectories.len()
    });
    measure("straight_skeleton/concave/global_contacts", 250, || {
        let Classification::Decided(events) = concave
            .straight_skeleton_global_contact_events(black_box(&policy))
            .expect("global contact query is exact")
        else {
            panic!("global contact query became uncertain")
        };
        events.len()
    });
    measure("straight_skeleton/concave/contour", 100, || {
        let evidence = concave
            .straight_skeleton(black_box(&policy))
            .expect("contour construction is exact");
        assert_eq!(evidence.stage(), StraightSkeletonStage2::Complete);
        let skeleton = evidence.skeleton().expect("construction completes");
        skeleton.nodes().len() + skeleton.arcs().len()
    });
    measure("straight_skeleton/concave/curve_path", 100, || {
        let evidence = concave_path
            .straight_skeleton(black_box(&policy))
            .expect("path dispatch is exact");
        assert_eq!(evidence.stage(), StraightSkeletonStage2::Complete);
        let skeleton = evidence.skeleton().expect("construction completes");
        skeleton.nodes().len() + skeleton.arcs().len()
    });

    for teeth in [1, 2, 4] {
        let source = orthogonal_comb(teeth);
        let iterations = match teeth {
            1 => 500,
            2 => 100,
            _ => 10,
        };
        measure(
            &format!("straight_skeleton/comb_{}/evidence", source.len()),
            iterations,
            || {
                let evidence = source
                    .straight_skeleton(black_box(&policy))
                    .expect("comb construction evidence is exact");
                evidence.event_count()
                    + evidence
                        .skeleton()
                        .map_or(0, |skeleton| skeleton.nodes().len() + skeleton.arcs().len())
                    + usize::from(evidence.blocker().is_some())
            },
        );
    }
}
