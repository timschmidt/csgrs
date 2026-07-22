#![no_main]

use hypercurve::{
    BulgeVertex2, Classification, Contour2, Curve2, CurvePath2, CurvePolicy, Point2, Real,
    Segment2, StraightSkeletonStage2,
};
use libfuzzer_sys::fuzz_target;

const FIXTURES: &[&[(i32, i32)]] = &[
    &[(0, 0), (4, 0), (4, 3), (0, 3)],
    &[(0, 0), (3, 0), (3, 1), (1, 1), (1, 3), (0, 3)],
    &[
        (0, 0),
        (30, 0),
        (30, 24),
        (20, 24),
        (20, 7),
        (17, 11),
        (17, 24),
        (0, 24),
    ],
    &[
        (0, 0),
        (6, 0),
        (6, 6),
        (4, 6),
        (4, 2),
        (2, 2),
        (2, 6),
        (0, 6),
    ],
];

fn transformed_fixture(data: &[u8]) -> Contour2 {
    let source = FIXTURES[data[0] as usize % FIXTURES.len()];
    let scale = i32::from(data[1] % 16 + 1);
    let translate_x = i32::from(data[2]) - 128;
    let translate_y = i32::from(data[3]) - 128;
    let mut points = source
        .iter()
        .map(|&(x, y)| {
            Point2::new(
                Real::from(x * scale + translate_x),
                Real::from(y * scale + translate_y),
            )
        })
        .collect::<Vec<_>>();
    if data[4] & 1 != 0 {
        points.reverse();
    }
    Contour2::from_bulge_vertices(
        &points
            .into_iter()
            .map(|point| BulgeVertex2::new(point, Real::zero()))
            .collect::<Vec<_>>(),
    )
    .expect("topology-preserving fixture transform remains valid")
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 5 {
        return;
    }
    let contour = transformed_fixture(data);
    let policy = CurvePolicy::certified();

    let Classification::Decided(trajectories) = contour
        .straight_skeleton_vertex_trajectories(&policy)
        .expect("integer fixture trajectories remain exact")
    else {
        panic!("integer fixture trajectories became uncertain")
    };
    assert_eq!(trajectories.len(), contour.len());

    let Classification::Decided(local_events) = contour
        .straight_skeleton_local_arc_events(&policy)
        .expect("line fixture local queue remains exact")
    else {
        panic!("line fixture local queue became uncertain")
    };
    assert!(local_events.is_empty());

    assert!(matches!(
        contour
            .straight_skeleton_splice_events(&policy)
            .expect("fixture splice query remains exact"),
        Classification::Decided(_)
    ));
    assert!(matches!(
        contour
            .straight_skeleton_global_contact_events(&policy)
            .expect("fixture contact query remains exact"),
        Classification::Decided(_)
    ));

    let contour_report = contour
        .straight_skeleton(&policy)
        .expect("fixture construction remains exact");
    assert_eq!(contour_report.stage(), StraightSkeletonStage2::Complete);
    let skeleton = contour_report
        .skeleton()
        .expect("fixture construction remains complete");
    assert_eq!(skeleton.source_edge_count(), contour.len());
    assert!(skeleton.arcs().iter().all(|arc| {
        arc.start_node() < skeleton.nodes().len()
            && arc.end_node() < skeleton.nodes().len()
            && arc.start_node() != arc.end_node()
    }));

    let path = CurvePath2::try_new(
        contour
            .segments()
            .iter()
            .cloned()
            .map(|segment| match segment {
                Segment2::Line(line) => Curve2::from(line),
                Segment2::Arc(arc) => Curve2::from(arc),
            })
            .collect(),
    )
    .expect("fixture path remains connected");
    assert_eq!(
        path.straight_skeleton(&policy)
            .expect("native path dispatch remains exact"),
        contour_report,
    );
});
