#![no_main]

use hyperbrep::{
    BrepFeatureId, BrepPlanarExtrusionConstruction, BrepSourceVersion, BrepVoxelHandoffBlocker,
};
use hypercurve::{Contour2, CurvePolicy, CurveRegion2, LineSeg2, Segment2};
use hyperreal::Real;
use libfuzzer_sys::fuzz_target;

fn r(value: i32) -> Real {
    Real::from(value)
}

fn uv(x: i32, y: i32) -> hypercurve::Point2 {
    hypercurve::Point2::new(r(x), r(y))
}

fn line(start: hypercurve::Point2, end: hypercurve::Point2) -> Segment2 {
    Segment2::Line(LineSeg2::try_new(start, end).unwrap())
}

fn rectangle(width: i32, depth: i32) -> CurveRegion2 {
    CurveRegion2::try_from_native_material_contours(
        vec![
        Contour2::try_new(vec![
            line(uv(0, 0), uv(width, 0)),
            line(uv(width, 0), uv(width, depth)),
            line(uv(width, depth), uv(0, depth)),
            line(uv(0, depth), uv(0, 0)),
        ])
        .unwrap(),
        ],
        &CurvePolicy::certified(),
    )
    .unwrap()
}

fuzz_target!(|data: (u8, u8, u8, bool)| {
    let (width_raw, depth_raw, height_raw, stale_source) = data;
    let width = i32::from((width_raw % 12) + 1);
    let depth = i32::from((depth_raw % 12) + 1);
    let height = i32::from((height_raw % 12) + 1);
    let constructed = BrepPlanarExtrusionConstruction::vertical_prism_from_region(
        &rectangle(width, depth),
        r(0),
        r(height),
        BrepFeatureId::new("fuzz:voxel-handoff").unwrap(),
        vec![BrepSourceVersion::new("fuzz:rectangle", 1).unwrap()],
    );
    assert!(constructed.exact_construction_ready);
    let shell = constructed.shell.as_ref().unwrap();
    let expected = hypervoxel::GridSource::new("fuzz:voxel-handoff", 1);
    let frame_source = if stale_source {
        hypervoxel::GridSource::new("fuzz:voxel-handoff", 0)
    } else {
        expected.clone()
    };
    let frame = hypervoxel::GridFrame::new(
        [r(0), r(0), r(0)],
        [r(1), r(1), r(1)],
        5,
        hypervoxel::LengthUnit::Unitless,
        Some(frame_source),
    )
    .unwrap();
    let report = shell.voxel_handoff_report(frame, Some(expected));
    assert!(report.exact_triangle_source_ready);
    assert_eq!(report.exact_triangle_voxelization_ready, !stale_source);
    if stale_source {
        assert!(
            report
                .blockers
                .contains(&BrepVoxelHandoffBlocker::StaleFrameSource)
        );
        assert!(report.prepared_triangle_solid.is_some());
    } else {
        assert!(report.blockers.is_empty());
        assert!(report.prepared_triangle_solid.is_some());
        assert!(report.prepared_triangle_solid_report.is_some());
    }
});
