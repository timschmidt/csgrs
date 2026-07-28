#![no_main]

use hyperbrep::vertical_prism_shell;
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
    let (width_raw, depth_raw, height_raw, _) = data;
    let width = i32::from((width_raw % 12) + 1);
    let depth = i32::from((depth_raw % 12) + 1);
    let height = i32::from((height_raw % 12) + 1);
    let shell = vertical_prism_shell(
        &rectangle(width, depth),
        r(0),
        r(height),
    )
    .unwrap();
    let frame = hypervoxel::GridFrame::new(
        [r(0), r(0), r(0)],
        [r(1), r(1), r(1)],
        5,
        hypervoxel::LengthUnit::Unitless,
    )
    .unwrap();
    let geometry = shell.voxel_geometry().unwrap();
    let _ = hypervoxel::voxelize_exact_triangle_solid(
        frame,
        &geometry.triangle_solid,
        hypervoxel::MaterialRegionId(1),
        hypervoxel::VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
});
