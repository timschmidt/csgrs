#![no_main]

use hyperlimit::Point3;
use hyperreal::Real;
use hypersdf::{
    SdfExpr, SdfGradientContourBlocker, SdfGradientContourReport, SdfPreviewGrid,
    SdfSamplingPrecision, prepare,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() < 9 {
        return;
    }
    let mode = data[0] % 5;
    let origin_x = i32::from(data[1] % 31) - 15;
    let origin_y = i32::from(data[2] % 9) - 4;
    let origin_z = i32::from(data[3] % 9) - 4;
    let step_x = i32::from(data[4] % 6) + 1;
    let step_y = i32::from(data[5] % 4) + 1;
    let step_z = i32::from(data[6] % 4) + 1;
    let nx = u32::from(data[7] % 4) + 2;
    let ny = u32::from(data[8] % 4) + 2;

    let sdf = match mode {
        0 => prepare(SdfExpr::x().sub_expr(SdfExpr::constant(r(origin_x + step_x)))),
        1 => prepare(SdfExpr::x()),
        2 => prepare(SdfExpr::sphere(p(0, 0, 0), r(25))),
        3 => prepare(SdfExpr::x().tan()),
        _ => prepare(SdfExpr::x().mul_expr(SdfExpr::x()).sub_expr(SdfExpr::constant(r(4)))),
    };
    let grid = SdfPreviewGrid::new(
        p(origin_x, origin_y, origin_z),
        p(step_x, step_y, step_z),
        [nx, ny, 2],
    );
    let report = sdf
        .gradient_contouring_report_from_grid(grid, SdfSamplingPrecision::F64)
        .expect("positive fuzz grid");
    assert!(report.is_self_consistent());
    assert!(!report.validation_handoff_ready);
    assert!(report
        .blockers
        .contains(&SdfGradientContourBlocker::LossyGradientApproximation));
    assert!(report.kept_projection_count <= report.active_cell_count);
    assert_eq!(
        report.rejected_projection_count,
        report.active_cell_count
            .saturating_sub(report.kept_projection_count)
    );
    for projection in &report.projections {
        if projection.is_kept() {
            let point = projection.projected_point.expect("kept projection point");
            for axis in 0..3 {
                assert!(point[axis] >= projection.cell_bounds[0][axis]);
                assert!(point[axis] <= projection.cell_bounds[1][axis]);
            }
        }
    }

    let sampled = SdfGradientContourReport::from_signed_grid_samples(report.grid_samples.clone());
    assert!(sampled.is_self_consistent());
    assert!(!sampled.validation_handoff_ready);
    assert!(sampled
        .blockers
        .contains(&SdfGradientContourBlocker::LossyGradientApproximation));
});

fn r(value: i32) -> Real {
    Real::from(value)
}

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}
