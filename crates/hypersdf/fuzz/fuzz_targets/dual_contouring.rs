#![no_main]

use hyperlimit::Point3;
use hyperreal::Real;
use hypersdf::{
    SdfDualContouringBlocker, SdfDualContouringReport, SdfExpr, SdfPreviewGrid,
    SdfSamplingPrecision, prepare,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() < 8 {
        return;
    }
    let mode = data[0] % 4;
    let origin_x = i32::from(data[1] % 25) - 12;
    let origin_y = i32::from(data[2] % 7) - 3;
    let origin_z = i32::from(data[3] % 7) - 3;
    let step_x = i32::from(data[4] % 6) + 1;
    let step_y = i32::from(data[5] % 4) + 1;
    let step_z = i32::from(data[6] % 4) + 1;
    let dim = u32::from(data[7] % 4) + 2;

    let sdf = match mode {
        0 => prepare(SdfExpr::x().sub_expr(SdfExpr::constant(r(origin_x + step_x)))),
        1 => prepare(
            SdfExpr::x()
                .mul_expr(SdfExpr::x())
                .sub_expr(SdfExpr::constant(r((origin_x + step_x).abs() + 1))),
        ),
        2 => prepare(SdfExpr::sphere(p(0, 0, 0), r(16))),
        _ => prepare(SdfExpr::x()),
    };
    let grid = SdfPreviewGrid::new(
        p(origin_x, origin_y, origin_z),
        p(step_x, step_y, step_z),
        [dim, 2, 2],
    );
    let report = sdf
        .dual_contouring_report_from_grid(grid, SdfSamplingPrecision::F64)
        .expect("positive fuzz grid");
    assert!(report.is_self_consistent());
    assert_eq!(report.validation_handoff_ready, report.blockers.is_empty());
    if report.validation_handoff_ready {
        assert_eq!(report.unknown_sample_count, 0);
        assert_eq!(report.sampled_edge_root_count, 0);
        assert!(report
            .crossings
            .iter()
            .all(|crossing| crossing.exact_hermite_ready()));
    }

    let sampled = SdfDualContouringReport::from_signed_grid_samples(report.grid_samples.clone());
    assert!(sampled.is_self_consistent());
    assert!(!sampled.validation_handoff_ready);
    assert!(sampled
        .blockers
        .contains(&SdfDualContouringBlocker::LossyPrimitiveSamples));
});

fn r(value: i32) -> Real {
    Real::from(value)
}

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}
