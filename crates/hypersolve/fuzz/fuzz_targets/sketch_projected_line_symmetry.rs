#![no_main]

use hyperreal::Real;
use hypersolve::{
    certify_candidate, context_from_problem, PreparedProblem, SketchGeneratedRowStatus,
    SketchResidualStrategy, SketchSolveProblem,
};
use libfuzzer_sys::fuzz_target;

fn value(data: &[u8], index: usize) -> i64 {
    i64::from(i16::from_le_bytes([data[index], data[index + 1]]))
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 18 {
        return;
    }
    let mode = data[0] % 5;
    let ox = value(data, 1);
    let oy = value(data, 3);
    let oz = value(data, 5);
    let sx = value(data, 7);
    let sy = value(data, 9);
    let ux = i64::from(data[11] % 15) - 7;
    let uy = i64::from(data[12] % 15) - 7;
    if ux == 0 && uy == 0 {
        return;
    }
    let t = i64::from(data[13] % 9) - 4;
    let offset = i64::from(data[14] % 9) - 4;
    let z0 = value(data, 15);
    let z1 = i64::from(data[17]) - 128;
    let midpoint_x = sx + t * ux;
    let midpoint_y = sy + t * uy;
    let normal_x = -uy;
    let normal_y = ux;

    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", Real::from(ox), Real::from(oy), Real::from(oz));
    let normal = sketch.add_normal3d(
        "normal",
        Real::from(1),
        Real::from(0),
        Real::from(0),
        Real::from(0),
    );
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let axis_start = sketch.add_point3d(
        "axis start",
        Real::from(ox + sx),
        Real::from(oy + sy),
        Real::from(oz + z0),
    );
    let axis_end = sketch.add_point3d(
        "axis end",
        Real::from(ox + sx + ux),
        Real::from(oy + sy + uy),
        Real::from(oz + z1),
    );
    let axis = sketch.add_line_segment3("axis", axis_start, axis_end);
    let point2 = sketch.add_point2d("point2", Real::from(sx), Real::from(sy));
    let wrong_shift = i64::from(data[17] % 7) + 1;
    let a = sketch.add_point3d(
        "a",
        Real::from(ox + midpoint_x + offset * normal_x),
        Real::from(oy + midpoint_y + offset * normal_y),
        Real::from(oz + z0 + z1),
    );
    let b = match mode {
        1 => sketch.add_point3d(
            "wrong midpoint",
            Real::from(ox + midpoint_x - offset * normal_x + wrong_shift),
            Real::from(oy + midpoint_y - offset * normal_y),
            Real::from(oz - z0 - z1),
        ),
        2 => sketch.add_point3d(
            "wrong perpendicular",
            Real::from(ox + midpoint_x - offset * normal_x + wrong_shift * ux),
            Real::from(oy + midpoint_y - offset * normal_y + wrong_shift * uy),
            Real::from(oz - z0 - z1),
        ),
        _ => sketch.add_point3d(
            "b",
            Real::from(ox + midpoint_x - offset * normal_x),
            Real::from(oy + midpoint_y - offset * normal_y),
            Real::from(oz - z0 - z1),
        ),
    };

    if mode == 3 {
        sketch.add_projected_symmetric_line3("wrong axis", workplane, a, b, point2);
        let lowered = sketch.lower_to_problem();
        assert_eq!(lowered.problem.constraints.len(), 0);
        assert!(matches!(
            lowered.rows[0].status,
            SketchGeneratedRowStatus::WrongEntityKind { .. }
        ));
        return;
    }

    sketch.add_projected_symmetric_line3("projected line symmetry", workplane, a, b, axis);
    let lowered = sketch.lower_to_problem();
    assert_eq!(lowered.problem.constraints.len(), 3);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert!(lowered.rows[1..].iter().all(|row| {
        row.status == SketchGeneratedRowStatus::Generated
            && row.strategy == Some(SketchResidualStrategy::ProjectedLineSymmetryPolynomial)
    }));
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    match mode {
        0 | 4 => assert!(certification.all_satisfied()),
        1 | 2 => assert!(!certification.all_satisfied()),
        _ => unreachable!(),
    }
});
