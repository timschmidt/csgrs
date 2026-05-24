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
    if data.len() < 16 {
        return;
    }
    let mode = data[0] % 6;
    let sx = value(data, 1);
    let sy = value(data, 3);
    let ux = i64::from(data[5] % 15) - 7;
    let uy = i64::from(data[6] % 15) - 7;
    if ux == 0 && uy == 0 {
        return;
    }
    let t = i64::from(data[7] % 9) - 4;
    let off = i64::from(data[8] % 7) + 1;
    let ox = value(data, 9);
    let oy = value(data, 11);
    let oz = value(data, 13);
    let normal_offset = i64::from(data[15]) - 128;

    let mut sketch = SketchSolveProblem::new();
    if mode < 3 {
        let start = sketch.add_point2d("start", Real::from(sx), Real::from(sy));
        let end = sketch.add_point2d("end", Real::from(sx + ux), Real::from(sy + uy));
        let line = sketch.add_line_segment2("line", start, end);
        let point = if mode == 1 {
            sketch.add_point2d(
                "off line",
                Real::from(sx + t * ux - off * uy),
                Real::from(sy + t * uy + off * ux),
            )
        } else {
            sketch.add_point2d("point", Real::from(sx + t * ux), Real::from(sy + t * uy))
        };
        if mode == 2 {
            let wrong_line = sketch.add_distance("wrong line", Real::from(off));
            sketch.add_point_on_line2("wrong line kind", point, wrong_line);
            let lowered = sketch.lower_to_problem();
            assert_eq!(lowered.problem.constraints.len(), 0);
            assert!(matches!(
                lowered.rows[0].status,
                SketchGeneratedRowStatus::WrongEntityKind { .. }
            ));
            return;
        }
        sketch.add_point_on_line2("point on line", point, line);
        let lowered = sketch.lower_to_problem();
        assert_eq!(lowered.problem.constraints.len(), 1);
        assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::PointLineIncidence)
        );
        let certification = certify_candidate(
            &PreparedProblem::new(&lowered.problem),
            &context_from_problem(&lowered.problem),
        );
        if mode == 0 {
            assert!(certification.all_satisfied());
        } else {
            assert!(!certification.all_satisfied());
        }
        return;
    }

    let origin = sketch.add_point3d("origin", Real::from(ox), Real::from(oy), Real::from(oz));
    let normal = sketch.add_normal3d(
        "normal",
        Real::from(1),
        Real::from(0),
        Real::from(0),
        Real::from(0),
    );
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let start = sketch.add_point3d(
        "start",
        Real::from(ox + sx),
        Real::from(oy + sy),
        Real::from(oz + normal_offset),
    );
    let end = sketch.add_point3d(
        "end",
        Real::from(ox + sx + ux),
        Real::from(oy + sy + uy),
        Real::from(oz - normal_offset),
    );
    let line = sketch.add_line_segment3("line", start, end);
    let point = if mode == 4 {
        sketch.add_point3d(
            "off projected line",
            Real::from(ox + sx + t * ux - off * uy),
            Real::from(oy + sy + t * uy + off * ux),
            Real::from(oz + off),
        )
    } else {
        sketch.add_point3d(
            "point",
            Real::from(ox + sx + t * ux),
            Real::from(oy + sy + t * uy),
            Real::from(oz + off),
        )
    };
    if mode == 5 {
        let wrong_line = sketch.add_point2d("wrong line", Real::from(sx), Real::from(sy));
        sketch.add_projected_point_on_line3("wrong line kind", workplane, point, wrong_line);
        let lowered = sketch.lower_to_problem();
        assert_eq!(lowered.problem.constraints.len(), 0);
        assert!(matches!(
            lowered.rows[0].status,
            SketchGeneratedRowStatus::WrongEntityKind { .. }
        ));
        return;
    }
    sketch.add_projected_point_on_line3("projected point on line", workplane, point, line);
    let lowered = sketch.lower_to_problem();
    assert_eq!(lowered.problem.constraints.len(), 2);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert_eq!(
        lowered.rows[1].strategy,
        Some(SketchResidualStrategy::ProjectedPointLineIncidence)
    );
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    if mode == 3 {
        assert!(certification.all_satisfied());
    } else {
        assert!(!certification.all_satisfied());
    }
});
