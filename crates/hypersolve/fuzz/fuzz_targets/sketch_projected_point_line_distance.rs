#![no_main]

use hyperreal::Real;
use hypersolve::{
    certify_candidate, context_from_problem, CertifiedCandidateStatus, PreparedProblem,
    SketchResidualFormKind, SketchResidualFormsStatus, SketchResidualStrategy, SketchSolveProblem,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() < 16 {
        return;
    }
    let value = |index: usize| i64::from(i16::from_le_bytes([data[index], data[index + 1]]));
    let lx = value(0);
    let ly = value(2);
    let lz = value(4);
    let mut length = value(6).abs();
    if length == 0 {
        length = 1;
    }
    let distance = value(8).abs();
    let point_x_offset = value(10);
    let point_z_offset = value(12);
    let line_z_offset = value(14);

    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(0));
    let normal = sketch.add_normal3d(
        "normal",
        Real::from(1),
        Real::from(0),
        Real::from(0),
        Real::from(0),
    );
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let start = sketch.add_point3d("start", Real::from(lx), Real::from(ly), Real::from(lz));
    let end = sketch.add_point3d(
        "end",
        Real::from(lx + length),
        Real::from(ly),
        Real::from(lz + line_z_offset),
    );
    let line = sketch.add_line_segment3("line", start, end);
    let point = sketch.add_point3d(
        "point",
        Real::from(lx + point_x_offset),
        Real::from(ly + distance),
        Real::from(lz + point_z_offset),
    );
    let distance = sketch.add_distance("distance", Real::from(distance));
    let handle = sketch.add_projected_point_line_distance(
        "projected point-line",
        workplane,
        point,
        line,
        distance,
    );

    let lowered = sketch.lower_to_problem();
    assert!(lowered.all_generated());
    assert_eq!(lowered.problem.constraints.len(), 2);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert_eq!(
        lowered.rows[1].strategy,
        Some(SketchResidualStrategy::SquaredProjectedPointLineDistance)
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 4);
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::SquaredProjectedPointLineDistancePolynomial
    );
});
