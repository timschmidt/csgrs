#![no_main]

use hyperreal::Real;
use hypersolve::{
    certify_candidate, context_from_problem, PreparedProblem, SketchResidualFormKind,
    SketchResidualFormsStatus, SketchResidualStrategy, SketchSolveProblem,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: [i16; 8]| {
    let ox = i64::from(data[0]);
    let oy = i64::from(data[1]);
    let oz = i64::from(data[2]);
    let ax = i64::from(data[3]).abs() + 1;
    let by = i64::from(data[4]).abs() + 1;
    let scale_a = i64::from(data[5]).abs() + 1;
    let az = i64::from(data[6]);
    let bz = i64::from(data[7]);

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
    let base = sketch.add_point3d("base", Real::from(ox), Real::from(oy), Real::from(oz));
    let a_end = sketch.add_point3d(
        "a end",
        Real::from(ox + ax),
        Real::from(oy),
        Real::from(oz + az),
    );
    let b_end = sketch.add_point3d(
        "b end",
        Real::from(ox),
        Real::from(oy + by),
        Real::from(oz + bz),
    );
    let c_end = sketch.add_point3d(
        "c end",
        Real::from(ox + scale_a * ax),
        Real::from(oy),
        Real::from(oz - az),
    );
    let d_end = sketch.add_point3d(
        "d end",
        Real::from(ox),
        Real::from(oy + scale_a * by),
        Real::from(oz - bz),
    );
    let a = sketch.add_line_segment3("a", base, a_end);
    let b = sketch.add_line_segment3("b", base, b_end);
    let c = sketch.add_line_segment3("c", base, c_end);
    let d = sketch.add_line_segment3("d", base, d_end);
    let handle =
        sketch.add_projected_equal_oriented_angle_lines3("projected angle", workplane, a, b, c, d);

    let lowered = sketch.lower_to_problem();
    assert!(lowered.all_generated());
    assert_eq!(lowered.problem.constraints.len(), 3);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert!(lowered.rows[1..].iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::ProjectedOrientedAngleEquality)
    }));

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    assert!(certification.all_satisfied());

    let forms = sketch.residual_forms_for_constraint(handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 3);
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::ProjectedOrientedAngleVectorCollinearityPolynomial
    );
});
