use criterion::{Criterion, criterion_group, criterion_main};
use hyperreal::{Rational, Real};
use hypersolve::{
    AlgebraicRootArithmeticOp, AlgebraicRootKind, AlgebraicRootRefinementComparisonConfig,
    AlgebraicRootRepresentation, AlgebraicRootValidationReport, AlgebraicRootValidationStatus,
    BatchPredicateScheduleConfig, BsplineKnotSpanSubstitutionConfig, Constraint,
    CurveResultantParameter, DraggedParameterWeight, EqualitySubstitution, Expr,
    IntervalBoxCertificationPackage, IsolatedRootInterval, PolynomialCurvePoint2,
    PolynomialParametricCurve2, PreparedProblem, PreparedSolverBlock, Problem, ProposalEngineKind,
    ProposalEnginePrecision, ProposalEngineReport, RationalCurveControlPoint2,
    RationalParametricCurve2, SolverConfig, SolverPoint2, SolverState, SparseResidualTerm,
    SymbolId, UnivariateResultantPairInput, VariableBall, analyze_exact_affine_rank,
    analyze_sparse_bareiss_elimination_pattern, apply_equality_substitution_classes,
    arithmetic_algebraic_root_representations, audit_active_set, audit_sketch_unit_tolerances,
    build_equality_substitution_classes, certify_affine_krawczyk_box, certify_candidate,
    certify_candidate_batch, certify_candidate_domains, certify_direct_univariate_quadratic_roots,
    certify_interval_box_candidate, certify_multivariate_quadratic_interval_candidate,
    certify_multivariate_quadratic_krawczyk_box, certify_quadratic_interval_candidate,
    certify_sketch_construction, certify_univariate_quadratic_alpha,
    certify_univariate_quadratic_krawczyk_box, compare_algebraic_root_representations,
    compare_algebraic_root_representations_by_difference,
    compare_algebraic_root_representations_with_refinement, context_from_problem,
    count_bernstein_univariate_polynomial_interval_roots,
    count_descartes_univariate_polynomial_roots, determinant_bareiss, diagnose_failed_constraints,
    diagnose_sketch_failed_constraints, eliminate_affine_rows_with_substitution_classes,
    enumerate_direct_univariate_quadratic_branches, evaluate_polynomial_at_algebraic_root,
    evaluate_rational_expression_at_algebraic_root, isolate_univariate_polynomial_roots,
    lift_sketch_point2_to_workplane3, preflight_sketch_degeneracies,
    preflight_sketch_entity_domains, preflight_sketch_parameter_domains,
    prepare_sparse_linear_residual_system, propose_active_set_update,
    regenerate_active_set_affine_candidate, regenerate_active_set_quadratic_candidates,
    replay_dense_linear_residuals, replay_sketch_compatibility_fixture,
    replay_sparse_linear_residuals, report_lossy_adapter_only_candidate,
    represent_univariate_algebraic_roots, resultant_parametric_curve_intersection,
    resultant_rational_parametric_curve_intersection, resultant_univariate_polynomials,
    run_active_set_update_loop, schedule_candidate_batch_predicates,
    schedule_univariate_resultant_pairs, search_failed_constraint_minimal_removals,
    search_failed_constraint_pair_removals, search_failed_constraint_set_removals,
    search_failed_constraint_single_removals, sketch_compatibility_fixtures,
    solve_damped_least_squares, solve_dense_linear_system_bareiss, solve_direct_affine_system,
    solve_direct_univariate_quadratic_equalities, solve_sparse_linear_system_bareiss,
    solve_sparse_linear_system_bareiss_pattern_preserving, squared_distance_equation,
    subdivide_bernstein_univariate_polynomial_interval_roots,
    subresultant_chain_univariate_polynomials, substitute_bezier_power_basis,
    substitute_bspline_knot_span_power_basis, substitute_nurbs_knot_span_power_basis,
    substitute_rational_bezier_power_basis, transform_algebraic_root_affine,
    transform_algebraic_root_mobius, transform_algebraic_root_polynomial_image,
    transform_algebraic_root_rational_image, transform_algebraic_roots_binary,
};

fn r(value: i64) -> Real {
    Real::new(Rational::new(value))
}

fn affine_problem(row_count: usize) -> Problem {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", r(2));
    for index in 0..row_count {
        problem.add_constraint(Constraint::equality(
            format!("row {index}"),
            x.clone() * Expr::int(index as i64 + 1) - Expr::int((index as i64 + 1) * 2),
        ));
    }
    problem
}

fn affine_krawczyk_problem() -> Problem {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", r(2));
    problem.add_variable("y", r(2));
    problem.add_constraint(Constraint::equality(
        "x plus y minus five",
        x.clone() + y.clone() - Expr::int(5),
    ));
    problem.add_constraint(Constraint::equality(
        "x minus y minus one",
        x - y - Expr::int(1),
    ));
    problem
}

fn substitution_elimination_problem(row_count: usize) -> Problem {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let z = Expr::symbol(SymbolId(2), "z");
    let mut problem = Problem::default();
    problem.add_variable("x", r(0));
    problem.add_variable("y", r(0));
    problem.add_variable("z", r(0));
    problem.add_constraint(Constraint::equality(
        "y - x - 3",
        y.clone() - x.clone() - Expr::int(3),
    ));
    problem.add_constraint(Constraint::equality(
        "z - x + 2",
        z.clone() - x.clone() + Expr::int(2),
    ));
    for index in 0..row_count {
        let scale = index as i64 + 1;
        problem.add_constraint(Constraint::equality(
            format!("reduced row {index}"),
            z.clone() * Expr::int(scale) + y.clone() * Expr::int(scale + 1) - Expr::int(scale),
        ));
    }
    problem
}

fn univariate_quadratic_problem(row_count: usize) -> Problem {
    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", r(3));
    for index in 0..row_count {
        let scale = index as i64 + 1;
        problem.add_constraint(Constraint::equality(
            format!("quadratic {index}"),
            x.clone() * x.clone() * Expr::int(scale) - x.clone() * Expr::int(2 * scale)
                + Expr::int(scale),
        ));
    }
    problem
}

fn multivariate_quadratic_problem(row_count: usize) -> Problem {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", r(3));
    problem.add_variable("y", r(5));
    for index in 0..row_count {
        let scale = index as i64 + 1;
        problem.add_constraint(Constraint::equality(
            format!("mixed quadratic {index}"),
            x.clone() * y.clone() * Expr::int(scale) + x.clone().powi(2) * Expr::int(scale + 1)
                - y.clone() * Expr::int(scale + 2)
                + Expr::int(scale),
        ));
    }
    problem
}

fn domain_problem(row_count: usize) -> Problem {
    let x = Expr::symbol(SymbolId(0), "x");
    let y = Expr::symbol(SymbolId(1), "y");
    let mut problem = Problem::default();
    problem.add_variable("x", r(9));
    problem.add_variable("y", r(3));
    for index in 0..row_count {
        problem.add_constraint(Constraint::equality(
            format!("domain row {index}"),
            (x.clone().sqrt()
                + (x.clone() / y.clone()).log10()
                + x.clone().ln()
                + y.clone().acos()
                + y.clone().atanh()
                + x.clone().acosh())
                / Expr::int(index as i64 + 1),
        ));
    }
    problem
}

fn sketch_problem(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let a = sketch.add_point2d(format!("a{index}"), r(index as i64), r(0));
        let b = sketch.add_point2d(format!("b{index}"), r(index as i64 + 3), r(4));
        let distance = sketch.add_distance(format!("d{index}"), r(5));
        let line = sketch.add_line_segment2(format!("line{index}"), a, b);
        let circle = sketch.add_circle2(format!("circle{index}"), a, distance);
        hypersolve::sketch_distance_builders::point_point_distance(
            &mut sketch,
            format!("distance {index}"),
            a,
            b,
            distance,
        );
        hypersolve::sketch_incidence_builders::point_on_circle(
            &mut sketch,
            format!("circle incidence {index}"),
            b,
            circle,
        );
        hypersolve::sketch_orientation_builders::horizontal(
            &mut sketch,
            format!("horizontal proposal {index}"),
            line,
        );
    }
    sketch
}

fn sketch_problem_with_point_on_arcs(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let y = index as i64;
        let center = sketch.add_point2d(format!("arcpnt{index}.center"), r(0), r(y));
        let start = sketch.add_point2d(format!("arcpnt{index}.start"), r(5), r(y));
        let end = sketch.add_point2d(format!("arcpnt{index}.end"), r(0), r(y + 5));
        let radius = sketch.add_distance(format!("arcpnt{index}.radius"), r(5));
        let arc =
            sketch.add_arc_of_circle2(format!("arcpnt{index}.arc"), center, start, end, radius);
        let point = sketch.add_point2d(format!("arcpnt{index}.point"), r(3), r(y + 4));
        hypersolve::sketch_incidence_builders::point_on_arc2(
            &mut sketch,
            format!("point on arc {index}"),
            point,
            arc,
            hypersolve::SketchArcPointSweep::CounterClockwiseMinor,
        );
    }
    sketch
}

fn sketch_problem_with_point_on_lines(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let y = index as i64;
        let start = sketch.add_point2d(format!("pntline{index}.start"), r(0), r(y));
        let end = sketch.add_point2d(format!("pntline{index}.end"), r(5), r(y + 10));
        let line = sketch.add_line_segment2(format!("pntline{index}.line"), start, end);
        let point = sketch.add_point2d(format!("pntline{index}.point"), r(2), r(y + 4));
        hypersolve::sketch_incidence_builders::point_on_line2(
            &mut sketch,
            format!("point on line {index}"),
            point,
            line,
        );
    }
    sketch
}

fn sketch_problem_with_metadata(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = sketch_problem(row_count);
    for index in 0..sketch.parameters().len() {
        let handle = sketch.parameters()[index].handle;
        sketch.set_parameter_metadata(
            handle,
            hypersolve::SketchRoundTripMetadata {
                source_unit: Some("mm".to_owned()),
                declared_tolerance: Some(r(1)),
                display_label: Some(format!("p{index}")),
                ..hypersolve::SketchRoundTripMetadata::default()
            },
        );
    }
    for index in 0..sketch.constraints().len() {
        let handle = sketch.constraints()[index].handle;
        sketch.set_constraint_metadata(
            handle,
            hypersolve::SketchRoundTripMetadata {
                display_label: Some(format!("constraint {index}")),
                lossy_adapter_label: Some("bench adapter".to_owned()),
                ..hypersolve::SketchRoundTripMetadata::default()
            },
        );
    }
    sketch
}

fn sketch_problem_with_ranges(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let parameter = sketch.add_parameter(format!("t{index}"), r(index as i64));
        hypersolve::sketch_range_builders::parameter_range(
            &mut sketch,
            format!("range {index}"),
            parameter,
            Some(r(-1)),
            Some(r(row_count as i64 + 1)),
        );
        hypersolve::sketch_objective_builders::stay_near_parameter(
            &mut sketch,
            format!("stay near {index}"),
            parameter,
            r(index as i64),
            r(1),
        );
    }
    sketch
}

fn sketch_problem_with_distance_ranges(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let a = sketch.add_point2d(format!("clearance{index}.a"), r(index as i64), r(0));
        let b = sketch.add_point2d(format!("clearance{index}.b"), r(index as i64 + 3), r(4));
        hypersolve::sketch_distance_builders::point_point_distance_range(
            &mut sketch,
            format!("clearance window {index}"),
            a,
            b,
            Some(r(4)),
            Some(r(6)),
        );
    }
    sketch
}

fn sketch_problem_with_equal_length_and_radius(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let y = index as i64;
        let a0 = sketch.add_point2d(format!("equal{index}.a0"), r(0), r(y));
        let a1 = sketch.add_point2d(format!("equal{index}.a1"), r(3), r(y + 4));
        let b0 = sketch.add_point2d(format!("equal{index}.b0"), r(10), r(y));
        let b1 = sketch.add_point2d(format!("equal{index}.b1"), r(6), r(y + 3));
        let line_a = sketch.add_line_segment2(format!("equal{index}.la"), a0, a1);
        let line_b = sketch.add_line_segment2(format!("equal{index}.lb"), b0, b1);
        hypersolve::sketch_distance_builders::equal_length_lines2(
            &mut sketch,
            format!("equal length {index}"),
            line_a,
            line_b,
        );
        let r0 = sketch.add_distance(format!("equal{index}.r0"), r(5));
        let r1 = sketch.add_distance(format!("equal{index}.r1"), r(5));
        let c0 = sketch.add_circle2(format!("equal{index}.c0"), a0, r0);
        let c1 = sketch.add_circle2(format!("equal{index}.c1"), b0, r1);
        hypersolve::sketch_distance_builders::equal_radius2(
            &mut sketch,
            format!("equal radius {index}"),
            c0,
            c1,
        );
    }
    sketch
}

fn sketch_problem_with_concentric_relations(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let y = index as i64;
        let center = sketch.add_point2d(format!("concentric{index}.center"), r(2), r(y));
        let start = sketch.add_point2d(format!("concentric{index}.start"), r(7), r(y));
        let end = sketch.add_point2d(format!("concentric{index}.end"), r(2), r(y + 5));
        let radius = sketch.add_distance(format!("concentric{index}.radius"), r(5));
        let circle = sketch.add_circle2(format!("concentric{index}.circle"), center, radius);
        let arc =
            sketch.add_arc_of_circle2(format!("concentric{index}.arc"), center, start, end, radius);
        hypersolve::sketch_distance_builders::concentric2(
            &mut sketch,
            format!("concentric {index}"),
            circle,
            arc,
        );
    }
    sketch
}

fn sketch_problem_with_projected_point_concentric_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("projected.concentric.origin", r(0), r(0), r(0));
    let normal = sketch.add_normal3d("projected.concentric.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("projected.concentric.workplane", origin, normal);
    for index in 0..row_count {
        let y = index as i64;
        let center = sketch.add_point2d(format!("projected.concentric{index}.center"), r(2), r(y));
        let start = sketch.add_point2d(format!("projected.concentric{index}.start"), r(7), r(y));
        let end = sketch.add_point2d(format!("projected.concentric{index}.end"), r(2), r(y + 5));
        let point = sketch.add_point3d(
            format!("projected.concentric{index}.point"),
            r(2),
            r(y),
            r(index as i64 + 11),
        );
        let radius = sketch.add_distance(format!("projected.concentric{index}.radius"), r(5));
        let arc = sketch.add_arc_of_circle2(
            format!("projected.concentric{index}.arc"),
            center,
            start,
            end,
            radius,
        );
        hypersolve::sketch_distance_builders::projected_point_concentric3(
            &mut sketch,
            format!("projected point concentric {index}"),
            workplane,
            point,
            arc,
        );
    }
    sketch
}

fn sketch_problem_with_length_ratio_and_point_line(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let y = index as i64;
        let long0 = sketch.add_point2d(format!("ratio{index}.long0"), r(0), r(y));
        let long1 = sketch.add_point2d(format!("ratio{index}.long1"), r(6), r(y + 8));
        let short0 = sketch.add_point2d(format!("ratio{index}.short0"), r(12), r(y));
        let short1 = sketch.add_point2d(format!("ratio{index}.short1"), r(15), r(y + 4));
        let long = sketch.add_line_segment2(format!("ratio{index}.long"), long0, long1);
        let short = sketch.add_line_segment2(format!("ratio{index}.short"), short0, short1);
        hypersolve::sketch_distance_builders::length_ratio_lines2(
            &mut sketch,
            format!("length ratio {index}"),
            long,
            short,
            r(2),
            r(1),
        );
        let p = sketch.add_point2d(format!("ptline{index}.p"), r(index as i64), r(y + 3));
        let line = sketch.add_line_segment2(format!("ptline{index}.line"), long0, short0);
        let distance = sketch.add_distance(format!("ptline{index}.d"), r(3));
        hypersolve::sketch_distance_builders::point_line_distance2(
            &mut sketch,
            format!("point line distance {index}"),
            p,
            line,
            distance,
        );
    }
    sketch
}

fn sketch_problem_with_length_differences(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let y = index as i64;
        let start = sketch.add_point2d(format!("diff{index}.start"), r(0), r(y));
        let shorter_end = sketch.add_point2d(format!("diff{index}.short"), r(4), r(y));
        let longer_end = sketch.add_point2d(format!("diff{index}.long"), r(7), r(y));
        let shorter = sketch.add_line_segment2(format!("diff{index}.shorter"), start, shorter_end);
        let longer = sketch.add_line_segment2(format!("diff{index}.longer"), start, longer_end);
        let difference = sketch.add_distance(format!("diff{index}.d"), r(3));
        hypersolve::sketch_distance_builders::length_difference_lines2(
            &mut sketch,
            format!("length difference {index}"),
            longer,
            shorter,
            difference,
        );
    }
    sketch
}

fn sketch_problem_with_line_arc_lengths(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let y = index as i64;
        let radius = (index % 5 + 1) as i64;
        let center = sketch.add_point2d(format!("linarc{index}.center"), r(0), r(y));
        let start = sketch.add_point2d(format!("linarc{index}.start"), r(radius), r(y));
        let end = sketch.add_point2d(format!("linarc{index}.end"), r(-radius), r(y));
        let radius_entity = sketch.add_distance(format!("linarc{index}.radius"), r(radius));
        let arc = sketch.add_arc_of_circle2(
            format!("linarc{index}.arc"),
            center,
            start,
            end,
            radius_entity,
        );
        let line_start = sketch.add_point2d(format!("linarc{index}.line_start"), r(0), r(y + 10));
        let line_end = sketch.add_point2d(
            format!("linarc{index}.line_end"),
            r(radius) * Real::pi(),
            r(y + 10),
        );
        let line = sketch.add_line_segment2(format!("linarc{index}.line"), line_start, line_end);
        hypersolve::sketch_distance_builders::equal_line_arc_length2(
            &mut sketch,
            format!("line arc length {index}"),
            line,
            arc,
        );
    }
    sketch
}

fn sketch_problem_with_line_arc_sweep_lengths(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let three_halves = Real::new(Rational::fraction(3, 2).unwrap());
    for index in 0..row_count {
        let y = index as i64;
        let radius = (index % 5 + 1) as i64;
        let center = sketch.add_point2d(format!("sweeplen{index}.center"), r(0), r(y));
        let start = sketch.add_point2d(format!("sweeplen{index}.start"), r(radius), r(y));
        let end = sketch.add_point2d(format!("sweeplen{index}.end"), r(0), r(y + radius));
        let radius_entity = sketch.add_distance(format!("sweeplen{index}.radius"), r(radius));
        let arc = sketch.add_arc_of_circle2(
            format!("sweeplen{index}.arc"),
            center,
            start,
            end,
            radius_entity,
        );
        let line_start = sketch.add_point2d(format!("sweeplen{index}.line_start"), r(0), r(y + 10));
        let line_end = sketch.add_point2d(
            format!("sweeplen{index}.line_end"),
            r(radius) * Real::pi() * three_halves.clone(),
            r(y + 10),
        );
        let line = sketch.add_line_segment2(format!("sweeplen{index}.line"), line_start, line_end);
        hypersolve::sketch_distance_builders::equal_line_arc_sweep_length2(
            &mut sketch,
            format!("line arc sweep length {index}"),
            line,
            arc,
            hypersolve::SketchArcLengthSweep::ClockwiseMajor,
        );
    }
    sketch
}

fn sketch_problem_with_projected_line_arc_sweep_lengths(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let three_halves = Real::new(Rational::fraction(3, 2).unwrap());
    let origin3 = sketch.add_point3d("projected_sweep.origin", r(0), r(0), r(0));
    let normal = sketch.add_normal3d("projected_sweep.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("projected_sweep.workplane", origin3, normal);
    for index in 0..row_count {
        let y = index as i64;
        let radius = (index % 5 + 1) as i64;
        let center = sketch.add_point2d(format!("psweeplen{index}.center"), r(0), r(y));
        let start = sketch.add_point2d(format!("psweeplen{index}.start"), r(radius), r(y));
        let end = sketch.add_point2d(format!("psweeplen{index}.end"), r(0), r(y + radius));
        let radius_entity = sketch.add_distance(format!("psweeplen{index}.radius"), r(radius));
        let arc = sketch.add_arc_of_circle2(
            format!("psweeplen{index}.arc"),
            center,
            start,
            end,
            radius_entity,
        );
        let line_start = sketch.add_point3d(
            format!("psweeplen{index}.line_start"),
            r(0),
            r(y + 10),
            r(5),
        );
        let line_end = sketch.add_point3d(
            format!("psweeplen{index}.line_end"),
            r(radius) * Real::pi() * three_halves.clone(),
            r(y + 10),
            r(9),
        );
        let line = sketch.add_line_segment3(format!("psweeplen{index}.line"), line_start, line_end);
        hypersolve::sketch_distance_builders::projected_equal_line_arc_sweep_length3(
            &mut sketch,
            format!("projected line arc sweep length {index}"),
            workplane,
            line,
            arc,
            hypersolve::SketchArcLengthSweep::ClockwiseMajor,
        );
    }
    sketch
}

fn sketch_problem_with_equal_point_line_distances(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let y = index as i64;
        let length_start = sketch.add_point2d(format!("eqpl{index}.ls"), r(0), r(y));
        let length_end = sketch.add_point2d(format!("eqpl{index}.le"), r(3), r(y));
        let line0 = sketch.add_point2d(format!("eqpl{index}.l0"), r(0), r(y + 5));
        let line1 = sketch.add_point2d(format!("eqpl{index}.l1"), r(5), r(y + 5));
        let point = sketch.add_point2d(format!("eqpl{index}.p"), r(2), r(y + 8));
        let length_line =
            sketch.add_line_segment2(format!("eqpl{index}.ll"), length_start, length_end);
        let distance_line = sketch.add_line_segment2(format!("eqpl{index}.dl"), line0, line1);
        hypersolve::sketch_distance_builders::equal_length_point_line_distance2(
            &mut sketch,
            format!("length point-line {index}"),
            length_line,
            point,
            distance_line,
        );

        let other0 = sketch.add_point2d(format!("eqpl{index}.o0"), r(9), r(y + 1));
        let other1 = sketch.add_point2d(format!("eqpl{index}.o1"), r(14), r(y + 1));
        let other_point = sketch.add_point2d(format!("eqpl{index}.op"), r(10), r(y + 4));
        let other_line = sketch.add_line_segment2(format!("eqpl{index}.ol"), other0, other1);
        hypersolve::sketch_distance_builders::equal_point_line_distances2(
            &mut sketch,
            format!("equal point-line distances {index}"),
            point,
            distance_line,
            other_point,
            other_line,
        );
    }
    sketch
}

fn sketch_problem_with_line_orientation_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let y = index as i64;
        let a0 = sketch.add_point2d(format!("orient{index}.a0"), r(0), r(y));
        let a1 = sketch.add_point2d(format!("orient{index}.a1"), r(3), r(y));
        let b0 = sketch.add_point2d(format!("orient{index}.b0"), r(1), r(y + 2));
        let b1 = sketch.add_point2d(format!("orient{index}.b1"), r(5), r(y + 2));
        let c0 = sketch.add_point2d(format!("orient{index}.c0"), r(2), r(y));
        let c1 = sketch.add_point2d(format!("orient{index}.c1"), r(2), r(y + 4));
        let horizontal = sketch.add_line_segment2(format!("orient{index}.h"), a0, a1);
        let parallel = sketch.add_line_segment2(format!("orient{index}.p"), b0, b1);
        let vertical = sketch.add_line_segment2(format!("orient{index}.v"), c0, c1);
        hypersolve::sketch_orientation_builders::parallel_lines2(
            &mut sketch,
            format!("parallel {index}"),
            horizontal,
            parallel,
        );
        hypersolve::sketch_orientation_builders::perpendicular_lines2(
            &mut sketch,
            format!("perpendicular {index}"),
            horizontal,
            vertical,
        );
    }
    sketch
}

fn sketch_problem_with_equal_angle_relations(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let y = index as i64;
        let a0 = sketch.add_point2d(format!("angle{index}.a0"), r(0), r(y));
        let a1 = sketch.add_point2d(format!("angle{index}.a1"), r(3), r(y));
        let b0 = sketch.add_point2d(format!("angle{index}.b0"), r(0), r(y));
        let b1 = sketch.add_point2d(format!("angle{index}.b1"), r(3), r(y + 4));
        let c0 = sketch.add_point2d(format!("angle{index}.c0"), r(10), r(y + 1));
        let c1 = sketch.add_point2d(format!("angle{index}.c1"), r(16), r(y + 1));
        let d0 = sketch.add_point2d(format!("angle{index}.d0"), r(10), r(y + 1));
        let d1 = sketch.add_point2d(format!("angle{index}.d1"), r(16), r(y + 9));
        let a = sketch.add_line_segment2(format!("angle{index}.a"), a0, a1);
        let b = sketch.add_line_segment2(format!("angle{index}.b"), b0, b1);
        let c = sketch.add_line_segment2(format!("angle{index}.c"), c0, c1);
        let d = sketch.add_line_segment2(format!("angle{index}.d"), d0, d1);
        hypersolve::sketch_angle_builders::equal_angle_lines2(
            &mut sketch,
            format!("equal angle {index}"),
            a,
            b,
            c,
            d,
        );
    }
    sketch
}

fn sketch_problem_with_oriented_angle_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let y = index as i64;
        let a0 = sketch.add_point2d(format!("oangle{index}.a0"), r(0), r(y));
        let a1 = sketch.add_point2d(format!("oangle{index}.a1"), r(3), r(y));
        let b0 = sketch.add_point2d(format!("oangle{index}.b0"), r(0), r(y));
        let b1 = sketch.add_point2d(format!("oangle{index}.b1"), r(3), r(y + 4));
        let c0 = sketch.add_point2d(format!("oangle{index}.c0"), r(10), r(y + 1));
        let c1 = sketch.add_point2d(format!("oangle{index}.c1"), r(16), r(y + 1));
        let d0 = sketch.add_point2d(format!("oangle{index}.d0"), r(10), r(y + 1));
        let d1 = sketch.add_point2d(format!("oangle{index}.d1"), r(16), r(y + 9));
        let a = sketch.add_line_segment2(format!("oangle{index}.a"), a0, a1);
        let b = sketch.add_line_segment2(format!("oangle{index}.b"), b0, b1);
        let c = sketch.add_line_segment2(format!("oangle{index}.c"), c0, c1);
        let d = sketch.add_line_segment2(format!("oangle{index}.d"), d0, d1);
        hypersolve::sketch_angle_builders::equal_oriented_angle_lines2(
            &mut sketch,
            format!("oriented angle {index}"),
            a,
            b,
            c,
            d,
        );
    }
    sketch
}

fn sketch_problem_with_projected_oriented_angle_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("projected-oangle.origin", r(0), r(0), r(0));
    let normal = sketch.add_normal3d("projected-oangle.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("projected-oangle.workplane", origin, normal);
    for index in 0..row_count {
        let y = index as i64;
        let base = sketch.add_point3d(format!("projected-oangle{index}.base"), r(0), r(y), r(0));
        let a_end = sketch.add_point3d(
            format!("projected-oangle{index}.a_end"),
            r(3),
            r(y),
            r(7 + y),
        );
        let b_end = sketch.add_point3d(
            format!("projected-oangle{index}.b_end"),
            r(0),
            r(y + 4),
            r(-9 - y),
        );
        let c_end = sketch.add_point3d(
            format!("projected-oangle{index}.c_end"),
            r(6),
            r(y),
            r(11 + y),
        );
        let d_end = sketch.add_point3d(
            format!("projected-oangle{index}.d_end"),
            r(0),
            r(y + 8),
            r(-3 - y),
        );
        let a = sketch.add_line_segment3(format!("projected-oangle{index}.a"), base, a_end);
        let b = sketch.add_line_segment3(format!("projected-oangle{index}.b"), base, b_end);
        let c = sketch.add_line_segment3(format!("projected-oangle{index}.c"), base, c_end);
        let d = sketch.add_line_segment3(format!("projected-oangle{index}.d"), base, d_end);
        hypersolve::sketch_angle_builders::projected_equal_oriented_angle_lines3(
            &mut sketch,
            format!("projected oriented angle {index}"),
            workplane,
            a,
            b,
            c,
            d,
        );
    }
    sketch
}

fn sketch_problem_with_projected_line_orientation_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("projected-orient.origin", r(0), r(0), r(0));
    let normal = sketch.add_normal3d("projected-orient.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("projected-orient.workplane", origin, normal);
    for index in 0..row_count {
        let y = index as i64;
        let base = sketch.add_point3d(format!("projected-orient{index}.base"), r(0), r(y), r(0));
        let x_end = sketch.add_point3d(format!("projected-orient{index}.x"), r(5), r(y), r(y));
        let x2_end =
            sketch.add_point3d(format!("projected-orient{index}.x2"), r(10), r(y), r(9 - y));
        let y_end = sketch.add_point3d(
            format!("projected-orient{index}.y"),
            r(0),
            r(y + 7),
            r(11 + y),
        );
        let x = sketch.add_line_segment3(format!("projected-orient{index}.lx"), base, x_end);
        let x2 = sketch.add_line_segment3(format!("projected-orient{index}.lx2"), base, x2_end);
        let y_line = sketch.add_line_segment3(format!("projected-orient{index}.ly"), base, y_end);
        hypersolve::sketch_orientation_builders::projected_parallel_lines3(
            &mut sketch,
            format!("projected parallel {index}"),
            workplane,
            x,
            x2,
        );
        hypersolve::sketch_orientation_builders::projected_perpendicular_lines3(
            &mut sketch,
            format!("projected perpendicular {index}"),
            workplane,
            x,
            y_line,
        );
        hypersolve::sketch_orientation_builders::projected_same_direction_lines3(
            &mut sketch,
            format!("projected same direction {index}"),
            workplane,
            x,
            x2,
        );
    }
    sketch
}

fn sketch_problem_with_midpoint_relations(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let x = index as i64;
        let midpoint = sketch.add_point2d(format!("mid{index}.m"), r(x), r(x + 1));
        let a = sketch.add_point2d(format!("mid{index}.a"), r(x - 2), r(x - 1));
        let b = sketch.add_point2d(format!("mid{index}.b"), r(x + 2), r(x + 3));
        hypersolve::sketch_symmetry_builders::at_midpoint2(
            &mut sketch,
            format!("midpoint {index}"),
            midpoint,
            a,
            b,
        );
    }
    sketch
}

fn sketch_problem_with_axis_symmetry_relations(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let axis = index as i64;
        let top = sketch.add_point2d(format!("axis{index}.top"), r(axis + 2), r(axis + 5));
        let bottom = sketch.add_point2d(format!("axis{index}.bottom"), r(axis + 2), r(axis - 1));
        let left = sketch.add_point2d(format!("axis{index}.left"), r(axis - 4), r(axis + 3));
        let right = sketch.add_point2d(format!("axis{index}.right"), r(axis + 6), r(axis + 3));
        hypersolve::sketch_symmetry_builders::symmetric_horizontal2(
            &mut sketch,
            format!("horizontal symmetry {index}"),
            top,
            bottom,
            r(axis + 2),
        );
        hypersolve::sketch_symmetry_builders::symmetric_vertical2(
            &mut sketch,
            format!("vertical symmetry {index}"),
            left,
            right,
            r(axis + 1),
        );
    }
    sketch
}

fn sketch_problem_with_line_symmetry_relations(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let x = index as i64;
        let axis_start = sketch.add_point2d(format!("line_sym{index}.axis0"), r(x), r(x + 1));
        let axis_end = sketch.add_point2d(format!("line_sym{index}.axis1"), r(x + 3), r(x + 5));
        let axis = sketch.add_line_segment2(format!("line_sym{index}.axis"), axis_start, axis_end);
        let a = sketch.add_point2d(format!("line_sym{index}.a"), r(x - 1), r(x + 5));
        let b = sketch.add_point2d(format!("line_sym{index}.b"), r(x + 7), r(x - 1));
        hypersolve::sketch_symmetry_builders::symmetric_line2(
            &mut sketch,
            format!("line symmetry {index}"),
            a,
            b,
            axis,
        );
    }
    sketch
}

fn sketch_problem_with_projected_line_symmetry_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-line-sym.origin", r(0), r(0), r(0));
    let normal = sketch.add_normal3d("project-line-sym.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-line-sym.workplane", origin, normal);
    for index in 0..row_count {
        let x = index as i64;
        let axis_start = sketch.add_point3d(
            format!("project-line-sym{index}.axis0"),
            r(x),
            r(x + 1),
            r(-x),
        );
        let axis_end = sketch.add_point3d(
            format!("project-line-sym{index}.axis1"),
            r(x + 3),
            r(x + 5),
            r(2 * x + 1),
        );
        let axis = sketch.add_line_segment3(
            format!("project-line-sym{index}.axis"),
            axis_start,
            axis_end,
        );
        let a = sketch.add_point3d(
            format!("project-line-sym{index}.a"),
            r(x - 1),
            r(x + 5),
            r(7),
        );
        let b = sketch.add_point3d(
            format!("project-line-sym{index}.b"),
            r(x + 7),
            r(x - 1),
            r(-11),
        );
        hypersolve::sketch_symmetry_builders::projected_symmetric_line3(
            &mut sketch,
            format!("projected line symmetry {index}"),
            workplane,
            a,
            b,
            axis,
        );
    }
    sketch
}

fn sketch_problem_with_same_direction_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let y = index as i64;
        let a0 = sketch.add_point2d(format!("same{index}.a0"), r(0), r(y));
        let a1 = sketch.add_point2d(format!("same{index}.a1"), r(3), r(y + 1));
        let b0 = sketch.add_point2d(format!("same{index}.b0"), r(2), r(y + 3));
        let b1 = sketch.add_point2d(format!("same{index}.b1"), r(8), r(y + 5));
        let a = sketch.add_line_segment2(format!("same{index}.a"), a0, a1);
        let b = sketch.add_line_segment2(format!("same{index}.b"), b0, b1);
        hypersolve::sketch_orientation_builders::same_direction_lines2(
            &mut sketch,
            format!("same direction {index}"),
            a,
            b,
        );
    }
    sketch
}

fn sketch_problem_with_tangent_same_direction_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let y = index as i64;
        let candidate0 = sketch.add_point2d(format!("tan{index}.c0"), r(0), r(y));
        let candidate1 = sketch.add_point2d(format!("tan{index}.c1"), r(3), r(y + 4));
        let target0 = sketch.add_point2d(format!("tan{index}.t0"), r(2), r(y + 3));
        let target1 = sketch.add_point2d(format!("tan{index}.t1"), r(8), r(y + 11));
        let candidate =
            sketch.add_line_segment2(format!("tan{index}.candidate"), candidate0, candidate1);
        let target = sketch.add_line_segment2(format!("tan{index}.target"), target0, target1);
        hypersolve::sketch_tangency_builders::tangent_same_direction_lines2(
            &mut sketch,
            format!("same tangent {index}"),
            candidate,
            target,
        );
    }
    sketch
}

fn sketch_problem_with_arc_line_tangent_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let x = index as i64;
        let center = sketch.add_point2d(format!("arctan{index}.center"), r(x), r(0));
        let start = sketch.add_point2d(format!("arctan{index}.start"), r(x + 5), r(0));
        let end = sketch.add_point2d(format!("arctan{index}.end"), r(x), r(5));
        let radius = sketch.add_distance(format!("arctan{index}.radius"), r(5));
        let arc =
            sketch.add_arc_of_circle2(format!("arctan{index}.arc"), center, start, end, radius);
        let tangent_end = sketch.add_point2d(format!("arctan{index}.tangent_end"), r(x + 5), r(3));
        let tangent = sketch.add_line_segment2(format!("arctan{index}.line"), start, tangent_end);
        hypersolve::sketch_tangency_builders::arc_line_tangent2(
            &mut sketch,
            format!("arc line tangent {index}"),
            arc,
            hypersolve::SketchArcEndpoint::Start,
            tangent,
            hypersolve::SketchLineEndpoint::Start,
            hypersolve::SketchTangentOrientation::CounterClockwise,
        );
    }
    sketch
}

fn sketch_problem_with_projected_arc_line_tangent_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-arctan.origin", r(0), r(0), r(0));
    let normal = sketch.add_normal3d("project-arctan.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-arctan.workplane", origin, normal);
    for index in 0..row_count {
        let x = index as i64;
        let center = sketch.add_point2d(format!("project-arctan{index}.center"), r(x), r(0));
        let start = sketch.add_point2d(format!("project-arctan{index}.start"), r(x + 5), r(0));
        let end = sketch.add_point2d(format!("project-arctan{index}.end"), r(x), r(5));
        let radius = sketch.add_distance(format!("project-arctan{index}.radius"), r(5));
        let arc = sketch.add_arc_of_circle2(
            format!("project-arctan{index}.arc"),
            center,
            start,
            end,
            radius,
        );
        let line_start = sketch.add_point3d(
            format!("project-arctan{index}.line_start"),
            r(x + 5),
            r(0),
            r(-x),
        );
        let line_end = sketch.add_point3d(
            format!("project-arctan{index}.line_end"),
            r(x + 5),
            r(3),
            r(20 - x),
        );
        let tangent =
            sketch.add_line_segment3(format!("project-arctan{index}.line"), line_start, line_end);
        hypersolve::sketch_tangency_builders::projected_arc_line_tangent3(
            &mut sketch,
            format!("projected arc line tangent {index}"),
            workplane,
            arc,
            hypersolve::SketchArcEndpoint::Start,
            tangent,
            hypersolve::SketchLineEndpoint::Start,
            hypersolve::SketchTangentOrientation::CounterClockwise,
        );
    }
    sketch
}

fn sketch_problem_with_projected_arc_cubic_curve_tangent_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-arccubic.origin", r(0), r(0), r(0));
    let normal = sketch.add_normal3d("project-arccubic.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-arccubic.workplane", origin, normal);
    for index in 0..row_count {
        let x = index as i64;
        let center = sketch.add_point2d(format!("project-arccubic{index}.center"), r(x), r(0));
        let start = sketch.add_point2d(format!("project-arccubic{index}.start"), r(x + 5), r(0));
        let end = sketch.add_point2d(format!("project-arccubic{index}.end"), r(x), r(5));
        let radius = sketch.add_distance(format!("project-arccubic{index}.radius"), r(5));
        let arc = sketch.add_arc_of_circle2(
            format!("project-arccubic{index}.arc"),
            center,
            start,
            end,
            radius,
        );
        let p0 = sketch.add_point3d(format!("project-arccubic{index}.p0"), r(x + 5), r(0), r(3));
        let p1 = sketch.add_point3d(format!("project-arccubic{index}.p1"), r(x + 5), r(1), r(5));
        let p2 = sketch.add_point3d(format!("project-arccubic{index}.p2"), r(x + 5), r(2), r(7));
        let p3 = sketch.add_point3d(format!("project-arccubic{index}.p3"), r(x + 5), r(3), r(11));
        let cubic = sketch.add_cubic3(format!("project-arccubic{index}.cubic"), p0, p1, p2, p3);
        let parameter = sketch.add_parameter(format!("project-arccubic{index}.t"), r(0));
        hypersolve::sketch_tangency_builders::projected_arc_cubic_curve_tangent3(
            &mut sketch,
            format!("projected arc cubic tangent {index}"),
            workplane,
            arc,
            hypersolve::SketchArcEndpoint::Start,
            cubic,
            parameter,
            hypersolve::SketchTangentOrientation::CounterClockwise,
        );
    }
    sketch
}

fn sketch_problem_with_projected_arc_cubic_curve_second_order_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-arcg2.origin", r(0), r(0), r(0));
    let normal = sketch.add_normal3d("project-arcg2.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-arcg2.workplane", origin, normal);
    for index in 0..row_count {
        let x = index as i64;
        let center = sketch.add_point2d(format!("project-arcg2{index}.center"), r(x), r(0));
        let start = sketch.add_point2d(format!("project-arcg2{index}.start"), r(x + 5), r(0));
        let end = sketch.add_point2d(format!("project-arcg2{index}.end"), r(x), r(5));
        let radius = sketch.add_distance(format!("project-arcg2{index}.radius"), r(5));
        let arc = sketch.add_arc_of_circle2(
            format!("project-arcg2{index}.arc"),
            center,
            start,
            end,
            radius,
        );
        let p0 = sketch.add_point3d(format!("project-arcg2{index}.p0"), r(x + 5), r(0), r(3));
        let p1 = sketch.add_point3d(format!("project-arcg2{index}.p1"), r(x + 5), r(50), r(5));
        let p2 = sketch.add_point3d(format!("project-arcg2{index}.p2"), r(x - 745), r(100), r(7));
        let p3 = sketch.add_point3d(
            format!("project-arcg2{index}.p3"),
            r(x - 1495),
            r(150),
            r(11),
        );
        let cubic = sketch.add_cubic3(format!("project-arcg2{index}.cubic"), p0, p1, p2, p3);
        let parameter = sketch.add_parameter(format!("project-arcg2{index}.t"), r(0));
        hypersolve::sketch_tangency_builders::projected_arc_cubic_curve_second_order_contact3(
            &mut sketch,
            format!("projected arc cubic second order contact {index}"),
            workplane,
            arc,
            hypersolve::SketchArcEndpoint::Start,
            cubic,
            parameter,
            hypersolve::SketchTangentOrientation::CounterClockwise,
        );
    }
    sketch
}

fn sketch_problem_with_arc_arc_tangent_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let x = index as i64 * 12;
        let first_center = sketch.add_point2d(format!("arcarc{index}.first_center"), r(x), r(0));
        let shared = sketch.add_point2d(format!("arcarc{index}.shared"), r(x + 5), r(0));
        let first_end = sketch.add_point2d(format!("arcarc{index}.first_end"), r(x), r(5));
        let first_radius = sketch.add_distance(format!("arcarc{index}.first_radius"), r(5));
        let first = sketch.add_arc_of_circle2(
            format!("arcarc{index}.first"),
            first_center,
            shared,
            first_end,
            first_radius,
        );
        let second_center =
            sketch.add_point2d(format!("arcarc{index}.second_center"), r(x + 10), r(0));
        let second_end = sketch.add_point2d(format!("arcarc{index}.second_end"), r(x + 10), r(5));
        let second_radius = sketch.add_distance(format!("arcarc{index}.second_radius"), r(5));
        let second = sketch.add_arc_of_circle2(
            format!("arcarc{index}.second"),
            second_center,
            shared,
            second_end,
            second_radius,
        );
        hypersolve::sketch_tangency_builders::arc_arc_tangent2(
            &mut sketch,
            format!("arc arc tangent {index}"),
            first,
            hypersolve::SketchArcEndpoint::Start,
            second,
            hypersolve::SketchArcEndpoint::Start,
            hypersolve::SketchArcTangencyBranch::OppositeRadiusDirection,
        );
    }
    sketch
}

fn sketch_problem_with_circle_circle_tangent_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let x = index as i64 * 12;
        let first_center = sketch.add_point2d(format!("circlecircle{index}.first"), r(x), r(0));
        let second_center =
            sketch.add_point2d(format!("circlecircle{index}.second"), r(x + 8), r(0));
        let first_radius = sketch.add_distance(format!("circlecircle{index}.first_radius"), r(5));
        let second_radius = sketch.add_distance(format!("circlecircle{index}.second_radius"), r(3));
        let first = sketch.add_circle2(
            format!("circlecircle{index}.first_circle"),
            first_center,
            first_radius,
        );
        let second = sketch.add_circle2(
            format!("circlecircle{index}.second_circle"),
            second_center,
            second_radius,
        );
        hypersolve::sketch_tangency_builders::circle_circle_tangent2(
            &mut sketch,
            format!("circle circle tangent {index}"),
            first,
            second,
            hypersolve::SketchCircleTangencyBranch::External,
        );
    }
    sketch
}

fn sketch_problem_with_arc_cubic_tangent_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let x = index as i64;
        let center = sketch.add_point2d(format!("arccubic{index}.center"), r(x), r(0));
        let start = sketch.add_point2d(format!("arccubic{index}.start"), r(x + 5), r(0));
        let end = sketch.add_point2d(format!("arccubic{index}.end"), r(x), r(5));
        let radius = sketch.add_distance(format!("arccubic{index}.radius"), r(5));
        let arc =
            sketch.add_arc_of_circle2(format!("arccubic{index}.arc"), center, start, end, radius);
        let p1 = sketch.add_point2d(format!("arccubic{index}.p1"), r(x + 5), r(1));
        let p2 = sketch.add_point2d(format!("arccubic{index}.p2"), r(x + 5), r(2));
        let p3 = sketch.add_point2d(format!("arccubic{index}.p3"), r(x + 5), r(3));
        let cubic = sketch.add_cubic2(format!("arccubic{index}.cubic"), start, p1, p2, p3);
        let parameter = sketch.add_parameter(format!("arccubic{index}.t"), r(0));
        hypersolve::sketch_tangency_builders::arc_cubic_tangent2(
            &mut sketch,
            format!("arc cubic tangent {index}"),
            arc,
            hypersolve::SketchArcEndpoint::Start,
            cubic,
            parameter,
            hypersolve::SketchTangentOrientation::CounterClockwise,
        );
    }
    sketch
}

fn sketch_problem_with_arc_cubic_second_order_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let x = index as i64;
        let center = sketch.add_point2d(format!("arcsecond{index}.center"), r(x), r(0));
        let start = sketch.add_point2d(format!("arcsecond{index}.start"), r(x + 5), r(0));
        let end = sketch.add_point2d(format!("arcsecond{index}.end"), r(x), r(5));
        let radius = sketch.add_distance(format!("arcsecond{index}.radius"), r(5));
        let arc =
            sketch.add_arc_of_circle2(format!("arcsecond{index}.arc"), center, start, end, radius);
        let p1 = sketch.add_point2d(format!("arcsecond{index}.p1"), r(x + 5), r(10));
        let p2 = sketch.add_point2d(format!("arcsecond{index}.p2"), r(x - 25), r(20));
        let p3 = sketch.add_point2d(format!("arcsecond{index}.p3"), r(x - 55), r(30));
        let cubic = sketch.add_cubic2(format!("arcsecond{index}.cubic"), start, p1, p2, p3);
        let parameter = sketch.add_parameter(format!("arcsecond{index}.t"), r(0));
        hypersolve::sketch_tangency_builders::arc_cubic_second_order_contact2(
            &mut sketch,
            format!("arc cubic second order {index}"),
            arc,
            hypersolve::SketchArcEndpoint::Start,
            cubic,
            parameter,
            hypersolve::SketchTangentOrientation::CounterClockwise,
        );
    }
    sketch
}

fn sketch_problem_with_point_on_cubic_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let x = index as i64;
        let p0 = sketch.add_point2d(format!("cubic{index}.p0"), r(x), r(x + 1));
        let p1 = sketch.add_point2d(format!("cubic{index}.p1"), r(x + 1), r(x + 2));
        let p2 = sketch.add_point2d(format!("cubic{index}.p2"), r(x + 2), r(x + 3));
        let p3 = sketch.add_point2d(format!("cubic{index}.p3"), r(x + 3), r(x + 4));
        let cubic = sketch.add_cubic2(format!("cubic{index}"), p0, p1, p2, p3);
        let parameter = sketch.add_parameter(format!("cubic{index}.t"), r(1));
        let point = sketch.add_point2d(format!("cubic{index}.point"), r(x + 3), r(x + 4));
        hypersolve::sketch_incidence_builders::point_on_cubic2(
            &mut sketch,
            format!("point on cubic {index}"),
            point,
            cubic,
            parameter,
        );
    }
    sketch
}

fn sketch_problem_with_projected_point_on_cubic_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-cubic.origin", r(1), r(2), r(3));
    let normal = sketch.add_normal3d("project-cubic.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-cubic.workplane", origin, normal);
    for index in 0..row_count {
        let x = index as i64;
        let p0 = sketch.add_point2d(format!("project-cubic{index}.p0"), r(x), r(x + 1));
        let p1 = sketch.add_point2d(format!("project-cubic{index}.p1"), r(x + 1), r(x + 2));
        let p2 = sketch.add_point2d(format!("project-cubic{index}.p2"), r(x + 2), r(x + 3));
        let p3 = sketch.add_point2d(format!("project-cubic{index}.p3"), r(x + 3), r(x + 4));
        let cubic = sketch.add_cubic2(format!("project-cubic{index}"), p0, p1, p2, p3);
        let parameter = sketch.add_parameter(format!("project-cubic{index}.t"), r(1));
        let point = sketch.add_point3d(
            format!("project-cubic{index}.point"),
            r(x + 4),
            r(x + 6),
            r(99 - x),
        );
        hypersolve::sketch_incidence_builders::projected_point_on_cubic3(
            &mut sketch,
            format!("projected point on cubic {index}"),
            workplane,
            point,
            cubic,
            parameter,
        );
    }
    sketch
}

fn sketch_problem_with_projected_point_on_cubic_curve_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-cubic3.origin", r(1), r(2), r(3));
    let normal = sketch.add_normal3d("project-cubic3.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-cubic3.workplane", origin, normal);
    for index in 0..row_count {
        let x = index as i64;
        let p0 = sketch.add_point3d(
            format!("project-cubic3-{index}.p0"),
            r(x + 1),
            r(x + 3),
            r(4),
        );
        let p1 = sketch.add_point3d(
            format!("project-cubic3-{index}.p1"),
            r(x + 2),
            r(x + 4),
            r(7),
        );
        let p2 = sketch.add_point3d(
            format!("project-cubic3-{index}.p2"),
            r(x + 3),
            r(x + 5),
            r(11),
        );
        let p3 = sketch.add_point3d(
            format!("project-cubic3-{index}.p3"),
            r(x + 4),
            r(x + 6),
            r(13),
        );
        let cubic = sketch.add_cubic3(format!("project-cubic3-{index}"), p0, p1, p2, p3);
        let parameter = sketch.add_parameter(format!("project-cubic3-{index}.t"), r(1));
        let point = sketch.add_point3d(
            format!("project-cubic3-{index}.point"),
            r(x + 4),
            r(x + 6),
            r(99 - x),
        );
        hypersolve::sketch_incidence_builders::projected_point_on_cubic_curve3(
            &mut sketch,
            format!("projected point on 3D cubic {index}"),
            workplane,
            point,
            cubic,
            parameter,
        );
    }
    sketch
}

fn sketch_problem_with_cubic_line_tangent_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let x = index as i64;
        let p0 = sketch.add_point2d(format!("cubictan{index}.p0"), r(x), r(x));
        let p1 = sketch.add_point2d(format!("cubictan{index}.p1"), r(x + 1), r(x + 2));
        let p2 = sketch.add_point2d(format!("cubictan{index}.p2"), r(x + 2), r(x + 4));
        let p3 = sketch.add_point2d(format!("cubictan{index}.p3"), r(x + 3), r(x + 6));
        let cubic = sketch.add_cubic2(format!("cubictan{index}"), p0, p1, p2, p3);
        let parameter = sketch.add_parameter(format!("cubictan{index}.t"), r(1));
        let tangent_end =
            sketch.add_point2d(format!("cubictan{index}.tangent_end"), r(x + 4), r(x + 8));
        let line = sketch.add_line_segment2(format!("cubictan{index}.line"), p3, tangent_end);
        hypersolve::sketch_tangency_builders::cubic_line_tangent2(
            &mut sketch,
            format!("cubic line tangent {index}"),
            cubic,
            parameter,
            line,
            hypersolve::SketchLineEndpoint::Start,
        );
    }
    sketch
}

fn sketch_problem_with_projected_cubic_line_tangent_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-cubic-tan.origin", r(1), r(2), r(3));
    let normal = sketch.add_normal3d("project-cubic-tan.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-cubic-tan.workplane", origin, normal);
    for index in 0..row_count {
        let x = index as i64;
        let p0 = sketch.add_point2d(format!("project-cubictan{index}.p0"), r(x), r(x));
        let p1 = sketch.add_point2d(format!("project-cubictan{index}.p1"), r(x + 1), r(x + 2));
        let p2 = sketch.add_point2d(format!("project-cubictan{index}.p2"), r(x + 2), r(x + 4));
        let p3 = sketch.add_point2d(format!("project-cubictan{index}.p3"), r(x + 3), r(x + 6));
        let cubic = sketch.add_cubic2(format!("project-cubictan{index}"), p0, p1, p2, p3);
        let parameter = sketch.add_parameter(format!("project-cubictan{index}.t"), r(1));
        let endpoint = sketch.add_point3d(
            format!("project-cubictan{index}.endpoint"),
            r(x + 4),
            r(x + 8),
            r(30 - x),
        );
        let tangent_end = sketch.add_point3d(
            format!("project-cubictan{index}.tangent_end"),
            r(x + 5),
            r(x + 10),
            r(20 + x),
        );
        let line = sketch.add_line_segment3(
            format!("project-cubictan{index}.line"),
            endpoint,
            tangent_end,
        );
        hypersolve::sketch_tangency_builders::projected_cubic_line_tangent3(
            &mut sketch,
            format!("projected cubic line tangent {index}"),
            workplane,
            cubic,
            parameter,
            line,
            hypersolve::SketchLineEndpoint::Start,
        );
    }
    sketch
}

fn sketch_problem_with_projected_cubic_curve_line_tangent_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-cubic3-tan.origin", r(1), r(2), r(3));
    let normal = sketch.add_normal3d("project-cubic3-tan.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-cubic3-tan.workplane", origin, normal);
    for index in 0..row_count {
        let x = index as i64;
        let p0 = sketch.add_point3d(
            format!("project-cubic3tan{index}.p0"),
            r(x + 1),
            r(x + 2),
            r(3),
        );
        let p1 = sketch.add_point3d(
            format!("project-cubic3tan{index}.p1"),
            r(x + 2),
            r(x + 4),
            r(5),
        );
        let p2 = sketch.add_point3d(
            format!("project-cubic3tan{index}.p2"),
            r(x + 3),
            r(x + 6),
            r(7),
        );
        let p3 = sketch.add_point3d(
            format!("project-cubic3tan{index}.p3"),
            r(x + 4),
            r(x + 8),
            r(11),
        );
        let cubic = sketch.add_cubic3(format!("project-cubic3tan{index}"), p0, p1, p2, p3);
        let parameter = sketch.add_parameter(format!("project-cubic3tan{index}.t"), r(1));
        let endpoint = sketch.add_point3d(
            format!("project-cubic3tan{index}.endpoint"),
            r(x + 4),
            r(x + 8),
            r(30 - x),
        );
        let tangent_end = sketch.add_point3d(
            format!("project-cubic3tan{index}.tangent_end"),
            r(x + 5),
            r(x + 10),
            r(20 + x),
        );
        let line = sketch.add_line_segment3(
            format!("project-cubic3tan{index}.line"),
            endpoint,
            tangent_end,
        );
        hypersolve::sketch_tangency_builders::projected_cubic_curve_line_tangent3(
            &mut sketch,
            format!("projected 3D cubic line tangent {index}"),
            workplane,
            cubic,
            parameter,
            line,
            hypersolve::SketchLineEndpoint::Start,
        );
    }
    sketch
}

fn sketch_problem_with_projected_cubic_curve_cubic_curve_tangent_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-cubic3-cubic3-tan.origin", r(1), r(2), r(3));
    let normal = sketch.add_normal3d("project-cubic3-cubic3-tan.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-cubic3-cubic3-tan.workplane", origin, normal);
    for index in 0..row_count {
        let x = index as i64;
        let a0 = sketch.add_point3d(
            format!("project-cubic3pair{index}.a0"),
            r(x + 1),
            r(x + 2),
            r(3),
        );
        let a1 = sketch.add_point3d(
            format!("project-cubic3pair{index}.a1"),
            r(x + 2),
            r(x + 4),
            r(5),
        );
        let a2 = sketch.add_point3d(
            format!("project-cubic3pair{index}.a2"),
            r(x + 3),
            r(x + 6),
            r(7),
        );
        let join = sketch.add_point3d(
            format!("project-cubic3pair{index}.join"),
            r(x + 4),
            r(x + 8),
            r(11),
        );
        let b1 = sketch.add_point3d(
            format!("project-cubic3pair{index}.b1"),
            r(x + 6),
            r(x + 12),
            r(13),
        );
        let b2 = sketch.add_point3d(
            format!("project-cubic3pair{index}.b2"),
            r(x + 8),
            r(x + 16),
            r(17),
        );
        let b3 = sketch.add_point3d(
            format!("project-cubic3pair{index}.b3"),
            r(x + 10),
            r(x + 20),
            r(19),
        );
        let first = sketch.add_cubic3(format!("project-cubic3pair{index}.first"), a0, a1, a2, join);
        let second = sketch.add_cubic3(
            format!("project-cubic3pair{index}.second"),
            join,
            b1,
            b2,
            b3,
        );
        let first_parameter = sketch.add_parameter(format!("project-cubic3pair{index}.ta"), r(1));
        let second_parameter = sketch.add_parameter(format!("project-cubic3pair{index}.tb"), r(0));
        hypersolve::sketch_tangency_builders::projected_cubic_curve_cubic_curve_tangent3(
            &mut sketch,
            format!("projected 3D cubic cubic tangent {index}"),
            workplane,
            first,
            first_parameter,
            second,
            second_parameter,
        );
    }
    sketch
}

fn sketch_problem_with_projected_cubic_curve_cubic_curve_c2_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-cubic3-cubic3-c2.origin", r(1), r(2), r(3));
    let normal = sketch.add_normal3d("project-cubic3-cubic3-c2.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-cubic3-cubic3-c2.workplane", origin, normal);
    for index in 0..row_count {
        let x = index as i64;
        let a0 = sketch.add_point3d(
            format!("project-cubic3c2{index}.a0"),
            r(x + 1),
            r(x + 2),
            r(3),
        );
        let a1 = sketch.add_point3d(
            format!("project-cubic3c2{index}.a1"),
            r(x + 2),
            r(x + 4),
            r(5),
        );
        let a2 = sketch.add_point3d(
            format!("project-cubic3c2{index}.a2"),
            r(x + 3),
            r(x + 6),
            r(7),
        );
        let join = sketch.add_point3d(
            format!("project-cubic3c2{index}.join"),
            r(x + 4),
            r(x + 8),
            r(11),
        );
        let b1 = sketch.add_point3d(
            format!("project-cubic3c2{index}.b1"),
            r(x + 5),
            r(x + 10),
            r(13),
        );
        let b2 = sketch.add_point3d(
            format!("project-cubic3c2{index}.b2"),
            r(x + 6),
            r(x + 12),
            r(17),
        );
        let b3 = sketch.add_point3d(
            format!("project-cubic3c2{index}.b3"),
            r(x + 7),
            r(x + 14),
            r(19),
        );
        let first = sketch.add_cubic3(format!("project-cubic3c2{index}.first"), a0, a1, a2, join);
        let second = sketch.add_cubic3(format!("project-cubic3c2{index}.second"), join, b1, b2, b3);
        let first_parameter = sketch.add_parameter(format!("project-cubic3c2{index}.ta"), r(1));
        let second_parameter = sketch.add_parameter(format!("project-cubic3c2{index}.tb"), r(0));
        hypersolve::sketch_tangency_builders::projected_cubic_curve_cubic_curve_c2_continuity3(
            &mut sketch,
            format!("projected 3D cubic cubic c2 {index}"),
            workplane,
            first,
            first_parameter,
            second,
            second_parameter,
        );
    }
    sketch
}

fn sketch_problem_with_projected_cubic_curve_cubic_curve_g2_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-cubic3-cubic3-g2.origin", r(1), r(2), r(3));
    let normal = sketch.add_normal3d("project-cubic3-cubic3-g2.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-cubic3-cubic3-g2.workplane", origin, normal);
    for index in 0..row_count {
        let x = index as i64;
        let a0 = sketch.add_point3d(format!("project-cubic3g2{index}.a0"), r(x), r(x), r(3));
        let a1 = sketch.add_point3d(format!("project-cubic3g2{index}.a1"), r(x + 1), r(x), r(5));
        let a2 = sketch.add_point3d(
            format!("project-cubic3g2{index}.a2"),
            r(x + 2),
            r(x + 1),
            r(7),
        );
        let join = sketch.add_point3d(
            format!("project-cubic3g2{index}.join"),
            r(x + 3),
            r(x + 3),
            r(11),
        );
        let b1 = sketch.add_point3d(
            format!("project-cubic3g2{index}.b1"),
            r(x + 4),
            r(x + 5),
            r(13),
        );
        let b2 = sketch.add_point3d(
            format!("project-cubic3g2{index}.b2"),
            r(x + 5),
            r(x + 8),
            r(17),
        );
        let b3 = sketch.add_point3d(
            format!("project-cubic3g2{index}.b3"),
            r(x + 6),
            r(x + 12),
            r(19),
        );
        let first = sketch.add_cubic3(format!("project-cubic3g2{index}.first"), a0, a1, a2, join);
        let second = sketch.add_cubic3(format!("project-cubic3g2{index}.second"), join, b1, b2, b3);
        let first_parameter = sketch.add_parameter(format!("project-cubic3g2{index}.ta"), r(1));
        let second_parameter = sketch.add_parameter(format!("project-cubic3g2{index}.tb"), r(0));
        hypersolve::sketch_tangency_builders::projected_cubic_curve_cubic_curve_g2_continuity3(
            &mut sketch,
            format!("projected 3D cubic cubic g2 {index}"),
            workplane,
            first,
            first_parameter,
            second,
            second_parameter,
        );
    }
    sketch
}

fn sketch_problem_with_cubic_cubic_tangent_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let x = index as i64;
        let a0 = sketch.add_point2d(format!("cubicpair{index}.a0"), r(x), r(x));
        let a1 = sketch.add_point2d(format!("cubicpair{index}.a1"), r(x + 1), r(x + 2));
        let a2 = sketch.add_point2d(format!("cubicpair{index}.a2"), r(x + 2), r(x + 4));
        let join = sketch.add_point2d(format!("cubicpair{index}.join"), r(x + 3), r(x + 6));
        let b1 = sketch.add_point2d(format!("cubicpair{index}.b1"), r(x + 4), r(x + 8));
        let b2 = sketch.add_point2d(format!("cubicpair{index}.b2"), r(x + 5), r(x + 10));
        let b3 = sketch.add_point2d(format!("cubicpair{index}.b3"), r(x + 6), r(x + 12));
        let first = sketch.add_cubic2(format!("cubicpair{index}.first"), a0, a1, a2, join);
        let second = sketch.add_cubic2(format!("cubicpair{index}.second"), join, b1, b2, b3);
        let first_parameter = sketch.add_parameter(format!("cubicpair{index}.ta"), r(1));
        let second_parameter = sketch.add_parameter(format!("cubicpair{index}.tb"), r(0));
        hypersolve::sketch_tangency_builders::cubic_cubic_tangent2(
            &mut sketch,
            format!("cubic cubic tangent {index}"),
            first,
            first_parameter,
            second,
            second_parameter,
        );
    }
    sketch
}

fn sketch_problem_with_cubic_cubic_c2_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let x = index as i64;
        let a0 = sketch.add_point2d(format!("cubicc2{index}.a0"), r(x), r(x));
        let a1 = sketch.add_point2d(format!("cubicc2{index}.a1"), r(x + 1), r(x + 2));
        let a2 = sketch.add_point2d(format!("cubicc2{index}.a2"), r(x + 2), r(x + 4));
        let join = sketch.add_point2d(format!("cubicc2{index}.join"), r(x + 3), r(x + 6));
        let b1 = sketch.add_point2d(format!("cubicc2{index}.b1"), r(x + 4), r(x + 8));
        let b2 = sketch.add_point2d(format!("cubicc2{index}.b2"), r(x + 5), r(x + 10));
        let b3 = sketch.add_point2d(format!("cubicc2{index}.b3"), r(x + 6), r(x + 12));
        let first = sketch.add_cubic2(format!("cubicc2{index}.first"), a0, a1, a2, join);
        let second = sketch.add_cubic2(format!("cubicc2{index}.second"), join, b1, b2, b3);
        let first_parameter = sketch.add_parameter(format!("cubicc2{index}.ta"), r(1));
        let second_parameter = sketch.add_parameter(format!("cubicc2{index}.tb"), r(0));
        hypersolve::sketch_tangency_builders::cubic_cubic_c2_continuity2(
            &mut sketch,
            format!("cubic c2 {index}"),
            first,
            first_parameter,
            second,
            second_parameter,
        );
    }
    sketch
}

fn sketch_problem_with_cubic_cubic_g2_relations(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let x = index as i64;
        let a0 = sketch.add_point2d(format!("cubicg2{index}.a0"), r(x), r(x));
        let a1 = sketch.add_point2d(format!("cubicg2{index}.a1"), r(x + 1), r(x + 2));
        let a2 = sketch.add_point2d(format!("cubicg2{index}.a2"), r(x + 2), r(x + 4));
        let join = sketch.add_point2d(format!("cubicg2{index}.join"), r(x + 3), r(x + 6));
        let b1 = sketch.add_point2d(format!("cubicg2{index}.b1"), r(x + 4), r(x + 8));
        let b2 = sketch.add_point2d(format!("cubicg2{index}.b2"), r(x + 5), r(x + 10));
        let b3 = sketch.add_point2d(format!("cubicg2{index}.b3"), r(x + 6), r(x + 12));
        let first = sketch.add_cubic2(format!("cubicg2{index}.first"), a0, a1, a2, join);
        let second = sketch.add_cubic2(format!("cubicg2{index}.second"), join, b1, b2, b3);
        let first_parameter = sketch.add_parameter(format!("cubicg2{index}.ta"), r(1));
        let second_parameter = sketch.add_parameter(format!("cubicg2{index}.tb"), r(0));
        hypersolve::sketch_tangency_builders::cubic_cubic_g2_continuity2(
            &mut sketch,
            format!("cubic g2 {index}"),
            first,
            first_parameter,
            second,
            second_parameter,
        );
    }
    sketch
}

fn sketch_problem_with_parameter_orderings(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let mut previous = sketch.add_parameter("order0", r(0));
    for index in 1..=row_count {
        let current = sketch.add_parameter(format!("order{index}"), r(index as i64));
        hypersolve::sketch_range_builders::parameter_ordering(
            &mut sketch,
            format!("nondecreasing {index}"),
            previous,
            current,
        );
        previous = current;
    }
    sketch
}

fn sketch_problem_with_parameter_margins(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let mut previous = sketch.add_parameter("margin0", r(0));
    for index in 1..=row_count {
        let current = sketch.add_parameter(format!("margin{index}"), r(index as i64 * 2));
        hypersolve::sketch_range_builders::parameter_margin(
            &mut sketch,
            format!("margin rule {index}"),
            previous,
            current,
            r(1),
        );
        previous = current;
    }
    sketch
}

fn sketch_problem_with_parameter_domains(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let parameter = sketch.add_parameter(format!("domain{index}"), r(index as i64 + 1));
        sketch.add_parameter_domain(parameter, hypersolve::SketchParameterDomain::Positive);
        sketch.add_parameter_domain(
            parameter,
            hypersolve::SketchParameterDomain::Bounded {
                lower: Some(r(0)),
                upper: Some(r(row_count as i64 + 1)),
            },
        );
    }
    sketch
}

fn sketch_problem_with_degeneracy_checks(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let a = sketch.add_point2d(format!("p{index}.a"), r(index as i64), r(0));
        let b = sketch.add_point2d(format!("p{index}.b"), r(index as i64 + 1), r(0));
        let radius = sketch.add_distance(format!("r{index}"), r(index as i64 + 1));
        let normal = sketch.add_normal2d(format!("n{index}"), r(1), r(0));
        sketch.add_line_segment2(format!("line{index}"), a, b);
        sketch.add_circle2(format!("circle{index}"), a, radius);
        sketch.add_arc_of_circle2(format!("arc{index}"), a, a, b, radius);
        sketch.add_workplane(format!("wp{index}"), a, normal);
    }
    sketch
}

fn sketch_problem_with_workplane_lifts(
    row_count: usize,
) -> (
    hypersolve::SketchSolveProblem,
    hypersolve::SketchEntityHandle,
    Vec<hypersolve::SketchEntityHandle>,
) {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", r(10), r(20), r(30));
    let normal = sketch.add_normal3d("normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let points = (0..row_count)
        .map(|index| {
            sketch.add_point2d(
                format!("uv{index}"),
                r(index as i64),
                r((row_count - index) as i64),
            )
        })
        .collect();
    (sketch, workplane, points)
}

fn sketch_problem_with_projected_distances(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project.origin", r(0), r(0), r(0));
    let normal = sketch.add_normal3d("project.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project.workplane", origin, normal);
    for index in 0..row_count {
        let x = index as i64;
        let a = sketch.add_point3d(format!("proj{index}.a"), r(x), r(x + 1), r(x + 2));
        let b = sketch.add_point3d(format!("proj{index}.b"), r(x + 3), r(x + 5), r(x + 99));
        let distance = sketch.add_distance(format!("proj{index}.d"), r(5));
        hypersolve::sketch_distance_builders::projected_point_point_distance(
            &mut sketch,
            format!("projected distance {index}"),
            workplane,
            a,
            b,
            distance,
        );
    }
    sketch
}

fn sketch_problem_with_projected_distance_ranges(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-range.origin", r(0), r(0), r(0));
    let normal = sketch.add_normal3d("project-range.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-range.workplane", origin, normal);
    for index in 0..row_count {
        let x = index as i64;
        let a = sketch.add_point3d(format!("projrange{index}.a"), r(x), r(x + 1), r(x + 2));
        let b = sketch.add_point3d(format!("projrange{index}.b"), r(x + 3), r(x + 5), r(x + 99));
        hypersolve::sketch_distance_builders::projected_point_point_distance_range(
            &mut sketch,
            format!("projected distance range {index}"),
            workplane,
            a,
            b,
            Some(r(4)),
            Some(r(6)),
        );
    }
    sketch
}

fn sketch_problem_with_projected_equal_point_distances(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-epp.origin", r(0), r(0), r(0));
    let normal = sketch.add_normal3d("project-epp.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-epp.workplane", origin, normal);
    for index in 0..row_count {
        let x = index as i64;
        let a = sketch.add_point3d(format!("projepp{index}.a"), r(x), r(x + 1), r(x + 2));
        let b = sketch.add_point3d(format!("projepp{index}.b"), r(x + 3), r(x + 5), r(x + 99));
        let c = sketch.add_point3d(format!("projepp{index}.c"), r(x + 20), r(x - 4), r(-x));
        let d = sketch.add_point3d(format!("projepp{index}.d"), r(x + 23), r(x), r(7 - x));
        hypersolve::sketch_distance_builders::projected_equal_point_point_distances3(
            &mut sketch,
            format!("projected equal point distances {index}"),
            workplane,
            a,
            b,
            c,
            d,
        );
    }
    sketch
}

fn sketch_problem_with_projected_point_line_distances(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-line.origin", r(0), r(0), r(0));
    let normal = sketch.add_normal3d("project-line.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-line.workplane", origin, normal);
    for index in 0..row_count {
        let y = index as i64;
        let start = sketch.add_point3d(format!("projline{index}.s"), r(0), r(y), r(-y));
        let end = sketch.add_point3d(format!("projline{index}.e"), r(4), r(y), r(12 + y));
        let line = sketch.add_line_segment3(format!("projline{index}.line"), start, end);
        let point = sketch.add_point3d(format!("projline{index}.p"), r(2), r(y + 3), r(99 - y));
        let distance = sketch.add_distance(format!("projline{index}.d"), r(3));
        hypersolve::sketch_distance_builders::projected_point_line_distance(
            &mut sketch,
            format!("projected point-line distance {index}"),
            workplane,
            point,
            line,
            distance,
        );
    }
    sketch
}

fn sketch_problem_with_projected_point_on_circles(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-circle.origin", r(1), r(2), r(3));
    let normal = sketch.add_normal3d("project-circle.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-circle.workplane", origin, normal);
    for index in 0..row_count {
        let y = index as i64;
        let center = sketch.add_point2d(format!("projcircle{index}.center"), r(2), r(y));
        let radius = sketch.add_distance(format!("projcircle{index}.radius"), r(5));
        let circle = sketch.add_circle2(format!("projcircle{index}.circle"), center, radius);
        let point = sketch.add_point3d(
            format!("projcircle{index}.point"),
            r(8),
            r(y + 2),
            r(99 - y),
        );
        hypersolve::sketch_incidence_builders::projected_point_on_circle3(
            &mut sketch,
            format!("projected point on circle {index}"),
            workplane,
            point,
            circle,
        );
    }
    sketch
}

fn sketch_problem_with_projected_line_circle_tangencies(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-lct.origin", r(1), r(2), r(3));
    let normal = sketch.add_normal3d("project-lct.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-lct.workplane", origin, normal);
    for index in 0..row_count {
        let y = index as i64;
        let center = sketch.add_point2d(format!("projlct{index}.center"), r(2), r(y));
        let radius = sketch.add_distance(format!("projlct{index}.radius"), r(5));
        let circle = sketch.add_circle2(format!("projlct{index}.circle"), center, radius);
        let start = sketch.add_point3d(format!("projlct{index}.start"), r(-8), r(y + 7), r(-y));
        let end = sketch.add_point3d(format!("projlct{index}.end"), r(12), r(y + 7), r(33 - y));
        let line = sketch.add_line_segment3(format!("projlct{index}.line"), start, end);
        hypersolve::sketch_tangency_builders::projected_line_circle_tangent3(
            &mut sketch,
            format!("projected line circle tangent {index}"),
            workplane,
            line,
            circle,
        );
    }
    sketch
}

fn sketch_problem_with_projected_point_on_arcs(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-arc.origin", r(1), r(2), r(3));
    let normal = sketch.add_normal3d("project-arc.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-arc.workplane", origin, normal);
    for index in 0..row_count {
        let y = index as i64;
        let center = sketch.add_point2d(format!("projarc{index}.center"), r(2), r(y));
        let start = sketch.add_point2d(format!("projarc{index}.start"), r(7), r(y));
        let end = sketch.add_point2d(format!("projarc{index}.end"), r(2), r(y + 5));
        let radius = sketch.add_distance(format!("projarc{index}.radius"), r(5));
        let arc =
            sketch.add_arc_of_circle2(format!("projarc{index}.arc"), center, start, end, radius);
        let point = sketch.add_point3d(format!("projarc{index}.point"), r(6), r(y + 6), r(99 - y));
        hypersolve::sketch_incidence_builders::projected_point_on_arc3(
            &mut sketch,
            format!("projected point on arc {index}"),
            workplane,
            point,
            arc,
            hypersolve::SketchArcPointSweep::CounterClockwiseMinor,
        );
    }
    sketch
}

fn sketch_problem_with_projected_point_on_lines(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-point-line.origin", r(1), r(2), r(3));
    let normal = sketch.add_normal3d("project-point-line.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-point-line.workplane", origin, normal);
    for index in 0..row_count {
        let y = index as i64;
        let start = sketch.add_point3d(format!("projline{index}.start"), r(1), r(y + 2), r(-y));
        let end = sketch.add_point3d(format!("projline{index}.end"), r(6), r(y + 12), r(99 - y));
        let line = sketch.add_line_segment3(format!("projline{index}.line"), start, end);
        let point = sketch.add_point3d(format!("projline{index}.point"), r(3), r(y + 6), r(50 - y));
        hypersolve::sketch_incidence_builders::projected_point_on_line3(
            &mut sketch,
            format!("projected point on line {index}"),
            workplane,
            point,
            line,
        );
    }
    sketch
}

fn sketch_problem_with_projected_point_line_distance_ranges(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-line-range.origin", r(0), r(0), r(0));
    let normal = sketch.add_normal3d("project-line-range.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-line-range.workplane", origin, normal);
    for index in 0..row_count {
        let y = index as i64;
        let start = sketch.add_point3d(format!("projlinerange{index}.s"), r(0), r(y), r(-y));
        let end = sketch.add_point3d(format!("projlinerange{index}.e"), r(4), r(y), r(12 + y));
        let line = sketch.add_line_segment3(format!("projlinerange{index}.line"), start, end);
        let point =
            sketch.add_point3d(format!("projlinerange{index}.p"), r(2), r(y + 3), r(99 - y));
        hypersolve::sketch_distance_builders::projected_point_line_distance_range(
            &mut sketch,
            format!("projected point-line distance range {index}"),
            workplane,
            point,
            line,
            Some(r(2)),
            Some(r(4)),
        );
    }
    sketch
}

fn sketch_problem_with_projected_equal_lengths(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-length.origin", r(0), r(0), r(0));
    let normal = sketch.add_normal3d("project-length.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-length.workplane", origin, normal);
    for index in 0..row_count {
        let y = index as i64;
        let a0 = sketch.add_point3d(format!("projlen{index}.a0"), r(0), r(y), r(-y));
        let a1 = sketch.add_point3d(format!("projlen{index}.a1"), r(3), r(y + 4), r(12 + y));
        let b0 = sketch.add_point3d(format!("projlen{index}.b0"), r(10), r(y + 8), r(y));
        let b1 = sketch.add_point3d(format!("projlen{index}.b1"), r(15), r(y + 8), r(99 - y));
        let a = sketch.add_line_segment3(format!("projlen{index}.a"), a0, a1);
        let b = sketch.add_line_segment3(format!("projlen{index}.b"), b0, b1);
        hypersolve::sketch_distance_builders::projected_equal_length_lines3(
            &mut sketch,
            format!("projected equal length {index}"),
            workplane,
            a,
            b,
        );
    }
    sketch
}

fn sketch_problem_with_projected_line_length_ranges(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-length-range.origin", r(0), r(0), r(0));
    let normal = sketch.add_normal3d("project-length-range.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-length-range.workplane", origin, normal);
    for index in 0..row_count {
        let y = index as i64;
        let start = sketch.add_point3d(format!("projlenrange{index}.s"), r(0), r(y), r(-y));
        let end = sketch.add_point3d(format!("projlenrange{index}.e"), r(5), r(y), r(12 + y));
        let line = sketch.add_line_segment3(format!("projlenrange{index}.line"), start, end);
        hypersolve::sketch_distance_builders::projected_line_length_range3(
            &mut sketch,
            format!("projected line length range {index}"),
            workplane,
            line,
            Some(r(4)),
            Some(r(6)),
        );
    }
    sketch
}

fn sketch_problem_with_projected_length_ratios(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-ratio.origin", r(0), r(0), r(0));
    let normal = sketch.add_normal3d("project-ratio.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-ratio.workplane", origin, normal);
    for index in 0..row_count {
        let y = index as i64;
        let a0 = sketch.add_point3d(format!("projratio{index}.a0"), r(0), r(y), r(-y));
        let a1 = sketch.add_point3d(format!("projratio{index}.a1"), r(6), r(y + 8), r(12 + y));
        let b0 = sketch.add_point3d(format!("projratio{index}.b0"), r(10), r(y + 8), r(y));
        let b1 = sketch.add_point3d(format!("projratio{index}.b1"), r(13), r(y + 12), r(99 - y));
        let a = sketch.add_line_segment3(format!("projratio{index}.a"), a0, a1);
        let b = sketch.add_line_segment3(format!("projratio{index}.b"), b0, b1);
        hypersolve::sketch_distance_builders::projected_length_ratio_lines3(
            &mut sketch,
            format!("projected length ratio {index}"),
            workplane,
            a,
            b,
            r(2),
            r(1),
        );
    }
    sketch
}

fn sketch_problem_with_projected_length_differences(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-diff.origin", r(0), r(0), r(0));
    let normal = sketch.add_normal3d("project-diff.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-diff.workplane", origin, normal);
    for index in 0..row_count {
        let y = index as i64;
        let long0 = sketch.add_point3d(format!("projdiff{index}.l0"), r(0), r(y), r(-y));
        let long1 = sketch.add_point3d(format!("projdiff{index}.l1"), r(10), r(y), r(12 + y));
        let short0 = sketch.add_point3d(format!("projdiff{index}.s0"), r(20), r(y + 4), r(y));
        let short1 = sketch.add_point3d(format!("projdiff{index}.s1"), r(25), r(y + 4), r(99 - y));
        let longer = sketch.add_line_segment3(format!("projdiff{index}.long"), long0, long1);
        let shorter = sketch.add_line_segment3(format!("projdiff{index}.short"), short0, short1);
        let difference = sketch.add_distance(format!("projdiff{index}.d"), r(5));
        hypersolve::sketch_distance_builders::projected_length_difference_lines3(
            &mut sketch,
            format!("projected length difference {index}"),
            workplane,
            longer,
            shorter,
            difference,
        );
    }
    sketch
}

fn sketch_problem_with_projected_length_point_line_distances(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-lpl.origin", r(0), r(0), r(0));
    let normal = sketch.add_normal3d("project-lpl.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-lpl.workplane", origin, normal);
    for index in 0..row_count {
        let y = index as i64;
        let len0 = sketch.add_point3d(format!("projlpl{index}.len0"), r(0), r(y + 10), r(-y));
        let len1 = sketch.add_point3d(format!("projlpl{index}.len1"), r(5), r(y + 10), r(12 + y));
        let point = sketch.add_point3d(format!("projlpl{index}.point"), r(0), r(y + 5), r(30 - y));
        let dist0 = sketch.add_point3d(format!("projlpl{index}.dist0"), r(-8), r(y), r(y));
        let dist1 = sketch.add_point3d(format!("projlpl{index}.dist1"), r(8), r(y), r(99 - y));
        let length_line = sketch.add_line_segment3(format!("projlpl{index}.length"), len0, len1);
        let distance_line =
            sketch.add_line_segment3(format!("projlpl{index}.distance"), dist0, dist1);
        hypersolve::sketch_distance_builders::projected_equal_length_point_line_distance3(
            &mut sketch,
            format!("projected length point-line distance {index}"),
            workplane,
            length_line,
            point,
            distance_line,
        );
    }
    sketch
}

fn sketch_problem_with_projected_equal_point_line_distances(
    row_count: usize,
) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("project-epl.origin", r(0), r(0), r(0));
    let normal = sketch.add_normal3d("project-epl.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("project-epl.workplane", origin, normal);
    for index in 0..row_count {
        let y = index as i64;
        let a_point = sketch.add_point3d(format!("projepl{index}.ap"), r(0), r(y + 5), r(30 - y));
        let a0 = sketch.add_point3d(format!("projepl{index}.a0"), r(-8), r(y), r(y));
        let a1 = sketch.add_point3d(format!("projepl{index}.a1"), r(8), r(y), r(99 - y));
        let b_point = sketch.add_point3d(format!("projepl{index}.bp"), r(1), r(y + 25), r(-30 + y));
        let b0 = sketch.add_point3d(format!("projepl{index}.b0"), r(-10), r(y + 20), r(-y));
        let b1 = sketch.add_point3d(format!("projepl{index}.b1"), r(10), r(y + 20), r(12 + y));
        let a_line = sketch.add_line_segment3(format!("projepl{index}.a"), a0, a1);
        let b_line = sketch.add_line_segment3(format!("projepl{index}.b"), b0, b1);
        hypersolve::sketch_distance_builders::projected_equal_point_line_distances3(
            &mut sketch,
            format!("projected equal point-line distances {index}"),
            workplane,
            a_point,
            a_line,
            b_point,
            b_line,
        );
    }
    sketch
}

fn sketch_problem_with_workplane_symmetry(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    let origin = sketch.add_point3d("sym.origin", r(0), r(0), r(0));
    let normal = sketch.add_normal3d("sym.normal", r(1), r(0), r(0), r(0));
    let workplane = sketch.add_workplane("sym.workplane", origin, normal);
    for index in 0..row_count {
        let x = index as i64;
        let offset = index as i64 + 1;
        let a = sketch.add_point3d(format!("sym{index}.a"), r(x), r(x + 2), r(offset));
        let b = sketch.add_point3d(format!("sym{index}.b"), r(x), r(x + 2), r(-offset));
        hypersolve::sketch_symmetry_builders::symmetric_workplane3(
            &mut sketch,
            format!("workplane symmetry {index}"),
            a,
            b,
            workplane,
        );
    }
    sketch
}

fn sketch_problem_with_entity_domains(row_count: usize) -> hypersolve::SketchSolveProblem {
    let mut sketch = hypersolve::SketchSolveProblem::new();
    for index in 0..row_count {
        let a = sketch.add_point2d(format!("domain{index}.a"), r(index as i64), r(0));
        let b = sketch.add_point2d(format!("domain{index}.b"), r(index as i64 + 1), r(0));
        let radius = sketch.add_distance(format!("domain{index}.r"), r(index as i64 + 1));
        let normal = sketch.add_normal2d(format!("domain{index}.n"), r(1), r(0));
        let line = sketch.add_line_segment2(format!("domain{index}.line"), a, b);
        let circle = sketch.add_circle2(format!("domain{index}.circle"), a, radius);
        let arc = sketch.add_arc_of_circle2(format!("domain{index}.arc"), a, a, b, radius);
        sketch.add_entity_domain(normal, hypersolve::SketchEntityDomain::UnitNormal);
        sketch.add_entity_domain(radius, hypersolve::SketchEntityDomain::PositiveRadius);
        sketch.add_entity_domain(circle, hypersolve::SketchEntityDomain::PositiveRadius);
        sketch.add_entity_domain(
            line,
            hypersolve::SketchEntityDomain::NonzeroTangentLineSegment2,
        );
        sketch.add_entity_domain(arc, hypersolve::SketchEntityDomain::NondegenerateArc2);
    }
    sketch
}

fn unary_endpoint_expression(row_count: usize) -> Expr {
    let mut expression = Expr::zero();
    for index in 1..=row_count {
        expression = expression
            + Expr::zero().sin()
            + Expr::zero().cos()
            + Expr::int(index as i64).sqrt()
            + Expr::int(1).log10()
            + Expr::int(1).ln()
            + Expr::zero().asin()
            + Expr::int(1).acos()
            + Expr::int(1).acosh()
            + Expr::zero().atanh();
    }
    expression
}

fn certification(c: &mut Criterion) {
    let problem = affine_problem(16);
    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);

    c.bench_function("prepared_solver_block_affine_rows", |b| {
        b.iter(|| PreparedSolverBlock::new(&prepared))
    });
    let sketch = sketch_problem(16);
    c.bench_function("sketch_lower_to_problem", |b| {
        b.iter(|| sketch.lower_to_problem())
    });
    let metadata_sketch = sketch_problem_with_metadata(16);
    c.bench_function("sketch_round_trip_metadata_lowering", |b| {
        b.iter(|| metadata_sketch.lower_to_problem())
    });
    c.bench_function("sketch_unit_tolerance_audit", |b| {
        b.iter(|| audit_sketch_unit_tolerances(&metadata_sketch))
    });
    let range_sketch = sketch_problem_with_ranges(16);
    c.bench_function("sketch_range_and_objective_lowering", |b| {
        b.iter(|| range_sketch.lower_to_problem())
    });
    let distance_range_sketch = sketch_problem_with_distance_ranges(16);
    c.bench_function("sketch_distance_range_lowering", |b| {
        b.iter(|| distance_range_sketch.lower_to_problem())
    });
    let equal_length_radius_sketch = sketch_problem_with_equal_length_and_radius(16);
    c.bench_function("sketch_equal_length_radius_lowering", |b| {
        b.iter(|| equal_length_radius_sketch.lower_to_problem())
    });
    let concentric_sketch = sketch_problem_with_concentric_relations(16);
    c.bench_function("sketch_concentric_lowering", |b| {
        b.iter(|| concentric_sketch.lower_to_problem())
    });
    let projected_point_concentric_sketch =
        sketch_problem_with_projected_point_concentric_relations(16);
    c.bench_function("sketch_projected_point_concentric_lowering", |b| {
        b.iter(|| projected_point_concentric_sketch.lower_to_problem())
    });
    let length_ratio_point_line_sketch = sketch_problem_with_length_ratio_and_point_line(16);
    c.bench_function("sketch_length_ratio_point_line_lowering", |b| {
        b.iter(|| length_ratio_point_line_sketch.lower_to_problem())
    });
    let length_difference_sketch = sketch_problem_with_length_differences(16);
    c.bench_function("sketch_length_difference_lowering", |b| {
        b.iter(|| length_difference_sketch.lower_to_problem())
    });
    let line_arc_length_sketch = sketch_problem_with_line_arc_lengths(16);
    c.bench_function("sketch_line_arc_length_lowering", |b| {
        b.iter(|| line_arc_length_sketch.lower_to_problem())
    });
    let line_arc_sweep_length_sketch = sketch_problem_with_line_arc_sweep_lengths(16);
    c.bench_function("sketch_line_arc_sweep_length_lowering", |b| {
        b.iter(|| line_arc_sweep_length_sketch.lower_to_problem())
    });
    let point_on_arc_sketch = sketch_problem_with_point_on_arcs(16);
    c.bench_function("sketch_point_on_arc_lowering", |b| {
        b.iter(|| point_on_arc_sketch.lower_to_problem())
    });
    let point_on_line_sketch = sketch_problem_with_point_on_lines(16);
    c.bench_function("sketch_point_on_line_lowering", |b| {
        b.iter(|| point_on_line_sketch.lower_to_problem())
    });
    let projected_line_arc_sweep_length_sketch =
        sketch_problem_with_projected_line_arc_sweep_lengths(16);
    c.bench_function("sketch_projected_line_arc_sweep_length_lowering", |b| {
        b.iter(|| projected_line_arc_sweep_length_sketch.lower_to_problem())
    });
    let equal_point_line_sketch = sketch_problem_with_equal_point_line_distances(16);
    c.bench_function("sketch_equal_point_line_distance_lowering", |b| {
        b.iter(|| equal_point_line_sketch.lower_to_problem())
    });
    let line_orientation_sketch = sketch_problem_with_line_orientation_relations(16);
    c.bench_function("sketch_line_orientation_lowering", |b| {
        b.iter(|| line_orientation_sketch.lower_to_problem())
    });
    let equal_angle_sketch = sketch_problem_with_equal_angle_relations(16);
    c.bench_function("sketch_equal_angle_lowering", |b| {
        b.iter(|| equal_angle_sketch.lower_to_problem())
    });
    let oriented_angle_sketch = sketch_problem_with_oriented_angle_relations(16);
    c.bench_function("sketch_oriented_angle_lowering", |b| {
        b.iter(|| oriented_angle_sketch.lower_to_problem())
    });
    let projected_oriented_angle_sketch =
        sketch_problem_with_projected_oriented_angle_relations(16);
    c.bench_function("sketch_projected_oriented_angle_lowering", |b| {
        b.iter(|| projected_oriented_angle_sketch.lower_to_problem())
    });
    let projected_line_orientation_sketch =
        sketch_problem_with_projected_line_orientation_relations(16);
    c.bench_function("sketch_projected_line_orientation_lowering", |b| {
        b.iter(|| projected_line_orientation_sketch.lower_to_problem())
    });
    let midpoint_sketch = sketch_problem_with_midpoint_relations(16);
    c.bench_function("sketch_midpoint_lowering", |b| {
        b.iter(|| midpoint_sketch.lower_to_problem())
    });
    let axis_symmetry_sketch = sketch_problem_with_axis_symmetry_relations(16);
    c.bench_function("sketch_axis_symmetry_lowering", |b| {
        b.iter(|| axis_symmetry_sketch.lower_to_problem())
    });
    let line_symmetry_sketch = sketch_problem_with_line_symmetry_relations(16);
    c.bench_function("sketch_line_symmetry_lowering", |b| {
        b.iter(|| line_symmetry_sketch.lower_to_problem())
    });
    let projected_line_symmetry_sketch = sketch_problem_with_projected_line_symmetry_relations(16);
    c.bench_function("sketch_projected_line_symmetry_lowering", |b| {
        b.iter(|| projected_line_symmetry_sketch.lower_to_problem())
    });
    let same_direction_sketch = sketch_problem_with_same_direction_relations(16);
    c.bench_function("sketch_same_direction_lowering", |b| {
        b.iter(|| same_direction_sketch.lower_to_problem())
    });
    let tangent_same_direction_sketch = sketch_problem_with_tangent_same_direction_relations(16);
    c.bench_function("sketch_tangent_same_direction_lowering", |b| {
        b.iter(|| tangent_same_direction_sketch.lower_to_problem())
    });
    let arc_line_tangent_sketch = sketch_problem_with_arc_line_tangent_relations(16);
    c.bench_function("sketch_arc_line_tangent_lowering", |b| {
        b.iter(|| arc_line_tangent_sketch.lower_to_problem())
    });
    let projected_arc_line_tangent_sketch =
        sketch_problem_with_projected_arc_line_tangent_relations(16);
    c.bench_function("sketch_projected_arc_line_tangent_lowering", |b| {
        b.iter(|| projected_arc_line_tangent_sketch.lower_to_problem())
    });
    let projected_arc_cubic_curve_tangent_sketch =
        sketch_problem_with_projected_arc_cubic_curve_tangent_relations(16);
    c.bench_function("sketch_projected_arc_cubic_curve_tangent_lowering", |b| {
        b.iter(|| projected_arc_cubic_curve_tangent_sketch.lower_to_problem())
    });
    let projected_arc_cubic_curve_second_order_sketch =
        sketch_problem_with_projected_arc_cubic_curve_second_order_relations(16);
    c.bench_function(
        "sketch_projected_arc_cubic_curve_second_order_lowering",
        |b| b.iter(|| projected_arc_cubic_curve_second_order_sketch.lower_to_problem()),
    );
    let arc_arc_tangent_sketch = sketch_problem_with_arc_arc_tangent_relations(16);
    c.bench_function("sketch_arc_arc_tangent_lowering", |b| {
        b.iter(|| arc_arc_tangent_sketch.lower_to_problem())
    });
    let circle_circle_tangent_sketch = sketch_problem_with_circle_circle_tangent_relations(16);
    c.bench_function("sketch_circle_circle_tangent_lowering", |b| {
        b.iter(|| circle_circle_tangent_sketch.lower_to_problem())
    });
    let arc_cubic_tangent_sketch = sketch_problem_with_arc_cubic_tangent_relations(16);
    c.bench_function("sketch_arc_cubic_tangent_lowering", |b| {
        b.iter(|| arc_cubic_tangent_sketch.lower_to_problem())
    });
    let arc_cubic_second_order_sketch = sketch_problem_with_arc_cubic_second_order_relations(16);
    c.bench_function("sketch_arc_cubic_second_order_lowering", |b| {
        b.iter(|| arc_cubic_second_order_sketch.lower_to_problem())
    });
    let point_on_cubic_sketch = sketch_problem_with_point_on_cubic_relations(16);
    c.bench_function("sketch_point_on_cubic_lowering", |b| {
        b.iter(|| point_on_cubic_sketch.lower_to_problem())
    });
    let projected_point_on_cubic_sketch =
        sketch_problem_with_projected_point_on_cubic_relations(16);
    c.bench_function("sketch_projected_point_on_cubic_lowering", |b| {
        b.iter(|| projected_point_on_cubic_sketch.lower_to_problem())
    });
    let projected_point_on_cubic_curve_sketch =
        sketch_problem_with_projected_point_on_cubic_curve_relations(16);
    c.bench_function("sketch_projected_point_on_cubic_curve_lowering", |b| {
        b.iter(|| projected_point_on_cubic_curve_sketch.lower_to_problem())
    });
    let cubic_line_tangent_sketch = sketch_problem_with_cubic_line_tangent_relations(16);
    c.bench_function("sketch_cubic_line_tangent_lowering", |b| {
        b.iter(|| cubic_line_tangent_sketch.lower_to_problem())
    });
    let projected_cubic_line_tangent_sketch =
        sketch_problem_with_projected_cubic_line_tangent_relations(16);
    c.bench_function("sketch_projected_cubic_line_tangent_lowering", |b| {
        b.iter(|| projected_cubic_line_tangent_sketch.lower_to_problem())
    });
    let projected_cubic_curve_line_tangent_sketch =
        sketch_problem_with_projected_cubic_curve_line_tangent_relations(16);
    c.bench_function("sketch_projected_cubic_curve_line_tangent_lowering", |b| {
        b.iter(|| projected_cubic_curve_line_tangent_sketch.lower_to_problem())
    });
    let projected_cubic_curve_cubic_curve_tangent_sketch =
        sketch_problem_with_projected_cubic_curve_cubic_curve_tangent_relations(16);
    c.bench_function(
        "sketch_projected_cubic_curve_cubic_curve_tangent_lowering",
        |b| b.iter(|| projected_cubic_curve_cubic_curve_tangent_sketch.lower_to_problem()),
    );
    let projected_cubic_curve_cubic_curve_c2_sketch =
        sketch_problem_with_projected_cubic_curve_cubic_curve_c2_relations(16);
    c.bench_function(
        "sketch_projected_cubic_curve_cubic_curve_c2_lowering",
        |b| b.iter(|| projected_cubic_curve_cubic_curve_c2_sketch.lower_to_problem()),
    );
    let projected_cubic_curve_cubic_curve_g2_sketch =
        sketch_problem_with_projected_cubic_curve_cubic_curve_g2_relations(16);
    c.bench_function(
        "sketch_projected_cubic_curve_cubic_curve_g2_lowering",
        |b| b.iter(|| projected_cubic_curve_cubic_curve_g2_sketch.lower_to_problem()),
    );
    let cubic_cubic_tangent_sketch = sketch_problem_with_cubic_cubic_tangent_relations(16);
    c.bench_function("sketch_cubic_cubic_tangent_lowering", |b| {
        b.iter(|| cubic_cubic_tangent_sketch.lower_to_problem())
    });
    let cubic_cubic_g2_sketch = sketch_problem_with_cubic_cubic_g2_relations(16);
    c.bench_function("sketch_cubic_cubic_g2_lowering", |b| {
        b.iter(|| cubic_cubic_g2_sketch.lower_to_problem())
    });
    let cubic_cubic_c2_sketch = sketch_problem_with_cubic_cubic_c2_relations(16);
    c.bench_function("sketch_cubic_cubic_c2_lowering", |b| {
        b.iter(|| cubic_cubic_c2_sketch.lower_to_problem())
    });
    let ordering_sketch = sketch_problem_with_parameter_orderings(16);
    c.bench_function("sketch_parameter_ordering_lowering", |b| {
        b.iter(|| ordering_sketch.lower_to_problem())
    });
    let margin_sketch = sketch_problem_with_parameter_margins(16);
    c.bench_function("sketch_parameter_margin_lowering", |b| {
        b.iter(|| margin_sketch.lower_to_problem())
    });
    let domain_sketch = sketch_problem_with_parameter_domains(16);
    c.bench_function("sketch_parameter_domain_preflight", |b| {
        b.iter(|| preflight_sketch_parameter_domains(&domain_sketch))
    });
    let degeneracy_sketch = sketch_problem_with_degeneracy_checks(16);
    c.bench_function("sketch_degeneracy_preflight", |b| {
        b.iter(|| preflight_sketch_degeneracies(&degeneracy_sketch))
    });
    let (workplane_sketch, workplane, workplane_points) = sketch_problem_with_workplane_lifts(16);
    c.bench_function("sketch_workplane_point_lifts", |b| {
        b.iter(|| {
            for point in &workplane_points {
                let _ = lift_sketch_point2_to_workplane3(&workplane_sketch, workplane, *point);
            }
        })
    });
    let projected_distance_sketch = sketch_problem_with_projected_distances(16);
    c.bench_function("sketch_projected_distance_lowering", |b| {
        b.iter(|| projected_distance_sketch.lower_to_problem())
    });
    let projected_distance_range_sketch = sketch_problem_with_projected_distance_ranges(16);
    c.bench_function("sketch_projected_distance_range_lowering", |b| {
        b.iter(|| projected_distance_range_sketch.lower_to_problem())
    });
    let projected_equal_point_distance_sketch =
        sketch_problem_with_projected_equal_point_distances(16);
    c.bench_function("sketch_projected_equal_point_distances_lowering", |b| {
        b.iter(|| projected_equal_point_distance_sketch.lower_to_problem())
    });
    let projected_point_line_distance_sketch =
        sketch_problem_with_projected_point_line_distances(16);
    c.bench_function("sketch_projected_point_line_distance_lowering", |b| {
        b.iter(|| projected_point_line_distance_sketch.lower_to_problem())
    });
    let projected_point_on_circle_sketch = sketch_problem_with_projected_point_on_circles(16);
    c.bench_function("sketch_projected_point_on_circle_lowering", |b| {
        b.iter(|| projected_point_on_circle_sketch.lower_to_problem())
    });
    let projected_line_circle_tangent_sketch =
        sketch_problem_with_projected_line_circle_tangencies(16);
    c.bench_function("sketch_projected_line_circle_tangent_lowering", |b| {
        b.iter(|| projected_line_circle_tangent_sketch.lower_to_problem())
    });
    let projected_point_on_arc_sketch = sketch_problem_with_projected_point_on_arcs(16);
    c.bench_function("sketch_projected_point_on_arc_lowering", |b| {
        b.iter(|| projected_point_on_arc_sketch.lower_to_problem())
    });
    let projected_point_on_line_sketch = sketch_problem_with_projected_point_on_lines(16);
    c.bench_function("sketch_projected_point_on_line_lowering", |b| {
        b.iter(|| projected_point_on_line_sketch.lower_to_problem())
    });
    let projected_point_line_distance_range_sketch =
        sketch_problem_with_projected_point_line_distance_ranges(16);
    c.bench_function("sketch_projected_point_line_distance_range_lowering", |b| {
        b.iter(|| projected_point_line_distance_range_sketch.lower_to_problem())
    });
    let projected_equal_length_sketch = sketch_problem_with_projected_equal_lengths(16);
    c.bench_function("sketch_projected_equal_length_lowering", |b| {
        b.iter(|| projected_equal_length_sketch.lower_to_problem())
    });
    let projected_line_length_range_sketch = sketch_problem_with_projected_line_length_ranges(16);
    c.bench_function("sketch_projected_line_length_range_lowering", |b| {
        b.iter(|| projected_line_length_range_sketch.lower_to_problem())
    });
    let projected_length_ratio_sketch = sketch_problem_with_projected_length_ratios(16);
    c.bench_function("sketch_projected_length_ratio_lowering", |b| {
        b.iter(|| projected_length_ratio_sketch.lower_to_problem())
    });
    let projected_length_difference_sketch = sketch_problem_with_projected_length_differences(16);
    c.bench_function("sketch_projected_length_difference_lowering", |b| {
        b.iter(|| projected_length_difference_sketch.lower_to_problem())
    });
    let projected_length_point_line_distance_sketch =
        sketch_problem_with_projected_length_point_line_distances(16);
    c.bench_function(
        "sketch_projected_length_point_line_distance_lowering",
        |b| b.iter(|| projected_length_point_line_distance_sketch.lower_to_problem()),
    );
    let projected_equal_point_line_distance_sketch =
        sketch_problem_with_projected_equal_point_line_distances(16);
    c.bench_function(
        "sketch_projected_equal_point_line_distances_lowering",
        |b| b.iter(|| projected_equal_point_line_distance_sketch.lower_to_problem()),
    );
    let workplane_symmetry_sketch = sketch_problem_with_workplane_symmetry(16);
    c.bench_function("sketch_workplane_symmetry_lowering", |b| {
        b.iter(|| workplane_symmetry_sketch.lower_to_problem())
    });
    let entity_domain_sketch = sketch_problem_with_entity_domains(16);
    c.bench_function("sketch_entity_domain_preflight", |b| {
        b.iter(|| preflight_sketch_entity_domains(&entity_domain_sketch))
    });
    c.bench_function("sketch_construction_certificate", |b| {
        b.iter(|| certify_sketch_construction(&sketch))
    });
    let form_handles = sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::PointPointDistance { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_distance_residual_forms", |b| {
        b.iter(|| {
            for handle in &form_handles {
                let _ = sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let concentric_form_handles = concentric_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::Concentric2 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_concentric_residual_forms", |b| {
        b.iter(|| {
            for handle in &concentric_form_handles {
                let _ = concentric_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let projected_point_concentric_form_handles = projected_point_concentric_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ProjectedPointConcentric3 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_projected_point_concentric_residual_forms", |b| {
        b.iter(|| {
            for handle in &projected_point_concentric_form_handles {
                let _ = projected_point_concentric_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let line_arc_form_handles = line_arc_length_sketch
        .constraints()
        .iter()
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_line_arc_length_residual_forms", |b| {
        b.iter(|| {
            for handle in &line_arc_form_handles {
                let _ = line_arc_length_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let line_arc_sweep_form_handles = line_arc_sweep_length_sketch
        .constraints()
        .iter()
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_line_arc_sweep_length_residual_forms", |b| {
        b.iter(|| {
            for handle in &line_arc_sweep_form_handles {
                let _ = line_arc_sweep_length_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let projected_line_arc_sweep_form_handles = projected_line_arc_sweep_length_sketch
        .constraints()
        .iter()
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function(
        "sketch_projected_line_arc_sweep_length_residual_forms",
        |b| {
            b.iter(|| {
                for handle in &projected_line_arc_sweep_form_handles {
                    let _ = projected_line_arc_sweep_length_sketch
                        .residual_forms_for_constraint(*handle);
                }
            })
        },
    );
    let circle_form_handles = sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::PointOnCircle { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_circle_incidence_residual_forms", |b| {
        b.iter(|| {
            for handle in &circle_form_handles {
                let _ = sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let point_on_arc_form_handles = point_on_arc_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::PointOnArc2 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_point_on_arc_residual_forms", |b| {
        b.iter(|| {
            for handle in &point_on_arc_form_handles {
                let _ = point_on_arc_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let point_on_line_form_handles = point_on_line_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::PointOnLine2 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_point_on_line_residual_forms", |b| {
        b.iter(|| {
            for handle in &point_on_line_form_handles {
                let _ = point_on_line_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let cubic_form_handles = point_on_cubic_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::PointOnCubic2 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_point_on_cubic_residual_forms", |b| {
        b.iter(|| {
            for handle in &cubic_form_handles {
                let _ = point_on_cubic_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let projected_cubic_form_handles = projected_point_on_cubic_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ProjectedPointOnCubic3 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_projected_point_on_cubic_residual_forms", |b| {
        b.iter(|| {
            for handle in &projected_cubic_form_handles {
                let _ = projected_point_on_cubic_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let projected_cubic_curve_form_handles = projected_point_on_cubic_curve_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ProjectedPointOnCubicCurve3 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function(
        "sketch_projected_point_on_cubic_curve_residual_forms",
        |b| {
            b.iter(|| {
                for handle in &projected_cubic_curve_form_handles {
                    let _ = projected_point_on_cubic_curve_sketch
                        .residual_forms_for_constraint(*handle);
                }
            })
        },
    );
    let cubic_tangent_form_handles = cubic_line_tangent_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::CubicLineTangent2 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_cubic_line_tangent_residual_forms", |b| {
        b.iter(|| {
            for handle in &cubic_tangent_form_handles {
                let _ = cubic_line_tangent_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let projected_cubic_tangent_form_handles = projected_cubic_line_tangent_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ProjectedCubicLineTangent3 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_projected_cubic_line_tangent_residual_forms", |b| {
        b.iter(|| {
            for handle in &projected_cubic_tangent_form_handles {
                let _ = projected_cubic_line_tangent_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let projected_cubic_curve_tangent_form_handles = projected_cubic_curve_line_tangent_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ProjectedCubicCurveLineTangent3 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function(
        "sketch_projected_cubic_curve_line_tangent_residual_forms",
        |b| {
            b.iter(|| {
                for handle in &projected_cubic_curve_tangent_form_handles {
                    let _ = projected_cubic_curve_line_tangent_sketch
                        .residual_forms_for_constraint(*handle);
                }
            })
        },
    );
    let projected_cubic_curve_cubic_curve_tangent_form_handles =
        projected_cubic_curve_cubic_curve_tangent_sketch
            .constraints()
            .iter()
            .filter(|constraint| {
                matches!(
                    constraint.kind,
                    hypersolve::SketchConstraintKind::ProjectedCubicCurveCubicCurveTangent3 { .. }
                )
            })
            .map(|constraint| constraint.handle)
            .collect::<Vec<_>>();
    c.bench_function(
        "sketch_projected_cubic_curve_cubic_curve_tangent_residual_forms",
        |b| {
            b.iter(|| {
                for handle in &projected_cubic_curve_cubic_curve_tangent_form_handles {
                    let _ = projected_cubic_curve_cubic_curve_tangent_sketch
                        .residual_forms_for_constraint(*handle);
                }
            })
        },
    );
    let projected_cubic_curve_cubic_curve_c2_form_handles =
        projected_cubic_curve_cubic_curve_c2_sketch
            .constraints()
            .iter()
            .filter(|constraint| {
                matches!(
                    constraint.kind,
                    hypersolve::SketchConstraintKind::ProjectedCubicCurveCubicCurveC2Continuity3 {
                        ..
                    }
                )
            })
            .map(|constraint| constraint.handle)
            .collect::<Vec<_>>();
    c.bench_function(
        "sketch_projected_cubic_curve_cubic_curve_c2_residual_forms",
        |b| {
            b.iter(|| {
                for handle in &projected_cubic_curve_cubic_curve_c2_form_handles {
                    let _ = projected_cubic_curve_cubic_curve_c2_sketch
                        .residual_forms_for_constraint(*handle);
                }
            })
        },
    );
    let projected_cubic_curve_cubic_curve_g2_form_handles =
        projected_cubic_curve_cubic_curve_g2_sketch
            .constraints()
            .iter()
            .filter(|constraint| {
                matches!(
                    constraint.kind,
                    hypersolve::SketchConstraintKind::ProjectedCubicCurveCubicCurveG2Continuity3 {
                        ..
                    }
                )
            })
            .map(|constraint| constraint.handle)
            .collect::<Vec<_>>();
    c.bench_function(
        "sketch_projected_cubic_curve_cubic_curve_g2_residual_forms",
        |b| {
            b.iter(|| {
                for handle in &projected_cubic_curve_cubic_curve_g2_form_handles {
                    let _ = projected_cubic_curve_cubic_curve_g2_sketch
                        .residual_forms_for_constraint(*handle);
                }
            })
        },
    );
    let cubic_cubic_tangent_form_handles = cubic_cubic_tangent_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::CubicCubicTangent2 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_cubic_cubic_tangent_residual_forms", |b| {
        b.iter(|| {
            for handle in &cubic_cubic_tangent_form_handles {
                let _ = cubic_cubic_tangent_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let cubic_cubic_g2_form_handles = cubic_cubic_g2_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::CubicCubicG2Continuity2 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_cubic_cubic_g2_residual_forms", |b| {
        b.iter(|| {
            for handle in &cubic_cubic_g2_form_handles {
                let _ = cubic_cubic_g2_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let cubic_cubic_c2_form_handles = cubic_cubic_c2_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::CubicCubicC2Continuity2 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_cubic_cubic_c2_residual_forms", |b| {
        b.iter(|| {
            for handle in &cubic_cubic_c2_form_handles {
                let _ = cubic_cubic_c2_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let point_line_form_handles = length_ratio_point_line_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::PointLineDistance2 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_point_line_residual_forms", |b| {
        b.iter(|| {
            for handle in &point_line_form_handles {
                let _ = length_ratio_point_line_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let angle_form_handles = equal_angle_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::EqualAngleLines2 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_equal_angle_residual_forms", |b| {
        b.iter(|| {
            for handle in &angle_form_handles {
                let _ = equal_angle_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let oriented_angle_form_handles = oriented_angle_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::EqualOrientedAngleLines2 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_oriented_angle_residual_forms", |b| {
        b.iter(|| {
            for handle in &oriented_angle_form_handles {
                let _ = oriented_angle_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let projected_oriented_angle_form_handles = projected_oriented_angle_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ProjectedEqualOrientedAngleLines3 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_projected_oriented_angle_residual_forms", |b| {
        b.iter(|| {
            for handle in &projected_oriented_angle_form_handles {
                let _ = projected_oriented_angle_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let projected_line_orientation_form_handles = projected_line_orientation_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ProjectedParallelLines3 { .. }
                    | hypersolve::SketchConstraintKind::ProjectedPerpendicularLines3 { .. }
                    | hypersolve::SketchConstraintKind::ProjectedSameDirectionLines3 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_projected_line_orientation_residual_forms", |b| {
        b.iter(|| {
            for handle in &projected_line_orientation_form_handles {
                let _ = projected_line_orientation_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let tangent_form_handles = tangent_same_direction_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::TangentSameDirectionLines2 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_tangent_residual_forms", |b| {
        b.iter(|| {
            for handle in &tangent_form_handles {
                let _ = tangent_same_direction_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let arc_line_tangent_form_handles = arc_line_tangent_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ArcLineTangent2 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_arc_line_tangent_residual_forms", |b| {
        b.iter(|| {
            for handle in &arc_line_tangent_form_handles {
                let _ = arc_line_tangent_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let projected_arc_line_tangent_form_handles = projected_arc_line_tangent_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ProjectedArcLineTangent3 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_projected_arc_line_tangent_residual_forms", |b| {
        b.iter(|| {
            for handle in &projected_arc_line_tangent_form_handles {
                let _ = projected_arc_line_tangent_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let projected_arc_cubic_curve_tangent_form_handles = projected_arc_cubic_curve_tangent_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ProjectedArcCubicCurveTangent3 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function(
        "sketch_projected_arc_cubic_curve_tangent_residual_forms",
        |b| {
            b.iter(|| {
                for handle in &projected_arc_cubic_curve_tangent_form_handles {
                    let _ = projected_arc_cubic_curve_tangent_sketch
                        .residual_forms_for_constraint(*handle);
                }
            })
        },
    );
    let projected_arc_cubic_curve_second_order_form_handles =
        projected_arc_cubic_curve_second_order_sketch
            .constraints()
            .iter()
            .filter(|constraint| {
                matches!(
                    constraint.kind,
                    hypersolve::SketchConstraintKind::ProjectedArcCubicCurveSecondOrderContact3 {
                        ..
                    }
                )
            })
            .map(|constraint| constraint.handle)
            .collect::<Vec<_>>();
    c.bench_function(
        "sketch_projected_arc_cubic_curve_second_order_residual_forms",
        |b| {
            b.iter(|| {
                for handle in &projected_arc_cubic_curve_second_order_form_handles {
                    let _ = projected_arc_cubic_curve_second_order_sketch
                        .residual_forms_for_constraint(*handle);
                }
            })
        },
    );
    let arc_arc_tangent_form_handles = arc_arc_tangent_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ArcArcTangent2 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_arc_arc_tangent_residual_forms", |b| {
        b.iter(|| {
            for handle in &arc_arc_tangent_form_handles {
                let _ = arc_arc_tangent_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let circle_circle_tangent_form_handles = circle_circle_tangent_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::CircleCircleTangent2 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_circle_circle_tangent_residual_forms", |b| {
        b.iter(|| {
            for handle in &circle_circle_tangent_form_handles {
                let _ = circle_circle_tangent_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let arc_cubic_tangent_form_handles = arc_cubic_tangent_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ArcCubicTangent2 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_arc_cubic_tangent_residual_forms", |b| {
        b.iter(|| {
            for handle in &arc_cubic_tangent_form_handles {
                let _ = arc_cubic_tangent_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let arc_cubic_second_order_form_handles = arc_cubic_second_order_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ArcCubicSecondOrderContact2 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_arc_cubic_second_order_residual_forms", |b| {
        b.iter(|| {
            for handle in &arc_cubic_second_order_form_handles {
                let _ = arc_cubic_second_order_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let projected_form_handles = projected_distance_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ProjectedPointPointDistance { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_projected_distance_residual_forms", |b| {
        b.iter(|| {
            for handle in &projected_form_handles {
                let _ = projected_distance_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let projected_range_form_handles = projected_distance_range_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ProjectedPointPointDistanceRange { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_projected_distance_range_residual_forms", |b| {
        b.iter(|| {
            for handle in &projected_range_form_handles {
                let _ = projected_distance_range_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let projected_equal_point_distance_form_handles = projected_equal_point_distance_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ProjectedEqualPointPointDistances3 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function(
        "sketch_projected_equal_point_distances_residual_forms",
        |b| {
            b.iter(|| {
                for handle in &projected_equal_point_distance_form_handles {
                    let _ = projected_equal_point_distance_sketch
                        .residual_forms_for_constraint(*handle);
                }
            })
        },
    );
    let projected_point_line_form_handles = projected_point_line_distance_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ProjectedPointLineDistance { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_projected_point_line_distance_residual_forms", |b| {
        b.iter(|| {
            for handle in &projected_point_line_form_handles {
                let _ = projected_point_line_distance_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let projected_point_on_circle_form_handles = projected_point_on_circle_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ProjectedPointOnCircle3 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_projected_point_on_circle_residual_forms", |b| {
        b.iter(|| {
            for handle in &projected_point_on_circle_form_handles {
                let _ = projected_point_on_circle_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let projected_line_circle_tangent_form_handles = projected_line_circle_tangent_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ProjectedLineCircleTangent3 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_projected_line_circle_tangent_residual_forms", |b| {
        b.iter(|| {
            for handle in &projected_line_circle_tangent_form_handles {
                let _ = projected_line_circle_tangent_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let projected_point_on_arc_form_handles = projected_point_on_arc_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ProjectedPointOnArc3 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_projected_point_on_arc_residual_forms", |b| {
        b.iter(|| {
            for handle in &projected_point_on_arc_form_handles {
                let _ = projected_point_on_arc_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let projected_point_on_line_form_handles = projected_point_on_line_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ProjectedPointOnLine3 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_projected_point_on_line_residual_forms", |b| {
        b.iter(|| {
            for handle in &projected_point_on_line_form_handles {
                let _ = projected_point_on_line_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let projected_point_line_range_form_handles = projected_point_line_distance_range_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ProjectedPointLineDistanceRange { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function(
        "sketch_projected_point_line_distance_range_residual_forms",
        |b| {
            b.iter(|| {
                for handle in &projected_point_line_range_form_handles {
                    let _ = projected_point_line_distance_range_sketch
                        .residual_forms_for_constraint(*handle);
                }
            })
        },
    );
    let projected_equal_length_form_handles = projected_equal_length_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ProjectedEqualLengthLines3 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_projected_equal_length_residual_forms", |b| {
        b.iter(|| {
            for handle in &projected_equal_length_form_handles {
                let _ = projected_equal_length_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let projected_line_length_range_form_handles = projected_line_length_range_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ProjectedLineLengthRange3 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_projected_line_length_range_residual_forms", |b| {
        b.iter(|| {
            for handle in &projected_line_length_range_form_handles {
                let _ = projected_line_length_range_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let projected_length_ratio_form_handles = projected_length_ratio_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ProjectedLengthRatioLines3 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_projected_length_ratio_residual_forms", |b| {
        b.iter(|| {
            for handle in &projected_length_ratio_form_handles {
                let _ = projected_length_ratio_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let projected_length_difference_form_handles = projected_length_difference_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ProjectedLengthDifferenceLines3 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_projected_length_difference_residual_forms", |b| {
        b.iter(|| {
            for handle in &projected_length_difference_form_handles {
                let _ = projected_length_difference_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let projected_length_point_line_distance_form_handles =
        projected_length_point_line_distance_sketch
            .constraints()
            .iter()
            .filter(|constraint| {
                matches!(
                    constraint.kind,
                    hypersolve::SketchConstraintKind::ProjectedEqualLengthPointLineDistance3 { .. }
                )
            })
            .map(|constraint| constraint.handle)
            .collect::<Vec<_>>();
    c.bench_function(
        "sketch_projected_length_point_line_distance_residual_forms",
        |b| {
            b.iter(|| {
                for handle in &projected_length_point_line_distance_form_handles {
                    let _ = projected_length_point_line_distance_sketch
                        .residual_forms_for_constraint(*handle);
                }
            })
        },
    );
    let projected_equal_point_line_distance_form_handles =
        projected_equal_point_line_distance_sketch
            .constraints()
            .iter()
            .filter(|constraint| {
                matches!(
                    constraint.kind,
                    hypersolve::SketchConstraintKind::ProjectedEqualPointLineDistances3 { .. }
                )
            })
            .map(|constraint| constraint.handle)
            .collect::<Vec<_>>();
    c.bench_function(
        "sketch_projected_equal_point_line_distances_residual_forms",
        |b| {
            b.iter(|| {
                for handle in &projected_equal_point_line_distance_form_handles {
                    let _ = projected_equal_point_line_distance_sketch
                        .residual_forms_for_constraint(*handle);
                }
            })
        },
    );
    let projected_line_symmetry_form_handles = projected_line_symmetry_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::ProjectedSymmetricLine3 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_projected_line_symmetry_residual_forms", |b| {
        b.iter(|| {
            for handle in &projected_line_symmetry_form_handles {
                let _ = projected_line_symmetry_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let workplane_symmetry_form_handles = workplane_symmetry_sketch
        .constraints()
        .iter()
        .filter(|constraint| {
            matches!(
                constraint.kind,
                hypersolve::SketchConstraintKind::SymmetricWorkplane3 { .. }
            )
        })
        .map(|constraint| constraint.handle)
        .collect::<Vec<_>>();
    c.bench_function("sketch_workplane_symmetry_residual_forms", |b| {
        b.iter(|| {
            for handle in &workplane_symmetry_form_handles {
                let _ = workplane_symmetry_sketch.residual_forms_for_constraint(*handle);
            }
        })
    });
    let compatibility_fixtures = sketch_compatibility_fixtures();
    c.bench_function("sketch_compatibility_fixture_replay", |b| {
        b.iter(|| {
            for fixture in &compatibility_fixtures {
                let _ = replay_sketch_compatibility_fixture(fixture);
            }
        })
    });
    let krawczyk_problem = affine_krawczyk_problem();
    let krawczyk_prepared = PreparedProblem::new(&krawczyk_problem);
    let krawczyk_context = context_from_problem(&krawczyk_problem);
    c.bench_function("certify_affine_krawczyk_box", |b| {
        b.iter(|| {
            certify_affine_krawczyk_box(
                &krawczyk_prepared,
                &krawczyk_context,
                &[
                    VariableBall {
                        symbol: SymbolId(0),
                        radius: r(1),
                    },
                    VariableBall {
                        symbol: SymbolId(1),
                        radius: r(0),
                    },
                ],
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("solve_direct_affine_system", |b| {
        b.iter(|| solve_direct_affine_system(&krawczyk_prepared))
    });
    c.bench_function("analyze_exact_affine_rank", |b| {
        b.iter(|| analyze_exact_affine_rank(&krawczyk_prepared, -64))
    });
    c.bench_function("determinant_bareiss", |b| {
        b.iter(|| determinant_bareiss(&[vec![r(2), r(1)], vec![r(1), r(-1)]], -64))
    });
    c.bench_function("solve_dense_linear_system_bareiss", |b| {
        b.iter(|| {
            solve_dense_linear_system_bareiss(
                &[vec![r(2), r(1)], vec![r(1), r(-1)]],
                &[r(5), r(1)],
                -64,
            )
        })
    });
    c.bench_function("solve_sparse_linear_system_bareiss", |b| {
        b.iter(|| {
            solve_sparse_linear_system_bareiss(
                2,
                2,
                &[
                    SparseResidualTerm {
                        row: 0,
                        column: 0,
                        coefficient: r(2),
                    },
                    SparseResidualTerm {
                        row: 0,
                        column: 1,
                        coefficient: r(1),
                    },
                    SparseResidualTerm {
                        row: 1,
                        column: 0,
                        coefficient: r(1),
                    },
                    SparseResidualTerm {
                        row: 1,
                        column: 1,
                        coefficient: r(-1),
                    },
                ],
                &[r(5), r(1)],
                -64,
            )
        })
    });
    c.bench_function(
        "solve_sparse_linear_system_bareiss_pattern_preserving",
        |b| {
            b.iter(|| {
                solve_sparse_linear_system_bareiss_pattern_preserving(
                    3,
                    3,
                    &[
                        SparseResidualTerm {
                            row: 0,
                            column: 0,
                            coefficient: r(2),
                        },
                        SparseResidualTerm {
                            row: 0,
                            column: 2,
                            coefficient: r(1),
                        },
                        SparseResidualTerm {
                            row: 1,
                            column: 0,
                            coefficient: r(1),
                        },
                        SparseResidualTerm {
                            row: 1,
                            column: 1,
                            coefficient: r(1),
                        },
                        SparseResidualTerm {
                            row: 2,
                            column: 2,
                            coefficient: r(3),
                        },
                    ],
                    &[r(4), r(3), r(6)],
                    -64,
                )
            })
        },
    );
    c.bench_function("analyze_sparse_bareiss_elimination_pattern", |b| {
        b.iter(|| {
            analyze_sparse_bareiss_elimination_pattern(
                3,
                3,
                &[
                    SparseResidualTerm {
                        row: 0,
                        column: 0,
                        coefficient: r(1),
                    },
                    SparseResidualTerm {
                        row: 0,
                        column: 2,
                        coefficient: r(1),
                    },
                    SparseResidualTerm {
                        row: 1,
                        column: 0,
                        coefficient: r(1),
                    },
                    SparseResidualTerm {
                        row: 1,
                        column: 1,
                        coefficient: r(1),
                    },
                    SparseResidualTerm {
                        row: 2,
                        column: 2,
                        coefficient: r(1),
                    },
                ],
                -64,
            )
        })
    });
    c.bench_function("resultant_univariate_polynomials", |b| {
        b.iter(|| resultant_univariate_polynomials(&[r(-1), r(0), r(1)], &[r(-2), r(1)], -64))
    });
    c.bench_function("subresultant_chain_univariate_polynomials", |b| {
        b.iter(|| {
            subresultant_chain_univariate_polynomials(&[r(2), r(-3), r(1)], &[r(-1), r(1)], -64)
        })
    });
    c.bench_function("schedule_univariate_resultant_pairs", |b| {
        b.iter(|| {
            schedule_univariate_resultant_pairs(
                &[
                    UnivariateResultantPairInput {
                        pair_index: 0,
                        left_coefficients: vec![r(-1), r(1)],
                        right_coefficients: vec![r(-2), r(1)],
                    },
                    UnivariateResultantPairInput {
                        pair_index: 1,
                        left_coefficients: vec![r(-1), r(1)],
                        right_coefficients: vec![r(-1), r(0), r(1)],
                    },
                ],
                -64,
            )
        })
    });
    let parabola = PolynomialParametricCurve2::new(vec![r(0), r(1)], vec![r(0), r(0), r(1)]);
    let horizontal = PolynomialParametricCurve2::new(vec![r(0), r(1)], vec![r(1)]);
    c.bench_function("resultant_parametric_curve_intersection", |b| {
        b.iter(|| {
            resultant_parametric_curve_intersection(
                &parabola,
                &horizontal,
                CurveResultantParameter::First,
                hypersolve::CurveIntersectionResultantConfig::default(),
            )
        })
    });
    let bezier_controls = [
        PolynomialCurvePoint2::new(r(0), r(0)),
        PolynomialCurvePoint2::new(r(1), r(2)),
        PolynomialCurvePoint2::new(r(2), r(0)),
    ];
    c.bench_function("substitute_bezier_power_basis", |b| {
        b.iter(|| {
            substitute_bezier_power_basis(
                &bezier_controls,
                hypersolve::BezierPowerBasisSubstitutionConfig::default(),
            )
        })
    });
    let rational_bezier_controls = [
        RationalCurveControlPoint2::new(r(0), r(0), r(1)),
        RationalCurveControlPoint2::new(r(1), r(2), r(2)),
        RationalCurveControlPoint2::new(r(2), r(0), r(1)),
    ];
    c.bench_function("substitute_rational_bezier_power_basis", |b| {
        b.iter(|| {
            substitute_rational_bezier_power_basis(
                &rational_bezier_controls,
                hypersolve::BezierPowerBasisSubstitutionConfig::default(),
            )
        })
    });
    let bspline_controls = [
        PolynomialCurvePoint2::new(r(0), r(0)),
        PolynomialCurvePoint2::new(r(1), r(2)),
        PolynomialCurvePoint2::new(r(3), r(2)),
        PolynomialCurvePoint2::new(r(4), r(0)),
    ];
    let bspline_knots = [r(0), r(0), r(0), r(1), r(2), r(2), r(2)];
    c.bench_function("substitute_bspline_knot_span_power_basis", |b| {
        b.iter(|| {
            substitute_bspline_knot_span_power_basis(
                &bspline_controls,
                &bspline_knots,
                2,
                2,
                BsplineKnotSpanSubstitutionConfig::default(),
            )
        })
    });
    let nurbs_controls = [
        RationalCurveControlPoint2::new(r(0), r(0), r(1)),
        RationalCurveControlPoint2::new(r(1), r(2), r(2)),
        RationalCurveControlPoint2::new(r(3), r(2), r(2)),
        RationalCurveControlPoint2::new(r(4), r(0), r(1)),
    ];
    c.bench_function("substitute_nurbs_knot_span_power_basis", |b| {
        b.iter(|| {
            substitute_nurbs_knot_span_power_basis(
                &nurbs_controls,
                &bspline_knots,
                2,
                2,
                BsplineKnotSpanSubstitutionConfig::default(),
            )
        })
    });
    let weighted_parabola =
        RationalParametricCurve2::new(vec![r(0), r(1)], vec![r(0), r(0), r(1)], vec![r(1), r(1)]);
    let rational_horizontal = RationalParametricCurve2::from_polynomial(&horizontal);
    c.bench_function("resultant_rational_parametric_curve_intersection", |b| {
        b.iter(|| {
            resultant_rational_parametric_curve_intersection(
                &weighted_parabola,
                &rational_horizontal,
                CurveResultantParameter::First,
                hypersolve::CurveIntersectionResultantConfig::default(),
            )
        })
    });
    c.bench_function("replay_dense_linear_residuals", |b| {
        b.iter(|| {
            replay_dense_linear_residuals(
                &[vec![r(2), r(1)], vec![r(1), r(-1)]],
                &[r(5), r(1)],
                &[r(2), r(1)],
                -64,
            )
        })
    });
    c.bench_function("replay_sparse_linear_residuals", |b| {
        b.iter(|| {
            replay_sparse_linear_residuals(
                2,
                2,
                &[
                    SparseResidualTerm {
                        row: 0,
                        column: 0,
                        coefficient: r(2),
                    },
                    SparseResidualTerm {
                        row: 0,
                        column: 1,
                        coefficient: r(1),
                    },
                    SparseResidualTerm {
                        row: 1,
                        column: 0,
                        coefficient: r(1),
                    },
                    SparseResidualTerm {
                        row: 1,
                        column: 1,
                        coefficient: r(-1),
                    },
                ],
                &[r(5), r(1)],
                &[r(2), r(1)],
                -64,
            )
        })
    });
    let sparse_batch_terms = vec![
        SparseResidualTerm {
            row: 0,
            column: 0,
            coefficient: r(2),
        },
        SparseResidualTerm {
            row: 0,
            column: 1,
            coefficient: r(1),
        },
        SparseResidualTerm {
            row: 1,
            column: 0,
            coefficient: r(1),
        },
        SparseResidualTerm {
            row: 1,
            column: 1,
            coefficient: r(-1),
        },
    ];
    let sparse_batch_system =
        prepare_sparse_linear_residual_system(2, 2, &sparse_batch_terms, &[r(5), r(1)]).unwrap();
    let sparse_batch_candidates = (0..16).map(|_| vec![r(2), r(1)]).collect::<Vec<_>>();
    c.bench_function("prepared_sparse_linear_batch_replay", |b| {
        b.iter(|| sparse_batch_system.replay_batch(&sparse_batch_candidates, -64))
    });
    let elimination_problem = substitution_elimination_problem(16);
    let elimination_prepared = PreparedProblem::new(&elimination_problem);
    let substitutions = vec![
        EqualitySubstitution {
            constraint_index: 0,
            left: SymbolId(1),
            right: SymbolId(0),
            offset: r(3),
        },
        EqualitySubstitution {
            constraint_index: 1,
            left: SymbolId(2),
            right: SymbolId(0),
            offset: r(-2),
        },
    ];
    let substitution_classes = build_equality_substitution_classes(&substitutions).unwrap();
    c.bench_function("eliminate_affine_rows_with_substitution_classes", |b| {
        b.iter(|| {
            eliminate_affine_rows_with_substitution_classes(
                &elimination_prepared,
                &substitution_classes,
            )
        })
    });
    c.bench_function("apply_equality_substitution_classes", |b| {
        b.iter(|| {
            let mut context = hypersolve::EvaluationContext::default();
            context.bind(SymbolId(2), r(8));
            apply_equality_substitution_classes(&mut context, &substitution_classes)
        })
    });
    let quadratic_problem = univariate_quadratic_problem(16);
    c.bench_function("prepared_univariate_quadratic_rows", |b| {
        b.iter(|| PreparedProblem::new(&quadratic_problem))
    });
    let multivariate_quadratic = multivariate_quadratic_problem(16);
    c.bench_function("prepared_multivariate_quadratic_rows", |b| {
        b.iter(|| PreparedProblem::new(&multivariate_quadratic))
    });
    let prepared_multivariate_quadratic = PreparedProblem::new(&multivariate_quadratic);
    let multivariate_quadratic_context = context_from_problem(&multivariate_quadratic);
    let prepared_quadratic = PreparedProblem::new(&quadratic_problem);
    let quadratic_context = context_from_problem(&quadratic_problem);
    c.bench_function("solve_direct_univariate_quadratic_rows", |b| {
        b.iter(|| solve_direct_univariate_quadratic_equalities(&prepared_quadratic))
    });
    c.bench_function("certify_direct_univariate_quadratic_roots", |b| {
        b.iter(|| {
            certify_direct_univariate_quadratic_roots(&prepared_quadratic, &quadratic_context)
        })
    });
    c.bench_function("enumerate_direct_univariate_quadratic_branches", |b| {
        b.iter(|| {
            enumerate_direct_univariate_quadratic_branches(&prepared_quadratic, &quadratic_context)
        })
    });
    c.bench_function("isolate_univariate_polynomial_roots_sturm", |b| {
        b.iter(|| {
            isolate_univariate_polynomial_roots(
                &prepared_quadratic,
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("represent_univariate_algebraic_roots", |b| {
        b.iter(|| {
            represent_univariate_algebraic_roots(
                &prepared_quadratic,
                hypersolve::RootIsolationConfig::default(),
            )
        })
    });
    let represented_roots = represent_univariate_algebraic_roots(
        &prepared_quadratic,
        hypersolve::RootIsolationConfig::default(),
    );
    c.bench_function("compare_algebraic_root_representations", |b| {
        b.iter(|| {
            compare_algebraic_root_representations(
                &represented_roots[0].roots[0],
                &represented_roots[0].roots[1],
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    let sqrt_two = AlgebraicRootRepresentation {
        constraint_index: 0,
        symbol: SymbolId(0),
        interval_index: 0,
        polynomial_coefficients: vec![r(-2), Real::zero(), Real::one()],
        interval: IsolatedRootInterval {
            lower: r(1),
            upper: r(2),
            exact_root: None,
            distinct_root_count: 1,
        },
        kind: AlgebraicRootKind::IsolatingInterval,
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    };
    let sqrt_three = AlgebraicRootRepresentation {
        constraint_index: 1,
        polynomial_coefficients: vec![r(-3), Real::zero(), Real::one()],
        ..sqrt_two.clone()
    };
    c.bench_function(
        "compare_algebraic_root_representations_with_refinement",
        |b| {
            b.iter(|| {
                compare_algebraic_root_representations_with_refinement(
                    &sqrt_two,
                    &sqrt_three,
                    AlgebraicRootRefinementComparisonConfig {
                        max_refinement_rounds: 8,
                        steps_per_round: 1,
                        ..AlgebraicRootRefinementComparisonConfig::default()
                    },
                )
            })
        },
    );
    c.bench_function(
        "compare_algebraic_root_representations_by_difference",
        |b| {
            b.iter(|| {
                compare_algebraic_root_representations_by_difference(
                    &sqrt_two,
                    &sqrt_three,
                    AlgebraicRootRefinementComparisonConfig {
                        max_refinement_rounds: 0,
                        ..AlgebraicRootRefinementComparisonConfig::default()
                    },
                )
            })
        },
    );
    let rational_two = AlgebraicRootRepresentation {
        constraint_index: 2,
        symbol: SymbolId(0),
        interval_index: 0,
        polynomial_coefficients: vec![r(-2), Real::one()],
        interval: IsolatedRootInterval {
            lower: r(2),
            upper: r(2),
            exact_root: Some(r(2)),
            distinct_root_count: 1,
        },
        kind: AlgebraicRootKind::ExactRationalWitness,
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    };
    let rational_three = AlgebraicRootRepresentation {
        constraint_index: 3,
        polynomial_coefficients: vec![r(-3), Real::one()],
        interval: IsolatedRootInterval {
            lower: r(3),
            upper: r(3),
            exact_root: Some(r(3)),
            distinct_root_count: 1,
        },
        ..rational_two.clone()
    };
    c.bench_function("arithmetic_algebraic_root_representations", |b| {
        b.iter(|| {
            arithmetic_algebraic_root_representations(
                &rational_two,
                Some(&rational_three),
                AlgebraicRootArithmeticOp::Multiply,
            )
        })
    });
    c.bench_function(
        "arithmetic_algebraic_root_representations_mixed_scalar",
        |b| {
            b.iter(|| {
                arithmetic_algebraic_root_representations(
                    &sqrt_two,
                    Some(&rational_three),
                    AlgebraicRootArithmeticOp::Add,
                )
            })
        },
    );
    c.bench_function("evaluate_polynomial_at_algebraic_root", |b| {
        b.iter(|| {
            evaluate_polynomial_at_algebraic_root(
                &sqrt_two,
                &[r(1), r(1), r(1)],
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("evaluate_rational_expression_at_algebraic_root", |b| {
        b.iter(|| {
            evaluate_rational_expression_at_algebraic_root(
                &sqrt_two,
                &[r(1), r(1)],
                &[r(2), r(1)],
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("transform_algebraic_root_affine", |b| {
        b.iter(|| {
            transform_algebraic_root_affine(
                &sqrt_two,
                r(2),
                r(3),
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("transform_algebraic_root_mobius", |b| {
        b.iter(|| {
            transform_algebraic_root_mobius(
                &sqrt_two,
                r(1),
                r(1),
                r(1),
                r(3),
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("transform_algebraic_root_polynomial_image", |b| {
        b.iter(|| {
            transform_algebraic_root_polynomial_image(
                &sqrt_two,
                &[r(0), r(1), r(1)],
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("transform_algebraic_root_rational_image", |b| {
        b.iter(|| {
            transform_algebraic_root_rational_image(
                &sqrt_two,
                &[r(0), r(1)],
                &[r(1), r(1)],
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("transform_algebraic_roots_binary", |b| {
        b.iter(|| {
            transform_algebraic_roots_binary(
                &sqrt_two,
                &sqrt_three,
                AlgebraicRootArithmeticOp::Add,
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("transform_algebraic_roots_binary_divide", |b| {
        b.iter(|| {
            transform_algebraic_roots_binary(
                &sqrt_two,
                &sqrt_three,
                AlgebraicRootArithmeticOp::Divide,
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("count_descartes_univariate_polynomial_roots", |b| {
        b.iter(|| {
            count_descartes_univariate_polynomial_roots(
                &prepared_quadratic,
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function(
        "count_bernstein_univariate_polynomial_interval_roots",
        |b| {
            b.iter(|| {
                count_bernstein_univariate_polynomial_interval_roots(
                    &prepared_quadratic,
                    r(0),
                    r(4),
                    hyperlimit::PredicatePolicy::default(),
                )
            })
        },
    );
    c.bench_function(
        "subdivide_bernstein_univariate_polynomial_interval_roots",
        |b| {
            b.iter(|| {
                subdivide_bernstein_univariate_polynomial_interval_roots(
                    &prepared_quadratic,
                    r(0),
                    r(4),
                    hypersolve::BernsteinSubdivisionConfig {
                        policy: hyperlimit::PredicatePolicy::default(),
                        max_depth: 8,
                    },
                )
            })
        },
    );
    c.bench_function("certify_quadratic_interval_rows", |b| {
        b.iter(|| {
            certify_quadratic_interval_candidate(
                &prepared_quadratic,
                &quadratic_context,
                &[VariableBall {
                    symbol: SymbolId(0),
                    radius: r(1),
                }],
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("certify_interval_box_candidate_report", |b| {
        b.iter(|| {
            certify_interval_box_candidate(
                &prepared_quadratic,
                &quadratic_context,
                &[VariableBall {
                    symbol: SymbolId(0),
                    radius: r(1),
                }],
                IntervalBoxCertificationPackage::UnivariateQuadratic,
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("certify_univariate_quadratic_alpha_rows", |b| {
        b.iter(|| {
            certify_univariate_quadratic_alpha(
                &prepared_quadratic,
                &quadratic_context,
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("certify_univariate_quadratic_krawczyk_rows", |b| {
        b.iter(|| {
            certify_univariate_quadratic_krawczyk_box(
                &prepared_quadratic,
                &quadratic_context,
                &[VariableBall {
                    symbol: SymbolId(0),
                    radius: r(0),
                }],
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("certify_multivariate_quadratic_interval_rows", |b| {
        b.iter(|| {
            certify_multivariate_quadratic_interval_candidate(
                &prepared_multivariate_quadratic,
                &multivariate_quadratic_context,
                &[
                    VariableBall {
                        symbol: SymbolId(0),
                        radius: r(1),
                    },
                    VariableBall {
                        symbol: SymbolId(1),
                        radius: r(1),
                    },
                ],
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("certify_multivariate_quadratic_krawczyk_rows", |b| {
        b.iter(|| {
            certify_multivariate_quadratic_krawczyk_box(
                &prepared_multivariate_quadratic,
                &multivariate_quadratic_context,
                &[
                    VariableBall {
                        symbol: SymbolId(0),
                        radius: r(0),
                    },
                    VariableBall {
                        symbol: SymbolId(1),
                        radius: r(0),
                    },
                ],
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("prepared_quadratic_candidate_replay", |b| {
        b.iter(|| {
            for row in 0..prepared_multivariate_quadratic.problem().constraints.len() {
                let _ = prepared_multivariate_quadratic
                    .evaluate_constraint_residual(row, &multivariate_quadratic_context);
            }
        })
    });
    c.bench_function("certify_affine_candidate_exact", |b| {
        b.iter(|| certify_candidate(&prepared, &context))
    });
    let batch_contexts = (0..16).map(|_| context.clone()).collect::<Vec<_>>();
    c.bench_function("certify_candidate_batch_affine", |b| {
        b.iter(|| certify_candidate_batch(&prepared, &batch_contexts))
    });
    c.bench_function("diagnose_failed_constraints_affine", |b| {
        b.iter(|| diagnose_failed_constraints(&prepared, &context))
    });
    let sketch_failed_problem = {
        let mut sketch = hypersolve::SketchSolveProblem::new();
        for index in 0..16 {
            let y = index as i64;
            let a = sketch.add_point2d(format!("failed{index}.a"), r(0), r(y));
            let b = sketch.add_point2d(format!("failed{index}.b"), r(3), r(y + 4));
            let distance = sketch.add_distance(format!("failed{index}.d"), r(4));
            sketch.add_point_point_distance(
                format!("failed sketch distance {index}"),
                a,
                b,
                distance,
            );
        }
        sketch
    };
    c.bench_function("diagnose_sketch_failed_constraints", |b| {
        b.iter(|| diagnose_sketch_failed_constraints(&sketch_failed_problem))
    });
    let failed_search_problem = {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", r(0));
        problem.add_constraint(Constraint::equality("x equals one", x - Expr::int(1)));
        problem
    };
    let failed_search_prepared = PreparedProblem::new(&failed_search_problem);
    let failed_search_context = context_from_problem(&failed_search_problem);
    c.bench_function("search_failed_constraint_single_removals", |b| {
        b.iter(|| {
            search_failed_constraint_single_removals(
                &failed_search_prepared,
                &failed_search_context,
            )
        })
    });
    let failed_pair_search_problem = {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", r(0));
        problem.add_constraint(Constraint::equality(
            "x equals one",
            x.clone() - Expr::int(1),
        ));
        problem.add_constraint(Constraint::equality("x equals two", x - Expr::int(2)));
        problem
    };
    let failed_pair_search_prepared = PreparedProblem::new(&failed_pair_search_problem);
    let failed_pair_search_context = context_from_problem(&failed_pair_search_problem);
    c.bench_function("search_failed_constraint_pair_removals", |b| {
        b.iter(|| {
            search_failed_constraint_pair_removals(
                &failed_pair_search_prepared,
                &failed_pair_search_context,
            )
        })
    });
    let failed_set_search_problem = {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", r(0));
        problem.add_constraint(Constraint::equality(
            "x equals one",
            x.clone() - Expr::int(1),
        ));
        problem.add_constraint(Constraint::equality(
            "x equals two",
            x.clone() - Expr::int(2),
        ));
        problem.add_constraint(Constraint::equality("x equals three", x - Expr::int(3)));
        problem
    };
    let failed_set_search_prepared = PreparedProblem::new(&failed_set_search_problem);
    let failed_set_search_context = context_from_problem(&failed_set_search_problem);
    c.bench_function("search_failed_constraint_set_removals", |b| {
        b.iter(|| {
            search_failed_constraint_set_removals(
                &failed_set_search_prepared,
                &failed_set_search_context,
                3,
            )
        })
    });
    c.bench_function("search_failed_constraint_minimal_removals", |b| {
        b.iter(|| {
            search_failed_constraint_minimal_removals(
                &failed_set_search_prepared,
                &failed_set_search_context,
                3,
            )
        })
    });
    c.bench_function("schedule_candidate_batch_predicates", |b| {
        b.iter(|| {
            schedule_candidate_batch_predicates(
                &prepared,
                16,
                BatchPredicateScheduleConfig {
                    max_rows_per_work_item: 4,
                },
            )
        })
    });
    c.bench_function("audit_active_set", |b| {
        b.iter(|| {
            audit_active_set(
                &prepared,
                &context,
                hypersolve::CandidateCertificationConfig::default(),
            )
        })
    });
    c.bench_function("propose_active_set_update", |b| {
        b.iter(|| {
            propose_active_set_update(
                &prepared,
                &context,
                hypersolve::CandidateCertificationConfig::default(),
            )
        })
    });
    let active_mask = prepared
        .problem()
        .constraints
        .iter()
        .map(|constraint| constraint.active)
        .collect::<Vec<_>>();
    c.bench_function("run_active_set_update_loop", |b| {
        b.iter(|| {
            run_active_set_update_loop(
                &prepared,
                &context,
                &active_mask,
                hypersolve::ActiveSetLoopConfig::default(),
            )
        })
    });
    let krawczyk_active_mask = krawczyk_prepared
        .problem()
        .constraints
        .iter()
        .map(|constraint| constraint.active)
        .collect::<Vec<_>>();
    c.bench_function("regenerate_active_set_affine_candidate", |b| {
        b.iter(|| {
            regenerate_active_set_affine_candidate(
                &krawczyk_prepared,
                &krawczyk_active_mask,
                hypersolve::CandidateCertificationConfig::default(),
            )
        })
    });
    let x = Expr::symbol(SymbolId(0), "x");
    let mut quadratic_regeneration_problem = Problem::default();
    quadratic_regeneration_problem.add_variable("x", r(0));
    quadratic_regeneration_problem.add_constraint(Constraint::equality(
        "bench active quadratic roots",
        x.clone() * x.clone() - Expr::int(4),
    ));
    let mut quadratic_bound = Constraint::equality("bench active quadratic bound", x);
    quadratic_bound.kind = hypersolve::ConstraintKind::GreaterOrEqual;
    quadratic_bound.active = false;
    quadratic_regeneration_problem.add_constraint(quadratic_bound);
    let quadratic_regeneration_prepared = PreparedProblem::new(&quadratic_regeneration_problem);
    c.bench_function("regenerate_active_set_quadratic_candidates", |b| {
        b.iter(|| {
            regenerate_active_set_quadratic_candidates(
                &quadratic_regeneration_prepared,
                &hypersolve::EvaluationContext::default(),
                &[true, false],
                hypersolve::ActiveSetQuadraticRegenerationConfig::default(),
            )
        })
    });
    c.bench_function("report_lossy_adapter_only_candidate", |b| {
        b.iter(|| {
            report_lossy_adapter_only_candidate(
                &prepared,
                ProposalEngineReport {
                    requested: ProposalEngineKind::DampedLeastSquares,
                    used: Some(ProposalEngineKind::DampedLeastSquares),
                    precision: ProposalEnginePrecision::LossyF64,
                    supported: true,
                },
            )
        })
    });
    c.bench_function("solve_levenberg_marquardt_affine", |b| {
        b.iter(|| {
            solve_damped_least_squares(SolverState {
                problem: affine_problem(4),
                config: SolverConfig {
                    max_iterations: 4,
                    proposal_engine: ProposalEngineKind::LevenbergMarquardt,
                    ..SolverConfig::default()
                },
            })
        })
    });
    c.bench_function("solve_modified_newton_least_squares_affine", |b| {
        b.iter(|| {
            solve_damped_least_squares(SolverState {
                problem: affine_problem(4),
                config: SolverConfig {
                    max_iterations: 4,
                    proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
                    ..SolverConfig::default()
                },
            })
        })
    });
    c.bench_function("solve_modified_newton_preprocessing", |b| {
        b.iter(|| {
            let x = hypersolve::Expr::symbol(hypersolve::SymbolId(0), "x");
            let y = hypersolve::Expr::symbol(hypersolve::SymbolId(1), "y");
            let z = hypersolve::Expr::symbol(hypersolve::SymbolId(2), "z");
            let mut problem = Problem::default();
            problem.add_variable("x", r(0));
            problem.add_variable("y", r(0));
            problem.add_variable("z", r(0));
            problem.add_constraint(hypersolve::Constraint::equality(
                "bench substitution",
                x - y.clone() - hypersolve::Expr::int(3),
            ));
            problem.add_constraint(hypersolve::Constraint::equality(
                "bench affine soluble alone",
                y - hypersolve::Expr::int(2),
            ));
            problem.add_constraint(hypersolve::Constraint::equality(
                "bench quadratic soluble alone",
                z.clone() * z - hypersolve::Expr::int(9),
            ));
            solve_damped_least_squares(SolverState {
                problem,
                config: SolverConfig {
                    max_iterations: 0,
                    proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
                    ..SolverConfig::default()
                },
            })
        })
    });
    c.bench_function("solve_modified_newton_affine_seed", |b| {
        b.iter(|| {
            let x = Expr::symbol(SymbolId(0), "x");
            let mut problem = Problem::default();
            problem.add_variable("x", r(0));
            problem.add_constraint(Constraint::equality("bench affine seed", x - Expr::int(7)));
            solve_damped_least_squares(SolverState {
                problem,
                config: SolverConfig {
                    max_iterations: 1,
                    proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
                    ..SolverConfig::default()
                },
            })
        })
    });
    c.bench_function("solve_modified_newton_substitution_seed", |b| {
        b.iter(|| {
            let x = Expr::symbol(SymbolId(0), "x");
            let y = Expr::symbol(SymbolId(1), "y");
            let mut problem = Problem::default();
            problem.add_variable("x", r(0));
            problem.add_variable("y", r(0));
            problem.add_constraint(Constraint::equality(
                "bench substitution seed",
                x - y.clone() - Expr::int(3),
            ));
            problem.add_constraint(Constraint::equality(
                "bench affine anchor",
                y - Expr::int(2),
            ));
            solve_damped_least_squares(SolverState {
                problem,
                config: SolverConfig {
                    max_iterations: 1,
                    proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
                    ..SolverConfig::default()
                },
            })
        })
    });
    c.bench_function("solve_modified_newton_bounded_substitution_seed", |b| {
        b.iter(|| {
            let x = Expr::symbol(SymbolId(0), "x");
            let y = Expr::symbol(SymbolId(1), "y");
            let mut problem = Problem::default();
            let x_id = problem.add_variable("x", r(100));
            problem.variables[x_id.0 as usize].upper = Some(r(4));
            let y_id = problem.add_variable("y", r(100));
            problem.variables[y_id.0 as usize].lower = Some(r(0));
            problem.add_constraint(Constraint::equality(
                "bench bounded substitution seed",
                x - y - Expr::int(3),
            ));
            solve_damped_least_squares(SolverState {
                problem,
                config: SolverConfig {
                    max_iterations: 1,
                    proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
                    ..SolverConfig::default()
                },
            })
        })
    });
    c.bench_function("solve_modified_newton_quadratic_seed", |b| {
        b.iter(|| {
            let x = Expr::symbol(SymbolId(0), "x");
            let mut problem = Problem::default();
            problem.add_variable("x", r(0));
            problem.add_constraint(Constraint::equality(
                "bench quadratic seed",
                x.clone() * x.clone() - Expr::int(6) * x + Expr::int(9),
            ));
            solve_damped_least_squares(SolverState {
                problem,
                config: SolverConfig {
                    max_iterations: 1,
                    proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
                    ..SolverConfig::default()
                },
            })
        })
    });
    c.bench_function("solve_modified_newton_bounded_quadratic_seed", |b| {
        b.iter(|| {
            let x = Expr::symbol(SymbolId(0), "x");
            let mut problem = Problem::default();
            let variable = problem.add_variable("x", r(0));
            problem.variables[variable.0 as usize].lower = Some(r(3));
            problem.add_constraint(Constraint::equality(
                "bench bounded quadratic seed",
                x.clone() * x.clone() - Expr::int(5) * x + Expr::int(6),
            ));
            solve_damped_least_squares(SolverState {
                problem,
                config: SolverConfig {
                    max_iterations: 1,
                    proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
                    ..SolverConfig::default()
                },
            })
        })
    });
    c.bench_function("solve_modified_newton_dragged_parameter", |b| {
        b.iter(|| {
            let mut problem = Problem::default();
            let x = problem.add_variable("x", r(0));
            solve_damped_least_squares(SolverState {
                problem,
                config: SolverConfig {
                    max_iterations: 4,
                    damping: r(0),
                    proposal_engine: ProposalEngineKind::ModifiedNewtonLeastSquares,
                    dragged_parameters: vec![DraggedParameterWeight {
                        variable: x,
                        target: r(10),
                        weight: r(1),
                    }],
                    ..SolverConfig::default()
                },
            })
        })
    });
    c.bench_function("solve_dogleg_affine", |b| {
        b.iter(|| {
            solve_damped_least_squares(SolverState {
                problem: affine_problem(4),
                config: SolverConfig {
                    max_iterations: 4,
                    damping: r(1),
                    proposal_engine: ProposalEngineKind::Dogleg,
                    ..SolverConfig::default()
                },
            })
        })
    });
    c.bench_function("solve_powell_hybrid_affine", |b| {
        b.iter(|| {
            solve_damped_least_squares(SolverState {
                problem: affine_problem(4),
                config: SolverConfig {
                    max_iterations: 4,
                    damping: r(1),
                    proposal_engine: ProposalEngineKind::PowellHybrid,
                    ..SolverConfig::default()
                },
            })
        })
    });
    c.bench_function("solve_bfgs_affine", |b| {
        b.iter(|| {
            solve_damped_least_squares(SolverState {
                problem: affine_problem(4),
                config: SolverConfig {
                    max_iterations: 4,
                    damping: r(1),
                    proposal_engine: ProposalEngineKind::Bfgs,
                    ..SolverConfig::default()
                },
            })
        })
    });
    c.bench_function("solve_sqp_affine", |b| {
        b.iter(|| {
            solve_damped_least_squares(SolverState {
                problem: affine_problem(4),
                config: SolverConfig {
                    max_iterations: 4,
                    damping: r(1),
                    proposal_engine: ProposalEngineKind::Sqp,
                    ..SolverConfig::default()
                },
            })
        })
    });
    let domain_problem = domain_problem(16);
    let domain_context = context_from_problem(&domain_problem);
    c.bench_function("certify_candidate_domains", |b| {
        b.iter(|| {
            certify_candidate_domains(
                &domain_problem,
                &domain_context,
                hyperlimit::PredicatePolicy::default(),
            )
        })
    });
    c.bench_function("simplify_unary_endpoint_expression", |b| {
        let expression = unary_endpoint_expression(16);
        b.iter(|| expression.clone().simplify())
    });

    let substitutions = (0..16)
        .map(|index| EqualitySubstitution {
            constraint_index: index,
            left: SymbolId(index as u32 + 1),
            right: SymbolId(index as u32),
            offset: r(index as i64),
        })
        .collect::<Vec<_>>();
    c.bench_function("build_equality_substitution_classes_exact", |b| {
        b.iter(|| build_equality_substitution_classes(&substitutions))
    });

    c.bench_function("domain_geometry_squared_distance_build", |b| {
        b.iter(|| {
            let mut problem = Problem::default();
            let ax = problem.add_variable("ax", r(0));
            let ay = problem.add_variable("ay", r(0));
            let bx = problem.add_variable("bx", r(3));
            let by = problem.add_variable("by", r(4));
            problem.add_constraint(squared_distance_equation(
                "3-4-5",
                SolverPoint2::new(ax, ay),
                SolverPoint2::new(bx, by),
                r(25),
            ));
            problem
        })
    });
}

criterion_group!(benches, certification);
criterion_main!(benches);
