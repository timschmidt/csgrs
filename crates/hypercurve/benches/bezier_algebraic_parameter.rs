use std::cmp::Ordering;
use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Classification, CurvePolicy, CurveResult, Point2, QuadraticBezier2,
    RationalQuadraticBezier2, Real,
};

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("benchmark setup became uncertain: {reason:?}"),
    }
}

fn main() -> CurveResult<()> {
    let policy = CurvePolicy::certified();
    let bernstein_coefficients = (0..=32).map(|index| r((index % 7) - 3)).collect::<Vec<_>>();
    let conversion_iterations = 20_000_u32;
    let started = Instant::now();
    let mut converted_degree = 0_usize;
    for _ in 0..conversion_iterations {
        let polynomial = decided(BezierParameterPolynomial::try_new_bernstein_basis(
            black_box(bernstein_coefficients.clone()),
            &policy,
        )?);
        converted_degree += black_box(polynomial.degree());
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_parameter_bernstein_32_to_power: {conversion_iterations} iterations in {elapsed:?} ({:?}/iter), degree_checksum={converted_degree}",
        elapsed / conversion_iterations
    );

    let polynomial = decided(BezierParameterPolynomial::try_new_power_basis(
        vec![q(1, 16), r(-1), r(1)],
        &policy,
    )?);
    let left = decided(BezierParameterInterval::try_new(r(0), q(1, 4), &policy)?);
    let right = decided(BezierParameterInterval::try_new(q(3, 4), r(1), &policy)?);

    let iterations = 20_000_u32;
    let started = Instant::now();
    let mut total = 0_usize;

    for _ in 0..iterations {
        let first = decided(BezierAlgebraicParameter2::try_isolate(
            polynomial.clone(),
            left.clone(),
            &policy,
        )?);
        let second = decided(BezierAlgebraicParameter2::try_isolate(
            polynomial.clone(),
            right.clone(),
            &policy,
        )?);
        total += black_box(first.root_count() + second.root_count());
    }

    let elapsed = started.elapsed();
    println!(
        "bezier_algebraic_parameter_sturm: {iterations} iterations in {elapsed:?} ({:?}/iter), total={total}",
        elapsed / iterations
    );

    let irrational_polynomial = decided(BezierParameterPolynomial::try_new_power_basis(
        vec![r(-1), r(0), r(2)],
        &policy,
    )?);
    let irrational_interval = decided(BezierParameterInterval::try_new(q(2, 3), q(3, 4), &policy)?);
    let irrational = BezierParameter2::algebraic(decided(BezierAlgebraicParameter2::try_isolate(
        irrational_polynomial,
        irrational_interval,
        &policy,
    )?));
    let close_rational = decided(BezierParameter2::exact(q(353_553, 500_000), &policy)?);
    let refinement_iterations = 10_000_u32;
    let started = Instant::now();
    let mut ordered = 0_usize;
    for _ in 0..refinement_iterations {
        ordered += black_box(
            decided(close_rational.cmp_by_refinement(&irrational, &policy)?) == Ordering::Less,
        ) as usize;
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_algebraic_parameter_refined_ordering: {refinement_iterations} iterations in {elapsed:?} ({:?}/iter), ordered={ordered}",
        elapsed / refinement_iterations
    );

    // This is (B(t)-P) dot B'(t) for the cubic with controls
    // (0,0), (6,10), (-8,-8), (-4,10) and query point (-3,3), using the
    // stationary-distance quintic for point-to-cubic distance minimization.
    // It has five irrational roots inside (0,1), so every possible stationary
    // distance candidate is exercised without an exact-midpoint shortcut.
    let quintic = decided(BezierParameterPolynomial::try_new_power_basis(
        vec![r(-36), r(1368), r(-11034), r(31728), r(-38280), r(16620)],
        &policy,
    )?);
    let quintic_trace = decided(quintic.isolate_unit_interval_roots_with_trace(&policy)?);
    let isolation_iterations = 2_000_u32;
    let started = Instant::now();
    let mut isolated = 0_usize;
    for _ in 0..isolation_iterations {
        isolated += black_box(decided(quintic.isolate_unit_interval_roots(&policy)?).len());
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_parameter_quintic_unit_isolation: {isolation_iterations} iterations in {elapsed:?} ({:?}/iter), isolated={isolated}, sturm_builds={}, interval_counts={}, bisections={}, rational_refinements={}, max_depth={}",
        elapsed / isolation_iterations,
        quintic_trace.trace().sturm_sequence_builds(),
        quintic_trace.trace().interval_root_counts(),
        quintic_trace.trace().bisections(),
        quintic_trace.trace().rational_reconstruction_refinements(),
        quintic_trace.trace().maximum_depth(),
    );

    let rational_polynomial = decided(BezierParameterPolynomial::try_new_power_basis(
        vec![r(-1), r(3), r(-1), r(3)],
        &policy,
    )?);
    let rational_interval = decided(BezierParameterInterval::try_new(q(1, 4), q(1, 2), &policy)?);
    let rational_parameter = decided(BezierAlgebraicParameter2::try_isolate(
        rational_polynomial,
        rational_interval,
        &policy,
    )?);
    let reconstruction_iterations = 5_000_u32;
    let started = Instant::now();
    let mut reconstructed = 0_usize;
    for _ in 0..reconstruction_iterations {
        reconstructed += black_box(
            decided(rational_parameter.represented_rational_root(&policy)?).is_some() as usize,
        );
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_algebraic_parameter_exact_rational_reconstruction: {reconstruction_iterations} iterations in {elapsed:?} ({:?}/iter), reconstructed={reconstructed}",
        elapsed / reconstruction_iterations
    );

    let midpoint_polynomial = decided(BezierParameterPolynomial::try_new_power_basis(
        vec![r(-1), r(2)],
        &policy,
    )?);
    let midpoint_interval = decided(BezierParameterInterval::try_new(q(2, 5), q(3, 5), &policy)?);
    let midpoint = decided(BezierAlgebraicParameter2::try_isolate(
        midpoint_polynomial,
        midpoint_interval,
        &policy,
    )?);
    let curve = QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::from_values(1, 3),
        Point2::from_values(4, 0),
    );

    let started = Instant::now();
    let mut transformed = 0_usize;
    for _ in 0..iterations {
        let point = curve.point_at_algebraic_parameter(&midpoint, &policy)?;
        let tangent = curve.tangent_at_algebraic_parameter(&midpoint, &policy)?;
        transformed += black_box(point.x().is_some() as usize + tangent.dx().is_some() as usize);
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_algebraic_point_tangent_image: {iterations} iterations in {elapsed:?} ({:?}/iter), transformed={transformed}",
        elapsed / iterations
    );

    let conic = RationalQuadraticBezier2::try_new(
        Point2::from_values(0, 0),
        Point2::from_values(2, 4),
        Point2::from_values(6, 0),
        r(1),
        r(2),
        r(3),
    )?;
    let started = Instant::now();
    let mut rational_transformed = 0_usize;
    for _ in 0..iterations {
        let point = conic.point_at_algebraic_parameter(&midpoint, &policy)?;
        let tangent = conic.tangent_at_algebraic_parameter(&midpoint, &policy)?;
        rational_transformed +=
            black_box(point.x().is_some() as usize + tangent.dx().is_some() as usize);
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_algebraic_point_tangent_image: {iterations} iterations in {elapsed:?} ({:?}/iter), transformed={rational_transformed}",
        elapsed / iterations
    );

    Ok(())
}
