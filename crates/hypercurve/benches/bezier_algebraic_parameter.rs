#[path = "../tests/support/mod.rs"]
mod support;
use std::cmp::Ordering;
use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Classification, CurveResult, Point2, PredicatePolicy,
    QuadraticBezier2, RationalQuadraticBezier2, Real,
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
    let policy = PredicatePolicy::STRICT;
    // Repeated exact comparisons must reuse a proved opaque cancellation.
    // The cold lane keeps the cost of constructing and proving a fresh value.
    let atom = (r(2).sqrt()? + Real::one()).sin();
    let normal_form_zero = || {
        let lower = &atom - Real::one();
        let upper = &atom + r(2);
        BezierParameter2::Exact(
            Real::diff_of_products(&Real::one(), &upper, &Real::one(), &lower) - r(3),
        )
    };
    let zero = BezierParameter2::Exact(Real::zero());
    let sign_iterations = 20_000_u32;
    for reuse in [false, true] {
        let retained = normal_form_zero();
        assert_eq!(
            decided(
                crate::support::under_classified_result(&policy, || {
                    retained.cmp_by_interval(&zero)
                })
                .expect("benchmark fixture remains exact")
            ),
            Ordering::Equal
        );
        let started = Instant::now();
        let mut equal = 0_u32;
        for _ in 0..sign_iterations {
            let parameter = if reuse {
                retained.clone()
            } else {
                normal_form_zero()
            };
            equal += (black_box(decided(
                crate::support::under_classified_result(&policy, || {
                    parameter.cmp_by_interval(&zero)
                })
                .expect("benchmark fixture remains exact"),
            )) == Ordering::Equal) as u32;
        }
        let elapsed = started.elapsed();
        assert_eq!(equal, sign_iterations);
        let label = if reuse { "reused" } else { "cold" };
        println!(
            "bezier_parameter_normal_form_zero_{label}: {sign_iterations} iterations in {elapsed:?} ({:?}/iter), equal={equal}",
            elapsed / sign_iterations,
        );
    }

    let bernstein_coefficients = (0..=32).map(|index| r((index % 7) - 3)).collect::<Vec<_>>();
    let conversion_iterations = 20_000_u32;
    let started = Instant::now();
    let mut converted_degree = 0_usize;
    for _ in 0..conversion_iterations {
        let polynomial = decided(
            crate::support::under_classified_result(&policy, || {
                BezierParameterPolynomial::try_new_bernstein_basis(black_box(
                    bernstein_coefficients.clone(),
                ))
            })
            .expect("benchmark fixture remains exact"),
        );
        converted_degree += black_box(polynomial.degree());
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_parameter_bernstein_32_to_power: {conversion_iterations} iterations in {elapsed:?} ({:?}/iter), degree_checksum={converted_degree}",
        elapsed / conversion_iterations
    );

    let polynomial = decided(
        crate::support::under_classified_result(&policy, || {
            BezierParameterPolynomial::try_new_power_basis(vec![q(1, 16), r(-1), r(1)])
        })
        .expect("benchmark fixture remains exact"),
    );
    let left = decided(
        crate::support::under_classified_result(&policy, || {
            BezierParameterInterval::try_new(r(0), q(1, 4))
        })
        .expect("benchmark fixture remains exact"),
    );
    let right = decided(
        crate::support::under_classified_result(&policy, || {
            BezierParameterInterval::try_new(q(3, 4), r(1))
        })
        .expect("benchmark fixture remains exact"),
    );

    let iterations = 20_000_u32;
    let started = Instant::now();
    let mut total = 0_usize;

    for _ in 0..iterations {
        let first = decided(
            crate::support::under_classified_result(&policy, || {
                BezierAlgebraicParameter2::try_isolate(polynomial.clone(), left.clone())
            })
            .expect("benchmark fixture remains exact"),
        );
        let second = decided(
            crate::support::under_classified_result(&policy, || {
                BezierAlgebraicParameter2::try_isolate(polynomial.clone(), right.clone())
            })
            .expect("benchmark fixture remains exact"),
        );
        total += black_box(first.root_count() + second.root_count());
    }

    let elapsed = started.elapsed();
    println!(
        "bezier_algebraic_parameter_sturm: {iterations} iterations in {elapsed:?} ({:?}/iter), total={total}",
        elapsed / iterations
    );

    let close_rational = BezierParameter2::Exact(q(353_553, 500_000));
    let refinement_iterations = 10_000_u32;
    for (label, coefficients) in [
        ("refined_ordering", vec![r(-1), r(0), r(2)]),
        (
            "refined_even_root_ordering",
            vec![r(1), r(0), r(-4), r(0), r(4)],
        ),
    ] {
        let irrational_polynomial = decided(
            crate::support::under_classified_result(&policy, || {
                BezierParameterPolynomial::try_new_power_basis(coefficients)
            })
            .expect("benchmark fixture remains exact"),
        );
        let irrational_interval = decided(
            crate::support::under_classified_result(&policy, || {
                BezierParameterInterval::try_new(q(2, 3), q(3, 4))
            })
            .expect("benchmark fixture remains exact"),
        );
        let irrational = BezierParameter2::Algebraic(decided(
            crate::support::under_classified_result(&policy, || {
                BezierAlgebraicParameter2::try_isolate(irrational_polynomial, irrational_interval)
            })
            .expect("benchmark fixture remains exact"),
        ));
        let started = Instant::now();
        let mut ordered = 0_usize;
        for _ in 0..refinement_iterations {
            ordered += black_box(
                decided(
                    crate::support::under_classified_result(&policy, || {
                        close_rational.cmp_by_refinement(&irrational)
                    })
                    .expect("benchmark fixture remains exact"),
                ) == Ordering::Less,
            ) as usize;
        }
        let elapsed = started.elapsed();
        println!(
            "bezier_algebraic_parameter_{label}: {refinement_iterations} iterations in {elapsed:?} ({:?}/iter), ordered={ordered}",
            elapsed / refinement_iterations
        );
    }

    // This is (B(t)-P) dot B'(t) for the cubic with controls
    // (0,0), (6,10), (-8,-8), (-4,10) and query point (-3,3), using the
    // stationary-distance quintic for point-to-cubic distance minimization.
    // It has five irrational roots inside (0,1), so every possible stationary
    // distance candidate is exercised without an exact-midpoint shortcut.
    let quintic = decided(
        crate::support::under_classified_result(&policy, || {
            BezierParameterPolynomial::try_new_power_basis(vec![
                r(-36),
                r(1368),
                r(-11034),
                r(31728),
                r(-38280),
                r(16620),
            ])
        })
        .expect("benchmark fixture remains exact"),
    );
    let quintic_trace = decided(
        crate::support::under_classified_result(&policy, || {
            quintic.isolate_unit_interval_roots_with_trace()
        })
        .expect("benchmark fixture remains exact"),
    );
    let isolation_iterations = 2_000_u32;
    let started = Instant::now();
    let mut isolated = 0_usize;
    for _ in 0..isolation_iterations {
        isolated += black_box(
            decided(
                crate::support::under_classified_result(&policy, || {
                    quintic.isolate_unit_interval_roots()
                })
                .expect("benchmark fixture remains exact"),
            )
            .len(),
        );
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

    let boundary_scale = Real::new(hyperreal::Rational::from_bigint(
        num::BigInt::from(3_u8) << 128_usize,
    ));
    let boundary_root = (Real::one() / &boundary_scale)?;
    let boundary = decided(
        crate::support::under_classified_result(&policy, || {
            BezierParameterPolynomial::try_new_power_basis(vec![-Real::one(), boundary_scale])
        })
        .expect("benchmark fixture remains exact"),
    );
    let boundary_trace = decided(
        crate::support::under_classified_result(&policy, || {
            boundary.isolate_unit_interval_roots_with_trace()
        })
        .expect("benchmark fixture remains exact"),
    );
    assert_eq!(
        boundary_trace.roots(),
        &[BezierParameter2::Exact(boundary_root)]
    );
    let boundary_iterations = 2_000_u32;
    let started = Instant::now();
    let mut boundary_isolated = 0_usize;
    for _ in 0..boundary_iterations {
        boundary_isolated += black_box(
            decided(
                crate::support::under_classified_result(&policy, || {
                    boundary.isolate_unit_interval_roots()
                })
                .expect("benchmark fixture remains exact"),
            )
            .len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_parameter_boundary_root_isolation: {boundary_iterations} iterations in {elapsed:?} ({:?}/iter), isolated={boundary_isolated}, interval_counts={}, bisections={}, max_depth={}",
        elapsed / boundary_iterations,
        boundary_trace.trace().interval_root_counts(),
        boundary_trace.trace().bisections(),
        boundary_trace.trace().maximum_depth(),
    );

    let rational_polynomial = decided(
        crate::support::under_classified_result(&policy, || {
            BezierParameterPolynomial::try_new_power_basis(vec![r(-1), r(3), r(-1), r(3)])
        })
        .expect("benchmark fixture remains exact"),
    );
    let rational_interval = decided(
        crate::support::under_classified_result(&policy, || {
            BezierParameterInterval::try_new(q(1, 4), q(1, 2))
        })
        .expect("benchmark fixture remains exact"),
    );
    let rational_parameter = decided(
        crate::support::under_classified_result(&policy, || {
            BezierAlgebraicParameter2::try_isolate(rational_polynomial, rational_interval)
        })
        .expect("benchmark fixture remains exact"),
    );
    let reconstruction_iterations = 500_000_u32;
    let started = Instant::now();
    let mut reconstructed = 0_usize;
    for _ in 0..reconstruction_iterations {
        reconstructed += black_box(
            decided(
                crate::support::under_classified_result(&policy, || {
                    rational_parameter.represented_exact_point()
                })
                .expect("benchmark fixture remains exact"),
            )
            .is_some() as usize,
        );
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_algebraic_parameter_exact_rational_reconstruction: {reconstruction_iterations} iterations in {elapsed:?} ({:?}/iter), reconstructed={reconstructed}",
        elapsed / reconstruction_iterations
    );

    let midpoint_polynomial = decided(
        crate::support::under_classified_result(&policy, || {
            BezierParameterPolynomial::try_new_power_basis(vec![r(-1), r(2)])
        })
        .expect("benchmark fixture remains exact"),
    );
    let midpoint_interval = decided(
        crate::support::under_classified_result(&policy, || {
            BezierParameterInterval::try_new(q(2, 5), q(3, 5))
        })
        .expect("benchmark fixture remains exact"),
    );
    let midpoint = decided(
        crate::support::under_classified_result(&policy, || {
            BezierAlgebraicParameter2::try_isolate(midpoint_polynomial, midpoint_interval)
        })
        .expect("benchmark fixture remains exact"),
    );
    let curve = QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::from_values(1, 3),
        Point2::from_values(4, 0),
    );

    let started = Instant::now();
    let mut transformed = 0_usize;
    let selected = hypercurve::CurveParameter2::from(BezierParameter2::Algebraic(midpoint.clone()));
    let general = hypercurve::Curve2::from(curve.clone());
    for _ in 0..iterations {
        let point = crate::support::under(&policy, || general.point_at(black_box(&selected)))
            .expect("the selected point remains exact")
            .into_value();
        let tangent =
            crate::support::under(&policy, || general.derivative_at(black_box(&selected)))
                .expect("the selected tangent remains exact")
                .into_value();
        transformed += black_box(
            point.coordinates().is_none() as usize
                + tangent.represented_coordinates().is_none() as usize,
        );
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
    let general_conic = hypercurve::Curve2::from(conic.clone());
    for _ in 0..iterations {
        let point = crate::support::under(&policy, || general_conic.point_at(black_box(&selected)))
            .expect("the selected conic point remains exact")
            .into_value();
        let tangent = crate::support::under(&policy, || {
            general_conic.derivative_at(black_box(&selected))
        })
        .expect("the selected conic tangent remains exact")
        .into_value();
        rational_transformed += black_box(
            point.coordinates().is_none() as usize
                + tangent.represented_coordinates().is_none() as usize,
        );
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_algebraic_point_tangent_image: {iterations} iterations in {elapsed:?} ({:?}/iter), transformed={rational_transformed}",
        elapsed / iterations
    );

    Ok(())
}
