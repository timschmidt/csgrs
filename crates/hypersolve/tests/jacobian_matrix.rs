#![cfg(feature = "interval-lattice")]
use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};
use hyperlattice::{IntervalMatrix, IntervalVector};
use hypersolve::{
    Expr, SymbolId,
    expression_program::{
        ExpressionBudget, ExpressionError, ExpressionProgram, RegularJacobianEnclosure,
    },
    work_budget::CancellationToken,
};

fn graph() -> ExpressionBudget {
    ExpressionBudget::new(100_000, 10_000_000)
}
fn arithmetic() -> DyadicBudget {
    DyadicBudget::new(4096, 10_000_000)
}
fn d(a: i64, b: i64) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), 0).unwrap()
}
fn source() -> ExpressionProgram {
    let x = || Expr::anonymous_symbol(SymbolId(0));
    let y = || Expr::anonymous_symbol(SymbolId(1));
    ExpressionProgram::compile(
        &[
            x().powi(2) + Expr::int(3) * y(),
            x() * y(),
            y().powi(2) - x(),
        ],
        &[SymbolId(0), SymbolId(1)],
        &mut graph(),
    )
    .unwrap()
}
fn evidence(arguments: &[DyadicInterval]) -> RegularJacobianEnclosure {
    source()
        .jacobian(&[SymbolId(1), SymbolId(0)], &mut graph())
        .unwrap()
        .enclose_regular(arguments, -16, &mut graph(), &mut arithmetic())
        .unwrap()
}

#[test]
fn rectangular_projection_retains_source_and_has_independent_derivatives() {
    for x in -4..=4 {
        for y in -4..=4 {
            let evidence = evidence(&[d(x, x), d(y, y)]);
            let projection = evidence
                .matrix::<3, 2>(&mut graph(), &mut arithmetic())
                .unwrap();
            assert!(core::ptr::eq(projection.evidence(), &evidence));
            assert_eq!(
                projection.evidence().program().source().definition(),
                source().definition()
            );
            assert_eq!(
                projection.evidence().program().variables(),
                &[SymbolId(1), SymbolId(0)]
            );
            assert_eq!(projection.evidence().arguments(), &[d(x, x), d(y, y)]);
            assert_eq!(projection.evidence().precision(), -16);
            let expected = [[3, 2 * x], [x, y], [2 * y, -1]].map(|row| row.map(|v| d(v, v)));
            assert_eq!(projection.intervals().rows(), Some(&expected));
            let vector = IntervalVector::new([d(2, 2), d(-1, -1)]).unwrap();
            let image = projection
                .intervals()
                .mul_vector(&vector, &mut arithmetic())
                .unwrap();
            assert_eq!(
                image.lanes(),
                Some(&[
                    d(6 - 2 * x, 6 - 2 * x),
                    d(2 * x - y, 2 * x - y),
                    d(4 * y + 1, 4 * y + 1)
                ])
            );
            assert_eq!(projection.into_intervals().rows(), Some(&expected));
        }
    }
}

#[test]
fn box_projection_composes_with_products_and_norms() {
    let evidence = evidence(&[d(-2, 4), d(-3, 2)]);
    let projection = evidence
        .matrix::<3, 2>(&mut graph(), &mut arithmetic())
        .unwrap();
    let matrix = projection.intervals();
    assert_eq!(
        matrix.rows(),
        Some(&[
            [d(3, 3), d(-4, 8)],
            [d(-2, 4), d(-3, 2)],
            [d(-6, 4), d(-1, -1)]
        ])
    );
    assert_eq!(matrix.norm_one(&mut arithmetic()).unwrap(), d(3, 13));
    assert_eq!(matrix.norm_infinity(&mut arithmetic()).unwrap(), d(3, 11));
    let vector = IntervalVector::new([d(2, 2), d(-1, -1)]).unwrap();
    assert_eq!(
        matrix
            .mul_vector(&vector, &mut arithmetic())
            .unwrap()
            .lanes(),
        Some(&[d(-2, 10), d(-6, 11), d(-11, 9)])
    );
    let swap = IntervalMatrix::new([[d(0, 0), d(1, 1)], [d(1, 1), d(0, 0)]]).unwrap();
    let switched = matrix.mul_matrix(&swap, &mut arithmetic()).unwrap();
    assert_eq!(
        switched.rows(),
        Some(&[
            [d(-4, 8), d(3, 3)],
            [d(-3, 2), d(-2, 4)],
            [d(-1, -1), d(-6, 4)]
        ])
    );
}

#[test]
fn shapes_empty_domains_and_unit_products_remain_distinct() {
    let evidence = evidence(&[DyadicInterval::empty(), d(0, 1)]);
    assert!(
        evidence
            .matrix::<3, 2>(&mut graph(), &mut arithmetic())
            .unwrap()
            .intervals()
            .is_empty()
    );
    assert!(matches!(
        evidence.matrix::<2, 3>(&mut graph(), &mut arithmetic()),
        Err(ExpressionError::JacobianRows {
            expected: 2,
            actual: 3
        })
    ));
    assert!(matches!(
        evidence.matrix::<3, 1>(&mut graph(), &mut arithmetic()),
        Err(ExpressionError::JacobianColumns {
            expected: 1,
            actual: 2
        })
    ));
    fn zero<const R: usize, const C: usize>(
        outputs: &[Expr],
        selected: &[SymbolId],
        arguments: &[DyadicInterval],
        empty: bool,
    ) {
        let bindings = if arguments.is_empty() {
            vec![]
        } else {
            vec![SymbolId(0)]
        };
        let source = ExpressionProgram::compile(outputs, &bindings, &mut graph()).unwrap();
        let evidence = source
            .jacobian(selected, &mut graph())
            .unwrap()
            .enclose_regular(arguments, -16, &mut graph(), &mut arithmetic())
            .unwrap();
        let projection = evidence
            .matrix::<R, C>(&mut graph(), &mut arithmetic())
            .unwrap();
        assert_eq!(projection.intervals().is_empty(), empty);
        assert_eq!(projection.intervals().is_singleton(), !empty);
        let rhs =
            IntervalVector::<DyadicInterval, C>::new(core::array::from_fn(|_| d(0, 0))).unwrap();
        assert_eq!(
            projection
                .intervals()
                .mul_vector(&rhs, &mut arithmetic())
                .unwrap()
                .is_empty(),
            empty
        );
    }
    for empty in [false, true] {
        let args = [if empty {
            DyadicInterval::empty()
        } else {
            d(-1, 1)
        }];
        zero::<0, 1>(&[], &[SymbolId(0)], &args, empty);
        zero::<1, 0>(&[Expr::int(7)], &[], &args, empty);
        zero::<0, 0>(&[], &[], &args, empty);
    }
    zero::<0, 0>(&[], &[], &[], false);
    zero::<1, 0>(&[Expr::int(7)], &[], &[], false);
}

#[test]
fn resource_failure_and_cancellation_preserve_source_and_retry() {
    let evidence = evidence(&[d(-2, 4), d(-3, 2)]);
    let before = evidence.values().to_vec();
    assert!(matches!(
        evidence.matrix::<3, 2>(&mut ExpressionBudget::new(0, 1000), &mut arithmetic()),
        Err(ExpressionError::SizeExhausted)
    ));
    assert!(matches!(
        evidence.matrix::<3, 2>(&mut ExpressionBudget::new(1000, 0), &mut arithmetic()),
        Err(ExpressionError::WorkExhausted)
    ));
    let error = evidence
        .matrix::<3, 2>(&mut graph(), &mut DyadicBudget::new(4096, 0))
        .unwrap_err();
    assert!(matches!(
        error,
        ExpressionError::Enclosure {
            source: IntervalError::WorkExhausted,
            ..
        }
    ));
    assert!(std::error::Error::source(&error).is_some());
    assert!(matches!(
        evidence.matrix::<3, 2>(&mut graph(), &mut DyadicBudget::new(0, 1000000)),
        Err(ExpressionError::Enclosure {
            source: IntervalError::SizeExhausted,
            ..
        })
    ));
    let token = CancellationToken::new();
    let mut cancelled = graph();
    cancelled.cancellation = Some(token.clone());
    token.cancel();
    assert!(matches!(
        evidence.matrix::<2, 3>(&mut cancelled, &mut arithmetic()),
        Err(ExpressionError::Cancelled)
    ));
    let mut budget = graph();
    let initial = budget.remaining_work();
    let expected = evidence
        .matrix::<3, 2>(&mut budget, &mut arithmetic())
        .unwrap()
        .into_intervals();
    let work = initial - budget.remaining_work();
    assert_eq!(work, 2 + 2 * (3 + 6) + 6);
    for limit in 0..work {
        assert!(matches!(
            evidence.matrix::<3, 2>(&mut ExpressionBudget::new(1000, limit), &mut arithmetic()),
            Err(ExpressionError::WorkExhausted)
        ));
    }
    assert_eq!(
        evidence
            .matrix::<3, 2>(&mut ExpressionBudget::new(1000, work), &mut arithmetic())
            .unwrap()
            .into_intervals(),
        expected
    );
    assert_eq!(evidence.values(), before);
}

#[test]
fn native_replay_rebuilds_source_association_before_projection() {
    let original = source();
    let replay =
        ExpressionProgram::from_definition(original.definition().clone(), &mut graph()).unwrap();
    let jacobian = replay
        .jacobian(&[SymbolId(1), SymbolId(0)], &mut graph())
        .unwrap();
    let evidence = jacobian
        .enclose_regular(&[d(-2, 4), d(-3, 2)], -16, &mut graph(), &mut arithmetic())
        .unwrap();
    let projection = evidence
        .matrix::<3, 2>(&mut graph(), &mut arithmetic())
        .unwrap();
    assert_eq!(
        projection.evidence().program().source().definition(),
        original.definition()
    );
    assert_eq!(
        projection.intervals().rows().unwrap()[1],
        [d(-2, 4), d(-3, 2)]
    );
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn native_projection_adds_no_scalar_predicates_or_rational_reductions() {
    use hyperreal::dispatch_trace;
    let evidence = evidence(&[d(-2, 4), d(-3, 2)]);
    for _ in 0..2 {
        dispatch_trace::reset();
        let projection = dispatch_trace::with_recording(|| {
            evidence
                .matrix::<3, 2>(&mut graph(), &mut arithmetic())
                .unwrap()
        });
        let trace = dispatch_trace::take_trace();
        assert!(trace.dispatch.is_empty(), "{trace:?}");
        assert_eq!(trace.rational, Default::default());
        for row in 0..3 {
            for column in 0..2 {
                let index = evidence.program().partial_output(row, column).unwrap();
                assert_eq!(
                    projection.intervals().rows().unwrap()[row][column].bounds(),
                    evidence.values()[index].bounds()
                );
            }
        }
    }
}

#[test]
fn nonselected_parameter_domains_survive_native_projection() {
    let x = Expr::anonymous_symbol(SymbolId(0));
    let time = Expr::anonymous_symbol(SymbolId(1));
    // State partials are respectively t^2 and sqrt(t). The latter is continuous
    // at t=0 and smooth in x, without claiming differentiability in time.
    for (coefficient, expected) in [(time.clone().powi(2), d(0, 16)), (time.sqrt(), d(0, 2))] {
        let source = ExpressionProgram::compile(
            &[x.clone() * coefficient],
            &[SymbolId(1), SymbolId(0)],
            &mut graph(),
        )
        .unwrap();
        let arguments = [d(0, 4), d(-2, 3)];
        let evidence = source
            .jacobian(&[SymbolId(0)], &mut graph())
            .unwrap()
            .enclose_regular(&arguments, -16, &mut graph(), &mut arithmetic())
            .unwrap();
        let projection = evidence
            .matrix::<1, 1>(&mut graph(), &mut arithmetic())
            .unwrap();
        assert_eq!(projection.evidence().arguments(), &arguments);
        assert_eq!(projection.evidence().program().variables(), &[SymbolId(0)]);
        assert_eq!(projection.intervals().rows(), Some(&[[expected.clone()]]));
        assert_eq!(
            projection
                .intervals()
                .norm_infinity(&mut arithmetic())
                .unwrap(),
            expected
        );
        assert!(matches!(
            evidence.matrix::<1, 2>(&mut graph(), &mut arithmetic()),
            Err(ExpressionError::JacobianColumns {
                expected: 2,
                actual: 1
            })
        ));
    }
}
