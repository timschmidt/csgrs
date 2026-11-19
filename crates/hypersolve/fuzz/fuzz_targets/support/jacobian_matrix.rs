//! Source-bound projection with independent affine derivative ranges.
use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionError, ExpressionProgram},
    work_budget::CancellationToken,
};

fn graph() -> ExpressionBudget {
    ExpressionBudget::new(100_000, 1_000_000)
}
fn arithmetic() -> DyadicBudget {
    DyadicBudget::new(4096, 1_000_000)
}
fn interval(lo: i64, hi: i64) -> DyadicInterval {
    DyadicInterval::new(lo.into(), hi.into(), 0).unwrap()
}
fn product(k: i64, lo: i64, hi: i64) -> DyadicInterval {
    interval((k * lo).min(k * hi), (k * lo).max(k * hi))
}

pub fn check(data: &[u8]) {
    let byte = |i| data.get(i).copied().unwrap_or(0);
    let coefficient = |i| i64::from(byte(i) as i8);
    let (a, b, c) = (coefficient(0), coefficient(1), coefficient(2));
    let xlo = coefficient(3);
    let xhi = xlo + i64::from(byte(4));
    let ylo = coefficient(5);
    let yhi = ylo + i64::from(byte(6));
    let empty = byte(7) & 1 != 0;
    let reversed = byte(7) & 2 != 0;
    let x = || Expr::anonymous_symbol(SymbolId(0));
    let y = || Expr::anonymous_symbol(SymbolId(1));
    let source = ExpressionProgram::compile(
        &[
            Expr::int(a) * x().powi(2) + Expr::int(b) * y(),
            Expr::int(c) * x() * y() + Expr::int(3),
        ],
        &[SymbolId(1), SymbolId(0)],
        &mut graph(),
    )
    .unwrap();
    let variables = if reversed {
        [SymbolId(1), SymbolId(0)]
    } else {
        [SymbolId(0), SymbolId(1)]
    };
    let arguments = [
        if empty {
            DyadicInterval::empty()
        } else {
            interval(ylo, yhi)
        },
        interval(xlo, xhi),
    ];
    let evidence = source
        .jacobian(&variables, &mut graph())
        .unwrap()
        .enclose_regular(&arguments, -32, &mut graph(), &mut arithmetic())
        .unwrap();
    let projection = evidence
        .matrix::<2, 2>(&mut graph(), &mut arithmetic())
        .unwrap();
    assert_eq!(
        projection.evidence().program().source().definition(),
        source.definition()
    );
    assert_eq!(projection.evidence().arguments(), &arguments);
    if empty {
        assert!(projection.intervals().is_empty());
    } else {
        let mut expected = [
            [product(2 * a, xlo, xhi), interval(b, b)],
            [product(c, ylo, yhi), product(c, xlo, xhi)],
        ];
        if reversed {
            for row in &mut expected {
                row.swap(0, 1);
            }
        }
        assert_eq!(projection.intervals().rows(), Some(&expected));
        for row in 0..2 {
            for column in 0..2 {
                let index = evidence.program().partial_output(row, column).unwrap();
                assert_eq!(
                    projection.intervals().rows().unwrap()[row][column].bounds(),
                    evidence.values()[index].bounds()
                );
            }
        }
    }
    assert!(matches!(
        evidence.matrix::<2, 1>(&mut graph(), &mut arithmetic()),
        Err(ExpressionError::JacobianColumns {
            expected: 1,
            actual: 2
        })
    ));
    let mut small_graph = ExpressionBudget::new(100_000, usize::from(byte(8) % 24));
    let mut small_arithmetic = DyadicBudget::new(4096, u64::from(byte(9) % 64));
    match evidence.matrix::<2, 2>(&mut small_graph, &mut small_arithmetic) {
        Ok(value) => assert_eq!(value.intervals(), projection.intervals()),
        Err(
            ExpressionError::WorkExhausted
            | ExpressionError::Enclosure {
                source: IntervalError::WorkExhausted,
                ..
            },
        ) => {}
        Err(error) => panic!("unexpected failure: {error}"),
    }
    let token = CancellationToken::new();
    token.cancel();
    let mut cancelled = graph();
    cancelled.cancellation = Some(token);
    assert!(matches!(
        evidence.matrix::<2, 2>(&mut cancelled, &mut arithmetic()),
        Err(ExpressionError::Cancelled)
    ));
    assert_eq!(
        evidence
            .matrix::<2, 2>(&mut graph(), &mut arithmetic())
            .unwrap()
            .intervals(),
        projection.intervals()
    );
}
