#![cfg(feature = "taylor-evolution")]
//! Opt-in Taylor-mode ODE recurrence coefficients for trajectory Taylor steps.
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    evolution::{EvolutionOptions, EvolutionRequest},
    taylor_flow::{FlowTaylorCoefficients, evolution::FlowTaylorEvolutionStatus},
};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};
const T: SymbolId = SymbolId(10);
const X: SymbolId = SymbolId(20);
const Y: SymbolId = SymbolId(21);
fn d(l: i64, u: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(l.into(), u.into(), e).unwrap()
}
fn a() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000)
}
fn budget() -> EnclosureBudget {
    EnclosureBudget::new(EnclosurePolicy::default())
}
fn field<const N: usize>(outputs: &[Expr], states: &[SymbolId]) -> ExpressionField<N> {
    // Time is bound last, so slots and state lanes are not the same order.
    let mut bindings = states.to_vec();
    bindings.push(T);
    let source = ExpressionProgram::compile(
        outputs,
        &bindings,
        &mut ExpressionBudget::new(1_000_000, 100_000_000),
    )
    .unwrap();
    ExpressionField::from_definition(
        FieldDefinition {
            source: source.definition().clone(),
            time: T,
            state: states.to_vec(),
        },
        &mut budget(),
    )
    .unwrap()
}
fn s(id: SymbolId) -> Expr {
    Expr::anonymous_symbol(id)
}

#[derive(PartialEq)]
enum Widths {
    // Polynomial fields: the same enclosure up to grid rounding.
    Rounding,
    // Strictly narrower endpoint lanes from the recurrence.
    Narrower,
}
// Both methods certify the same horizon. Every recurrence coefficient meets the
// formula coefficient of the same step (both contain the exact derivative).
fn agree<const N: usize>(
    f: ExpressionField<N>,
    initial: BoxN<DyadicInterval, N>,
    order: usize,
    widths: Widths,
) {
    let prepared = f.flow_taylor(order, &mut budget()).unwrap();
    assert_eq!(prepared.coefficients(), FlowTaylorCoefficients::Recurrence);
    let formulas = prepared.with_coefficients(FlowTaylorCoefficients::Formulas);
    assert_eq!(formulas.coefficients(), FlowTaylorCoefficients::Formulas);
    let recurrence = formulas
        .clone()
        .with_coefficients(FlowTaylorCoefficients::Recurrence);
    assert_eq!(
        recurrence.coefficients(),
        FlowTaylorCoefficients::Recurrence
    );
    let restricted = recurrence.restrict_order(order - 1, &mut budget()).unwrap();
    assert_eq!(
        restricted.coefficients(),
        FlowTaylorCoefficients::Recurrence
    );
    let run = |program: &hyperdynamics::field::taylor_flow::FlowTaylorProgram<N>| {
        program.integrate(
            EvolutionRequest {
                start: d(0, 0, 0),
                end: d(1, 1, -2),
                initial: initial.clone(),
            },
            EvolutionOptions {
                maximum_step: Some(d(1, 1, -4)),
                ..Default::default()
            },
            &mut budget(),
        )
    };
    let (old, new) = (run(&formulas), run(&recurrence));
    assert!(
        matches!(old.status(), FlowTaylorEvolutionStatus::Completed),
        "{:?}",
        old.status()
    );
    assert!(matches!(new.status(), FlowTaylorEvolutionStatus::Completed));
    assert_eq!(old.segments().len(), new.segments().len());
    for (old, new) in old.segments().zip(new.segments()) {
        let (old, new) = (old.evidence(), new.evidence());
        assert_eq!(old.coefficients().len(), new.coefficients().len());
        let pairs = old
            .coefficients()
            .iter()
            .zip(new.coefficients())
            .chain([(old.remainder().unwrap(), new.remainder().unwrap())]);
        for (old, new) in pairs {
            for (old, new) in old.lanes().unwrap().iter().zip(new.lanes().unwrap()) {
                assert!(!old.intersection(new, &mut a()).unwrap().is_empty());
            }
        }
    }
    let (old, new) = (old.end_enclosure().unwrap(), new.end_enclosure().unwrap());
    for (old, new) in old.lanes().unwrap().iter().zip(new.lanes().unwrap()) {
        assert!(!old.intersection(new, &mut a()).unwrap().is_empty());
        // |width(new) - width(old)| <= 2^-30 width(old) + 2^-60.
        let (wo, wn) = (old.width(&mut a()).unwrap(), new.width(&mut a()).unwrap());
        let slack = wo
            .mul(&d(1, 1, -30), &mut a())
            .unwrap()
            .add(&d(1, 1, -60), &mut a())
            .unwrap();
        let difference = wn.sub(&wo, &mut a()).unwrap();
        let bound = slack.neg(&mut a()).unwrap().hull(&slack, &mut a()).unwrap();
        if widths == Widths::Rounding {
            assert!(
                difference.subset_of(&bound, &mut a()).unwrap(),
                "{wo:?} {wn:?}"
            );
        } else {
            assert_eq!(
                wn.compare_singleton(&wo, &mut a()).unwrap(),
                std::cmp::Ordering::Less,
                "{wo:?} {wn:?}"
            );
        }
    }
}

#[test]
fn recurrence_coefficients_certify_the_same_horizons_as_stage_formulas() {
    // Rotation from an uncertain box, a Riccati blow-up direction, a
    // time-dependent growth and a transcendental, guarded right-hand side.
    agree(
        field::<2>(&[s(Y), -s(X)], &[X, Y]),
        BoxN::new([d(1023, 1025, -10), d(-1, 1, -10)]).unwrap(),
        3,
        Widths::Rounding,
    );
    agree(
        field::<1>(&[s(X) * s(X)], &[X]),
        BoxN::new([d(1, 1, 0)]).unwrap(),
        4,
        Widths::Rounding,
    );
    agree(
        field::<1>(&[s(T) * s(X)], &[X]),
        BoxN::new([d(1023, 1025, -10)]).unwrap(),
        3,
        Widths::Rounding,
    );
    agree(
        field::<1>(&[s(X).sin() + (Expr::int(2) + s(T)).ln() / s(X)], &[X]),
        BoxN::new([d(1023, 1025, -10)]).unwrap(),
        3,
        // The quotient and transcendental recurrences avoid the dependency of
        // the expanded Lie-derivative formulas (about 5% narrower here).
        Widths::Narrower,
    );
}
