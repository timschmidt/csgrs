//! Complete fresh preparation, integration, local remainder checks and proof destruction.
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    evolution::{EvolutionOptions, EvolutionRequest},
    taylor_flow::{
        adaptive::{AdaptiveTaylorOptions, AdaptiveTaylorStatus},
        evolution::FlowTaylorEvolutionStatus,
    },
};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};
use std::{hint::black_box, time::Instant};
fn d(l: i64, u: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(l.into(), u.into(), e).unwrap()
}
fn budget() -> EnclosureBudget {
    EnclosureBudget::new(EnclosurePolicy::default())
}
fn field(exponential: bool) -> ExpressionField<1> {
    let t = SymbolId(0);
    let x = SymbolId(1);
    let source = ExpressionProgram::compile(
        &[if exponential {
            Expr::anonymous_symbol(x)
        } else {
            Expr::int(2)
        }],
        &[t, x],
        &mut ExpressionBudget::new(1_000_000, 100_000_000),
    )
    .unwrap();
    ExpressionField::from_definition(
        FieldDefinition {
            source: source.definition().clone(),
            time: t,
            state: vec![x],
        },
        &mut budget(),
    )
    .unwrap()
}
fn main() {
    let iterations: u32 = std::env::var("ADAPTIVE_TAYLOR_ITERATIONS")
        .map(|v| v.parse().unwrap())
        .unwrap_or(1000);
    assert!(iterations > 0);
    for exponential in [false, true] {
        let f = field(exponential);
        let name = if exponential {
            "exponential"
        } else {
            "constant"
        };
        let request = EvolutionRequest {
            start: d(0, 0, 0),
            end: d(1, 1, -1),
            initial: BoxN::new([if exponential { d(1, 1, 0) } else { d(0, 0, 0) }]).unwrap(),
        };
        let native = EvolutionOptions {
            maximum_step: Some(d(1, 1, -2)),
            ..Default::default()
        };
        let options = AdaptiveTaylorOptions {
            maximum_order: 3,
            remainder_limits: [d(1, 1, -10)],
            native: native.clone(),
            ..Default::default()
        };
        let controller = f.adaptive_taylor(options);
        let expected = f.flow_taylor(3, &mut budget()).unwrap().integrate(
            request.clone(),
            native.clone(),
            &mut budget(),
        );
        assert!(matches!(
            expected.status(),
            FlowTaylorEvolutionStatus::Completed
        ));
        let check_endpoint = |end: &BoxN<DyadicInterval, 1>| {
            assert_eq!(Some(end), expected.end_enclosure());
            let mut arithmetic = DyadicBudget::new(16384, 1_000_000_000);
            if exponential {
                use hyperreal::Rational as Q;
                let mut term = Q::from(1);
                let mut lo = term.clone();
                for k in 1..=12 {
                    term = term / Q::from(2 * k);
                    lo = lo + &term;
                }
                let hi = &lo + (&term / Q::from(26)) / (Q::from(1) - Q::from(1) / Q::from(28));
                let out = end.lanes().unwrap()[0]
                    .to_rational(&mut arithmetic)
                    .unwrap();
                assert!(out.contains_point(&lo));
                assert!(out.contains_point(&hi));
                assert!(
                    end.widths(&mut arithmetic).unwrap().unwrap()[0]
                        .subset_of(&d(0, 1, -9), &mut arithmetic)
                        .unwrap()
                );
            } else {
                assert_eq!(end, &BoxN::new([d(1, 1, 0)]).unwrap());
            }
        };
        let check_remainder = |term: &DyadicInterval| {
            let mut a = DyadicBudget::new(16384, 1_000_000_000);
            assert!(
                term.abs(&mut a)
                    .unwrap()
                    .subset_of(&d(0, 1, -10), &mut a)
                    .unwrap()
            );
        };
        for mode in ["fixed3", "adaptive"] {
            let run = || {
                let mut b = budget();
                if mode == "fixed3" {
                    let result = f.flow_taylor(3, &mut b).unwrap().integrate(
                        black_box(request.clone()),
                        native.clone(),
                        &mut b,
                    );
                    assert!(matches!(
                        result.status(),
                        FlowTaylorEvolutionStatus::Completed
                    ));
                    check_endpoint(result.end_enclosure().unwrap());
                    assert_eq!(result.segments().len(), 2);
                    for s in result.segments() {
                        check_remainder(
                            &s.evidence().end_terms().last().unwrap().lanes().unwrap()[0],
                        );
                    }
                    drop(black_box(result));
                } else {
                    let result = controller.integrate(black_box(request.clone()), &mut b);
                    assert!(matches!(result.status(), AdaptiveTaylorStatus::Completed));
                    check_endpoint(result.end_enclosure().unwrap());
                    assert_eq!(result.segments().len(), 2);
                    for s in result.segments() {
                        assert_eq!(s.refinement().meets_limits(), Some(true));
                        check_remainder(
                            &s.refinement()
                                .evidence()
                                .end_terms()
                                .last()
                                .unwrap()
                                .lanes()
                                .unwrap()[0],
                        );
                    }
                    drop(black_box(result));
                }
            };
            for _ in 0..16 {
                run();
            }
            let started = Instant::now();
            for _ in 0..iterations {
                run();
            }
            let elapsed = started.elapsed();
            println!(
                "{name}/{mode}: iterations={iterations} total_ns={} ns_per_operation={}",
                elapsed.as_nanos(),
                elapsed.as_nanos() / u128::from(iterations)
            );
        }
    }
}
