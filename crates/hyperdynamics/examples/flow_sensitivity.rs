//! Certified sensitivity of a pendulum to its release angle and speed.
//!
//! θ'' = -sin θ, written as θ' = ω, ω' = -sin θ, released near θ = 1/2 rad at
//! rest. The evolution encloses every trajectory from the initial box; the
//! sensitivity encloses the derivative of the final state with respect to the
//! initial state along all of them. The flow is Hamiltonian, so Liouville's
//! theorem says the derivative preserves phase-space area: its determinant is
//! exactly 1. The example checks that the certified determinant contains 1.
use std::sync::Arc;

use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    evolution::{EvolutionOptions, EvolutionRequest},
    sensitivity::SensitivityOptions,
};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};

// Rounded decimal display of an exact dyadic bound.
fn approx(bound: Option<(&impl std::fmt::Display, i32)>) -> f64 {
    let (mantissa, exponent) = bound.expect("nonempty interval");
    mantissa.to_string().parse::<f64>().unwrap() * 2f64.powi(exponent)
}
fn show(x: &DyadicInterval) -> String {
    format!("[{:.12}, {:.12}]", approx(x.lower()), approx(x.upper()))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let [time, theta, omega] = [SymbolId(0), SymbolId(1), SymbolId(2)];
    let source = ExpressionProgram::compile(
        &[
            Expr::anonymous_symbol(omega),
            -Expr::anonymous_symbol(theta).sin(),
        ],
        &[theta, omega, time],
        &mut ExpressionBudget::new(10_000, 1_000_000),
    )?;
    let mut budget = EnclosureBudget::new(EnclosurePolicy::default());
    let field = ExpressionField::<2>::from_definition(
        FieldDefinition {
            source: source.definition().clone(),
            time,
            state: vec![theta, omega],
        },
        &mut budget,
    )?;
    // Release angle 1/2 ± 2^-12 rad, speed 0 ± 2^-12 rad/s, for one second in
    // 256 steps of 2^-8 s, the default step limit. Larger steps stay sound but
    // widen every enclosure.
    let request = EvolutionRequest {
        start: DyadicInterval::point(0.into(), 0)?,
        end: DyadicInterval::point(1.into(), 0)?,
        initial: BoxN::new([
            DyadicInterval::new(2047.into(), 2049.into(), -12)?,
            DyadicInterval::new((-1).into(), 1.into(), -12)?,
        ])?,
    };
    let options = EvolutionOptions {
        maximum_step: Some(DyadicInterval::point(1.into(), -8)?),
        ..Default::default()
    };
    let evolution = Arc::new(field.integrate(request, options, &mut budget));
    let end = evolution
        .end_enclosure()
        .ok_or("the evolution did not complete")?;
    let sensitivity = evolution.sensitivity(SensitivityOptions::default(), &mut budget)?;
    let s = sensitivity
        .end_matrix()
        .and_then(|m| m.rows())
        .ok_or("no whole-horizon derivative")?;

    let lanes = end.lanes().expect("nonempty endpoint");
    println!("certified segments: {}", sensitivity.steps().len());
    println!("θ(1) in {}", show(&lanes[0]));
    println!("ω(1) in {}", show(&lanes[1]));
    println!("∂θ(1)/∂θ0 in {}", show(&s[0][0]));
    println!("∂θ(1)/∂ω0 in {}", show(&s[0][1]));
    println!("∂ω(1)/∂θ0 in {}", show(&s[1][0]));
    println!("∂ω(1)/∂ω0 in {}", show(&s[1][1]));

    let mut scalar = DyadicBudget::new(16_384, 1_000_000_000);
    let determinant = s[0][0]
        .mul(&s[1][1], &mut scalar)?
        .sub(&s[0][1].mul(&s[1][0], &mut scalar)?, &mut scalar)?;
    println!("det ∂x(1)/∂x0 in {}", show(&determinant));
    let one = DyadicInterval::point(1.into(), 0)?;
    assert!(
        one.subset_of(&determinant, &mut scalar)?,
        "Liouville's theorem: the certified determinant must contain 1"
    );
    println!("contains 1, as phase-space area conservation requires");
    Ok(())
}
