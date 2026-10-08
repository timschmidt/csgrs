//! Cross every finite optimized Hyperreal certificate through Hypersolve.

#![no_main]

use std::collections::HashMap;

use hyperlimit::{PredicateOutcome, PredicatePolicy, Sign};
use hyperreal::{
    CertifiedRealEquality, CertifiedRealSign, Rational, Real, RealSign, StructuralKind,
};
use hypersolve::{
    Constraint, Expr, Problem, SparseResidualTerm, SymbolId, certify_candidate,
    certify_candidate_domains, classify_real_sign_predicate, context_from_problem,
    evaluate_residuals, replay_dense_linear_residuals, replay_sparse_linear_residuals,
};
use libfuzzer_sys::fuzz_target;

const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;
const MIN_PRECISION: i32 = -256;

#[derive(Clone, Copy, Debug)]
struct Input {
    representation_stride: u8,
    graph_depth: u8,
    graph_opcode: u8,
    candidate_scale: u8,
    expression_opcode: u8,
}

fuzz_target!(|data: &[u8]| {
    // Missing bytes default to zero so libFuzzer's empty/minimal corpus still
    // executes the complete finite inventory instead of being rejected by a
    // fixed-size decoder.
    let byte = |index| data.get(index).copied().unwrap_or(0);
    let input = Input {
        representation_stride: byte(0),
        graph_depth: byte(1),
        graph_opcode: byte(2),
        candidate_scale: byte(3),
        expression_opcode: byte(4),
    };

    let finite = representative_values();
    assert_eq!(finite.len(), 22, "update the finite Real fuzz inventory");
    let scale = Real::from(i64::from(input.candidate_scale % 7) + 1);

    // Every execution visits every optimized finite class. This makes a short
    // sanitizer run meaningful and prevents corpus minimization from deleting
    // a representation family altogether.
    for (certificate, expected_kind, value) in &finite {
        assert_eq!(value.detailed_facts().symbolic.kind, *expected_kind);
        assert!(
            format!("{value:?}").contains(&format!("class: {certificate}")),
            "representation recipe drifted"
        );
        assert!(value.certified_dyadic_interval(MIN_PRECISION).is_some());
        exercise_solver_pipeline(value, &scale);
        exercise_positive_domains(value);
    }

    // Rotate all finite left-hand certificates across a fuzzer-selected right
    // stride. A campaign covers the full ordered 22x22 matrix without paying
    // for all 484 high-precision pairs in each execution.
    let finite_stride = usize::from(input.representation_stride) % finite.len();
    for (index, (_, _, left)) in finite.iter().enumerate() {
        let right = &finite[(index + finite_stride) % finite.len()].2;
        exercise_pair(left, right, input.expression_opcode);
    }

    // The finite certificate enum is exhaustively inventoried above. Generic
    // computables additionally have an unbounded representation space, so
    // vary depth, node sharing, unary kernels, and binary topology here.
    let opaque = opaque_graph_values(input.graph_depth, input.graph_opcode);
    let mut all_values = finite
        .iter()
        .map(|(_, _, value)| value.clone())
        .collect::<Vec<_>>();
    all_values.extend(opaque.iter().cloned());
    let stride = usize::from(input.representation_stride) % all_values.len();
    for (index, left) in all_values.iter().enumerate() {
        let right = &all_values[(index + stride) % all_values.len()];
        exercise_total_pair(left, right, input.expression_opcode);
    }
    for value in opaque {
        assert_eq!(
            value.detailed_facts().symbolic.kind,
            StructuralKind::ComputableOpaque
        );
        assert!(value.certified_dyadic_interval(MIN_PRECISION).is_some());
        exercise_solver_pipeline(&value, &scale);
    }
});

fn exercise_solver_pipeline(value: &Real, scale: &Real) {
    let mut problem = Problem::default();
    let variable = problem.add_variable("x", value.clone());
    let symbol = SymbolId(variable.0);
    let delta = || Expr::symbol(symbol, "x") - Expr::real(value.clone());
    problem.add_constraint(Constraint::equality("affine", delta()));
    problem.add_constraint(Constraint::equality("quadratic", delta() * delta()));
    problem.add_constraint(Constraint::equality("non-polynomial", delta().sin()));

    let analysis = problem.analyze();
    assert_eq!(analysis.facts().active_constraint_count, 3);
    let context = context_from_problem(&problem);
    for residual in evaluate_residuals(&problem, &context).expect("bound tree evaluation") {
        assert_certified_zero(&residual.value);
    }
    for residual in analysis
        .evaluate_residuals(&context)
        .expect("bound retained evaluation")
    {
        assert_certified_zero(&residual.value);
    }
    assert!(certify_candidate(&analysis, &context).all_satisfied());

    let matrix = [vec![value.clone()]];
    let rhs = [value * scale];
    let dense = replay_dense_linear_residuals(
        &matrix,
        &rhs,
        std::slice::from_ref(scale),
        MIN_PRECISION,
        APPROX,
    )
    .expect("bounded dense replay");
    assert!(dense.accepted);

    // Author the sparse residual with the same coefficient/RHS graph. The
    // strict sparse proof is expected to certify this identity without an f64
    // terminal policy.
    let sparse = replay_sparse_linear_residuals(
        1,
        1,
        &[SparseResidualTerm {
            row: 0,
            column: 0,
            coefficient: value.clone(),
        }],
        &rhs,
        std::slice::from_ref(scale),
        MIN_PRECISION,
    )
    .expect("identity-preserving sparse replay");
    assert!(sparse.accepted);
}

fn exercise_positive_domains(value: &Real) {
    let mut problem = Problem::default();
    let variable = problem.add_variable("positive", value.clone());
    let symbol = SymbolId(variable.0);
    problem.add_constraint(Constraint::equality(
        "positive domains",
        Expr::symbol(symbol, "positive").ln()
            + Expr::symbol(symbol, "positive").sqrt()
            + (Expr::int(1) / Expr::symbol(symbol, "positive")),
    ));
    let context = context_from_problem(&problem);
    let domains = certify_candidate_domains(&problem, &context, APPROX);
    assert!(domains.all_checks_certified_valid());
    assert!(matches!(
        classify_real_sign_predicate(value, APPROX),
        PredicateOutcome::Decided {
            value: Sign::Positive,
            ..
        }
    ));
}

fn exercise_pair(left: &Real, right: &Real, opcode: u8) {
    let x = SymbolId(0);
    let y = SymbolId(1);
    let mut bindings = HashMap::new();
    bindings.insert(x, left.clone());
    bindings.insert(y, right.clone());

    let expression = match opcode % 4 {
        0 => Expr::symbol(x, "x") + Expr::symbol(y, "y") - Expr::real(left + right),
        1 => Expr::symbol(x, "x") - Expr::symbol(y, "y") - Expr::real(left - right),
        2 => Expr::symbol(x, "x") * Expr::symbol(y, "y") - Expr::real(left * right),
        _ => {
            Expr::symbol(x, "x") / Expr::symbol(y, "y")
                - Expr::real((left / right).expect("finite representatives are nonzero"))
        }
    };
    assert_eq!(expression.structural_facts().dependencies.len(), 2);
    let result = expression
        .eval_real(&bindings)
        .expect("bound pair expression");
    assert_bounded_equal(&result, &Real::zero());
}

fn exercise_total_pair(left: &Real, right: &Real, opcode: u8) {
    let expected = match opcode % 3 {
        0 => left + right,
        1 => left - right,
        _ => left * right,
    };
    let x = SymbolId(0);
    let y = SymbolId(1);
    let mut bindings = HashMap::new();
    bindings.insert(x, left.clone());
    bindings.insert(y, right.clone());
    let expression = match opcode % 3 {
        0 => Expr::symbol(x, "x") + Expr::symbol(y, "y"),
        1 => Expr::symbol(x, "x") - Expr::symbol(y, "y"),
        _ => Expr::symbol(x, "x") * Expr::symbol(y, "y"),
    };
    let actual = expression
        .eval_real(&bindings)
        .expect("bound total expression");
    assert_bounded_equal(&actual, &expected);
}

fn assert_certified_zero(value: &Real) {
    assert!(matches!(
        value.certified_sign_until(MIN_PRECISION),
        CertifiedRealSign::Known {
            sign: RealSign::Zero,
            ..
        }
    ));
}

fn assert_bounded_equal(left: &Real, right: &Real) {
    if matches!(
        left.certified_eq_until(right, MIN_PRECISION),
        CertifiedRealEquality::Equal { .. }
    ) || matches!(
        (left - right).certified_sign_until(MIN_PRECISION),
        CertifiedRealSign::Known {
            sign: RealSign::Zero,
            ..
        }
    ) {
        return;
    }

    let [left_lower, left_upper] = left
        .certified_dyadic_interval(MIN_PRECISION)
        .expect("bounded left value");
    let [right_lower, right_upper] = right
        .certified_dyadic_interval(MIN_PRECISION)
        .expect("bounded right value");
    assert!(left_lower <= right_upper && right_lower <= left_upper);
}

fn fraction(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).expect("nonzero denominator"))
}

fn representative_values() -> Vec<(&'static str, StructuralKind, Real)> {
    let pi = Real::pi();
    let e = Real::e();
    let pi_squared = &pi * &pi;
    let sqrt_two = Real::from(2).sqrt().expect("positive radicand");
    let ln_two = Real::from(2).ln().expect("positive logarithm input");
    let ln_three = Real::from(3).ln().expect("positive logarithm input");

    vec![
        ("One", StructuralKind::ExactRational, fraction(3, 2)),
        ("Pi", StructuralKind::PiLike, pi.clone()),
        ("PiPow", StructuralKind::PiLike, pi_squared.clone()),
        (
            "PiInv",
            StructuralKind::PiLike,
            pi.clone().inverse().expect("pi is nonzero"),
        ),
        ("PiExp", StructuralKind::ExpLike, &pi * &e),
        (
            "PiInvExp",
            StructuralKind::ExpLike,
            (&e / &pi).expect("pi is nonzero"),
        ),
        ("PiSqrt", StructuralKind::SqrtLike, &pi * &sqrt_two),
        (
            "ConstProduct",
            StructuralKind::ProductConstant,
            &pi_squared * &e,
        ),
        (
            "ConstOffset",
            StructuralKind::ProductConstant,
            &pi - Real::from(3),
        ),
        (
            "ConstProductSqrt",
            StructuralKind::ProductConstant,
            &(&pi_squared * &e) * &sqrt_two,
        ),
        ("Sqrt", StructuralKind::SqrtLike, sqrt_two),
        (
            "Exp",
            StructuralKind::ExpLike,
            Real::from(2).exp().expect("finite exponential"),
        ),
        ("Ln", StructuralKind::LogLike, ln_three.clone()),
        (
            "LnAffine",
            StructuralKind::LogLike,
            (Real::from(2) * &e).ln().expect("positive logarithm input"),
        ),
        ("LnProduct", StructuralKind::LogLike, &ln_two * &ln_three),
        (
            "Log10",
            StructuralKind::LogLike,
            Real::from(2).log10().expect("positive input"),
        ),
        (
            "Log2",
            StructuralKind::LogLike,
            Real::from(3).log2().expect("positive input"),
        ),
        (
            "Pow10",
            StructuralKind::ExpLike,
            fraction(1, 7)
                .exp10()
                .expect("finite rational base-ten power"),
        ),
        (
            "Pow2",
            StructuralKind::ExpLike,
            fraction(1, 7)
                .exp2()
                .expect("finite rational base-two power"),
        ),
        ("SinPi", StructuralKind::TrigExact, fraction(1, 5).sin_pi()),
        (
            "TanPi",
            StructuralKind::TrigExact,
            fraction(1, 5)
                .tan_pi()
                .expect("one fifth of a turn is not a tangent pole"),
        ),
        (
            "Irrational",
            StructuralKind::ComputableOpaque,
            Real::one().sin(),
        ),
    ]
}

fn opaque_graph_values(depth_seed: u8, opcode_seed: u8) -> Vec<Real> {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let identity_residual = &sine * &sine + &cosine * &cosine - Real::one();
    let mut recursive = &sine + &cosine;

    for level in 0..=usize::from(depth_seed % 12) {
        recursive = match (usize::from(opcode_seed) + level) % 7 {
            0 => recursive.sin(),
            1 => recursive.cos(),
            2 => recursive.atan().expect("finite graph arctangent"),
            3 => &recursive * &recursive + &sine,
            4 => &recursive + &cosine,
            5 => (&recursive * &sine) - &cosine,
            6 => {
                let shared = recursive.clone();
                recursive + shared
            }
            _ => unreachable!(),
        };
    }

    let values = vec![sine, cosine, identity_residual, recursive];
    assert!(
        values.iter().all(|value| {
            value.detailed_facts().symbolic.kind == StructuralKind::ComputableOpaque
        })
    );
    values
}
