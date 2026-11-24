//! Performance atlas for Hypersolve: exact univariate polynomial algebra and
//! root isolation, resultants and elimination, algebraic-number arithmetic,
//! exact linear algebra, candidate and interval certification, expression
//! programs, Taylor models and work budgets, swept across polynomial degree,
//! coefficient size, root separation, dimension, expression size, Taylor order
//! and domain width. Run through `scripts/perf-atlas/run.sh`, or directly:
//! `cargo run --release -p hypersolve --all-features --example perf_atlas -- --out atlas.json`.
//!
//! Every polynomial is stored with ascending coefficients. Inputs are built
//! outside the timer. Work budgets and graph/arithmetic budgets are generous so
//! that, except in the explicit "work budget" group, a failed point is a
//! genuine refusal (undecided, depth limit, domain error), not exhaustion.
use std::sync::Arc;

use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperreal::{Rational, Real};
use hypersolve::expression_program::{
    CoefficientRequest, ExpressionBudget, ExpressionProgram, TaylorAffineMap, TaylorAffineModel,
    TaylorAffineRow, TaylorModel, TaylorNoiseModel, TaylorPolynomialModel, TaylorRequest,
};
use hypersolve::work_budget::{
    CancellationToken, WorkBudget, with_budget, with_cancellable_budget,
};
use hypersolve::{
    AffineKrawczykStatus, AlgebraicRootAffineTransformStatus, AlgebraicRootArithmeticOp,
    AlgebraicRootArithmeticStatus, AlgebraicRootBinaryTransformStatus,
    AlgebraicRootComparisonStatus, AlgebraicRootMobiusTransformStatus,
    AlgebraicRootPolynomialEvaluationStatus, AlgebraicRootPolynomialImageStatus,
    AlgebraicRootRefinementComparisonConfig, AlgebraicRootRepresentation,
    AlgebraicRootSquareRootStatus, BernsteinRootCountStatus, BernsteinSubdivisionConfig,
    BernsteinSubdivisionStatus, BivariatePolynomial, Constraint, ConvergenceReason,
    CurveIntersectionResultantConfig, CurveIntersectionResultantStatus, CurveResultantParameter,
    DenseLinearBackend, DescartesRootCountStatus, ExactAffineRankStatus, Expr,
    IntervalBoxCertificationPackage, IntervalBoxCertificationStatus, IsolatedRootInterval,
    IsolatedRootRefinementStatus, LinearBackend, ModularCoprimality,
    MultivariateQuadraticKrawczykStatus, PolynomialParametricCurve2, PredicatePolicy, Problem,
    ProposalEngineKind, RootIsolationConfig, RootIsolationStatus, SimplexProjectionStatus,
    SolverConfig, SolverState, SparseResidualTerm, SymbolId, TrivariateConstraintResultantStatus,
    TrivariatePolynomial, TrivariatePolynomialAxis, UnivariateSturmSequence, VariableBall,
    analyze_exact_affine_rank, arithmetic_algebraic_root_representations,
    certify_affine_krawczyk_box, certify_candidate, certify_interval_box_candidate,
    certify_multivariate_quadratic_krawczyk_box, certify_univariate_polynomial_nonnegative,
    compare_algebraic_root_representations_by_difference,
    compare_algebraic_root_representations_with_refinement,
    compose_univariate_polynomial_linear_fractional, context_from_problem,
    count_bernstein_univariate_polynomial_interval_expr,
    count_descartes_univariate_polynomial_expr, determinant_bareiss,
    divide_bivariate_polynomial_exact, divide_univariate_polynomial_exact,
    evaluate_polynomial_at_algebraic_root, greatest_common_divisor_univariate_polynomials_exact,
    isolate_univariate_polynomial_expr, isolate_univariate_polynomial_expr_with_config,
    polynomial_has_one_distinct_root_in_open_interval, project_origin_onto_simplex,
    refine_isolated_univariate_polynomial_interval, refined_represented_root,
    replay_sparse_linear_residuals, represent_univariate_algebraic_roots,
    resultant_bivariate_polynomial_system, resultant_parametric_curve_intersection,
    resultant_trivariate_polynomial_univariate_constraint, resultant_univariate_polynomials,
    sign_at_selected_root, solve_damped_least_squares, solve_dense_linear_system_bareiss,
    solve_sparse_linear_system_bareiss, solve_sparse_linear_system_bareiss_minimum_degree,
    solve_sparse_linear_system_bareiss_pattern_preserving, square_free_part,
    square_root_algebraic_root_representation,
    subdivide_bernstein_univariate_polynomial_interval_expr,
    subresultant_chain_univariate_polynomials, transform_algebraic_root_affine,
    transform_algebraic_root_mobius, transform_algebraic_root_polynomial_image,
    transform_algebraic_roots_binary, univariate_polynomials_modular_coprimality,
};
use num::bigint::Sign;
use num::{BigInt, BigUint};
use perf_atlas::{Atlas, Rng, ladder, powers_of_two};

/// The crate's default exact-comparison policy: certified stages first, with
/// bounded approximate refinement to 512 bits.
const POLICY: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;
/// Refinement floor for exact Real sign decisions in linear algebra.
const MIN_PRECISION: i32 = -256;

// ---------------------------------------------------------------- scalars ---

fn int(n: i64) -> Real {
    Real::new(Rational::new(n))
}

fn frac(n: i64, d: u64) -> Real {
    Real::new(Rational::fraction(n, d).expect("nonzero denominator"))
}

/// `n / 2^k` exactly.
fn dyadic_real(n: i64, k: u32) -> Real {
    Real::new(
        Rational::from_bigint_fraction(BigInt::from(n), BigUint::from(1u8) << k as usize)
            .expect("nonzero denominator"),
    )
}

/// A random nonzero signed integer with exactly `bits` magnitude bits.
fn big(rng: &mut Rng, bits: u32) -> BigInt {
    let sign = if rng.coin() { Sign::Minus } else { Sign::Plus };
    BigInt::from_biguint(sign, BigUint::new(rng.limbs(bits)))
}

fn big_real(value: BigInt) -> Real {
    Real::new(Rational::from_bigint(value))
}

/// A random signed integer coefficient of `bits` bits.
fn rand_int(rng: &mut Rng, bits: u32) -> Real {
    big_real(big(rng, bits))
}

/// A random signed rational whose numerator and denominator have `bits` bits.
fn rand_rational(rng: &mut Rng, bits: u32) -> Real {
    let numerator = big(rng, bits);
    let denominator = BigUint::new(rng.limbs(bits.max(2)));
    Real::new(Rational::from_bigint_fraction(numerator, denominator).expect("nonzero"))
}

fn sqrt2() -> Real {
    int(2).sqrt().expect("positive radicand")
}

fn d(lower: i64, upper: i64, exponent: i32) -> DyadicInterval {
    DyadicInterval::new(lower.into(), upper.into(), exponent).expect("ordered dyadic interval")
}

fn graph() -> ExpressionBudget {
    ExpressionBudget::new(10_000_000, 1_000_000_000)
}

fn arithmetic() -> DyadicBudget {
    DyadicBudget::new(1 << 22, 1 << 62)
}

// ------------------------------------------------------------ polynomials ---

fn imul(a: &[BigInt], b: &[BigInt]) -> Vec<BigInt> {
    let mut out = vec![BigInt::from(0); a.len() + b.len() - 1];
    for (i, x) in a.iter().enumerate() {
        for (j, y) in b.iter().enumerate() {
            out[i + j] += x * y;
        }
    }
    out
}

/// Product of linear factors `q x - p`, given as `(q, p)`.
fn from_factors(factors: &[(BigInt, BigInt)]) -> Vec<BigInt> {
    factors.iter().fold(vec![BigInt::from(1)], |acc, (q, p)| {
        imul(&acc, &[-p.clone(), q.clone()])
    })
}

fn reals(coefficients: &[BigInt]) -> Vec<Real> {
    coefficients.iter().cloned().map(big_real).collect()
}

fn rmul(a: &[Real], b: &[Real]) -> Vec<Real> {
    let mut out = vec![Real::zero(); a.len() + b.len() - 1];
    for (i, x) in a.iter().enumerate() {
        for (j, y) in b.iter().enumerate() {
            out[i + j] = out[i + j].clone() + x.clone() * y;
        }
    }
    out
}

/// Wilkinson's polynomial with roots `1..=degree`.
fn wilkinson(degree: usize) -> Vec<BigInt> {
    let factors: Vec<_> = (1..=degree as i64)
        .map(|i| (BigInt::from(1), BigInt::from(i)))
        .collect();
    from_factors(&factors)
}

/// Mignotte's polynomial `x^d - 2 (a x - 1)^2`: two real roots within about
/// `a^-(d+2)/2` of `1/a`, the classical separation stress case.
fn mignotte(degree: usize, a: i64) -> Vec<BigInt> {
    let degree = degree.max(3);
    let a = BigInt::from(a);
    let mut out = vec![BigInt::from(0); degree + 1];
    out[0] = BigInt::from(-2);
    out[1] = BigInt::from(4) * &a;
    out[2] = BigInt::from(-2) * &a * &a;
    out[degree] += BigInt::from(1);
    out
}

/// Roots `1/2` and `1/2 + 2^-k`, plus `-1, ..., -(degree - 2)`.
fn close_pair(degree: usize, k: u32) -> Vec<BigInt> {
    let two_k = BigInt::from(1) << k as usize;
    let mut factors = vec![(BigInt::from(2), BigInt::from(1)), (&two_k * 2, &two_k + 1)];
    for i in 1..=degree.saturating_sub(2) as i64 {
        factors.push((BigInt::from(1), BigInt::from(-i)));
    }
    from_factors(&factors)
}

fn dense_int(rng: &mut Rng, degree: usize, bits: u32) -> Vec<Real> {
    (0..=degree).map(|_| rand_int(rng, bits)).collect()
}

fn dense_rational(rng: &mut Rng, degree: usize, bits: u32) -> Vec<Real> {
    (0..=degree).map(|_| rand_rational(rng, bits)).collect()
}

/// `x^degree - c`.
fn radical(degree: usize, c: i64) -> Vec<Real> {
    let mut out = vec![Real::zero(); degree + 1];
    out[0] = int(-c);
    out[degree] = Real::one();
    out
}

/// Builds a random polynomial.
type MakePoly = fn(&mut Rng) -> Vec<Real>;
/// Builds a fixed polynomial.
type FixedPoly = fn() -> Vec<Real>;

/// Polynomial degrees: geometric, with intermediate steps so that a cliff
/// does not overshoot the single-call ceiling by a large factor.
const DEGREES: [f64; 10] = [2.0, 4.0, 8.0, 16.0, 24.0, 32.0, 48.0, 64.0, 96.0, 128.0];

const POLY_KINDS: [&str; 4] = [
    "dense 32-bit",
    "rational 16-bit",
    "Wilkinson",
    "Mignotte a=16",
];

fn poly(kind: &str, degree: usize, rng: &mut Rng) -> Vec<Real> {
    match kind {
        "dense 32-bit" => dense_int(rng, degree, 32),
        "rational 16-bit" => dense_rational(rng, degree, 16),
        "Wilkinson" => reals(&wilkinson(degree)),
        "Mignotte a=16" => reals(&mignotte(degree, 16)),
        "sparse trinomial" => {
            let mut out = vec![Real::zero(); degree + 1];
            out[0] = rand_int(rng, 32);
            out[(degree / 2).max(1)] = rand_int(rng, 32);
            out[degree] = out[degree].clone() + rand_int(rng, 32);
            out
        }
        other => unreachable!("unknown polynomial kind {other}"),
    }
}

/// A random polynomial of random kind, degree in `1..=max_degree`.
fn fuzz_poly(max_degree: f64, rng: &mut Rng) -> Vec<Real> {
    let degree = rng.range(1, max_degree as i64) as usize;
    match rng.below(5) {
        0 => {
            let bits = rng.range(1, 128) as u32;
            dense_int(rng, degree, bits)
        }
        1 => {
            let bits = rng.range(1, 32) as u32;
            dense_rational(rng, degree, bits)
        }
        2 => reals(&wilkinson(degree)),
        3 => reals(&mignotte(degree, rng.range(2, 64))),
        _ => poly("sparse trinomial", degree.max(2), rng),
    }
}

fn poly_expr(coefficients: &[Real], x: &Expr) -> Expr {
    let mut terms = coefficients
        .iter()
        .enumerate()
        .filter(|(_, c)| !c.definitely_zero())
        .map(|(i, c)| {
            if i == 0 {
                Expr::real(c.clone())
            } else {
                Expr::Mul(
                    Box::new(Expr::real(c.clone())),
                    Box::new(x.clone().powi(i as i64)),
                )
            }
        });
    let first = terms.next().unwrap_or_else(Expr::zero);
    terms.fold(first, |acc, term| Expr::Add(Box::new(acc), Box::new(term)))
}

/// One univariate equality row `p(x) = 0` in its own problem.
struct Univariate {
    problem: Problem,
    expr: Expr,
}

fn univariate(coefficients: Vec<Real>) -> Univariate {
    let mut problem = Problem::default();
    let variable = problem.add_variable("x", Real::zero());
    let x = Expr::symbol(SymbolId(variable.0), "x");
    let expr = poly_expr(&coefficients, &x);
    problem.add_constraint(Constraint::equality("p", expr.clone()));
    Univariate { problem, expr }
}

fn isolated(status: &RootIsolationStatus) -> bool {
    matches!(
        status,
        RootIsolationStatus::Isolated
            | RootIsolationStatus::MultipleRoot
            | RootIsolationStatus::NoRealRoots
    )
}

fn isolate(u: &Univariate) -> bool {
    isolated(&isolate_univariate_polynomial_expr(0, &u.expr, &u.problem, POLICY).status)
}

fn intervals(u: &Univariate) -> Vec<IsolatedRootInterval> {
    isolate_univariate_polynomial_expr(0, &u.expr, &u.problem, POLICY).intervals
}

/// Represented real roots of `p`, in ascending order.
fn roots_of(coefficients: Vec<Real>) -> Vec<AlgebraicRootRepresentation> {
    let u = univariate(coefficients);
    let analysis = u.problem.analyze();
    represent_univariate_algebraic_roots(
        &analysis,
        RootIsolationConfig {
            policy: POLICY,
            ..RootIsolationConfig::default()
        },
    )
    .into_iter()
    .flat_map(|report| report.roots)
    .collect()
}

/// The largest real root of `p`.
fn top_root(coefficients: Vec<Real>) -> AlgebraicRootRepresentation {
    roots_of(coefficients)
        .pop()
        .expect("polynomial has a real root")
}

/// Bisection steps applied to operand roots before algebraic-number
/// operations, so that unrefined isolating intervals (which can straddle zero
/// or overlap a resultant's other roots) are not what is measured.
const REFINEMENT_STEPS: usize = 32;

/// The largest real root of `p`, refined by [`REFINEMENT_STEPS`] bisections.
fn refined_root(coefficients: Vec<Real>) -> AlgebraicRootRepresentation {
    refined_represented_root(&top_root(coefficients), REFINEMENT_STEPS)
}

// --------------------------------------------------------- bivariate etc. ---

fn bivariate_dense(
    rng: &mut Rng,
    degree: usize,
    coefficient: impl Fn(&mut Rng) -> Real,
) -> BivariatePolynomial {
    let mut grid = vec![vec![Real::zero(); degree + 1]; degree + 1];
    for (i, row) in grid.iter_mut().enumerate() {
        for (j, entry) in row.iter_mut().enumerate() {
            if i + j <= degree {
                *entry = coefficient(rng);
            }
        }
    }
    BivariatePolynomial::new(grid)
}

fn bivariate_mul(a: &BivariatePolynomial, b: &BivariatePolynomial) -> BivariatePolynomial {
    let (ar, ac) = (a.coefficients.len(), a.coefficients[0].len());
    let (br, bc) = (b.coefficients.len(), b.coefficients[0].len());
    let mut out = vec![vec![Real::zero(); ac + bc - 1]; ar + br - 1];
    for i in 0..ar {
        for j in 0..ac {
            for k in 0..br {
                for l in 0..bc {
                    out[i + k][j + l] = out[i + k][j + l].clone()
                        + a.coefficients[i][j].clone() * &b.coefficients[k][l];
                }
            }
        }
    }
    BivariatePolynomial::new(out)
}

fn resultant_config() -> CurveIntersectionResultantConfig {
    CurveIntersectionResultantConfig {
        max_resultant_degree: 1 << 16,
        ..CurveIntersectionResultantConfig::default()
    }
}

fn matrix(kind: &str, n: usize, bits: u32, rng: &mut Rng) -> Vec<Vec<Real>> {
    let root = sqrt2();
    (0..n)
        .map(|_| {
            (0..n)
                .map(|_| match kind {
                    "integer" => rand_int(rng, bits),
                    "rational" => rand_rational(rng, bits),
                    "√2 field" => rand_int(rng, bits) + rand_int(rng, bits) * &root,
                    other => unreachable!("unknown matrix kind {other}"),
                })
                .collect()
        })
        .collect()
}

/// Tridiagonal, diagonally dominant sparse system with a known rational
/// solution.
fn tridiagonal(
    n: usize,
    rational: bool,
    rng: &mut Rng,
) -> (Vec<SparseResidualTerm>, Vec<Real>, Vec<Real>) {
    let entry = |rng: &mut Rng| {
        if rational {
            rand_rational(rng, 8)
        } else {
            rand_int(rng, 8)
        }
    };
    let mut terms = Vec::new();
    for row in 0..n {
        for column in row.saturating_sub(1)..(row + 2).min(n) {
            let coefficient = if row == column {
                int(1024) + entry(rng)
            } else {
                entry(rng)
            };
            terms.push(SparseResidualTerm {
                row,
                column,
                coefficient,
            });
        }
    }
    let solution: Vec<Real> = (0..n).map(|i| frac(i as i64 + 1, 3)).collect();
    let mut rhs = vec![Real::zero(); n];
    for term in &terms {
        rhs[term.row] = rhs[term.row].clone() + term.coefficient.clone() * &solution[term.column];
    }
    (terms, rhs, solution)
}

/// `n` coupled quadratic equations `x_i^2 + x_{i+1} = r_i^2 + r_{i+1}` with an
/// exact root `r`; every variable starts at `r_i + offset`.
fn quadratic_system(
    n: usize,
    root_kind: &str,
    offset: &Real,
) -> (Problem, Vec<SymbolId>, Vec<Real>) {
    let roots: Vec<Real> = (0..n)
        .map(|i| match root_kind {
            "√2" => sqrt2() + frac(i as i64, (2 * n) as u64),
            _ => frac((2 * n + i) as i64, (2 * n) as u64),
        })
        .collect();
    let mut problem = Problem::default();
    let symbols: Vec<SymbolId> = (0..n)
        .map(|i| {
            SymbolId(
                problem
                    .add_variable(format!("x{i}"), roots[i].clone() + offset)
                    .0,
            )
        })
        .collect();
    for i in 0..n {
        let j = (i + 1) % n;
        let xi = Expr::symbol(symbols[i], format!("x{i}"));
        let xj = Expr::symbol(symbols[j], format!("x{j}"));
        let constant = roots[i].clone() * &roots[i] + &roots[j];
        problem.add_constraint(Constraint::equality(
            format!("q{i}"),
            xi.powi(2) + xj - Expr::real(constant),
        ));
    }
    (problem, symbols, roots)
}

/// Diagonally dominant affine system `A x = A r` with rational solution `r`.
fn affine_system(
    n: usize,
    rank_deficient: bool,
    offset: &Real,
    rng: &mut Rng,
) -> (Problem, Vec<SymbolId>) {
    let mut problem = Problem::default();
    let symbols: Vec<SymbolId> = (0..n)
        .map(|i| {
            SymbolId(
                problem
                    .add_variable(format!("x{i}"), frac(i as i64 + 1, 3) + offset)
                    .0,
            )
        })
        .collect();
    let mut rows: Vec<Vec<i64>> = (0..n)
        .map(|i| {
            (0..n)
                .map(|j| {
                    if i == j {
                        8 * n as i64
                    } else {
                        rng.range(-4, 4)
                    }
                })
                .collect()
        })
        .collect();
    if rank_deficient {
        for i in n / 2..n {
            rows[i] = rows[i - n / 2].clone();
        }
    }
    for (i, row) in rows.iter().enumerate() {
        let mut expr = Expr::zero();
        let mut constant = Real::zero();
        for (j, &a) in row.iter().enumerate() {
            expr = Expr::Add(
                Box::new(expr),
                Box::new(Expr::Mul(
                    Box::new(Expr::int(a)),
                    Box::new(Expr::symbol(symbols[j], format!("x{j}"))),
                )),
            );
            constant += int(a) * frac(j as i64 + 1, 3);
        }
        problem.add_constraint(Constraint::equality(
            format!("a{i}"),
            expr - Expr::real(constant),
        ));
    }
    (problem, symbols)
}

fn balls(symbols: &[SymbolId], radius: &Real) -> Vec<VariableBall> {
    symbols
        .iter()
        .map(|&symbol| VariableBall {
            symbol,
            radius: radius.clone(),
        })
        .collect()
}

// ------------------------------------------------------------ expressions ---

const EXPR_KINDS: [&str; 4] = [
    "Horner chain",
    "balanced product tree",
    "transcendental chain",
    "multivariate quadratic",
];

fn var(i: usize) -> Expr {
    Expr::anonymous_symbol(SymbolId(i as u32))
}

fn add(a: Expr, b: Expr) -> Expr {
    Expr::Add(Box::new(a), Box::new(b))
}

fn mul(a: Expr, b: Expr) -> Expr {
    Expr::Mul(Box::new(a), Box::new(b))
}

fn balanced(leaves: &[Expr]) -> Expr {
    match leaves {
        [] => Expr::int(1),
        [one] => one.clone(),
        _ => {
            let (left, right) = leaves.split_at(leaves.len() / 2);
            add(balanced(left), balanced(right))
        }
    }
}

/// One source expression of about `size` operation nodes, and its variables.
fn expression(kind: &str, size: usize) -> (Expr, Vec<SymbolId>) {
    match kind {
        "Horner chain" => {
            let mut e = Expr::int(1);
            for i in 0..size {
                e = add(mul(e, var(0)), Expr::int(i as i64 % 7 - 3));
            }
            (e, vec![SymbolId(0)])
        }
        "balanced product tree" => {
            let leaves: Vec<Expr> = (0..size.max(1))
                .map(|i| mul(var(i % 4), var((i + 1) % 4)))
                .collect();
            (
                balanced(&leaves),
                (0..4).map(|i| SymbolId(i as u32)).collect(),
            )
        }
        "transcendental chain" => {
            let mut e = var(0);
            for i in 0..size {
                e = match i % 3 {
                    0 => e.sin(),
                    1 => add(e.cos(), Expr::int(2)),
                    _ => add(e.powi(2), Expr::int(1)).sqrt(),
                };
            }
            (e, vec![SymbolId(0)])
        }
        "multivariate quadratic" => {
            let n = size.max(1);
            let leaves: Vec<Expr> = (0..n)
                .map(|i| add(mul(var(i), var((i + 1) % n)), mul(var(i), var(i))))
                .collect();
            (
                balanced(&leaves),
                (0..n).map(|i| SymbolId(i as u32)).collect(),
            )
        }
        other => unreachable!("unknown expression kind {other}"),
    }
}

fn program(kind: &str, size: usize) -> ExpressionProgram {
    let (e, variables) = expression(kind, size);
    ExpressionProgram::compile(&[e], &variables, &mut graph()).expect("compiles")
}

/// Exact sample point `x_i = (i + 1) / 3`.
fn values(count: usize) -> Vec<Real> {
    (0..count).map(|i| frac(i as i64 + 1, 3)).collect()
}

/// Dyadic boxes `[(i+1)/4, (i+1)/4 + 2^-width]`.
fn boxes(count: usize, width: u32) -> Vec<DyadicInterval> {
    (0..count)
        .map(|i| {
            let e = width as i32 + 2;
            let lower = (i as i64 + 1) << width;
            d(lower, lower + 4, -e)
        })
        .collect()
}

// ------------------------------------------------------------------ Taylor ---

const TAYLOR_KINDS: [&str; 4] = ["1/(1−x)", "sin(x)·cos(x)", "sqrt(2+x)", "(1+x)^8 expanded"];

fn taylor_source(kind: &str) -> Expr {
    let x = var(0);
    match kind {
        "1/(1−x)" => Expr::int(1) / (Expr::int(1) - x),
        "sin(x)·cos(x)" => mul(x.clone().sin(), x.cos()),
        "sqrt(2+x)" => add(Expr::int(2), x).sqrt(),
        "(1+x)^8 expanded" => {
            let binomial = [1, 8, 28, 56, 70, 56, 28, 8, 1];
            let mut e = Expr::int(1);
            for c in binomial.iter().rev().skip(1) {
                e = add(mul(e, var(0)), Expr::int(*c));
            }
            e
        }
        other => unreachable!("unknown Taylor kind {other}"),
    }
}

fn taylor_program(kind: &str) -> ExpressionProgram {
    ExpressionProgram::compile(&[taylor_source(kind)], &[SymbolId(0)], &mut graph())
        .expect("compiles")
}

/// Order-`order` request centered at 0 on `[-2^-k, 2^-k]` in `n` variables.
fn taylor_request(n: usize, order: usize, k: i32) -> TaylorRequest {
    TaylorRequest {
        center: vec![d(0, 0, 0); n],
        domain: vec![d(-1, 1, -k); n],
        order,
        precision: -64,
    }
}

fn taylor_model(program: &ExpressionProgram, request: TaylorRequest) -> Option<TaylorModel> {
    program
        .taylor_model(request, &mut graph(), &mut arithmetic())
        .ok()
}

/// `m` outputs `sin(x + j) * x` of one variable, order 2 on `[-1/16, 1/16]`.
fn multi_output_model(m: usize) -> Arc<TaylorModel> {
    let sources: Vec<Expr> = (0..m)
        .map(|j| mul(add(var(0), Expr::int(j as i64)).sin(), var(0)))
        .collect();
    let program =
        ExpressionProgram::compile(&sources, &[SymbolId(0)], &mut graph()).expect("compiles");
    Arc::new(taylor_model(&program, taylor_request(1, 2, 4)).expect("model"))
}

fn affine_map(inputs: usize, outputs: usize, rng: &mut Rng) -> TaylorAffineMap {
    TaylorAffineMap {
        input_count: inputs,
        rows: (0..outputs)
            .map(|_| TaylorAffineRow {
                coefficients: (0..inputs).map(|_| int(rng.range(-8, 8))).collect(),
                offset: int(rng.range(-8, 8)),
            })
            .collect(),
    }
}

fn main() {
    let mut atlas = Atlas::from_args("hypersolve");
    univariate_operations(&mut atlas);
    polynomial_algebra(&mut atlas);
    algebraic_numbers(&mut atlas);
    elimination(&mut atlas);
    linear_algebra(&mut atlas);
    certification(&mut atlas);
    expression_programs(&mut atlas);
    taylor_models(&mut atlas);
    work_budgets(&mut atlas);
    atlas.finish();
}

fn univariate_operations(atlas: &mut Atlas) {
    let degrees = DEGREES.to_vec();
    let group = "univariate roots";

    atlas.operation(
        group,
        "isolate real roots",
        "polynomial degree",
        "Sturm isolation of every distinct real root of one exact polynomial row, including extraction from the expression tree and square-free reduction. Wilkinson has huge coefficients and well-separated roots; Mignotte has two roots about 16^-(d+2)/2 apart.",
    );
    for kind in POLY_KINDS {
        atlas.sweep(
            kind,
            &degrees,
            |m, rng| univariate(poly(kind, m as usize, rng)),
            isolate,
        );
    }
    atlas.fuzz(
        "mixed kinds, degree ≤ m",
        &powers_of_two(1, 6),
        |m, rng| univariate(fuzz_poly(m, rng)),
        isolate,
    );
    atlas.grid(
        "dense integer",
        ("polynomial degree", &powers_of_two(1, 6)),
        ("coefficient bits", &powers_of_two(0, 10)),
        |degree, bits, rng| univariate(dense_int(rng, degree as usize, bits as u32)),
        isolate,
    );

    atlas.operation(
        group,
        "isolate by root separation",
        "log2 of root gap (gap = 2^-k)",
        "Sturm isolation of a polynomial with roots 1/2 and 1/2 + 2^-k (plus well-separated roots -1, -2, ... up to the degree). Bisection depth grows with k.",
    );
    let separations = powers_of_two(0, 10);
    for degree in [2usize, 8, 32] {
        atlas.sweep(
            &format!("degree {degree}"),
            &separations,
            |k, _| univariate(reals(&close_pair(degree, k as u32))),
            isolate,
        );
    }
    atlas.sweep(
        "Mignotte degree 8, a = 2^k",
        &powers_of_two(0, 7),
        |k, _| univariate(reals(&mignotte(8, 1i64 << (k as u32).min(62)))),
        isolate,
    );

    atlas.operation(
        group,
        "Sturm chain construction",
        "polynomial degree",
        "Builds the exact fraction-free Sturm sequence of p and p'.",
    );
    for kind in POLY_KINDS {
        atlas.sweep(
            kind,
            &degrees,
            |m, rng| poly(kind, m as usize, rng),
            |p| UnivariateSturmSequence::new(p, POLICY),
        );
    }

    atlas.operation(
        group,
        "Sturm root count",
        "polynomial degree",
        "Counts distinct roots in (-2^80, 2^80] with a prebuilt Sturm chain (sign variations at both endpoints).",
    );
    let bound = Real::new(Rational::from_bigint(BigInt::from(1) << 80usize));
    for kind in POLY_KINDS {
        let (lower, upper) = (-bound.clone(), bound.clone());
        atlas.sweep(
            kind,
            &powers_of_two(1, 6),
            |m, rng| UnivariateSturmSequence::new(&poly(kind, m as usize, rng), POLICY),
            move |chain| {
                chain
                    .as_ref()
                    .and_then(|c| c.count_distinct_roots(&lower, &upper, POLICY))
            },
        );
    }

    atlas.operation(
        group,
        "Descartes sign bound",
        "polynomial degree",
        "Descartes' rule of signs on p(x) and p(-x): positive/negative root bounds from one expression row.",
    );
    for kind in POLY_KINDS {
        atlas.sweep(
            kind,
            &degrees,
            |m, rng| univariate(poly(kind, m as usize, rng)),
            |u| {
                count_descartes_univariate_polynomial_expr(0, &u.expr, &u.problem, POLICY).status
                    == DescartesRootCountStatus::Counted
            },
        );
    }

    atlas.operation(
        group,
        "Bernstein root bound",
        "polynomial degree",
        "Exact conversion to Bernstein form on [-(d+1), d+1] and sign-variation bound.",
    );
    for kind in POLY_KINDS {
        atlas.sweep(
            kind,
            &degrees,
            |m, rng| (univariate(poly(kind, m as usize, rng)), int(m as i64 + 1)),
            |(u, r)| {
                count_bernstein_univariate_polynomial_interval_expr(
                    0,
                    &u.expr,
                    &u.problem,
                    -r.clone(),
                    r.clone(),
                    POLICY,
                )
                .status
                    == BernsteinRootCountStatus::Counted
            },
        );
    }

    atlas.operation(
        group,
        "Bernstein subdivision",
        "polynomial degree",
        "Recursive Bernstein bisection on [-(d+1), d+1] to depth 96. Fails (depth limit) when two roots are closer than the depth resolves, e.g. Mignotte at high degree.",
    );
    for kind in POLY_KINDS {
        atlas.sweep(
            kind,
            &powers_of_two(1, 6),
            |m, rng| (univariate(poly(kind, m as usize, rng)), int(m as i64 + 1)),
            |(u, r)| {
                subdivide_bernstein_univariate_polynomial_interval_expr(
                    0,
                    &u.expr,
                    &u.problem,
                    -r.clone(),
                    r.clone(),
                    BernsteinSubdivisionConfig {
                        policy: POLICY,
                        max_depth: 96,
                    },
                )
                .status
                    == BernsteinSubdivisionStatus::Completed
            },
        );
    }

    atlas.operation(
        group,
        "refine isolated root",
        "bisection steps",
        "Validates one isolating interval and bisects it the given number of times, keeping exactly one root (Sturm counts or endpoint signs).",
    );
    let refine_kinds: [(&str, MakePoly); 4] = [
        ("√2 (x²−2)", |_| radical(2, 2)),
        ("∛3 (x³−3)", |_| radical(3, 3)),
        ("Mignotte deg 8 cluster root", |_| reals(&mignotte(8, 16))),
        ("dense deg 16, 32-bit", |rng| dense_int(rng, 16, 32)),
    ];
    for (kind, make_poly) in refine_kinds {
        atlas.sweep(
            kind,
            &powers_of_two(0, 10),
            |m, rng| {
                let coefficients = make_poly(rng);
                let u = univariate(coefficients.clone());
                let all = intervals(&u);
                let interval = all
                    .iter()
                    .filter(|i| i.exact_root.is_none() && i.distinct_root_count == 1)
                    .nth(if kind.starts_with("Mignotte") { 1 } else { 0 })
                    .or_else(|| all.first())
                    .cloned()
                    .expect("a real root");
                (coefficients, interval, m as usize)
            },
            |(p, interval, steps)| {
                let report = refine_isolated_univariate_polynomial_interval(
                    p,
                    interval,
                    RootIsolationConfig {
                        policy: POLICY,
                        max_interval_width: None,
                        max_refinement_steps: *steps,
                    },
                );
                matches!(
                    report.status,
                    IsolatedRootRefinementStatus::Refined | IsolatedRootRefinementStatus::ExactRoot
                )
            },
        );
    }

    atlas.operation(
        group,
        "square-free part",
        "polynomial degree",
        "p / gcd(p, p') by exact polynomial gcd and division.",
    );
    let square_free_kinds = [
        "square-free dense 32-bit",
        "p²·q (dense)",
        "Wilkinson² (all double)",
    ];
    for kind in square_free_kinds {
        atlas.sweep(
            kind,
            &degrees,
            |m, rng| {
                let degree = m as usize;
                match kind {
                    "square-free dense 32-bit" => dense_int(rng, degree, 32),
                    "p²·q (dense)" => {
                        let p = dense_int(rng, (degree / 4).max(1), 16);
                        let q = dense_int(rng, (degree / 2).max(1), 16);
                        rmul(&rmul(&p, &p), &q)
                    }
                    _ => {
                        let w = wilkinson((degree / 2).max(1));
                        reals(&imul(&w, &w))
                    }
                }
            },
            |p| square_free_part(p.clone(), POLICY),
        );
    }

    atlas.operation(
        group,
        "nonnegativity certificate",
        "polynomial degree",
        "Decides p ≥ 0 on a closed interval with a Sturm chain and inward Taylor signs. Both proved answers count as success.",
    );
    let nonneg_kinds = [
        "q² + 1 on [-2, 2] (true)",
        "(x − 1/3)²·(q² + 1) (touching)",
        "Wilkinson on [0, d+1] (false)",
    ];
    for kind in nonneg_kinds {
        atlas.sweep(
            kind,
            &degrees,
            |m, rng| {
                let degree = m as usize;
                let q = dense_int(rng, (degree / 2).max(1), 16);
                let mut positive = rmul(&q, &q);
                positive[0] = positive[0].clone() + Real::one();
                match kind {
                    "q² + 1 on [-2, 2] (true)" => (positive, int(-2), int(2)),
                    "(x − 1/3)²·(q² + 1) (touching)" => {
                        let factor = [frac(-1, 3), Real::one()];
                        (rmul(&rmul(&factor, &factor), &positive), int(-2), int(2))
                    }
                    _ => (
                        reals(&wilkinson(degree)),
                        Real::zero(),
                        int(degree as i64 + 1),
                    ),
                }
            },
            |(p, lower, upper)| certify_univariate_polynomial_nonnegative(p, lower, upper, POLICY),
        );
    }

    atlas.operation(
        group,
        "one root in open interval",
        "polynomial degree",
        "Decides whether exactly one distinct root lies in an open interval (endpoint signs, monotonicity, Bernstein, then Sturm). Either proved answer counts as success.",
    );
    let one_root_kinds = [
        "x^d − 2 on (1, 2) (yes)",
        "Mignotte a=16 on (0, 1/8) (two roots)",
        "Wilkinson on (1/2, 3/2) (yes)",
    ];
    for kind in one_root_kinds {
        atlas.sweep(
            kind,
            &degrees,
            |m, _| {
                let degree = m as usize;
                match kind {
                    "x^d − 2 on (1, 2) (yes)" => (radical(degree, 2), int(1), int(2)),
                    "Mignotte a=16 on (0, 1/8) (two roots)" => {
                        (reals(&mignotte(degree, 16)), Real::zero(), frac(1, 8))
                    }
                    _ => (reals(&wilkinson(degree)), frac(1, 2), frac(3, 2)),
                }
            },
            |(p, lower, upper)| {
                polynomial_has_one_distinct_root_in_open_interval(p, lower, upper, POLICY)
            },
        );
    }
}

fn polynomial_algebra(atlas: &mut Atlas) {
    let degrees = DEGREES.to_vec();
    let group = "polynomial algebra";

    atlas.operation(
        group,
        "exact division",
        "dividend degree",
        "Exact univariate division a·b / b with deg a = deg b = d/2.",
    );
    for kind in ["integer 32-bit", "rational 16-bit", "Wilkinson / half"] {
        atlas.sweep(
            kind,
            &degrees,
            |m, rng| {
                let half = (m as usize / 2).max(1);
                match kind {
                    "integer 32-bit" => {
                        let (a, b) = (dense_int(rng, half, 32), dense_int(rng, half, 32));
                        (rmul(&a, &b), b)
                    }
                    "rational 16-bit" => {
                        let (a, b) = (dense_rational(rng, half, 16), dense_rational(rng, half, 16));
                        (rmul(&a, &b), b)
                    }
                    _ => {
                        let w = wilkinson(2 * half);
                        let factors: Vec<_> = (1..=half as i64)
                            .map(|i| (BigInt::from(1), BigInt::from(i)))
                            .collect();
                        (reals(&w), reals(&from_factors(&factors)))
                    }
                }
            },
            |(a, b)| divide_univariate_polynomial_exact(a, b),
        );
    }

    atlas.operation(
        group,
        "exact gcd",
        "operand degree",
        "Exact univariate gcd of two polynomials of degree d.",
    );
    for kind in [
        "coprime dense 32-bit",
        "shared factor deg d/2 (integer)",
        "shared factor deg d/2 (rational)",
    ] {
        atlas.sweep(
            kind,
            &degrees,
            |m, rng| shared_factor_pair(kind, m as usize, rng),
            |(a, b)| greatest_common_divisor_univariate_polynomials_exact(a, b),
        );
    }
    atlas.fuzz(
        "random pair, degree ≤ m",
        &powers_of_two(1, 6),
        |m, rng| (fuzz_poly(m, rng), fuzz_poly(m, rng)),
        |(a, b)| greatest_common_divisor_univariate_polynomials_exact(a, b),
    );

    atlas.operation(
        group,
        "modular coprimality",
        "operand degree",
        "Modular-image coprimality filter; success when the filter decides (coprime or common factor likely).",
    );
    for kind in ["coprime dense 32-bit", "shared factor deg d/2 (integer)"] {
        atlas.sweep(
            kind,
            &degrees,
            |m, rng| shared_factor_pair(kind, m as usize, rng),
            |(a, b)| {
                univariate_polynomials_modular_coprimality(a, b) != ModularCoprimality::Inconclusive
            },
        );
    }

    atlas.operation(
        group,
        "resultant",
        "operand degree",
        "Exact Sylvester resultant of two degree-d polynomials.",
    );
    let resultant_kinds = [
        "integer 8-bit",
        "integer 64-bit",
        "rational 16-bit",
        "shared root (zero)",
    ];
    for kind in resultant_kinds {
        atlas.sweep(
            kind,
            &degrees,
            |m, rng| resultant_pair(kind, m as usize, rng),
            |(a, b)| resultant_univariate_polynomials(a, b, MIN_PRECISION).map(|r| r.resultant),
        );
    }
    atlas.fuzz(
        "random pair, degree ≤ m",
        &powers_of_two(1, 6),
        |m, rng| (fuzz_poly(m, rng), fuzz_poly(m, rng)),
        |(a, b)| resultant_univariate_polynomials(a, b, MIN_PRECISION).map(|r| r.resultant),
    );
    atlas.grid(
        "dense integer",
        ("operand degree", &powers_of_two(1, 6)),
        ("coefficient bits", &powers_of_two(0, 10)),
        |degree, bits, rng| {
            (
                dense_int(rng, degree as usize, bits as u32),
                dense_int(rng, degree as usize, bits as u32),
            )
        },
        |(a, b)| resultant_univariate_polynomials(a, b, MIN_PRECISION).map(|r| r.resultant),
    );

    atlas.operation(
        group,
        "subresultant chain",
        "operand degree",
        "Full exact subresultant chain of two degree-d polynomials.",
    );
    for kind in resultant_kinds {
        atlas.sweep(
            kind,
            &[2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0, 12.0, 14.0, 16.0],
            |m, rng| resultant_pair(kind, m as usize, rng),
            |(a, b)| {
                subresultant_chain_univariate_polynomials(a, b, MIN_PRECISION)
                    .map(|r| r.last_nonzero_degree)
            },
        );
    }

    atlas.operation(
        group,
        "linear-fractional composition",
        "polynomial degree",
        "(cx + d)^n · p((ax + b)/(cx + d)) with a, b, c, d = 3, 1, 2, 5.",
    );
    for kind in ["dense 32-bit", "rational 16-bit", "Wilkinson"] {
        atlas.sweep(
            kind,
            &degrees,
            |m, rng| poly(kind, m as usize, rng),
            |p| {
                compose_univariate_polynomial_linear_fractional(
                    p,
                    &int(3),
                    &int(1),
                    &int(2),
                    &int(5),
                    POLICY,
                )
            },
        );
    }
}

fn shared_factor_pair(kind: &str, degree: usize, rng: &mut Rng) -> (Vec<Real>, Vec<Real>) {
    let half = (degree / 2).max(1);
    match kind {
        "coprime dense 32-bit" => (dense_int(rng, degree, 32), dense_int(rng, degree, 32)),
        "shared factor deg d/2 (integer)" => {
            let g = dense_int(rng, half, 16);
            (
                rmul(&g, &dense_int(rng, half, 16)),
                rmul(&g, &dense_int(rng, half, 16)),
            )
        }
        _ => {
            let g = dense_rational(rng, half, 8);
            (
                rmul(&g, &dense_rational(rng, half, 8)),
                rmul(&g, &dense_rational(rng, half, 8)),
            )
        }
    }
}

fn resultant_pair(kind: &str, degree: usize, rng: &mut Rng) -> (Vec<Real>, Vec<Real>) {
    match kind {
        "integer 8-bit" => (dense_int(rng, degree, 8), dense_int(rng, degree, 8)),
        "integer 64-bit" => (dense_int(rng, degree, 64), dense_int(rng, degree, 64)),
        "rational 16-bit" => (
            dense_rational(rng, degree, 16),
            dense_rational(rng, degree, 16),
        ),
        _ => {
            let root = [int(-rng.range(1, 9)), Real::one()];
            let (a, b) = (
                dense_int(rng, degree.saturating_sub(1).max(1), 16),
                dense_int(rng, degree.saturating_sub(1).max(1), 16),
            );
            (rmul(&root, &a), rmul(&root, &b))
        }
    }
}

fn algebraic_numbers(atlas: &mut Atlas) {
    let group = "algebraic numbers";

    atlas.operation(
        group,
        "compare close roots",
        "log2 of separation (≈ 2^-k)",
        "Orders the positive roots of x^n − 2 and x^n − (2 + 2^-k) by alternating interval refinement (up to 4096 rounds of 8 bisections), or by the sign of their difference.",
    );
    for (kind, n, by_difference) in [
        ("quadratic", 2usize, false),
        ("cubic", 3, false),
        ("quintic", 5, false),
        ("cubic, difference method", 3, true),
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 9),
            |k, _| {
                let mut shifted = radical(n, 2);
                shifted[0] = -(int(2) + dyadic_real(1, k as u32));
                (top_root(radical(n, 2)), top_root(shifted))
            },
            |(left, right)| {
                let config = AlgebraicRootRefinementComparisonConfig {
                    policy: POLICY,
                    max_refinement_rounds: 4096,
                    steps_per_round: 8,
                };
                let status = if by_difference {
                    compare_algebraic_root_representations_by_difference(left, right, config)
                        .comparison
                        .status
                } else {
                    compare_algebraic_root_representations_with_refinement(left, right, config)
                        .comparison
                        .status
                };
                status == AlgebraicRootComparisonStatus::Compared
            },
        );
    }

    atlas.operation(
        group,
        "root arithmetic",
        "defining degree n (operands x^n − 2, x^n − 3)",
        "Exact arithmetic on two represented roots (operands pre-refined by 32 bisections); the result's defining polynomial has degree up to n², built by resultants. From n = 4 (degree 16) the bounded resultant package refuses (genuine Undecided).",
    );
    let degrees = ladder(1, 8, 8);
    for (kind, op) in [
        ("add", AlgebraicRootArithmeticOp::Add),
        ("subtract", AlgebraicRootArithmeticOp::Subtract),
        ("multiply", AlgebraicRootArithmeticOp::Multiply),
        ("divide", AlgebraicRootArithmeticOp::Divide),
    ] {
        atlas.sweep(
            kind,
            &degrees,
            |m, _| {
                (
                    refined_root(radical(m as usize, 2)),
                    refined_root(radical(m as usize, 3)),
                )
            },
            move |(left, right)| {
                matches!(
                    arithmetic_algebraic_root_representations(left, Some(right), op, POLICY).status,
                    AlgebraicRootArithmeticStatus::ComputedExactRationalWitness
                        | AlgebraicRootArithmeticStatus::ComputedExactRealWitness
                        | AlgebraicRootArithmeticStatus::ComputedRepresentation
                )
            },
        );
    }
    atlas.fuzz(
        "add/multiply, random monic roots of degree ≤ m",
        &ladder(1, 6, 6),
        |m, rng| {
            let root = |rng: &mut Rng| loop {
                let degree = rng.range(1, m as i64) as usize;
                let mut p = dense_int(rng, degree, 8);
                p[degree] = Real::one();
                if let Some(r) = roots_of(p).pop() {
                    break refined_represented_root(&r, REFINEMENT_STEPS);
                }
            };
            let op = if rng.coin() {
                AlgebraicRootArithmeticOp::Add
            } else {
                AlgebraicRootArithmeticOp::Multiply
            };
            (root(rng), root(rng), op)
        },
        |(left, right, op)| {
            matches!(
                arithmetic_algebraic_root_representations(left, Some(right), *op, POLICY).status,
                AlgebraicRootArithmeticStatus::ComputedExactRationalWitness
                    | AlgebraicRootArithmeticStatus::ComputedExactRealWitness
                    | AlgebraicRootArithmeticStatus::ComputedRepresentation
            )
        },
    );

    atlas.operation(
        group,
        "binary root transform",
        "defining degree n (operands x^n − 2, x^n − 3)",
        "Resultant-based image of two represented roots (pre-refined by 32 bisections) under +/×, with image isolation and validation. From n = 4 the bounded package reports UnsupportedDegree.",
    );
    for (kind, op) in [
        ("add", AlgebraicRootArithmeticOp::Add),
        ("multiply", AlgebraicRootArithmeticOp::Multiply),
    ] {
        atlas.sweep(
            kind,
            &degrees,
            |m, _| {
                (
                    refined_root(radical(m as usize, 2)),
                    refined_root(radical(m as usize, 3)),
                )
            },
            move |(left, right)| {
                transform_algebraic_roots_binary(left, right, op, POLICY).status
                    == AlgebraicRootBinaryTransformStatus::Transformed
            },
        );
    }

    atlas.operation(
        group,
        "square root of a root",
        "defining degree n",
        "Represents the positive square root of a represented positive root (pre-refined by 32 bisections). n = 1 is the exact rational point 3.",
    );
    for kind in ["x^n − 3", "x^n − x − 1"] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 6),
            |m, _| {
                let n = m as usize;
                let mut p = radical(n.max(1), 3);
                if kind == "x^n − x − 1" {
                    p = radical(n.max(2), 1);
                    p[1] = int(-1);
                }
                refined_root(p)
            },
            |root| {
                square_root_algebraic_root_representation(root, 1).status
                    == AlgebraicRootSquareRootStatus::Transformed
            },
        );
    }

    let root_kinds: [(&str, FixedPoly); 3] = [
        ("√2", || radical(2, 2)),
        ("∛2", || radical(3, 2)),
        ("root of x^8 − x − 1", || {
            let mut p = radical(8, 1);
            p[1] = int(-1);
            p
        }),
    ];

    atlas.operation(
        group,
        "evaluate polynomial at root",
        "evaluated polynomial degree",
        "Sign/enclosure of a random 16-bit polynomial at a represented root pre-refined by 32 bisections (exact witness or certified interval).",
    );
    for (kind, defining) in root_kinds {
        atlas.sweep(
            kind,
            &powers_of_two(1, 7),
            |m, rng| (refined_root(defining()), dense_int(rng, m as usize, 16)),
            |(root, p)| {
                matches!(
                    evaluate_polynomial_at_algebraic_root(root, p, POLICY).status,
                    AlgebraicRootPolynomialEvaluationStatus::EvaluatedExactRationalWitness
                        | AlgebraicRootPolynomialEvaluationStatus::EvaluatedExactRealWitness
                        | AlgebraicRootPolynomialEvaluationStatus::IntervalCertifiedPositive
                        | AlgebraicRootPolynomialEvaluationStatus::IntervalCertifiedNegative
                )
            },
        );
    }

    atlas.operation(
        group,
        "sign at selected root",
        "predicate degree",
        "Exact sign of a predicate polynomial at the root selected by an isolating interval (gcd/remainder sequence).",
    );
    for (kind, defining) in root_kinds {
        atlas.sweep(
            kind,
            &powers_of_two(1, 7),
            |m, rng| {
                let root = refined_root(defining());
                (
                    root.polynomial_coefficients.clone(),
                    dense_int(rng, m as usize, 16),
                    root.interval,
                )
            },
            |(defining, predicate, interval)| sign_at_selected_root(defining, predicate, interval),
        );
    }
    atlas.sweep(
        "∛2, vanishing predicate",
        &powers_of_two(1, 7),
        |m, rng| {
            let root = refined_root(radical(3, 2));
            let predicate = rmul(
                &root.polynomial_coefficients,
                &dense_int(rng, m as usize, 16),
            );
            (
                root.polynomial_coefficients.clone(),
                predicate,
                root.interval,
            )
        },
        |(defining, predicate, interval)| sign_at_selected_root(defining, predicate, interval),
    );

    atlas.operation(
        group,
        "polynomial image of root",
        "image polynomial degree",
        "Represents q(α) for a random 8-bit q and a pre-refined root α: resultant elimination, image isolation and validation. Larger image degrees exceed the bounded resultant package (genuine UnsupportedDegree).",
    );
    for (kind, defining) in root_kinds {
        atlas.sweep(
            kind,
            &ladder(1, 16, 8),
            |m, rng| (refined_root(defining()), dense_int(rng, m as usize, 8)),
            |(root, image)| {
                transform_algebraic_root_polynomial_image(root, image, POLICY).status
                    == AlgebraicRootPolynomialImageStatus::Transformed
            },
        );
    }

    atlas.operation(
        group,
        "affine and Möbius transforms",
        "defining degree n (root of x^n − 2)",
        "Represents 3α/2 + 1/3 (affine) or (3α + 1)/(2α + 5) (Möbius) for a represented root α.",
    );
    for kind in ["affine", "Möbius"] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 6),
            |m, _| refined_root(radical(m as usize, 2)),
            move |root| {
                if kind == "affine" {
                    transform_algebraic_root_affine(root, frac(3, 2), frac(1, 3), POLICY).status
                        == AlgebraicRootAffineTransformStatus::Transformed
                } else {
                    transform_algebraic_root_mobius(root, int(3), int(1), int(2), int(5), POLICY)
                        .status
                        == AlgebraicRootMobiusTransformStatus::Transformed
                }
            },
        );
    }
}

fn elimination(atlas: &mut Atlas) {
    let group = "elimination";
    let degrees = [
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0, 12.0, 14.0, 16.0,
    ];

    atlas.operation(
        group,
        "bivariate system resultant",
        "total degree of each equation",
        "Eliminates y from two dense bivariate polynomials (resultant degree up to d²) by exact sampling and interpolation.",
    );
    for kind in [
        "dense 8-bit",
        "dense 64-bit",
        "rational 8-bit",
        "sparse x^d + y^d − 2, y^d − x − 1",
    ] {
        atlas.sweep(
            kind,
            &degrees,
            |m, rng| bivariate_pair(kind, m as usize, 0, rng),
            |(f, g)| {
                resultant_bivariate_polynomial_system(
                    f,
                    g,
                    CurveResultantParameter::First,
                    resultant_config(),
                )
                .status
                    == CurveIntersectionResultantStatus::Constructed
            },
        );
    }
    atlas.grid(
        "dense integer",
        ("total degree", &[1.0, 2.0, 3.0, 4.0, 6.0, 8.0]),
        ("coefficient bits", &powers_of_two(0, 8)),
        |degree, bits, rng| bivariate_pair("dense", degree as usize, bits as u32, rng),
        |(f, g)| {
            resultant_bivariate_polynomial_system(
                f,
                g,
                CurveResultantParameter::First,
                resultant_config(),
            )
            .status
                == CurveIntersectionResultantStatus::Constructed
        },
    );

    atlas.operation(
        group,
        "parametric curve intersection",
        "curve degree",
        "Implicit resultant in one curve's parameter for two polynomial parametric plane curves.",
    );
    for (kind, bits, rational) in [
        ("8-bit", 8u32, false),
        ("64-bit", 64, false),
        ("rational 8-bit", 8, true),
    ] {
        atlas.sweep(
            kind,
            &ladder(1, 16, 7),
            |m, rng| {
                let n = m as usize;
                let curve = |rng: &mut Rng| {
                    let mut c = || {
                        if rational {
                            dense_rational(rng, n, bits)
                        } else {
                            dense_int(rng, n, bits)
                        }
                    };
                    let x = c();
                    PolynomialParametricCurve2::new(x, c())
                };
                (curve(rng), curve(rng))
            },
            |(a, b)| {
                resultant_parametric_curve_intersection(
                    a,
                    b,
                    CurveResultantParameter::First,
                    resultant_config(),
                )
                .status
                    == CurveIntersectionResultantStatus::Constructed
            },
        );
    }

    atlas.operation(
        group,
        "bivariate exact division",
        "factor total degree",
        "Exact division f·g / g of dense bivariate polynomials.",
    );
    for (kind, bits) in [("integer 8-bit", 8u32), ("integer 64-bit", 64)] {
        atlas.sweep(
            kind,
            &degrees,
            |m, rng| {
                let f = bivariate_dense(rng, m as usize, |rng| rand_int(rng, bits));
                let g = bivariate_dense(rng, m as usize, |rng| rand_int(rng, bits));
                (bivariate_mul(&f, &g), g)
            },
            |(product, g)| divide_bivariate_polynomial_exact(product, g),
        );
    }

    atlas.operation(
        group,
        "trivariate resultant under constraint",
        "degree per variable",
        "Eliminates the third variable of a dense trivariate polynomial against a univariate constraint on it.",
    );
    for kind in [
        "dense 8-bit, constraint z^n − 2",
        "dense 8-bit, random constraint",
    ] {
        atlas.sweep(
            kind,
            &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            |m, rng| {
                let n = m as usize;
                let tensor = (0..=n)
                    .map(|_| (0..=n).map(|_| dense_int(rng, n, 8)).collect())
                    .collect();
                let constraint = if kind.contains("z^n") {
                    radical(n, 2)
                } else {
                    dense_int(rng, n, 8)
                };
                (TrivariatePolynomial::new(tensor), constraint)
            },
            |(polynomial, constraint)| {
                resultant_trivariate_polynomial_univariate_constraint(
                    polynomial,
                    constraint,
                    TrivariatePolynomialAxis::Third,
                    resultant_config(),
                )
                .status
                    == TrivariateConstraintResultantStatus::Constructed
            },
        );
    }
}

fn bivariate_pair(
    kind: &str,
    degree: usize,
    grid_bits: u32,
    rng: &mut Rng,
) -> (BivariatePolynomial, BivariatePolynomial) {
    let mut pair = |coefficient: &dyn Fn(&mut Rng) -> Real| {
        (
            bivariate_dense(rng, degree, coefficient),
            bivariate_dense(rng, degree, coefficient),
        )
    };
    match kind {
        "dense" => pair(&|rng| rand_int(rng, grid_bits)),
        "dense 8-bit" => pair(&|rng| rand_int(rng, 8)),
        "dense 64-bit" => pair(&|rng| rand_int(rng, 64)),
        "rational 8-bit" => pair(&|rng| rand_rational(rng, 8)),
        _ => {
            let mut f = vec![vec![Real::zero(); degree + 1]; degree + 1];
            f[degree][0] = Real::one();
            f[0][degree] = Real::one();
            f[0][0] = int(-2);
            let mut g = vec![vec![Real::zero(); degree + 1]; degree + 1];
            g[0][degree] = Real::one();
            g[1.min(degree)][0] = int(-1);
            g[0][0] = g[0][0].clone() - Real::one();
            (BivariatePolynomial::new(f), BivariatePolynomial::new(g))
        }
    }
}

fn linear_algebra(atlas: &mut Atlas) {
    let group = "linear algebra";
    let dims = [
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0, 24.0, 32.0, 48.0, 64.0,
    ];
    let kinds = [
        ("integer 8-bit", "integer", 8u32),
        ("integer 64-bit", "integer", 64),
        ("rational 8-bit", "rational", 8),
        ("√2 field, 4-bit", "√2 field", 4),
    ];

    atlas.operation(
        group,
        "Bareiss determinant",
        "matrix dimension",
        "Fraction-free exact determinant of a dense random matrix.",
    );
    for (kind, entries, bits) in kinds {
        atlas.sweep(
            kind,
            &dims,
            |m, rng| matrix(entries, m as usize, bits, rng),
            |a| determinant_bareiss(a, MIN_PRECISION).map(|r| r.determinant),
        );
    }
    atlas.fuzz(
        "integer, random dimension ≤ m and bits",
        &[2.0, 4.0, 8.0, 16.0, 32.0],
        |m, rng| {
            let n = rng.range(1, m as i64) as usize;
            let bits = rng.range(1, 128) as u32;
            matrix("integer", n, bits, rng)
        },
        |a| determinant_bareiss(a, MIN_PRECISION).map(|r| r.determinant),
    );
    atlas.grid(
        "integer",
        ("matrix dimension", &[2.0, 4.0, 8.0, 16.0, 32.0, 64.0]),
        ("entry bits", &powers_of_two(0, 10)),
        |n, bits, rng| matrix("integer", n as usize, bits as u32, rng),
        |a| determinant_bareiss(a, MIN_PRECISION).map(|r| r.determinant),
    );

    atlas.operation(
        group,
        "dense Bareiss solve",
        "matrix dimension",
        "Exact solve A x = b with Cramer numerators and exact residual replay.",
    );
    for (kind, entries, bits) in kinds {
        atlas.sweep(
            kind,
            &dims,
            |m, rng| {
                let n = m as usize;
                let a = matrix(entries, n, bits, rng);
                let b: Vec<Real> = (0..n).map(|_| rand_int(rng, bits)).collect();
                (a, b)
            },
            |(a, b)| {
                solve_dense_linear_system_bareiss(a, b, MIN_PRECISION, POLICY).map(|r| r.solution)
            },
        );
    }

    atlas.operation(
        group,
        "sparse Bareiss solve",
        "system dimension (tridiagonal)",
        "Exact solve of a diagonally dominant tridiagonal system with three sparse strategies.",
    );
    for kind in ["general", "pattern-preserving", "minimum-degree"] {
        atlas.sweep(
            kind,
            &powers_of_two(1, 8),
            |m, rng| {
                let n = m as usize;
                let (terms, rhs, _) = tridiagonal(n, false, rng);
                (n, terms, rhs)
            },
            move |(n, terms, rhs)| match kind {
                "general" => {
                    solve_sparse_linear_system_bareiss(*n, *n, terms, rhs, MIN_PRECISION).is_ok()
                }
                "pattern-preserving" => solve_sparse_linear_system_bareiss_pattern_preserving(
                    *n,
                    *n,
                    terms,
                    rhs,
                    MIN_PRECISION,
                )
                .is_ok(),
                _ => solve_sparse_linear_system_bareiss_minimum_degree(
                    *n,
                    *n,
                    terms,
                    rhs,
                    MIN_PRECISION,
                )
                .is_ok(),
            },
        );
    }

    atlas.operation(
        group,
        "sparse residual replay",
        "system dimension (tridiagonal)",
        "Exact replay of A x − b for a candidate that solves the system exactly.",
    );
    for (kind, rational) in [("integer", false), ("rational", true)] {
        atlas.sweep(
            kind,
            &powers_of_two(1, 12),
            |m, rng| {
                let n = m as usize;
                let (terms, rhs, solution) = tridiagonal(n, rational, rng);
                (n, terms, rhs, solution)
            },
            |(n, terms, rhs, x)| {
                replay_sparse_linear_residuals(*n, *n, terms, rhs, x, MIN_PRECISION).is_ok()
            },
        );
    }

    atlas.operation(
        group,
        "f64 damped normal step",
        "matrix dimension",
        "Lossy dense f64 damped normal-equation proposal step (Levenberg damping 1e-9).",
    );
    for kind in ["random well-conditioned", "Hilbert (ill-conditioned)"] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 8),
            |m, rng| {
                let n = m as usize;
                let jacobian: Vec<Vec<f64>> = (0..n)
                    .map(|i| {
                        (0..n)
                            .map(|j| {
                                if kind.starts_with("Hilbert") {
                                    1.0 / (i + j + 1) as f64
                                } else if i == j {
                                    4.0 + rng.unit()
                                } else {
                                    rng.unit() - 0.5
                                }
                            })
                            .collect()
                    })
                    .collect();
                let residuals: Vec<f64> = (0..n).map(|_| rng.unit()).collect();
                (jacobian, residuals)
            },
            |(jacobian, residuals)| {
                DenseLinearBackend
                    .solve_damped_normal(jacobian, residuals, 1e-9)
                    .map(|(step, _)| step)
            },
        );
    }

    atlas.operation(
        group,
        "simplex projection",
        "ambient dimension (n + 1 vertices)",
        "Certified closest point of a simplex to the origin by exact face enumeration (the package admits at most 8 vertices).",
    );
    for (kind, rational) in [("integer 8-bit", false), ("rational 8-bit", true)] {
        atlas.sweep(
            kind,
            &ladder(1, 7, 7),
            |m, rng| {
                let n = m as usize;
                (0..=n)
                    .map(|_| {
                        (0..n)
                            .map(|_| {
                                if rational {
                                    rand_rational(rng, 8)
                                } else {
                                    rand_int(rng, 8)
                                }
                            })
                            .collect::<Vec<_>>()
                    })
                    .collect::<Vec<_>>()
            },
            |vertices| {
                project_origin_onto_simplex(vertices)
                    .is_ok_and(|r| r.status == SimplexProjectionStatus::Certified)
            },
        );
    }
}

fn certification(atlas: &mut Atlas) {
    let group = "certification";
    let dims = [1.0, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0, 16.0, 24.0, 32.0];

    atlas.operation(
        group,
        "candidate certification",
        "number of variables",
        "Structural analysis plus exact residual replay of n coupled quadratics at a candidate. Success means decided: all satisfied or a certified violation.",
    );
    for (kind, root, offset) in [
        ("rational root", "rational", Real::zero()),
        ("√2-shifted root", "√2", Real::zero()),
        ("rational, off by 2^-64", "rational", dyadic_real(1, 64)),
    ] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 8),
            |m, _| quadratic_system(m as usize, root, &offset).0,
            |problem| {
                let analysis = problem.analyze();
                let report = certify_candidate(&analysis, &context_from_problem(problem));
                report.all_satisfied() || report.has_certified_violation()
            },
        );
    }

    atlas.operation(
        group,
        "quadratic Krawczyk box",
        "number of variables",
        "Interval Krawczyk existence/uniqueness test for n coupled quadratics on a box around a candidate offset by 1/64. The radius-1/2 box is too wide to contract (genuine refusal).",
    );
    for (kind, root, radius) in [
        ("rational, radius 1/16", "rational", frac(1, 16)),
        ("√2-shifted, radius 1/16", "√2", frac(1, 16)),
        ("rational, radius 1/2", "rational", frac(1, 2)),
    ] {
        atlas.sweep(
            kind,
            &dims,
            |m, _| {
                let (problem, symbols, _) = quadratic_system(m as usize, root, &frac(1, 64));
                (problem, balls(&symbols, &radius))
            },
            |(problem, balls)| {
                let analysis = problem.analyze();
                certify_multivariate_quadratic_krawczyk_box(
                    &analysis,
                    &context_from_problem(problem),
                    balls,
                    POLICY,
                )
                .status
                    == MultivariateQuadraticKrawczykStatus::CertifiedUniqueRoot
            },
        );
    }

    atlas.operation(
        group,
        "interval box certification",
        "number of variables",
        "Interval-box residual certification with the retained multivariate-quadratic enclosure package. Success means decided. A box around a root cannot be decided (its residual enclosures contain zero: genuine Unknown); a box 1/2 away from the root proves a violation.",
    );
    for (kind, root, offset) in [
        ("rational, box excludes root", "rational", frac(1, 2)),
        ("√2-shifted, box excludes root", "√2", frac(1, 2)),
        (
            "rational, box around root (undecidable)",
            "rational",
            frac(1, 64),
        ),
    ] {
        atlas.sweep(
            kind,
            &dims,
            |m, _| {
                let (problem, symbols, _) = quadratic_system(m as usize, root, &offset);
                (problem, balls(&symbols, &frac(1, 16)))
            },
            |(problem, balls)| {
                let analysis = problem.analyze();
                let status = certify_interval_box_candidate(
                    &analysis,
                    &context_from_problem(problem),
                    balls,
                    IntervalBoxCertificationPackage::MultivariateQuadratic,
                    POLICY,
                )
                .status;
                matches!(
                    status,
                    IntervalBoxCertificationStatus::Certified
                        | IntervalBoxCertificationStatus::Violation
                )
            },
        );
    }

    atlas.operation(
        group,
        "affine Krawczyk box",
        "number of variables",
        "Krawczyk test for a diagonally dominant affine system on a box of radius 1/4 around a candidate offset by 1/64.",
    );
    atlas.sweep(
        "dense diagonally dominant",
        &dims,
        |m, rng| {
            let (problem, symbols) = affine_system(m as usize, false, &frac(1, 64), rng);
            (problem, balls(&symbols, &frac(1, 4)))
        },
        |(problem, balls)| {
            let analysis = problem.analyze();
            certify_affine_krawczyk_box(&analysis, &context_from_problem(problem), balls, POLICY)
                .status
                == AffineKrawczykStatus::CertifiedUniqueRoot
        },
    );

    atlas.operation(
        group,
        "exact affine rank",
        "number of variables",
        "Exact rank analysis of the affine rows of a problem.",
    );
    for (kind, deficient) in [("full rank", false), ("rank n/2 (duplicated rows)", true)] {
        // The rank-deficient path is super-exponential in n; keep its steps small.
        let deficient_dims = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0];
        atlas.sweep(
            kind,
            if deficient {
                &deficient_dims[..]
            } else {
                &dims[..]
            },
            |m, rng| affine_system(m as usize, deficient, &Real::zero(), rng).0,
            |problem| {
                analyze_exact_affine_rank(&problem.analyze(), MIN_PRECISION).status
                    == ExactAffineRankStatus::Certified
            },
        );
    }

    atlas.operation(
        group,
        "nonlinear proposal solve",
        "number of variables",
        "Lossy f64 proposal iteration (up to 64 steps) for n coupled quadratics started 1/8 from a rational root, including exact residual evaluation each step. Success means converged; the dogleg and BFGS routes stop at the iteration limit on this family (genuine non-convergence).",
    );
    for (kind, engine) in [
        (
            "damped least squares",
            ProposalEngineKind::DampedLeastSquares,
        ),
        ("dogleg", ProposalEngineKind::Dogleg),
        ("BFGS", ProposalEngineKind::Bfgs),
        (
            "modified Newton",
            ProposalEngineKind::ModifiedNewtonLeastSquares,
        ),
    ] {
        atlas.sweep(
            kind,
            &[1.0, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0, 16.0],
            |m, _| quadratic_system(m as usize, "rational", &frac(1, 8)).0,
            move |problem| {
                solve_damped_least_squares(SolverState {
                    problem: problem.clone(),
                    config: SolverConfig {
                        max_iterations: 64,
                        proposal_engine: engine,
                        ..SolverConfig::default()
                    },
                })
                .reason
                    == ConvergenceReason::Converged
            },
        );
    }
}

fn expression_programs(atlas: &mut Atlas) {
    let group = "expression programs";
    let sizes = powers_of_two(0, 11);

    atlas.operation(
        group,
        "compile",
        "expression size (operations)",
        "Compiles one source expression tree into a flat validated program (includes source traversal and deduplication).",
    );
    for kind in EXPR_KINDS {
        atlas.sweep(
            kind,
            &sizes,
            |m, _| expression(kind, m as usize),
            |(e, variables)| {
                ExpressionProgram::compile(std::slice::from_ref(e), variables, &mut graph())
            },
        );
    }

    atlas.operation(
        group,
        "replay definition",
        "expression size (operations)",
        "Validates and rebuilds a program from its owned definition (includes cloning the definition).",
    );
    for kind in EXPR_KINDS {
        atlas.sweep(
            kind,
            &sizes,
            |m, _| program(kind, m as usize),
            |p| ExpressionProgram::from_definition(p.definition().clone(), &mut graph()),
        );
    }

    atlas.operation(
        group,
        "evaluate exact (Real)",
        "expression size (operations)",
        "Exact Real evaluation at x_i = (i+1)/3. Transcendental results stay symbolic/lazy; rational Horner values grow by ~1.6 bits per step.",
    );
    for kind in EXPR_KINDS {
        atlas.sweep(
            kind,
            &sizes,
            |m, _| {
                let p = program(kind, m as usize);
                let v = values(p.definition().variables.len());
                (p, v)
            },
            |(p, v)| p.eval_real(v, &mut graph()),
        );
    }
    atlas.fuzz(
        "random kind and size ≤ m",
        &powers_of_two(2, 6),
        |m, rng| {
            let kind = EXPR_KINDS[rng.below(4) as usize];
            let p = program(kind, rng.range(1, m as i64) as usize);
            let v = values(p.definition().variables.len());
            (p, v)
        },
        |(p, v)| p.eval_real(v, &mut graph()),
    );

    atlas.operation(
        group,
        "evaluate tree (Expr::eval_real)",
        "expression size (operations)",
        "Recursive exact evaluation of the source tree with a hash-map binding, for comparison with program evaluation.",
    );
    for kind in EXPR_KINDS {
        atlas.sweep(
            kind,
            &sizes,
            |m, _| {
                let (e, variables) = expression(kind, m as usize);
                let bindings = variables
                    .iter()
                    .copied()
                    .zip(values(variables.len()))
                    .collect();
                (e, bindings)
            },
            |(e, bindings): &(Expr, std::collections::HashMap<SymbolId, Real>)| {
                e.eval_real(bindings)
            },
        );
    }

    atlas.operation(
        group,
        "evaluate dyadic interval",
        "expression size (operations)",
        "Certified dyadic interval image on boxes of width 2^-20 at output precision 2^-64.",
    );
    for kind in EXPR_KINDS {
        atlas.sweep(
            kind,
            &sizes,
            |m, _| {
                let p = program(kind, m as usize);
                let b = boxes(p.definition().variables.len(), 20);
                (p, b)
            },
            |(p, b)| p.eval_dyadic(b, -64, &mut graph(), &mut arithmetic()),
        );
    }

    atlas.operation(
        group,
        "dyadic evaluation by domain width",
        "log2 of inverse box width (width 2^-k)",
        "Certified dyadic image of fixed expressions as the input box narrows; wide boxes expose dependency blow-up and domain failures.",
    );
    for (kind, source, size) in [
        ("Horner chain, 32 steps", "Horner chain", 32usize),
        ("transcendental chain, 16 steps", "transcendental chain", 16),
        (
            "multivariate quadratic, 16 variables",
            "multivariate quadratic",
            16,
        ),
    ] {
        atlas.sweep(
            kind,
            &(0..=12).map(|k| f64::from(k * 4)).collect::<Vec<_>>(),
            |k, _| {
                let p = program(source, size);
                let b = boxes(p.definition().variables.len(), k as u32);
                (p, b)
            },
            |(p, b)| p.eval_dyadic(b, -64, &mut graph(), &mut arithmetic()),
        );
    }

    atlas.operation(
        group,
        "float proposal estimate",
        "expression size (operations)",
        "Lossy f64 evaluation of a prepared float-proposal schedule.",
    );
    for kind in EXPR_KINDS {
        atlas.sweep(
            kind,
            &sizes,
            |m, _| {
                let p = program(kind, m as usize);
                let args: Vec<f64> = (0..p.definition().variables.len())
                    .map(|i| (i as f64 + 1.0) / 3.0)
                    .collect();
                (
                    p.float_proposal(&mut graph()).expect("float proposal"),
                    args,
                )
            },
            |(proposal, args)| proposal.estimate(args, &mut graph()),
        );
    }

    atlas.operation(
        group,
        "symbolic derivative (Expr)",
        "expression size (operations)",
        "Expr::derivative with respect to the first variable (tree rewriting, no simplification).",
    );
    for kind in EXPR_KINDS {
        atlas.sweep(
            kind,
            &powers_of_two(0, 9),
            |m, _| expression(kind, m as usize).0,
            |e| {
                let derivative = e.derivative(SymbolId(0));
                std::hint::black_box(&derivative);
                true
            },
        );
    }

    let jacobian_kinds = [
        "chain Σ x_i x_{i+1}",
        "dense Σ_{i,j} x_i x_j",
        "sin(x_i)·x_{i+1}",
    ];
    let jacobian_source = |kind: &str, n: usize| -> ExpressionProgram {
        let sources: Vec<Expr> = (0..n)
            .map(|i| match kind {
                "chain Σ x_i x_{i+1}" => add(mul(var(i), var((i + 1) % n)), var(i)),
                "dense Σ_{i,j} x_i x_j" => {
                    balanced(&(0..n).map(|j| mul(var(i), var(j))).collect::<Vec<_>>())
                }
                _ => mul(var(i).sin(), var((i + 1) % n)),
            })
            .collect();
        let variables: Vec<SymbolId> = (0..n).map(|i| SymbolId(i as u32)).collect();
        ExpressionProgram::compile(&sources, &variables, &mut graph()).expect("compiles")
    };
    let jacobian_dims = powers_of_two(0, 6);

    atlas.operation(
        group,
        "Jacobian program",
        "number of variables and outputs",
        "Builds the symbolic Jacobian program of n outputs in n variables.",
    );
    for kind in jacobian_kinds {
        atlas.sweep(
            kind,
            &jacobian_dims,
            |m, _| {
                let p = jacobian_source(kind, m as usize);
                let variables = p.definition().variables.clone();
                (p, variables)
            },
            |(p, variables)| p.jacobian(variables, &mut graph()),
        );
    }

    atlas.operation(
        group,
        "regular Jacobian enclosure",
        "number of variables and outputs",
        "Certified C1-regularity proof plus dyadic enclosure of all Jacobian entries on a box of width 2^-10.",
    );
    for kind in jacobian_kinds {
        atlas.sweep(
            kind,
            &jacobian_dims,
            |m, _| {
                let p = jacobian_source(kind, m as usize);
                let variables = p.definition().variables.clone();
                let jacobian = p.jacobian(&variables, &mut graph()).expect("jacobian");
                (jacobian, boxes(variables.len(), 10))
            },
            |(jacobian, b)| jacobian.enclose_regular(b, -64, &mut graph(), &mut arithmetic()),
        );
    }

    atlas.operation(
        group,
        "program composition",
        "outer expression size (operations)",
        "Composes an outer one-variable program with one output of an inner program, as a shared flat DAG.",
    );
    for (kind, outer_kind) in [
        ("Horner outer ∘ (x + sin y)", "Horner chain"),
        ("transcendental outer ∘ (x + sin y)", "transcendental chain"),
    ] {
        atlas.sweep(
            kind,
            &sizes,
            |m, _| {
                let outer = program(outer_kind, m as usize);
                let inner = ExpressionProgram::compile(
                    &[add(var(0), var(1).sin())],
                    &[SymbolId(0), SymbolId(1)],
                    &mut graph(),
                )
                .expect("compiles");
                (outer, inner)
            },
            |(outer, inner)| outer.compose(inner, &[0], &mut graph()),
        );
    }
}

fn taylor_models(atlas: &mut Atlas) {
    let group = "Taylor models";
    let orders = [
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 20.0, 24.0, 32.0,
    ];

    atlas.operation(
        group,
        "Taylor coefficient program",
        "Taylor order",
        "Builds the program computing every exact coefficient up to the order (repeated symbolic differentiation).",
    );
    for kind in TAYLOR_KINDS {
        atlas.sweep(
            kind,
            &orders,
            |m, _| {
                let requests: Vec<CoefficientRequest> = (0..=m as usize)
                    .map(|k| CoefficientRequest {
                        output: 0,
                        powers: if k == 0 {
                            vec![]
                        } else {
                            vec![(SymbolId(0), k)]
                        },
                    })
                    .collect();
                (taylor_program(kind), requests)
            },
            |(p, requests)| p.coefficients(requests, &mut graph()),
        );
    }

    atlas.operation(
        group,
        "Taylor model by order",
        "Taylor order",
        "Certified univariate Taylor model (exact coefficients plus dyadic remainder) on [-1/16, 1/16]. For 1/(1−x) from order 10 the unsimplified high-order derivative's denominator enclosure contains zero, so the model is refused (genuine ContainsZero).",
    );
    for kind in TAYLOR_KINDS {
        atlas.sweep(
            kind,
            &orders,
            |m, _| (taylor_program(kind), taylor_request(1, m as usize, 4)),
            |(p, request)| taylor_model(p, request.clone()),
        );
    }
    atlas.fuzz(
        "random kind, order ≤ m, width 2^-k random",
        &[2.0, 4.0, 8.0, 16.0],
        |m, rng| {
            let kind = TAYLOR_KINDS[rng.below(4) as usize];
            let request =
                taylor_request(1, rng.range(1, m as i64) as usize, rng.range(1, 20) as i32);
            (taylor_program(kind), request)
        },
        |(p, request)| taylor_model(p, request.clone()),
    );
    atlas.grid(
        "sin(x)·cos(x)",
        ("Taylor order", &[1.0, 2.0, 4.0, 8.0, 16.0]),
        (
            "log2 of inverse domain radius",
            &[0.0, 2.0, 4.0, 8.0, 16.0, 32.0],
        ),
        |order, k, _| {
            (
                taylor_program("sin(x)·cos(x)"),
                taylor_request(1, order as usize, k as i32),
            )
        },
        |(p, request)| taylor_model(p, request.clone()),
    );

    atlas.operation(
        group,
        "Taylor model by domain width",
        "log2 of inverse domain radius (domain [-2^-k, 2^-k])",
        "Order-4 Taylor model as the domain narrows. 1/(1−x) on [-1, 1] contains its pole and is refused.",
    );
    for kind in TAYLOR_KINDS {
        atlas.sweep(
            kind,
            &(0..=10).map(|k| f64::from(k * 4)).collect::<Vec<_>>(),
            |k, _| (taylor_program(kind), taylor_request(1, 4, k as i32)),
            |(p, request)| taylor_model(p, request.clone()),
        );
    }

    atlas.operation(
        group,
        "multivariate Taylor model",
        "number of variables",
        "Taylor model of one output Σ sin(x_i)·x_{i+1} in n variables on [-1/16, 1/16]^n; term count grows like C(n + order, order).",
    );
    for order in [1usize, 2, 3, 4] {
        atlas.sweep(
            &format!("order {order}"),
            &[1.0, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0, 16.0],
            |m, _| {
                let n = m as usize;
                let source = balanced(
                    &(0..n)
                        .map(|i| mul(var(i).sin(), var((i + 1) % n)))
                        .collect::<Vec<_>>(),
                );
                let variables: Vec<SymbolId> = (0..n).map(|i| SymbolId(i as u32)).collect();
                let p = ExpressionProgram::compile(&[source], &variables, &mut graph())
                    .expect("compiles");
                (p, taylor_request(n, order, 4))
            },
            |(p, request)| taylor_model(p, request.clone()),
        );
    }

    atlas.operation(
        group,
        "Taylor enclosure on subdomain",
        "Taylor order",
        "Encloses a prebuilt univariate Taylor model on the subdomain [0, 2^-5].",
    );
    for kind in TAYLOR_KINDS {
        atlas.sweep(
            kind,
            &orders[..9],
            |m, _| taylor_model(&taylor_program(kind), taylor_request(1, m as usize, 4)),
            |model| {
                // A missing model means its construction was refused.
                model.as_ref().and_then(|model| {
                    model
                        .enclose_on(&[d(0, 1, -5)], &mut graph(), &mut arithmetic())
                        .ok()
                })
            },
        );
    }

    atlas.operation(
        group,
        "Taylor affine model",
        "number of model outputs",
        "Applies a random integer affine map to an m-output order-2 Taylor model (exact coefficient combination plus remainder propagation).",
    );
    let outputs = [1.0, 2.0, 4.0, 8.0, 16.0, 32.0];
    atlas.sweep(
        "square map",
        &outputs,
        |m, rng| {
            let m = m as usize;
            (multi_output_model(m), affine_map(m, m, rng))
        },
        |(model, map)| {
            TaylorAffineModel::new(model.clone(), map.clone(), &mut graph(), &mut arithmetic())
        },
    );
    atlas.sweep(
        "one observation row",
        &outputs,
        |m, rng| {
            let m = m as usize;
            (multi_output_model(m), affine_map(m, 1, rng))
        },
        |(model, map)| {
            TaylorAffineModel::new(model.clone(), map.clone(), &mut graph(), &mut arithmetic())
        },
    );

    atlas.operation(
        group,
        "Taylor noise model",
        "number of model outputs",
        "Prepares a noise-symbol model (one retained noise term) from an m-output affine Taylor model, or encloses one observation with a prepared model.",
    );
    let affine_source = |m: usize, rng: &mut Rng| {
        Arc::new(
            TaylorAffineModel::new(
                multi_output_model(m),
                affine_map(m, m, rng),
                &mut graph(),
                &mut arithmetic(),
            )
            .expect("affine model"),
        )
    };
    atlas.sweep(
        "prepare",
        &outputs,
        |m, rng| affine_source(m as usize, rng),
        |source| TaylorNoiseModel::new(source.clone(), 1, &mut graph(), &mut arithmetic()),
    );
    atlas.sweep(
        "enclose one observation (prepared)",
        &outputs,
        |m, rng| {
            let m = m as usize;
            let source = affine_source(m, rng);
            let model = TaylorNoiseModel::new(source, 1, &mut graph(), &mut arithmetic())
                .expect("noise model");
            (model, affine_map(m, 1, rng))
        },
        |(model, map)| model.enclose(map, &mut graph(), &mut arithmetic()),
    );

    atlas.operation(
        group,
        "Taylor polynomial map",
        "map polynomial degree",
        "Pushes a prebuilt Taylor model of 1/(1−x) through a polynomial map u ↦ Horner(u) of growing degree (truncated products with remainder bounds).",
    );
    for order in [2usize, 4, 8] {
        atlas.sweep(
            &format!("seed order {order}"),
            &ladder(1, 16, 7),
            |m, _| {
                let seed = Arc::new(
                    taylor_model(&taylor_program("1/(1−x)"), taylor_request(1, order, 4))
                        .expect("seed model"),
                );
                (seed, program("Horner chain", m as usize))
            },
            |(seed, map)| {
                TaylorPolynomialModel::new(
                    seed.clone(),
                    map.clone(),
                    &mut graph(),
                    &mut arithmetic(),
                )
            },
        );
    }

    atlas.operation(
        group,
        "Taylor model composition",
        "Taylor order",
        "Composes an outer Taylor model (centered at 1 on [7/8, 9/8]) with an inner model on [-1/16, 1/16], reusing both certificates.",
    );
    for (kind, inner, outer) in [
        (
            "1/(2+u) ∘ (1 + x/2)",
            add(Expr::int(1), var(0) / Expr::int(2)),
            Expr::int(1) / add(Expr::int(2), var(1)),
        ),
        (
            "sqrt(1+u) ∘ (1 + x/2)",
            add(Expr::int(1), var(0) / Expr::int(2)),
            add(Expr::int(1), var(1)).sqrt(),
        ),
        (
            "sin(u) ∘ 1/(1−x)",
            Expr::int(1) / (Expr::int(1) - var(0)),
            var(1).sin(),
        ),
    ] {
        atlas.sweep(
            kind,
            &[1.0, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0, 16.0],
            |m, _| {
                let order = m as usize;
                let inner_program = ExpressionProgram::compile(
                    std::slice::from_ref(&inner),
                    &[SymbolId(0)],
                    &mut graph(),
                )
                .expect("compiles");
                let outer_program = ExpressionProgram::compile(
                    std::slice::from_ref(&outer),
                    &[SymbolId(1)],
                    &mut graph(),
                )
                .expect("compiles");
                let inner_model = taylor_model(&inner_program, taylor_request(1, order, 4));
                let outer_model = taylor_model(
                    &outer_program,
                    TaylorRequest {
                        center: vec![d(1, 1, 0)],
                        domain: vec![d(7, 9, -3)],
                        order,
                        precision: -64,
                    },
                );
                inner_model
                    .zip(outer_model)
                    .map(|(i, o)| (Arc::new(i), Arc::new(o)))
            },
            |models| {
                // A missing model means the outer or inner construction was refused.
                models.as_ref().and_then(|(inner, outer)| {
                    inner
                        .clone()
                        .compose(outer.clone(), &mut graph(), &mut arithmetic())
                        .ok()
                })
            },
        );
    }
}

fn work_budgets(atlas: &mut Atlas) {
    let group = "work budgets";

    atlas.operation(
        group,
        "isolation under a refinement budget",
        "admitted root-refinement units",
        "Isolates and refines every root to width 2^-32 inside with_budget. Small budgets fail with typed exhaustion (intended, shown as errors); the curve flattens once the budget suffices.",
    );
    let budget_kinds: [(&str, FixedPoly); 3] = [
        ("Mignotte deg 16, a=16", || reals(&mignotte(16, 16))),
        ("close pair 2^-64, deg 8", || reals(&close_pair(8, 64))),
        ("Wilkinson 24", || reals(&wilkinson(24))),
    ];
    for (kind, make_poly) in budget_kinds {
        atlas.sweep(
            kind,
            &powers_of_two(0, 14),
            |m, _| (univariate(make_poly()), m as usize),
            |(u, units)| {
                with_budget(
                    WorkBudget {
                        refinements: *units,
                        ..WorkBudget::UNLIMITED
                    },
                    || {
                        isolated(
                            &isolate_univariate_polynomial_expr_with_config(
                                0,
                                &u.expr,
                                &u.problem,
                                RootIsolationConfig {
                                    policy: POLICY,
                                    max_interval_width: Some(dyadic_real(1, 32)),
                                    max_refinement_steps: 64,
                                },
                            )
                            .status,
                        )
                    },
                )
                .result
                .unwrap_or(false)
            },
        );
    }

    atlas.operation(
        group,
        "budget scope overhead",
        "polynomial degree",
        "The same dense 32-bit isolation with no budget scope, an unlimited budget scope, and a cancellable unlimited scope.",
    );
    for kind in ["no scope", "unlimited budget", "cancellable"] {
        atlas.sweep(
            kind,
            &powers_of_two(1, 6),
            |m, rng| {
                (
                    univariate(dense_int(rng, m as usize, 32)),
                    CancellationToken::new(),
                )
            },
            move |(u, token)| match kind {
                "no scope" => isolate(u),
                "unlimited budget" => with_budget(WorkBudget::UNLIMITED, || isolate(u))
                    .result
                    .unwrap_or(false),
                _ => with_cancellable_budget(WorkBudget::UNLIMITED, token, || isolate(u))
                    .result
                    .unwrap_or(false),
            },
        );
    }
}
