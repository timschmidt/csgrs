//! Shared inputs and independent oracles for the represented-root suites.
//!
//! The oracles below never call the represented-root module. A selected
//! value is checked by exact `Real` replay of its defining polynomial,
//! certified `hyperreal` signs against its isolating interval, and an
//! isolation count from a test-local rational Sturm chain.

#![allow(dead_code)]

use std::cell::Cell;

use hyperreal::{Problem, Rational, Real, RealSign, ZeroKnowledge};
use hypersolve::classification::{Classification, UncertaintyReason};
use hypersolve::{
    AlgebraicRootRepresentation, AlgebraicRootValidationReport, AlgebraicRootValidationStatus,
    ApproximationPolicy, DenseTensorPolynomial, FieldInvariantError, IsolatedRootInterval,
    PredicatePolicy, RealInterval, SelectedAlgebraContext, SymbolId,
    validate_algebraic_root_representation,
};

pub fn int(value: i64) -> Real {
    Real::from(value)
}

pub fn q(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).unwrap())
}

pub fn sqrt(value: i64) -> Real {
    Real::from(value).sqrt().unwrap()
}

pub fn cbrt(value: i64) -> Real {
    Real::from(value).cbrt().unwrap()
}

/// `sin(1)^2 + cos(1)^2 - 1`: exactly zero by the Pythagorean identity, but
/// not structurally recognized, so STRICT predicates cannot decide its sign.
pub fn unknown_zero() -> Real {
    let sine = Real::from(1).sin();
    let cosine = Real::from(1).cos();
    &sine * &sine + &cosine * &cosine - Real::from(1)
}

pub fn ints(values: &[i64]) -> Vec<Real> {
    values.iter().copied().map(Real::from).collect()
}

/// A selected root with the given defining polynomial and isolator. The
/// stored validation is recomputed and may be invalid.
pub fn unchecked_root(
    coefficients: Vec<Real>,
    lower: Real,
    upper: Real,
) -> AlgebraicRootRepresentation {
    let mut root = AlgebraicRootRepresentation {
        constraint_index: 0,
        symbol: SymbolId(0),
        interval_index: 0,
        polynomial_coefficients: coefficients,
        interval: IsolatedRootInterval {
            lower,
            upper,
            exact_root: None,
            distinct_root_count: 1,
        },
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    };
    root.validation = validate_algebraic_root_representation(&root, PredicatePolicy::STRICT);
    root
}

/// A valid selected root whose isolation is proved by the test-local Sturm
/// chain before it is used as an input.
pub fn root(coefficients: &[i64], lower: Real, upper: Real) -> AlgebraicRootRepresentation {
    let root = unchecked_root(ints(coefficients), lower, upper);
    assert!(root.is_valid(), "{:?}", root.validation);
    let polynomial = coefficients
        .iter()
        .map(|coefficient| Rational::new(*coefficient))
        .collect::<Vec<_>>();
    assert_eq!(
        sturm_root_count(
            &polynomial,
            &rational(&root.interval.lower),
            &rational(&root.interval.upper)
        ),
        1,
        "test input must isolate one root"
    );
    root
}

/// A selected root isolated by the degenerate interval `[value, value]`
/// without an exact point witness; `value` must be a root of `coefficients`.
pub fn point_isolator(coefficients: &[i64], value: Real) -> AlgebraicRootRepresentation {
    let root = unchecked_root(ints(coefficients), value.clone(), value);
    assert!(root.is_valid(), "{:?}", root.validation);
    assert!(root.exact_point_witness().is_none());
    root
}

/// A selected root whose payload fails structural validation: its claimed
/// distinct-root count is two.
pub fn non_unit_root() -> AlgebraicRootRepresentation {
    let mut root = unchecked_root(ints(&[-2, 0, 1]), int(-2), int(2));
    root.interval.distinct_root_count = 2;
    root.validation = validate_algebraic_root_representation(&root, PredicatePolicy::STRICT);
    assert_eq!(
        root.validation.status,
        AlgebraicRootValidationStatus::NonUnitIsolation
    );
    root
}

pub fn sqrt2() -> AlgebraicRootRepresentation {
    root(&[-2, 0, 1], int(1), int(2))
}

pub fn negative_sqrt2() -> AlgebraicRootRepresentation {
    root(&[-2, 0, 1], int(-2), int(-1))
}

pub fn sqrt3() -> AlgebraicRootRepresentation {
    root(&[-3, 0, 1], int(1), int(2))
}

pub fn cbrt2() -> AlgebraicRootRepresentation {
    root(&[-2, 0, 0, 1], int(1), int(2))
}

/// `sqrt(2)` selected from `(x^2 - 2)(x - 3)`, a different defining
/// polynomial for the same algebraic number.
pub fn sqrt2_from_cubic() -> AlgebraicRootRepresentation {
    root(&[6, -2, -3, 1], int(1), int(2))
}

pub fn exact(value: &Real) -> AlgebraicRootRepresentation {
    AlgebraicRootRepresentation::from_exact_value(value)
}

pub fn rational(value: &Real) -> Rational {
    value
        .exact_rational_normal_form()
        .unwrap_or_else(|| panic!("{value} is not an exact rational"))
}

/// Certified sign from `hyperreal`, independent of Hypersolve's predicates.
pub fn certified_sign(value: &Real) -> RealSign {
    if value.zero_status() == ZeroKnowledge::Zero {
        return RealSign::Zero;
    }
    value
        .certified_sign_until(-2048)
        .sign()
        .unwrap_or_else(|| panic!("the oracle could not certify the sign of {value}"))
}

pub fn assert_exact_zero(value: &Real) {
    assert_eq!(
        value.zero_status(),
        ZeroKnowledge::Zero,
        "{value} is not exactly zero"
    );
}

pub fn assert_equal(left: &Real, right: &Real) {
    assert_exact_zero(&(left - right));
}

/// Strict order `lower < value < upper`, certified by `hyperreal`.
pub fn assert_strictly_between(value: &Real, lower: &Real, upper: &Real) {
    assert_eq!(certified_sign(&(value - lower)), RealSign::Positive);
    assert_eq!(certified_sign(&(upper - value)), RealSign::Positive);
}

/// The closed interval contains `value`.
pub fn assert_encloses(interval: &RealInterval, value: &Real) {
    assert_ne!(
        certified_sign(&(value - &interval.lower)),
        RealSign::Negative
    );
    assert_ne!(
        certified_sign(&(&interval.upper - value)),
        RealSign::Negative
    );
}

fn trim(mut polynomial: Vec<Rational>) -> Vec<Rational> {
    while polynomial.last().is_some_and(Rational::is_zero) {
        polynomial.pop();
    }
    polynomial
}

fn remainder(dividend: &[Rational], divisor: &[Rational]) -> Vec<Rational> {
    let divisor = trim(divisor.to_vec());
    let mut remainder = trim(dividend.to_vec());
    let leading = divisor.last().expect("nonzero divisor").clone();
    while remainder.len() >= divisor.len() {
        let shift = remainder.len() - divisor.len();
        let factor = remainder.last().unwrap() / &leading;
        for (index, coefficient) in divisor.iter().enumerate() {
            remainder[shift + index] = &remainder[shift + index] - &(&factor * coefficient);
        }
        remainder.pop();
        remainder = trim(remainder);
    }
    remainder
}

fn derivative(polynomial: &[Rational]) -> Vec<Rational> {
    polynomial
        .iter()
        .enumerate()
        .skip(1)
        .map(|(power, coefficient)| coefficient * &Rational::new(power as i64))
        .collect()
}

fn evaluate_rational(polynomial: &[Rational], point: &Rational) -> Rational {
    polynomial
        .iter()
        .rev()
        .fold(Rational::zero(), |value, coefficient| {
            &(&value * point) + coefficient
        })
}

fn sign_variations(chain: &[Vec<Rational>], point: &Rational) -> usize {
    let signs = chain
        .iter()
        .map(|polynomial| evaluate_rational(polynomial, point))
        .filter(|value| !value.is_zero())
        .map(|value| value.is_positive())
        .collect::<Vec<_>>();
    signs.windows(2).filter(|pair| pair[0] != pair[1]).count()
}

/// Distinct real roots of a rational polynomial in `(lower, upper]`, by a
/// Sturm chain written only for these tests.
pub fn sturm_root_count(polynomial: &[Rational], lower: &Rational, upper: &Rational) -> usize {
    let polynomial = trim(polynomial.to_vec());
    let mut chain = vec![polynomial.clone(), trim(derivative(&polynomial))];
    while chain.last().is_some_and(|last| !last.is_empty()) {
        let length = chain.len();
        let next = remainder(&chain[length - 2], &chain[length - 1])
            .into_iter()
            .map(|coefficient| -coefficient)
            .collect::<Vec<_>>();
        chain.push(trim(next));
    }
    chain.pop();
    sign_variations(&chain, lower) - sign_variations(&chain, upper)
}

/// Independent oracle that `representation` selects exactly `value`:
/// structural validation, exact polynomial replay at `value`, certified
/// containment in the isolator, and uniqueness of that root inside it.
pub fn assert_selects(representation: &AlgebraicRootRepresentation, value: &Real) {
    assert_selects_impl(representation, value, None);
}

/// [`assert_selects`] for a value whose exact polynomial replay `hyperreal`
/// cannot simplify to zero. `minimal` is the hand-derived rational minimal
/// polynomial of `value` (ascending powers); it must divide the defining
/// polynomial exactly, and must change sign across a certified enclosure of
/// `value`, which guards the derivation itself.
pub fn assert_selects_root_of(
    representation: &AlgebraicRootRepresentation,
    value: &Real,
    minimal: &[i64],
) {
    let minimal = minimal
        .iter()
        .map(|coefficient| Rational::new(*coefficient))
        .collect::<Vec<_>>();
    assert_selects_impl(representation, value, Some(&minimal));
}

/// [`assert_selects_root_of`] with arbitrary rational coefficients.
pub fn assert_selects_root_of_rationals(
    representation: &AlgebraicRootRepresentation,
    value: &Real,
    minimal: &[Rational],
) {
    assert_selects_impl(representation, value, Some(minimal));
}

fn assert_selects_impl(
    representation: &AlgebraicRootRepresentation,
    value: &Real,
    minimal: Option<&[Rational]>,
) {
    assert!(representation.is_valid(), "{:?}", representation.validation);
    assert_eq!(
        validate_algebraic_root_representation(representation, PredicatePolicy::STRICT).status,
        AlgebraicRootValidationStatus::Valid
    );
    assert_eq!(representation.interval.distinct_root_count, 1);
    let coefficients = &representation.polynomial_coefficients;
    let rational_polynomial = coefficients
        .iter()
        .map(Real::exact_rational_normal_form)
        .collect::<Option<Vec<_>>>();
    match (minimal, &rational_polynomial) {
        (Some(minimal), Some(polynomial)) => {
            let [below, above] = value.certified_rational_interval(-512).unwrap();
            let below = evaluate_rational(minimal, &below);
            let above = evaluate_rational(minimal, &above);
            assert!(
                below.is_zero() || above.is_zero() || below.is_positive() != above.is_positive(),
                "the minimal polynomial does not vanish near {value}"
            );
            assert!(
                remainder(polynomial, minimal).is_empty(),
                "the minimal polynomial of {value} does not divide the defining polynomial"
            );
        }
        _ => assert_exact_zero(&Real::eval_poly(coefficients, value)),
    }
    let interval = RealInterval {
        lower: representation.interval.lower.clone(),
        upper: representation.interval.upper.clone(),
    };
    assert_encloses(&interval, value);
    if let Some(witness) = representation.exact_point_witness() {
        assert_equal(witness, value);
        return;
    }
    // Uniqueness of the selected root inside the isolating interval.
    if let Some(polynomial) = rational_polynomial {
        let [mut lower, _] = interval.lower.certified_rational_interval(-160).unwrap();
        let [_, mut upper] = interval.upper.certified_rational_interval(-160).unwrap();
        let step = Rational::fraction(1, 1 << 40).unwrap();
        while evaluate_rational(&polynomial, &lower).is_zero() {
            lower = &lower - &step;
        }
        while evaluate_rational(&polynomial, &upper).is_zero() {
            upper = &upper + &step;
        }
        assert_eq!(sturm_root_count(&polynomial, &lower, &upper), 1);
    } else {
        // A quadratic over an irrational coefficient field: its other root
        // `-b/a - value` must lie outside the isolator.
        assert_eq!(coefficients.len(), 3, "oracle supports linear or quadratic");
        let other = -(&coefficients[1] / &coefficients[2]).unwrap() - value;
        let below = certified_sign(&(&other - &interval.lower)) == RealSign::Negative;
        let above = certified_sign(&(&other - &interval.upper)) == RealSign::Positive;
        assert!(below || above, "the conjugate root lies in the isolator");
    }
}

pub fn decided<T: std::fmt::Debug>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("expected a decided value, got {reason:?}"),
    }
}

pub fn uncertain<T: std::fmt::Debug>(classification: Classification<T>) -> UncertaintyReason {
    match classification {
        Classification::Decided(value) => panic!("expected uncertainty, got {value:?}"),
        Classification::Uncertain(reason) => reason,
    }
}

/// Dense tensor helper in row-major order.
pub fn tensor(dimensions: &[usize], coefficients: Vec<Real>) -> DenseTensorPolynomial {
    DenseTensorPolynomial::try_new(dimensions.to_vec(), coefficients).unwrap()
}

/// `coefficients` on `axis` of a rank-`rank` tensor.
pub fn on_axis(rank: usize, axis: usize, coefficients: Vec<Real>) -> DenseTensorPolynomial {
    DenseTensorPolynomial::from_axis_polynomial(rank, axis, &coefficients).unwrap()
}

pub fn constant(rank: usize, value: Real) -> DenseTensorPolynomial {
    on_axis(rank, 0, vec![value])
}

/// Exact evaluation of a dense tensor at a point, written for these tests.
pub fn evaluate_tensor(polynomial: &DenseTensorPolynomial, point: &[Real]) -> Real {
    let dimensions = polynomial.dimensions();
    assert_eq!(dimensions.len(), point.len());
    let mut total = Real::zero();
    for (flat, coefficient) in polynomial.coefficients().iter().enumerate() {
        let mut remaining = flat;
        let mut term = coefficient.clone();
        for axis in (0..dimensions.len()).rev() {
            let exponent = remaining % dimensions[axis];
            remaining /= dimensions[axis];
            for _ in 0..exponent {
                term = &term * &point[axis];
            }
        }
        total += term;
    }
    total
}

/// Which approximation protocol a test policy follows.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Mode {
    /// Certified STRICT decisions only.
    Strict,
    /// `APPROXIMATE_512` selected and consumable.
    Approximate,
    /// `APPROXIMATE_512` selected, inside a pass that forbids consuming it.
    StrictPassOfApproximate,
}

/// A recording approximation policy.
#[derive(Debug)]
pub struct Policy {
    pub mode: Mode,
    pub observed: Cell<usize>,
}

impl Policy {
    pub fn new(mode: Mode) -> Self {
        Self {
            mode,
            observed: Cell::new(0),
        }
    }
}

impl ApproximationPolicy for Policy {
    fn selects_approximate_512(&self) -> bool {
        self.mode != Mode::Strict
    }

    fn permits_approximate_512(&self) -> bool {
        self.mode == Mode::Approximate
    }

    fn observe_approximate_512(&self) {
        self.observed.set(self.observed.get() + 1);
    }
}

/// The context's one-root sign authority.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Authority {
    /// Declines every query, leaving the decision to Hypersolve.
    Defer,
    /// Fails with a typed context error.
    Fail,
}

#[derive(Debug, PartialEq)]
pub enum ContextError {
    Invariant(String),
    Scalar(Problem),
    Authority,
}

impl From<FieldInvariantError> for ContextError {
    fn from(error: FieldInvariantError) -> Self {
        Self::Invariant(error.0)
    }
}

impl From<Problem> for ContextError {
    fn from(error: Problem) -> Self {
        Self::Scalar(error)
    }
}

/// A selected-algebra context with a recording policy and a configurable
/// one-root authority.
#[derive(Debug)]
pub struct Context {
    pub policy: Policy,
    pub authority: Authority,
    pub bounded: bool,
    pub authority_calls: Cell<usize>,
}

impl Context {
    pub fn new(mode: Mode, authority: Authority, bounded: bool) -> Self {
        Self {
            policy: Policy::new(mode),
            authority,
            bounded,
            authority_calls: Cell::new(0),
        }
    }

    pub fn strict() -> Self {
        Self::new(Mode::Strict, Authority::Defer, false)
    }
}

impl ApproximationPolicy for Context {
    fn selects_approximate_512(&self) -> bool {
        self.policy.selects_approximate_512()
    }

    fn permits_approximate_512(&self) -> bool {
        self.policy.permits_approximate_512()
    }

    fn observe_approximate_512(&self) {
        self.policy.observe_approximate_512();
    }
}

impl SelectedAlgebraContext for Context {
    type Error = ContextError;

    fn real_sign(&self, value: &Real) -> Option<RealSign> {
        (value.zero_status() == ZeroKnowledge::Zero)
            .then_some(RealSign::Zero)
            .or_else(|| value.certified_sign_until(-256).sign())
    }

    fn strict_counterpart(&self) -> Self {
        Self::new(Mode::Strict, self.authority, self.bounded)
    }

    fn strict_predicate_pass<T>(&self, evaluate: impl FnOnce() -> T) -> T {
        evaluate()
    }

    fn bounded_exact_predicate_pass<T>(&self, evaluate: impl FnOnce() -> T) -> T {
        evaluate()
    }

    fn has_bounded_exact_predicate_budget(&self) -> bool {
        self.bounded
    }

    fn univariate_sign_at_root(
        &self,
        _coefficients: &[Real],
        _root: &AlgebraicRootRepresentation,
    ) -> Result<Classification<RealSign>, ContextError> {
        self.authority_calls.set(self.authority_calls.get() + 1);
        match self.authority {
            Authority::Defer => Ok(Classification::Uncertain(UncertaintyReason::Predicate)),
            Authority::Fail => Err(ContextError::Authority),
        }
    }
}
