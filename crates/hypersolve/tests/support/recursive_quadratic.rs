//! Shared fixtures for the recursive quadratic field suites.
//!
//! Every expected value in these suites is an independently constructed
//! exact `Real` closed form, compared through Hyperlimit's STRICT predicates.
//! The selected roots are authored isolators whose validity is certified by
//! STRICT validation before use, never assumed.

use std::cell::Cell;
use std::cmp::Ordering;
use std::rc::Rc;
use std::sync::Arc;

use hyperlimit::{Certainty, PredicateOutcome, PredicatePolicy, Sign};
use hyperreal::{Problem, Rational, Real, RealSign};
use hypersolve::classification::{Classification, UncertaintyReason};
use hypersolve::radical_expression::TwoSquareRootExpression;
use hypersolve::recursive_quadratic_field::*;
use hypersolve::{
    AlgebraicRootRepresentation, AlgebraicRootValidationStatus, ApproximationPolicy,
    DenseTensorPolynomial, FieldInvariantError, IsolatedRootInterval, RealInterval,
    SelectedAlgebraContext, SymbolId, sign_at_selected_root,
    validate_algebraic_root_representation,
};

/// Error type of the test evaluation context.
#[derive(Debug)]
pub enum TestError {
    Field(FieldInvariantError),
    Problem(Problem),
}

impl From<FieldInvariantError> for TestError {
    fn from(error: FieldInvariantError) -> Self {
        Self::Field(error)
    }
}

impl From<Problem> for TestError {
    fn from(error: Problem) -> Self {
        Self::Problem(error)
    }
}

/// How the context answers a univariate sign query at one selected root.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RootAuthority {
    /// Exact witness substitution or a Sturm-Tarski query on the isolator.
    Exact,
    /// Every query stays undecided, as a specialized authority may.
    Undecided,
}

/// Pass bookkeeping shared by a context and its strict counterpart.
#[derive(Debug, Default)]
pub struct PassState {
    strict_depth: Cell<usize>,
    bounded_depth: Cell<usize>,
    approximations: Cell<usize>,
    root_queries: Cell<usize>,
}

/// A minimal selected-algebra context. It selects either STRICT or
/// APPROXIMATE_512 and records every approximate decision it consumes.
/// Consuming an approximation inside a strict pass panics, so any test
/// that reaches such a decision fails loudly.
#[derive(Debug)]
pub struct TestContext {
    approximate: bool,
    authority: RootAuthority,
    state: Rc<PassState>,
}

impl TestContext {
    pub fn strict() -> Self {
        Self {
            approximate: false,
            authority: RootAuthority::Exact,
            state: Rc::default(),
        }
    }

    pub fn approximate() -> Self {
        Self {
            approximate: true,
            authority: RootAuthority::Exact,
            state: Rc::default(),
        }
    }

    pub fn with_authority(mut self, authority: RootAuthority) -> Self {
        self.authority = authority;
        self
    }

    pub fn approximations(&self) -> usize {
        self.state.approximations.get()
    }

    pub fn root_queries(&self) -> usize {
        self.state.root_queries.get()
    }

    fn policy(&self) -> PredicatePolicy {
        if self.permits_approximate_512() {
            PredicatePolicy::APPROXIMATE_512
        } else {
            PredicatePolicy::STRICT
        }
    }
}

struct DepthGuard<'a>(&'a Cell<usize>);

impl<'a> DepthGuard<'a> {
    fn enter(depth: &'a Cell<usize>) -> Self {
        depth.set(depth.get() + 1);
        Self(depth)
    }
}

impl Drop for DepthGuard<'_> {
    fn drop(&mut self) {
        self.0.set(self.0.get() - 1);
    }
}

impl ApproximationPolicy for TestContext {
    fn selects_approximate_512(&self) -> bool {
        self.approximate
    }

    fn permits_approximate_512(&self) -> bool {
        self.approximate && self.state.strict_depth.get() == 0
    }

    fn observe_approximate_512(&self) {
        assert!(
            self.permits_approximate_512(),
            "an approximate terminal was consumed where it is forbidden"
        );
        self.state
            .approximations
            .set(self.state.approximations.get() + 1);
    }
}

fn hyperlimit_sign(sign: Sign) -> RealSign {
    match sign {
        Sign::Negative => RealSign::Negative,
        Sign::Zero => RealSign::Zero,
        Sign::Positive => RealSign::Positive,
    }
}

impl SelectedAlgebraContext for TestContext {
    type Error = TestError;

    fn real_sign(&self, value: &Real) -> Option<RealSign> {
        match hyperlimit::classify_real_sign(value, self.policy()) {
            PredicateOutcome::Decided {
                value, certainty, ..
            } => {
                if certainty == Certainty::Approximate {
                    self.observe_approximate_512();
                }
                Some(hyperlimit_sign(value))
            }
            PredicateOutcome::Unknown { .. } => None,
        }
    }

    fn strict_counterpart(&self) -> Self {
        Self {
            approximate: false,
            authority: self.authority,
            state: self.state.clone(),
        }
    }

    fn strict_predicate_pass<T>(&self, evaluate: impl FnOnce() -> T) -> T {
        let _strict = DepthGuard::enter(&self.state.strict_depth);
        evaluate()
    }

    fn bounded_exact_predicate_pass<T>(&self, evaluate: impl FnOnce() -> T) -> T {
        let _strict = DepthGuard::enter(&self.state.strict_depth);
        let _bounded = DepthGuard::enter(&self.state.bounded_depth);
        evaluate()
    }

    fn has_bounded_exact_predicate_budget(&self) -> bool {
        self.state.bounded_depth.get() > 0
    }

    fn univariate_sign_at_root(
        &self,
        coefficients: &[Real],
        root: &AlgebraicRootRepresentation,
    ) -> Result<Classification<RealSign>, TestError> {
        self.state
            .root_queries
            .set(self.state.root_queries.get() + 1);
        if self.authority == RootAuthority::Undecided {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        if let Some(witness) = root.exact_point_witness() {
            return Ok(strict_sign(&Real::eval_poly(coefficients, witness)).map_or(
                Classification::Uncertain(UncertaintyReason::RealSign),
                Classification::Decided,
            ));
        }
        Ok(
            match sign_at_selected_root(&root.polynomial_coefficients, coefficients, &root.interval)
            {
                Some(Ordering::Less) => Classification::Decided(RealSign::Negative),
                Some(Ordering::Equal) => Classification::Decided(RealSign::Zero),
                Some(Ordering::Greater) => Classification::Decided(RealSign::Positive),
                None => Classification::Uncertain(UncertaintyReason::Predicate),
            },
        )
    }
}

pub fn real(value: i64) -> Real {
    Real::from(value)
}

pub fn fraction(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).expect("nonzero denominator"))
}

pub fn sqrt(value: Real) -> Real {
    value.sqrt().expect("nonnegative radicand")
}

pub fn root_of(value: i64) -> Real {
    sqrt(real(value))
}

/// A STRICT Hyperlimit sign, or `None` when STRICT cannot decide it.
pub fn strict_sign(value: &Real) -> Option<RealSign> {
    hyperlimit::classify_real_sign(value, PredicatePolicy::STRICT)
        .value()
        .map(hyperlimit_sign)
}

/// Asserts exact equality of two independently built reals under STRICT.
#[track_caller]
pub fn assert_real_eq(actual: &Real, expected: &Real) {
    assert_eq!(
        strict_sign(&(actual - expected)),
        Some(RealSign::Zero),
        "expected {expected:?}, found {actual:?}"
    );
}

/// Asserts a certified strict order `lower < upper`.
#[track_caller]
pub fn assert_real_lt(lower: &Real, upper: &Real) {
    assert_eq!(
        strict_sign(&(upper - lower)),
        Some(RealSign::Positive),
        "expected {lower:?} < {upper:?}"
    );
}

/// Asserts `lower <= value <= upper` for a certified enclosure.
#[track_caller]
pub fn assert_encloses(interval: &RealInterval, value: &Real) {
    assert_ne!(
        strict_sign(&(value - &interval.lower)),
        Some(RealSign::Negative),
        "lower endpoint {:?} exceeds {value:?}",
        interval.lower
    );
    assert_ne!(
        strict_sign(&(&interval.upper - value)),
        Some(RealSign::Negative),
        "upper endpoint {:?} is below {value:?}",
        interval.upper
    );
    assert!(
        strict_sign(&(value - &interval.lower)).is_some()
            && strict_sign(&(&interval.upper - value)).is_some(),
        "enclosure of {value:?} was not certified"
    );
}

/// Asserts the field value's retained exact witness equals `expected`.
#[track_caller]
pub fn assert_value(value: &RecursiveQuadraticValue, expected: &Real) {
    let actual = value
        .exact_real_value_with_retained_witnesses()
        .expect("every generator in this value carries an exact witness");
    assert_real_eq(&actual, expected);
}

/// Asserts a bounded-pass outcome for a value that is zero by construction:
/// the pass may certify zero or decline, but never reports a nonzero sign.
#[track_caller]
pub fn assert_zero_or_undecided<E: std::fmt::Debug>(result: Result<Classification<RealSign>, E>) {
    match result {
        Ok(Classification::Decided(RealSign::Zero) | Classification::Uncertain(_)) => {}
        other => panic!("a zero was misclassified: {other:?}"),
    }
}

/// Asserts a bounded-pass outcome for a value of known nonzero sign.
#[track_caller]
pub fn assert_sign_or_undecided<E: std::fmt::Debug>(
    result: Result<Classification<RealSign>, E>,
    expected: RealSign,
) {
    match result {
        Ok(Classification::Uncertain(_)) => {}
        Ok(Classification::Decided(sign)) => assert_eq!(sign, expected),
        other => panic!("expected {expected:?} or undecided, found {other:?}"),
    }
}

/// Asserts a decided sign.
#[track_caller]
pub fn assert_sign<E: std::fmt::Debug>(
    result: Result<Classification<RealSign>, E>,
    expected: RealSign,
) {
    match result {
        Ok(Classification::Decided(sign)) => assert_eq!(sign, expected),
        other => panic!("expected decided {expected:?}, found {other:?}"),
    }
}

/// A selected root of `coefficients` in `[lower, upper]`, certified by
/// STRICT validation before it is used as a field source.
pub fn selected_root(
    coefficients: Vec<Real>,
    lower: Real,
    upper: Real,
    symbol: u32,
) -> AlgebraicRootRepresentation {
    let mut root = AlgebraicRootRepresentation {
        constraint_index: symbol as usize,
        symbol: SymbolId(symbol),
        interval_index: 0,
        polynomial_coefficients: coefficients,
        interval: IsolatedRootInterval {
            lower,
            upper,
            exact_root: None,
            distinct_root_count: 1,
        },
        validation: hypersolve::AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::NonUnitIsolation,
            message: None,
        },
    };
    root.validation = validate_algebraic_root_representation(&root, PredicatePolicy::STRICT);
    assert_eq!(
        root.validation.status,
        AlgebraicRootValidationStatus::Valid,
        "authored isolator must own exactly one root"
    );
    root
}

/// `sqrt(2)` as the unique root of `x^2 - 2` in `[1, 2]`.
pub fn sqrt_two_root() -> AlgebraicRootRepresentation {
    selected_root(vec![real(-2), real(0), real(1)], real(1), real(2), 1)
}

/// `2^(1/3)` as the unique root of `x^3 - 2` in `[1, 2]`. No compact
/// low-degree witness exists, so dense interval refinement is required.
pub fn cube_root_two() -> AlgebraicRootRepresentation {
    selected_root(
        vec![real(-2), real(0), real(0), real(1)],
        real(1),
        real(2),
        2,
    )
}

/// `3^(1/3)` as the unique root of `x^3 - 3` in `[1, 2]`.
pub fn cube_root_three() -> AlgebraicRootRepresentation {
    selected_root(
        vec![real(-3), real(0), real(0), real(1)],
        real(1),
        real(2),
        3,
    )
}

pub fn cube_root_two_real() -> Real {
    // 2^(1/3) = exp(ln(2) / 3), independent of every isolator.
    ((real(2).ln().expect("positive") / real(3)).expect("nonzero"))
        .exp()
        .expect("finite exponent")
}

/// `sin(1)^2 + cos(1)^2 - 1`: zero by the Pythagorean identity, but beyond
/// Hyperreal's STRICT sign authority. APPROXIMATE_512 may call it zero.
pub fn undecidable_zero() -> Real {
    let one = real(1);
    let sine = one.clone().sin();
    let cosine = one.cos();
    let value = &sine * &sine + &cosine * &cosine - real(1);
    assert_eq!(
        strict_sign(&value),
        None,
        "fixture premise: STRICT cannot decide this zero"
    );
    value
}

/// `sqrt(2 + sqrt3) - (sqrt6 + sqrt2)/2`: zero by denesting, and certified
/// zero structurally without being stored as an exact rational.
pub fn denested_zero() -> Real {
    let value =
        sqrt(real(2) + root_of(3)) - ((root_of(6) + root_of(2)) / real(2)).expect("nonzero");
    assert!(
        value.exact_rational_ref().is_none()
            && value.zero_status() == hyperreal::ZeroKnowledge::Zero,
        "fixture premise: a structural zero that is not a stored rational"
    );
    value
}

pub fn constant_tensor(rank: usize, value: Real) -> DenseTensorPolynomial {
    DenseTensorPolynomial::try_new(vec![1; rank], vec![value]).expect("constant tensor")
}

/// The ascending univariate polynomial `coefficients` on axis 0 of rank 1.
pub fn axis_tensor(coefficients: &[Real]) -> DenseTensorPolynomial {
    DenseTensorPolynomial::from_axis_polynomial(1, 0, coefficients).expect("axis tensor")
}

pub fn base_arc(field: &RecursiveQuadraticField) -> Arc<RecursiveQuadraticBaseField> {
    field.base_and_extension_path().0
}

pub fn extension_arc(field: &RecursiveQuadraticField) -> Arc<RecursiveQuadraticExtension> {
    match field {
        RecursiveQuadraticField::Extension(extension) => extension.clone(),
        RecursiveQuadraticField::Base(_) => panic!("expected an extension field"),
    }
}

/// `Q(sqrt(first), sqrt(second))` over the empty selected tuple.
pub fn quadratic_base(first: i64, second: i64) -> RecursiveQuadraticField {
    RecursiveQuadraticField::base(
        Vec::new(),
        constant_tensor(0, real(first)),
        constant_tensor(0, real(second)),
    )
    .expect("rank-zero speeds match the empty tuple")
}

/// `a + b*sqrt(A) + c*sqrt(B) + d*sqrt(A*B)` in a rank-zero base.
pub fn base_value(field: &RecursiveQuadraticField, parts: [Real; 4]) -> RecursiveQuadraticValue {
    let [rational, first, second, product] = parts;
    RecursiveQuadraticValue::from_base(
        base_arc(field),
        TwoSquareRootExpression {
            rational: constant_tensor(0, rational),
            first: constant_tensor(0, first),
            second: constant_tensor(0, second),
            product: constant_tensor(0, product),
        },
    )
    .expect("rank-zero value")
}

pub fn int_value(field: &RecursiveQuadraticField, parts: [i64; 4]) -> RecursiveQuadraticValue {
    base_value(field, parts.map(real))
}

/// The closed form of [`base_value`] with speeds `A` and `B`.
pub fn base_closed_form(first: i64, second: i64, parts: [i64; 4]) -> Real {
    real(parts[0])
        + real(parts[1]) * root_of(first)
        + real(parts[2]) * root_of(second)
        + real(parts[3]) * root_of(first * second)
}

/// The positive generator of an extension field.
pub fn generator(field: &RecursiveQuadraticField) -> RecursiveQuadraticValue {
    let extension = extension_arc(field);
    field
        .element(
            extension.parent.constant(Real::zero()).expect("zero"),
            extension.parent.constant(Real::one()).expect("one"),
        )
        .expect("generator")
}

/// A sparse-coefficient value over a rank-one base. Every polynomial is
/// supplied in ascending powers of the selected source axis.
pub fn axis_value(
    field: &RecursiveQuadraticField,
    rational: &[Real],
    first: &[Real],
    second: &[Real],
    product: &[Real],
) -> Option<RecursiveQuadraticValue> {
    RecursiveQuadraticValue::from_base(
        base_arc(field),
        TwoSquareRootExpression {
            rational: axis_tensor(rational),
            first: axis_tensor(first),
            second: axis_tensor(second),
            product: axis_tensor(product),
        },
    )
}

/// The standard tower `Q(sqrt2, sqrt3)(sqrt(1 + sqrt2))(sqrt(sqrt(1 + sqrt2)))`.
pub struct Tower {
    pub base: RecursiveQuadraticField,
    pub first: RecursiveQuadraticField,
    pub second: RecursiveQuadraticField,
}

impl Tower {
    pub fn new() -> Self {
        Self::over(quadratic_base(2, 3))
    }

    pub fn over(base: RecursiveQuadraticField) -> Self {
        let first = base
            .extension(int_value(&base, [1, 1, 0, 0]))
            .expect("radicand belongs to the base");
        let first_generator = generator(&first);
        let second = first
            .extension(first_generator)
            .expect("radicand belongs to the first extension");
        Self {
            base,
            first,
            second,
        }
    }

    /// `sqrt(1 + sqrt2)`.
    pub fn first_closed_form() -> Real {
        sqrt(real(1) + root_of(2))
    }

    /// `(1 + sqrt2)^(1/4)`.
    pub fn second_closed_form() -> Real {
        sqrt(Self::first_closed_form())
    }
}
