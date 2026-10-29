//! Finite polynomial extensions with retained real-root selections.
//!
//! A value is a rational function in one selected generator over its parent
//! field. Parent fields are shared, so dependent roots retain a construction
//! graph rather than becoming independently projected coordinates. Recursive
//! quadratic fields remain the native base specialization. Local monic
//! reduction preserves values on the original relation, including when that
//! relation is reducible or repeated; denominators need only be nonzero at
//! the selected root. No irreducibility or global inverse is assumed.
//!
//! One shared promotion unit admits each new extension's selection proof.
//! Arithmetic and remainder work inside that proof are not a total work,
//! coefficient-size, memory or cancellation bound. Common fields retain exact
//! embeddings of independent selections; construction-identity checks alone
//! do not decide equality of arbitrary algebraic values.

mod definition;
pub use definition::{
    SelectedPolynomialDefinition, SelectedPolynomialDefinitionNode, SelectedPolynomialNodeLimit,
    SelectedPolynomialReplayError, SelectedPolynomialRootDefinition,
};

mod bounds;
pub use bounds::SelectedPolynomialEnclosure;
mod join;
pub use join::SelectedPolynomialJoin;

use std::{cmp::Ordering, sync::Arc};

use hyperreal::{Real, RealSign};

use crate::classification::{Classification, UncertaintyReason};
use crate::ordered_field_roots::OrderedFieldPolynomialContext;
use crate::recursive_quadratic_field::{
    RecursiveQuadraticField, RecursiveQuadraticProjectiveScalar, RecursiveQuadraticValue,
};
use crate::{
    IsolatedRootInterval, SelectedAlgebraContext, ordered_field_count_roots,
    ordered_field_sign_at_selected_root,
};

/// Arithmetic that could not retain the requested field value.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SelectedPolynomialArithmeticError {
    /// The operands need an embedding not supplied by their ancestor chain.
    IncompatibleField,
    /// Checked size or underlying field construction was unavailable.
    /// This does not assert that a declared resource budget was exhausted.
    ConstructionUnavailable,
    /// A native base has no adjoined polynomial generator.
    NoPolynomialGenerator,
}

/// A failed selection or exact field operation.
#[derive(Debug)]
pub enum SelectedPolynomialError<E> {
    /// Failure from the caller's selected-algebra authority.
    Context(E),
    /// A field or storage construction could not be completed.
    Arithmetic(SelectedPolynomialArithmeticError),
    /// The open selector is reversed, empty, or has a defining root at an endpoint.
    InvalidSelector,
    /// Supplied closed bounds are reversed or do not contain the retained value.
    InvalidEnclosure,
    /// The defining equation vanishes identically in the parent field.
    ZeroPolynomial,
    /// The selector contains no real root of the defining equation.
    NoRoot,
    /// The selector contains this many distinct roots and selects none uniquely.
    MultipleRoots(usize),
    /// A rational value's denominator vanishes at the selected root.
    ZeroDenominator,
    /// Internal ordered-field contexts propagate an undecided predicate.
    /// Public selection and value predicates return this as `Classification`.
    Uncertain(UncertaintyReason),
}

impl<E> From<SelectedPolynomialArithmeticError> for SelectedPolynomialError<E> {
    fn from(error: SelectedPolynomialArithmeticError) -> Self {
        Self::Arithmetic(error)
    }
}

type ArithmeticResult<T> = Result<T, SelectedPolynomialArithmeticError>;
type ExactResult<T, E> = Result<Classification<T>, SelectedPolynomialError<E>>;

/// An admitted coefficient field or one selected polynomial extension.
#[derive(Clone, Debug)]
pub struct SelectedPolynomialField(Arc<FieldData>);

#[derive(Debug)]
enum FieldData {
    Recursive(RecursiveQuadraticField),
    Extension {
        parent: SelectedPolynomialField,
        defining: Vec<SelectedPolynomialValue>,
        monic: Vec<SelectedPolynomialValue>,
        interval: IsolatedRootInterval,
        // A join transported this original generator and revalidated its
        // singleton selection. This is a construction relation, not a cache.
        origin: Option<GeneratorOrigin>,
    },
}

struct GeneratorOrigin(SelectedPolynomialField);

impl std::fmt::Debug for GeneratorOrigin {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Do not expand a shared proof graph into a diagnostic tree.
        f.debug_struct("GeneratorOrigin").finish_non_exhaustive()
    }
}

/// A finite rational value retaining its selected field and denominator proof.
#[derive(Clone, Debug)]
pub struct SelectedPolynomialValue(Arc<ValueData>);

#[derive(Debug)]
struct ValueData {
    field: SelectedPolynomialField,
    coordinates: Coordinates,
}

#[derive(Debug)]
enum Coordinates {
    Recursive(RecursiveQuadraticProjectiveScalar),
    Extension {
        numerator: Vec<SelectedPolynomialValue>,
        // Strictly positive at this selected generator, by construction.
        denominator: Vec<SelectedPolynomialValue>,
    },
}

fn finish<T, E>(result: Result<T, SelectedPolynomialError<E>>) -> ExactResult<T, E> {
    if crate::work_budget::exhaustion().is_some() {
        return Ok(Classification::Uncertain(
            UncertaintyReason::BudgetExhausted,
        ));
    }
    match result {
        Ok(value) => Ok(Classification::Decided(value)),
        Err(SelectedPolynomialError::Uncertain(reason)) => Ok(Classification::Uncertain(reason)),
        Err(error) => Err(error),
    }
}

fn unavailable<T>(value: Option<T>) -> ArithmeticResult<T> {
    value.ok_or(SelectedPolynomialArithmeticError::ConstructionUnavailable)
}

fn recursive_ancestor(field: &RecursiveQuadraticField, ancestor: &RecursiveQuadraticField) -> bool {
    if field.same_field(ancestor) {
        return true;
    }
    match field {
        RecursiveQuadraticField::Base(_) => false,
        RecursiveQuadraticField::Extension(extension) => {
            recursive_ancestor(&extension.parent, ancestor)
        }
    }
}

impl SelectedPolynomialField {
    /// Retains an existing admitted native coefficient field without projection.
    /// Its existing selected-root and positive-radical preconditions still apply.
    pub fn from_recursive(field: RecursiveQuadraticField) -> Self {
        Self(Arc::new(FieldData::Recursive(field)))
    }

    /// Tests shared construction authority, not equality of algebraic fields.
    pub fn shares_authority(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
            || match (self.0.as_ref(), other.0.as_ref()) {
                (FieldData::Recursive(a), FieldData::Recursive(b)) => a.same_field(b),
                _ => false,
            }
    }

    /// The retained coefficient field of a polynomial extension.
    pub fn parent(&self) -> Option<&Self> {
        match self.0.as_ref() {
            FieldData::Extension { parent, .. } => Some(parent),
            _ => None,
        }
    }

    /// Original ascending coefficients, including repeated factors.
    pub fn defining_polynomial(&self) -> Option<&[SelectedPolynomialValue]> {
        match self.0.as_ref() {
            FieldData::Extension { defining, .. } => Some(defining),
            _ => None,
        }
    }

    /// Certified open singleton bounds; the defining equation is nonzero at both.
    pub fn isolating_bounds(&self) -> Option<[&Real; 2]> {
        match self.0.as_ref() {
            FieldData::Extension { interval, .. } => Some([&interval.lower, &interval.upper]),
            _ => None,
        }
    }

    /// Embeds one exact represented scalar.
    pub fn constant(&self, value: Real) -> ArithmeticResult<SelectedPolynomialValue> {
        match self.0.as_ref() {
            FieldData::Recursive(field) => self.recursive_value(
                unavailable(field.constant(value))?,
                unavailable(field.constant(Real::one()))?,
            ),
            FieldData::Extension { parent, .. } => self.extension_value(
                vec![parent.constant(value)?],
                vec![parent.constant(Real::one())?],
            ),
        }
    }

    /// Imports a native value through the same native field or its descendants.
    pub fn import_recursive(
        &self,
        value: &RecursiveQuadraticValue,
    ) -> ArithmeticResult<SelectedPolynomialValue> {
        match self.0.as_ref() {
            FieldData::Recursive(field) => {
                if !recursive_ancestor(field, &value.field()) {
                    return Err(SelectedPolynomialArithmeticError::IncompatibleField);
                }
                self.recursive_value(
                    unavailable(field.lift(value))?,
                    unavailable(field.constant(Real::one()))?,
                )
            }
            FieldData::Extension { parent, .. } => self.lift(&parent.import_recursive(value)?),
        }
    }

    /// Lifts a value through certified ancestor inclusions, retaining selections.
    pub fn lift(
        &self,
        value: &SelectedPolynomialValue,
    ) -> ArithmeticResult<SelectedPolynomialValue> {
        if self.shares_authority(value.field()) {
            return Ok(value.clone());
        }
        match (self.0.as_ref(), &value.0.coordinates) {
            (FieldData::Recursive(field), Coordinates::Recursive(scalar)) => {
                if !recursive_ancestor(field, &scalar.numerator.field()) {
                    return Err(SelectedPolynomialArithmeticError::IncompatibleField);
                }
                self.recursive_value(
                    unavailable(field.lift(&scalar.numerator))?,
                    unavailable(field.lift(&scalar.denominator))?,
                )
            }
            (FieldData::Extension { parent, .. }, _) => self.extension_value(
                vec![parent.lift(value)?],
                vec![parent.constant(Real::one())?],
            ),
            _ => Err(SelectedPolynomialArithmeticError::IncompatibleField),
        }
    }

    /// Returns the selected root, retaining its full original relation.
    pub fn generator(&self) -> ArithmeticResult<SelectedPolynomialValue> {
        let Some(parent) = self.parent() else {
            return Err(SelectedPolynomialArithmeticError::NoPolynomialGenerator);
        };
        self.extension_value(
            vec![
                parent.constant(Real::zero())?,
                parent.constant(Real::one())?,
            ],
            vec![parent.constant(Real::one())?],
        )
    }

    /// Evaluates ascending coefficients at an exact represented scalar.
    /// Coefficients must belong to this field or an admitted ancestor. The
    /// existing field arithmetic retains their selections and denominator
    /// proofs; this operation makes no root-selection or nonzero claim.
    pub fn evaluate_polynomial(
        &self,
        coefficients: &[SelectedPolynomialValue],
        point: &Real,
    ) -> ArithmeticResult<SelectedPolynomialValue> {
        self.lift(&evaluate(coefficients, point, self)?)
    }

    /// Adjoins the unique distinct root in the supplied open interval.
    ///
    /// Strict Sturm root counting replays the selector; repeated and reducible
    /// relations remain valid. No caller-supplied root count is trusted. Zero
    /// equations, no root, multiple roots and invalid selectors are distinct
    /// errors. Unavailable decisions and reached work limits remain uncertain.
    pub fn select<C: SelectedAlgebraContext>(
        &self,
        coefficients: Vec<SelectedPolynomialValue>,
        lower: Real,
        upper: Real,
        policy: &C,
    ) -> ExactResult<Self, C::Error> {
        finish(
            policy.strict_predicate_pass(|| self.select_strict(coefficients, lower, upper, policy)),
        )
    }

    fn select_strict<C: SelectedAlgebraContext>(
        &self,
        coefficients: Vec<SelectedPolynomialValue>,
        lower: Real,
        upper: Real,
        policy: &C,
    ) -> Result<Self, SelectedPolynomialError<C::Error>> {
        self.select_strict_with_origin(coefficients, lower, upper, None, policy)
    }

    // Only a join supplies an origin, after embedding that source's original
    // coefficients. The same open singleton below then proves correspondence.
    fn select_strict_with_origin<C: SelectedAlgebraContext>(
        &self,
        coefficients: Vec<SelectedPolynomialValue>,
        lower: Real,
        upper: Real,
        origin: Option<Self>,
        policy: &C,
    ) -> Result<Self, SelectedPolynomialError<C::Error>> {
        match policy.real_sign(&(&upper - &lower)) {
            Some(RealSign::Positive) => {}
            Some(_) => return Err(SelectedPolynomialError::InvalidSelector),
            None => {
                return Err(SelectedPolynomialError::Uncertain(
                    UncertaintyReason::RealSign,
                ));
            }
        }
        let defining = coefficients
            .iter()
            .map(|value| self.lift(value))
            .collect::<ArithmeticResult<Vec<_>>>()?;
        crate::work_budget::consume(crate::work_budget::WorkKind::AlgebraicPromotion, 1)
            .map_err(|_| SelectedPolynomialError::Uncertain(UncertaintyReason::BudgetExhausted))?;
        let mut reduced = defining.clone();
        while let Some(value) = reduced.last() {
            if value.sign_strict(policy)? != RealSign::Zero {
                break;
            }
            reduced.pop();
        }
        if reduced.is_empty() {
            return Err(SelectedPolynomialError::ZeroPolynomial);
        }
        if reduced.len() == 1 {
            return Err(SelectedPolynomialError::NoRoot);
        }
        for endpoint in [&lower, &upper] {
            if evaluate(&reduced, endpoint, self)?.sign_strict(policy)? == RealSign::Zero {
                return Err(SelectedPolynomialError::InvalidSelector);
            }
        }
        let mut context = SelectedPolynomialContext {
            field: self.clone(),
            policy,
        };
        match ordered_field_count_roots(&reduced, &lower, &upper, &mut context)? {
            Some(0) => return Err(SelectedPolynomialError::NoRoot),
            Some(1) => {}
            Some(count) => return Err(SelectedPolynomialError::MultipleRoots(count)),
            None => {
                return Err(SelectedPolynomialError::Uncertain(
                    UncertaintyReason::Predicate,
                ));
            }
        }
        let leading = reduced
            .last()
            .expect("a nonconstant relation has a leading term");
        let inverse = leading.reciprocal_strict(policy)?;
        let monic = reduced
            .iter()
            .map(|value| value.multiply(&inverse))
            .collect::<ArithmeticResult<Vec<_>>>()?;
        Ok(Self(Arc::new(FieldData::Extension {
            parent: self.clone(),
            defining,
            monic,
            origin: origin.map(GeneratorOrigin),
            interval: IsolatedRootInterval {
                lower,
                upper,
                exact_root: None,
                distinct_root_count: 1,
            },
        })))
    }

    // Both constructors below are private. Positivity follows from constants,
    // multiplication, ancestor lifting, or a strict reciprocal sign proof.
    fn recursive_value(
        &self,
        numerator: RecursiveQuadraticValue,
        denominator: RecursiveQuadraticValue,
    ) -> ArithmeticResult<SelectedPolynomialValue> {
        let mut pair = [numerator, denominator];
        RecursiveQuadraticValue::normalize_positive_scale(&mut pair);
        let [numerator, denominator] = pair;
        Ok(SelectedPolynomialValue(Arc::new(ValueData {
            field: self.clone(),
            coordinates: Coordinates::Recursive(RecursiveQuadraticProjectiveScalar {
                numerator,
                denominator,
            }),
        })))
    }

    fn extension_value(
        &self,
        numerator: Vec<SelectedPolynomialValue>,
        denominator: Vec<SelectedPolynomialValue>,
    ) -> ArithmeticResult<SelectedPolynomialValue> {
        Ok(SelectedPolynomialValue(Arc::new(ValueData {
            field: self.clone(),
            coordinates: Coordinates::Extension {
                numerator: self.reduce(numerator)?,
                denominator: self.reduce(denominator)?,
            },
        })))
    }

    fn reduce(
        &self,
        mut polynomial: Vec<SelectedPolynomialValue>,
    ) -> ArithmeticResult<Vec<SelectedPolynomialValue>> {
        let FieldData::Extension { parent, monic, .. } = self.0.as_ref() else {
            return Err(SelectedPolynomialArithmeticError::NoPolynomialGenerator);
        };
        let degree = monic.len() - 1;
        while polynomial.len() > degree {
            let leading = polynomial.pop().expect("one coefficient is eliminated");
            let shift = polynomial.len() - degree;
            if leading.stored_zero() {
                continue;
            }
            for (index, coefficient) in monic[..degree].iter().enumerate() {
                polynomial[shift + index] =
                    polynomial[shift + index].subtract(&leading.multiply(coefficient)?)?;
            }
        }
        while polynomial.len() > 1
            && polynomial
                .last()
                .is_some_and(SelectedPolynomialValue::stored_zero)
        {
            polynomial.pop();
        }
        if polynomial.is_empty() {
            polynomial.push(parent.constant(Real::zero())?);
        }
        Ok(polynomial)
    }
}

impl SelectedPolynomialValue {
    /// Field retaining this value's selected dependencies.
    pub fn field(&self) -> &SelectedPolynomialField {
        &self.0.field
    }

    fn stored_zero(&self) -> bool {
        match &self.0.coordinates {
            Coordinates::Recursive(scalar) => scalar.numerator.is_structurally_zero(),
            Coordinates::Extension { numerator, .. } => numerator.iter().all(Self::stored_zero),
        }
    }

    fn stored_one(&self) -> bool {
        match &self.0.coordinates {
            Coordinates::Recursive(scalar) => {
                scalar.numerator.is_coefficientwise_stored_one()
                    && scalar.denominator.is_coefficientwise_stored_one()
            }
            Coordinates::Extension {
                numerator,
                denominator,
            } => {
                matches!(numerator.as_slice(), [one] if one.stored_one())
                    && matches!(denominator.as_slice(), [one] if one.stored_one())
            }
        }
    }

    fn align(&self, other: &Self) -> ArithmeticResult<(SelectedPolynomialField, Self, Self)> {
        match self.field().lift(other) {
            Ok(other) => Ok((self.field().clone(), self.clone(), other)),
            Err(SelectedPolynomialArithmeticError::IncompatibleField) => Ok((
                other.field().clone(),
                other.field().lift(self)?,
                other.clone(),
            )),
            Err(error) => Err(error),
        }
    }

    /// Exact addition through the common ancestor chain.
    pub fn add(&self, other: &Self) -> ArithmeticResult<Self> {
        let (field, left, right) = self.align(other)?;
        // Values already own their denominator proofs. These structural
        // identities preserve the aligned field and its selected dependencies.
        if left.stored_zero() {
            return Ok(right);
        }
        if right.stored_zero() {
            return Ok(left);
        }
        match (&left.0.coordinates, &right.0.coordinates) {
            (Coordinates::Recursive(a), Coordinates::Recursive(b)) => field.recursive_value(
                unavailable(
                    unavailable(a.numerator.multiply(&b.denominator))?
                        .add(&unavailable(b.numerator.multiply(&a.denominator))?),
                )?,
                unavailable(a.denominator.multiply(&b.denominator))?,
            ),
            (
                Coordinates::Extension {
                    numerator: a,
                    denominator: b,
                },
                Coordinates::Extension {
                    numerator: c,
                    denominator: d,
                },
            ) => {
                let parent = field.parent().expect("extension coordinates have a parent");
                field.extension_value(
                    add(&multiply(a, d, parent)?, &multiply(c, b, parent)?, parent)?,
                    multiply(b, d, parent)?,
                )
            }
            _ => Err(SelectedPolynomialArithmeticError::IncompatibleField),
        }
    }

    /// Exact subtraction without changing either operand's selection.
    pub fn subtract(&self, other: &Self) -> ArithmeticResult<Self> {
        self.add(&other.scale(&Real::from(-1))?)
    }

    /// Exact multiplication with local defining-relation reduction.
    pub fn multiply(&self, other: &Self) -> ArithmeticResult<Self> {
        let (field, left, right) = self.align(other)?;
        if left.stored_zero() || right.stored_zero() {
            return field.constant(Real::zero());
        }
        if left.stored_one() {
            return Ok(right);
        }
        if right.stored_one() {
            return Ok(left);
        }
        match (&left.0.coordinates, &right.0.coordinates) {
            (Coordinates::Recursive(a), Coordinates::Recursive(b)) => field.recursive_value(
                unavailable(a.numerator.multiply(&b.numerator))?,
                unavailable(a.denominator.multiply(&b.denominator))?,
            ),
            (
                Coordinates::Extension {
                    numerator: a,
                    denominator: b,
                },
                Coordinates::Extension {
                    numerator: c,
                    denominator: d,
                },
            ) => {
                let parent = field.parent().expect("extension coordinates have a parent");
                field.extension_value(multiply(a, c, parent)?, multiply(b, d, parent)?)
            }
            _ => Err(SelectedPolynomialArithmeticError::IncompatibleField),
        }
    }

    /// Exact scalar multiplication; retained denominator guards remain valid.
    pub fn scale(&self, scale: &Real) -> ArithmeticResult<Self> {
        match &self.0.coordinates {
            Coordinates::Recursive(scalar) => self.field().recursive_value(
                unavailable(scalar.numerator.scale(scale))?,
                scalar.denominator.clone(),
            ),
            Coordinates::Extension {
                numerator,
                denominator,
            } => self.field().extension_value(
                numerator
                    .iter()
                    .map(|value| value.scale(scale))
                    .collect::<ArithmeticResult<Vec<_>>>()?,
                denominator.clone(),
            ),
        }
    }

    /// Divides only after proving the denominator nonzero at its selected root.
    pub fn divide<C: SelectedAlgebraContext>(
        &self,
        other: &Self,
        policy: &C,
    ) -> ExactResult<Self, C::Error> {
        finish(policy.strict_predicate_pass(|| {
            let (_, left, right) = self.align(other)?;
            Ok(left.multiply(&right.reciprocal_strict(policy)?)?)
        }))
    }

    fn reciprocal_strict<C: SelectedAlgebraContext>(
        &self,
        policy: &C,
    ) -> Result<Self, SelectedPolynomialError<C::Error>> {
        let scale = match self.sign_strict(policy)? {
            RealSign::Zero => return Err(SelectedPolynomialError::ZeroDenominator),
            RealSign::Positive => Real::one(),
            RealSign::Negative => Real::from(-1),
        };
        Ok(match &self.0.coordinates {
            Coordinates::Recursive(scalar) => self.field().recursive_value(
                unavailable(scalar.denominator.scale(&scale))?,
                unavailable(scalar.numerator.scale(&scale))?,
            )?,
            Coordinates::Extension {
                numerator,
                denominator,
            } => self.field().extension_value(
                denominator
                    .iter()
                    .map(|value| value.scale(&scale))
                    .collect::<ArithmeticResult<Vec<_>>>()?,
                numerator
                    .iter()
                    .map(|value| value.scale(&scale))
                    .collect::<ArithmeticResult<Vec<_>>>()?,
            )?,
        })
    }

    /// Exact sign on the retained selection, with no approximate terminal.
    pub fn sign<C: SelectedAlgebraContext>(&self, policy: &C) -> ExactResult<RealSign, C::Error> {
        finish(policy.strict_predicate_pass(|| self.sign_strict(policy)))
    }

    fn sign_strict<C: SelectedAlgebraContext>(
        &self,
        policy: &C,
    ) -> Result<RealSign, SelectedPolynomialError<C::Error>> {
        if crate::work_budget::exhaustion().is_some() {
            return Err(SelectedPolynomialError::Uncertain(
                UncertaintyReason::BudgetExhausted,
            ));
        }
        if self.stored_zero() {
            return Ok(RealSign::Zero);
        }
        match (&self.field().0.as_ref(), &self.0.coordinates) {
            (_, Coordinates::Recursive(scalar)) => match scalar
                .numerator
                .sign(policy)
                .map_err(SelectedPolynomialError::Context)?
            {
                Classification::Decided(sign) => Ok(sign),
                Classification::Uncertain(reason) => {
                    Err(SelectedPolynomialError::Uncertain(reason))
                }
            },
            (
                FieldData::Extension {
                    parent,
                    defining,
                    interval,
                    ..
                },
                Coordinates::Extension { numerator, .. },
            ) => {
                if let [constant] = numerator.as_slice() {
                    return constant.sign_strict(policy);
                }
                let mut context = SelectedPolynomialContext {
                    field: parent.clone(),
                    policy,
                };
                match ordered_field_sign_at_selected_root(
                    defining,
                    numerator,
                    interval,
                    &mut context,
                )? {
                    Some(Ordering::Less) => Ok(RealSign::Negative),
                    Some(Ordering::Equal) => Ok(RealSign::Zero),
                    Some(Ordering::Greater) => Ok(RealSign::Positive),
                    None => Err(SelectedPolynomialError::Uncertain(
                        UncertaintyReason::Predicate,
                    )),
                }
            }
            _ => Err(SelectedPolynomialArithmeticError::IncompatibleField.into()),
        }
    }
}

/// Adapter to the shared ordered-field isolator and remainder/sign machinery.
/// All coefficient signs run inside a strict pass, even with an approximate policy.
pub struct SelectedPolynomialContext<'a, C> {
    /// Shared selected coefficient field.
    pub field: SelectedPolynomialField,
    /// Caller-owned decision and resource context.
    pub policy: &'a C,
}

impl<C: SelectedAlgebraContext> OrderedFieldPolynomialContext<SelectedPolynomialValue>
    for SelectedPolynomialContext<'_, C>
{
    type Error = SelectedPolynomialError<C::Error>;
    fn constant(&mut self, value: &Real) -> Result<SelectedPolynomialValue, Self::Error> {
        Ok(self.field.constant(value.clone())?)
    }
    fn add(
        &mut self,
        a: &SelectedPolynomialValue,
        b: &SelectedPolynomialValue,
    ) -> Result<SelectedPolynomialValue, Self::Error> {
        Ok(self.field.lift(a)?.add(&self.field.lift(b)?)?)
    }
    fn multiply(
        &mut self,
        a: &SelectedPolynomialValue,
        b: &SelectedPolynomialValue,
    ) -> Result<SelectedPolynomialValue, Self::Error> {
        Ok(self.field.lift(a)?.multiply(&self.field.lift(b)?)?)
    }
    fn scale(
        &mut self,
        value: &SelectedPolynomialValue,
        scale: &Real,
    ) -> Result<SelectedPolynomialValue, Self::Error> {
        Ok(self.field.lift(value)?.scale(scale)?)
    }
    fn normalize_positive_scale(&mut self, values: &mut [SelectedPolynomialValue]) {
        let mut numerators = Vec::with_capacity(values.len());
        for value in values.iter() {
            let Coordinates::Recursive(scalar) = &value.0.coordinates else {
                return;
            };
            if !scalar.denominator.is_coefficientwise_stored_one() {
                return;
            }
            numerators.push(scalar.numerator.clone());
        }
        RecursiveQuadraticValue::normalize_positive_scale(&mut numerators);
        for (value, numerator) in values.iter_mut().zip(numerators) {
            let Coordinates::Recursive(scalar) = &value.0.coordinates else {
                unreachable!()
            };
            *value = SelectedPolynomialValue(Arc::new(ValueData {
                field: value.field().clone(),
                coordinates: Coordinates::Recursive(RecursiveQuadraticProjectiveScalar {
                    numerator,
                    denominator: scalar.denominator.clone(),
                }),
            }));
        }
    }
    fn sign(&mut self, value: &SelectedPolynomialValue) -> Result<Ordering, Self::Error> {
        self.policy
            .strict_predicate_pass(|| self.field.lift(value)?.sign_strict(self.policy))
            .map(sign_order)
    }
    fn sign_if_separated(
        &mut self,
        value: &SelectedPolynomialValue,
    ) -> Result<Option<Ordering>, Self::Error> {
        let value = self.field.lift(value)?;
        if value.stored_zero() {
            return Ok(Some(Ordering::Equal));
        }
        Ok(match &value.0.coordinates {
            Coordinates::Recursive(scalar) => scalar
                .numerator
                .bounded_or_exact_real_witness_sign()
                .map(sign_order),
            Coordinates::Extension { .. } => None,
        })
    }
    fn remainder_elimination_budget(&self) -> Option<usize> {
        // A speculative bounded pass declines after ten leading eliminations;
        // an unbounded strict pass retains the complete finite remainder chain.
        self.policy
            .has_bounded_exact_predicate_budget()
            .then_some(10)
    }
}

fn sign_order(sign: RealSign) -> Ordering {
    match sign {
        RealSign::Negative => Ordering::Less,
        RealSign::Zero => Ordering::Equal,
        RealSign::Positive => Ordering::Greater,
    }
}

fn evaluate(
    polynomial: &[SelectedPolynomialValue],
    point: &Real,
    field: &SelectedPolynomialField,
) -> ArithmeticResult<SelectedPolynomialValue> {
    let mut result = field.constant(Real::zero())?;
    for coefficient in polynomial.iter().rev() {
        result = result.scale(point)?.add(coefficient)?;
    }
    Ok(result)
}

fn add(
    a: &[SelectedPolynomialValue],
    b: &[SelectedPolynomialValue],
    field: &SelectedPolynomialField,
) -> ArithmeticResult<Vec<SelectedPolynomialValue>> {
    let zero = field.constant(Real::zero())?;
    (0..a.len().max(b.len()))
        .map(|i| a.get(i).unwrap_or(&zero).add(b.get(i).unwrap_or(&zero)))
        .collect()
}

fn multiply(
    a: &[SelectedPolynomialValue],
    b: &[SelectedPolynomialValue],
    field: &SelectedPolynomialField,
) -> ArithmeticResult<Vec<SelectedPolynomialValue>> {
    let count = a
        .len()
        .checked_add(b.len())
        .and_then(|n| n.checked_sub(1))
        .ok_or(SelectedPolynomialArithmeticError::ConstructionUnavailable)?;
    let mut result = Vec::new();
    result
        .try_reserve_exact(count)
        .map_err(|_| SelectedPolynomialArithmeticError::ConstructionUnavailable)?;
    result.resize(count, field.constant(Real::zero())?);
    for (i, left) in a.iter().enumerate() {
        for (j, right) in b.iter().enumerate() {
            result[i + j] = result[i + j].add(&left.multiply(right)?)?;
        }
    }
    Ok(result)
}
