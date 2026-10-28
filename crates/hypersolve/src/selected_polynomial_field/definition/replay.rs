use super::super::{FieldData, SelectedPolynomialField, unavailable};
use super::*;
use crate::classification::{Classification, UncertaintyReason};
use crate::recursive_quadratic_field::{RecursiveQuadraticField, RecursiveQuadraticValue};
use crate::{
    AlgebraicField, AlgebraicFieldError, AlgebraicRootRepresentation,
    AlgebraicRootValidationReport, AlgebraicRootValidationStatus, IsolatedRootInterval,
    SelectedAlgebraContext,
};
use hyperreal::RealSign;

type Error<E> = SelectedPolynomialReplayError<E>;
type Result<T, E> = std::result::Result<T, Error<E>>;

fn exact<T, E>(result: Classification<T>) -> Result<T, E> {
    match result {
        Classification::Decided(value) => Ok(value),
        Classification::Uncertain(reason) => Err(SelectedPolynomialError::Uncertain(reason).into()),
    }
}

fn promotion<E>() -> Result<(), E> {
    crate::work_budget::consume(crate::work_budget::WorkKind::AlgebraicPromotion, 1)
        .map_err(|_| SelectedPolynomialError::Uncertain(UncertaintyReason::BudgetExhausted).into())
}

fn native_sign<C: SelectedAlgebraContext>(
    value: &RecursiveQuadraticValue,
    policy: &C,
) -> Result<RealSign, C::Error> {
    exact(
        value
            .sign(policy)
            .map_err(SelectedPolynomialError::Context)?,
    )
}

enum Admitted {
    NativeField(RecursiveQuadraticField),
    NativeValue(RecursiveQuadraticValue),
    Field(SelectedPolynomialField),
    Value(SelectedPolynomialValue),
}

struct Reader<'a>(&'a [Admitted]);

impl Reader<'_> {
    fn invalid<E>(&self, reference: usize) -> Error<E> {
        Error::InvalidReference {
            node: self.0.len(),
            reference,
        }
    }

    fn native_field<E>(&self, reference: usize) -> Result<RecursiveQuadraticField, E> {
        match self.0.get(reference) {
            Some(Admitted::NativeField(value)) => Ok(value.clone()),
            _ => Err(self.invalid(reference)),
        }
    }

    fn native_value<E>(&self, reference: usize) -> Result<RecursiveQuadraticValue, E> {
        match self.0.get(reference) {
            Some(Admitted::NativeValue(value)) => Ok(value.clone()),
            _ => Err(self.invalid(reference)),
        }
    }

    fn field<E>(&self, reference: usize) -> Result<SelectedPolynomialField, E> {
        match self.0.get(reference) {
            Some(Admitted::Field(value)) => Ok(value.clone()),
            _ => Err(self.invalid(reference)),
        }
    }

    fn value<E>(&self, reference: usize) -> Result<SelectedPolynomialValue, E> {
        match self.0.get(reference) {
            Some(Admitted::Value(value)) => Ok(value.clone()),
            _ => Err(self.invalid(reference)),
        }
    }

    fn values<E>(&self, references: &[usize]) -> Result<Vec<SelectedPolynomialValue>, E> {
        references.iter().map(|id| self.value(*id)).collect()
    }

    fn coordinates<E>(&self, valid: bool) -> Result<(), E> {
        if valid {
            Ok(())
        } else {
            Err(Error::InvalidCoordinates { node: self.0.len() })
        }
    }

    fn admit<C: SelectedAlgebraContext>(
        &self,
        definition: &SelectedPolynomialDefinitionNode,
        policy: &C,
    ) -> Result<Admitted, C::Error> {
        use SelectedPolynomialDefinitionNode as Node;
        let node = self.0.len();
        Ok(match definition {
            Node::NativeBase {
                sources,
                first_speed_squared,
                second_speed_squared,
            } => {
                self.coordinates(
                    [first_speed_squared, second_speed_squared]
                        .iter()
                        .all(|p| p.dimensions().len() == sources.len()),
                )?;
                promotion()?;
                let sources = sources
                    .iter()
                    .enumerate()
                    .map(|(source, definition)| {
                        let root = AlgebraicRootRepresentation {
                            constraint_index: definition.constraint_index,
                            symbol: definition.symbol,
                            interval_index: definition.interval_index,
                            polynomial_coefficients: definition.polynomial.clone(),
                            interval: IsolatedRootInterval {
                                lower: definition.lower.clone(),
                                upper: definition.upper.clone(),
                                exact_root: definition.exact_root.clone(),
                                distinct_root_count: 1,
                            },
                            // This provisional payload is never published without
                            // full strict selection replay below. A count of one or
                            // this flag alone is not a uniqueness proof.
                            validation: AlgebraicRootValidationReport {
                                status: AlgebraicRootValidationStatus::Valid,
                                message: None,
                            },
                        };
                        match AlgebraicField::new(&root) {
                            Ok(_) => Ok(root),
                            Err(AlgebraicFieldError::Undecided) => Err(Error::Field(
                                SelectedPolynomialError::Uncertain(UncertaintyReason::Predicate),
                            )),
                            Err(
                                AlgebraicFieldError::InvalidEvidence
                                | AlgebraicFieldError::DivisionByZero,
                            ) => Err(Error::InvalidNativeRoot { node, source }),
                        }
                    })
                    .collect::<Result<Vec<_>, C::Error>>()?;
                // Prove native speed squares in an already valid unit-radical
                // base, before constructing any field with the supplied squares.
                let one = unavailable(DenseTensorPolynomial::try_new(
                    vec![1; sources.len()],
                    vec![Real::one()],
                ))?;
                let RecursiveQuadraticField::Base(validation) = unavailable(
                    RecursiveQuadraticField::base(sources.clone(), one.clone(), one),
                )?
                else {
                    unreachable!()
                };
                for square in [first_speed_squared, second_speed_squared] {
                    let expression =
                        unavailable(TwoSquareRootExpression::from_rational(square.clone()))?;
                    let value = unavailable(RecursiveQuadraticValue::from_base(
                        validation.clone(),
                        expression,
                    ))?;
                    if native_sign(&value, policy)? == RealSign::Negative {
                        return Err(Error::InvalidRadicand { node });
                    }
                }
                Admitted::NativeField(unavailable(RecursiveQuadraticField::base(
                    sources,
                    first_speed_squared.clone(),
                    second_speed_squared.clone(),
                ))?)
            }
            Node::NativeExtension { parent, radicand } => {
                let parent = self.native_field(*parent)?;
                let radicand = self.native_value(*radicand)?;
                self.coordinates(parent.same_field(&radicand.field()))?;
                promotion()?;
                if native_sign(&radicand, policy)? != RealSign::Positive {
                    return Err(Error::InvalidRadicand { node });
                }
                Admitted::NativeField(unavailable(parent.extension(radicand))?)
            }
            Node::NativeBaseValue { field, expression } => {
                let RecursiveQuadraticField::Base(field) = self.native_field(*field)? else {
                    return Err(Error::InvalidCoordinates { node });
                };
                self.coordinates(
                    [
                        &expression.rational,
                        &expression.first,
                        &expression.second,
                        &expression.product,
                    ]
                    .iter()
                    .all(|p| p.dimensions().len() == field.sources.len()),
                )?;
                Admitted::NativeValue(unavailable(RecursiveQuadraticValue::from_base(
                    field,
                    expression.clone(),
                ))?)
            }
            Node::NativeExtensionValue {
                field,
                retained,
                radical,
            } => {
                let RecursiveQuadraticField::Extension(field) = self.native_field(*field)? else {
                    return Err(Error::InvalidCoordinates { node });
                };
                let retained = self.native_value(*retained)?;
                let radical = self.native_value(*radical)?;
                self.coordinates(
                    field.parent.same_field(&retained.field())
                        && field.parent.same_field(&radical.field()),
                )?;
                Admitted::NativeValue(unavailable(RecursiveQuadraticValue::from_extension(
                    field, retained, radical,
                ))?)
            }
            Node::NativeField { native } => Admitted::Field(
                SelectedPolynomialField::from_recursive(self.native_field(*native)?),
            ),
            Node::PolynomialField {
                parent,
                coefficients,
                lower,
                upper,
            } => {
                let parent = self.field(*parent)?;
                Admitted::Field(parent.select_strict(
                    self.values(coefficients)?,
                    lower.clone(),
                    upper.clone(),
                    policy,
                )?)
            }
            Node::NativeValue {
                field,
                numerator,
                denominator,
            } => {
                let field = self.field(*field)?;
                let FieldData::Recursive(native) = field.0.as_ref() else {
                    return Err(Error::InvalidCoordinates { node });
                };
                let numerator = self.native_value(*numerator)?;
                let denominator = self.native_value(*denominator)?;
                self.coordinates(
                    native.same_field(&numerator.field())
                        && native.same_field(&denominator.field()),
                )?;
                let numerator = field.import_recursive(&numerator)?;
                let denominator = field.import_recursive(&denominator)?;
                Admitted::Value(numerator.multiply(&denominator.reciprocal_strict(policy)?)?)
            }
            Node::PolynomialValue {
                field,
                numerator,
                denominator,
            } => {
                let field = self.field(*field)?;
                let Some(parent) = field.parent() else {
                    return Err(Error::InvalidCoordinates { node });
                };
                let numerator = self.values(numerator)?;
                let denominator = self.values(denominator)?;
                self.coordinates(
                    numerator
                        .iter()
                        .chain(&denominator)
                        .all(|v| parent.shares_authority(v.field())),
                )?;
                let one = vec![parent.constant(Real::one())?];
                let numerator = field.extension_value(numerator, one.clone())?;
                let denominator = field.extension_value(denominator, one)?;
                Admitted::Value(numerator.multiply(&denominator.reciprocal_strict(policy)?)?)
            }
        })
    }
}

impl SelectedPolynomialDefinition {
    /// Re-establishes every selection and denominator, including unused records.
    /// No root is exposed until all records and root references pass.
    ///
    /// The caller's record limit is separate from the shared promotion counter:
    /// each native base/extension admission uses one promotion, and polynomial
    /// selection uses its existing promotion charge. Neither bounds internal
    /// arithmetic work, coefficient size, memory, elapsed time or cancellation.
    /// Strict passes forbid approximate terminals even in an approximate context.
    pub fn replay<C: SelectedAlgebraContext>(
        &self,
        node_limit: usize,
        policy: &C,
    ) -> Result<Classification<Vec<SelectedPolynomialValue>>, C::Error> {
        if self.nodes.len() > node_limit {
            return Err(Error::NodeLimit(SelectedPolynomialNodeLimit {
                limit: node_limit,
            }));
        }
        let result = policy.strict_predicate_pass(|| {
            let mut admitted = Vec::new();
            for node in &self.nodes {
                let value = Reader(&admitted).admit(node, policy)?;
                admitted.push(value);
            }
            Reader(&admitted).values(&self.roots)
        });
        if crate::work_budget::exhaustion().is_some() {
            return Ok(Classification::Uncertain(
                UncertaintyReason::BudgetExhausted,
            ));
        }
        match result {
            Ok(values) => Ok(Classification::Decided(values)),
            Err(Error::Field(SelectedPolynomialError::Uncertain(reason))) => {
                Ok(Classification::Uncertain(reason))
            }
            Err(error) => Err(error),
        }
    }
}
