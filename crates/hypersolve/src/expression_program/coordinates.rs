//! Certified finite affine coordinate changes of vector-valued programs.
use super::{jacobian::Builder, *};
use hyperlimit::{Certainty, PredicateOutcome, PredicatePolicy, Sign, classify_real_sign};
use std::collections::BTreeSet;

/// Replay data for `x = origin + to_source*y` and `y' = from_source*f(x)`.
/// All matrices are square in `variables` order. Other bindings are unchanged.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CoordinateChange {
    /// Distinct source inputs reinterpreted as the new coordinates.
    pub variables: Vec<SymbolId>,
    /// Exact matrix A taking new coordinates to original coordinates.
    pub to_source: Vec<Vec<Real>>,
    /// Proposed exact inverse B. Admission proves B*A=I under STRICT.
    pub from_source: Vec<Vec<Real>>,
    /// Exact original-coordinate translation c.
    pub origin: Vec<Real>,
}
/// Shape, inverse-proof or original source/resource failure.
#[derive(Clone, Debug)]
pub enum CoordinateError {
    /// A vector, matrix or row has an incorrect dimension.
    Shape {
        field: &'static str,
        row: Option<usize>,
        expected: usize,
        actual: usize,
    },
    /// A product entry differs exactly from the identity entry.
    NotInverse {
        row: usize,
        column: usize,
        outcome: PredicateOutcome<Sign>,
    },
    /// An identity entry could not be proved exactly; not evidence of singularity.
    InverseUndecided {
        row: usize,
        column: usize,
        outcome: PredicateOutcome<Sign>,
    },
    /// Original binding, source or graph-budget diagnostic.
    Expression(ExpressionError),
    /// Original exact product-sum diagnostic.
    Scalar(Problem),
}
impl fmt::Display for CoordinateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "coordinate change: {self:?}")
    }
}
impl std::error::Error for CoordinateError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Expression(e) => Some(e),
            Self::Scalar(e) => Some(e),
            _ => None,
        }
    }
}
impl From<ExpressionError> for CoordinateError {
    fn from(e: ExpressionError) -> Self {
        Self::Expression(e)
    }
}
/// Source-bound exact chart and derivative program. No executable proof is deserialized.
///
/// In finite dimension B*A=I proves invertibility. The retained STRICT predicate
/// outcomes certify each entry; replay regenerates them from the exact input data.
/// This is an algebraic change of coordinates, not an ODE existence certificate.
#[derive(Clone, Debug)]
pub struct CoordinateProgram {
    source: ExpressionProgram,
    change: Arc<CoordinateChange>,
    program: ExpressionProgram,
    mapping: Arc<[usize]>,
    inverse_evidence: Arc<[PredicateOutcome<Sign>]>,
}
impl CoordinateProgram {
    /// Original complete source and binding metadata.
    pub fn source(&self) -> &ExpressionProgram {
        &self.source
    }
    /// Exact retained chart and inverse proposal, validated at construction.
    pub fn change(&self) -> &CoordinateChange {
        &self.change
    }
    /// Transformed derivative program in original binding/output order.
    pub fn program(&self) -> &ExpressionProgram {
        &self.program
    }
    /// Generated node implementing the original node before output mixing.
    pub fn transformed_node(&self, source_node: usize) -> Option<usize> {
        self.mapping.get(source_node).copied()
    }
    /// Row-major proofs that B*A minus identity is exactly zero.
    pub fn inverse_evidence(&self) -> &[PredicateOutcome<Sign>] {
        &self.inverse_evidence
    }
}
fn shape(
    field: &'static str,
    row: Option<usize>,
    actual: usize,
    expected: usize,
) -> Result<(), CoordinateError> {
    if actual == expected {
        Ok(())
    } else {
        Err(CoordinateError::Shape {
            field,
            row,
            actual,
            expected,
        })
    }
}
impl ExpressionProgram {
    /// Certify an exact affine chart and retain `B*f(c+A*y)` as a shared DAG.
    ///
    /// Preserves source operations and guards, including zero-weighted outputs;
    /// all selected-variable occurrences share one generated affine expression.
    /// No expression tree is reconstructed. The original bodies and all inverse
    /// residuals are consumed in deterministic row/column order.
    ///
    /// Graph work/entries cover shape scans, structural product-sum preparation,
    /// generated nodes, mapping, validation and publication. Exact scalar payload,
    /// arithmetic/refinement and predicate internals, heap bytes, serialization and
    /// cancellation inside one scalar/predicate stage remain outside those bounds.
    /// An undecided inverse is typed, never silently treated as invertible.
    pub fn change_coordinates(
        &self,
        change: CoordinateChange,
        budget: &mut ExpressionBudget,
    ) -> Result<CoordinateProgram, CoordinateError> {
        budget.charge(0)?;
        let source = self.definition();
        let n = change.variables.len();
        shape("outputs", None, source.outputs.len(), n)?;
        shape("origin", None, change.origin.len(), n)?;
        let square = n.checked_mul(n).ok_or(ExpressionError::SizeExhausted)?;
        let row_nodes = n
            .checked_mul(3)
            .and_then(|x| x.checked_add(1))
            .ok_or(ExpressionError::SizeExhausted)?;
        let nodes = n
            .checked_mul(row_nodes)
            .and_then(|x| x.checked_mul(2))
            .and_then(|x| x.checked_add(n))
            .and_then(|x| x.checked_add(source.nodes.len()))
            .ok_or(ExpressionError::SizeExhausted)?;
        budget.check_entries(&[
            nodes,
            source.variables.len(),
            source.nodes.len(),
            square,
            square,
            square,
            n,
            n,
            n,
            n,
            n,
            n,
            n,
            n,
            n,
        ])?;
        for (field, matrix) in [
            ("to_source", &change.to_source),
            ("from_source", &change.from_source),
        ] {
            shape(field, None, matrix.len(), n)?;
            for (row, values) in matrix.iter().enumerate() {
                budget.charge(1)?;
                shape(field, Some(row), values.len(), n)?;
            }
        }
        let mut seen = BTreeSet::new();
        for variable in &change.variables {
            budget.charge(1)?;
            if !seen.insert(*variable) {
                return Err(ExpressionError::DuplicateVariable(*variable).into());
            }
            budget.charge(source.variables.len())?;
            if !source.variables.contains(variable) {
                return Err(ExpressionError::UnboundVariable(*variable).into());
            }
        }
        let mut inverse_evidence = Vec::new();
        for (row, values) in change.from_source.iter().enumerate() {
            budget.charge(1)?;
            for column in 0..n {
                budget.charge(n)?;
                let axis: Vec<_> = change.to_source.iter().map(|r| r[column].clone()).collect();
                budget.charge(n)?;
                let residual = Real::sum_products(values, &axis)
                    .map_err(CoordinateError::Scalar)?
                    - Real::from(u8::from(row == column));
                budget.charge(1)?;
                let outcome = classify_real_sign(&residual, PredicatePolicy::STRICT);
                match outcome {
                    PredicateOutcome::Decided {
                        value: Sign::Zero,
                        certainty: Certainty::Exact | Certainty::Filtered,
                        ..
                    } => inverse_evidence.push(outcome),
                    PredicateOutcome::Decided {
                        value: Sign::Negative | Sign::Positive,
                        certainty: Certainty::Exact | Certainty::Filtered,
                        ..
                    } => {
                        return Err(CoordinateError::NotInverse {
                            row,
                            column,
                            outcome,
                        });
                    }
                    _ => {
                        return Err(CoordinateError::InverseUndecided {
                            row,
                            column,
                            outcome,
                        });
                    }
                }
            }
        }
        budget.charge(source.variables.len())?;
        let mut builder = Builder {
            definition: ExpressionDefinition {
                variables: source.variables.clone(),
                nodes: Vec::new(),
                outputs: Vec::new(),
            },
            outputs: n,
            budget,
        };
        let mut coordinates = Vec::new();
        for &variable in &change.variables {
            coordinates
                .push(builder.push(ExpressionNode::Variable(SymbolRef::new(variable, None)))?);
        }
        let mut replacements = BTreeMap::new();
        for (row, &variable) in change.variables.iter().enumerate() {
            builder.budget.charge(1)?;
            let root = affine_row(
                &mut builder,
                &change.to_source[row],
                &coordinates,
                &change.origin[row],
            )?;
            replacements.insert(variable, root);
        }
        let mut mapping = Vec::new();
        for node in &source.nodes {
            builder.budget.charge(1)?;
            if let ExpressionNode::Variable(reference) = node
                && let Some(&root) = replacements.get(&reference.id)
            {
                mapping.push(root);
                continue;
            }
            let node = match node {
                ExpressionNode::Constant(_) | ExpressionNode::Variable(_) => node.clone(),
                ExpressionNode::Unary { operation, input } => ExpressionNode::Unary {
                    operation: *operation,
                    input: mapping[*input],
                },
                ExpressionNode::Binary {
                    operation,
                    left,
                    right,
                } => ExpressionNode::Binary {
                    operation: *operation,
                    left: mapping[*left],
                    right: mapping[*right],
                },
            };
            mapping.push(builder.push(node)?);
        }
        builder.budget.charge(n)?;
        let roots: Vec<_> = source.outputs.iter().map(|&root| mapping[root]).collect();
        for row in &change.from_source {
            builder.budget.charge(1)?;
            let root = affine_row(&mut builder, row, &roots, &Real::zero())?;
            builder.definition.outputs.push(root);
        }
        let program = Self::from_definition(builder.definition, builder.budget)?;
        budget.charge(source.nodes.len())?;
        budget.charge(square)?;
        budget.charge(3)?;
        let result = CoordinateProgram {
            source: self.clone(),
            change: Arc::new(change),
            program,
            mapping: mapping.into(),
            inverse_evidence: inverse_evidence.into(),
        };
        budget.charge(0)?;
        Ok(result)
    }
}
// A graph row is not exact scalar arithmetic or a domain-simplifying polynomial
// reducer: even zero weights preserve evaluation of the corresponding operand.
fn affine_row(
    builder: &mut Builder<'_>,
    weights: &[Real],
    inputs: &[usize],
    offset: &Real,
) -> Result<usize, ExpressionError> {
    let mut root = builder.push(ExpressionNode::Constant(offset.clone()))?;
    for (weight, &input) in weights.iter().zip(inputs) {
        builder.budget.charge(1)?;
        let coefficient = builder.push(ExpressionNode::Constant(weight.clone()))?;
        let product = builder.push(ExpressionNode::Binary {
            operation: BinaryOperation::Mul,
            left: coefficient,
            right: input,
        })?;
        root = builder.push(ExpressionNode::Binary {
            operation: BinaryOperation::Add,
            left: root,
            right: product,
        })?;
    }
    Ok(root)
}

#[cfg(feature = "coordinate-transition")]
mod transition;
#[cfg(feature = "coordinate-transition")]
pub use transition::{CoordinateTransition, CoordinateTransitionError};

#[cfg(feature = "qr-coordinates")]
mod qr;
#[cfg(feature = "qr-coordinates")]
pub use qr::{QrCoordinateError, QrCoordinateProgram, QrCoordinateRequest};
