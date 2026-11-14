//! Native coefficient grids compiled through the shared expression builder.
use super::{
    BinaryOperation, ExpressionBudget, ExpressionDefinition, ExpressionError, ExpressionNode,
    ExpressionProgram, jacobian::Builder,
};
use crate::{BivariatePolynomial, SymbolId, SymbolRef};
use hyperreal::Real;

impl ExpressionProgram {
    /// Compile ordered bivariate numerators over one shared denominator.
    ///
    /// The binding order is the polynomial's first axis, then its second axis.
    /// Coefficient rows may be ragged; empty rows and grids are zero. Exact
    /// coefficients and explicit division guards are retained, including for a
    /// zero numerator. No cancellation, sign test or scalar demotion is applied.
    /// This constructs a definition, not evidence that the denominator is nonzero
    /// on any domain. Evaluation and regularity certification remain separate.
    ///
    /// Graph limits charge visited rows, emitted nodes, binding/output records
    /// and validation. Entry checks precede graph allocation; scalar payload
    /// cloning/destruction, arithmetic, coefficient growth and total heap bytes
    /// retain the expression budget's exclusions. Cancellation is checked during
    /// construction and before publication. Failure stores no global state.
    pub fn from_bivariate_quotients(
        numerators: &[BivariatePolynomial],
        denominator: &BivariatePolynomial,
        variables: [SymbolId; 2],
        budget: &mut ExpressionBudget,
    ) -> Result<Self, ExpressionError> {
        budget.charge(0)?;
        budget.check_entries(&[2, 2, numerators.len()])?;
        budget.charge(2)?;
        if variables[0] == variables[1] {
            return Err(ExpressionError::DuplicateVariable(variables[0]));
        }
        let mut builder = Builder {
            definition: ExpressionDefinition {
                variables: variables.to_vec(),
                nodes: Vec::new(),
                outputs: Vec::new(),
            },
            outputs: numerators.len(),
            budget,
        };
        let u = builder.push(ExpressionNode::Variable(SymbolRef::new(variables[0], None)))?;
        let v = builder.push(ExpressionNode::Variable(SymbolRef::new(variables[1], None)))?;
        let weight = append_bivariate(&mut builder, denominator, [u, v])?;
        for numerator in numerators {
            let numerator = append_bivariate(&mut builder, numerator, [u, v])?;
            let quotient = builder.push(ExpressionNode::Binary {
                operation: BinaryOperation::Div,
                left: numerator,
                right: weight,
            })?;
            builder.budget.charge(1)?;
            builder.definition.outputs.push(quotient);
        }
        let program = Self::from_definition(builder.definition, builder.budget)?;
        budget.charge(0)?;
        Ok(program)
    }
}

fn append_bivariate(
    builder: &mut Builder<'_>,
    polynomial: &BivariatePolynomial,
    [u, v]: [usize; 2],
) -> Result<usize, ExpressionError> {
    let mut outer = None;
    for row in polynomial.coefficients.iter().rev() {
        builder.budget.charge(1)?;
        let mut inner = None;
        for coefficient in row.iter().rev() {
            let product = inner
                .map(|left| {
                    builder.push(ExpressionNode::Binary {
                        operation: BinaryOperation::Mul,
                        left,
                        right: v,
                    })
                })
                .transpose()?;
            let constant = builder.push(ExpressionNode::Constant(coefficient.clone()))?;
            inner = Some(match product {
                None => constant,
                Some(left) => builder.push(ExpressionNode::Binary {
                    operation: BinaryOperation::Add,
                    left,
                    right: constant,
                })?,
            });
        }
        let value = match inner {
            Some(value) => value,
            None => builder.push(ExpressionNode::Constant(Real::zero()))?,
        };
        outer = Some(match outer {
            None => value,
            Some(left) => {
                let product = builder.push(ExpressionNode::Binary {
                    operation: BinaryOperation::Mul,
                    left,
                    right: u,
                })?;
                builder.push(ExpressionNode::Binary {
                    operation: BinaryOperation::Add,
                    left: product,
                    right: value,
                })?
            }
        });
    }
    match outer {
        Some(value) => Ok(value),
        None => builder.push(ExpressionNode::Constant(Real::zero())),
    }
}
