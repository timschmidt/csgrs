//! Demanded normalized mixed partials, with source guards and shared derivatives.
mod model;
use super::jacobian::Builder;
use super::{
    ExpressionBudget, ExpressionDefinition, ExpressionError, ExpressionNode, ExpressionProgram,
};
use crate::SymbolId;
use hyperreal::{Rational, Real};
pub use model::{TaylorModel, TaylorRemainderTerm, TaylorRequest, TaylorTerm};
use num::BigInt;
use std::{collections::BTreeMap, fmt, sync::Arc};

/// One normalized partial D^alpha f / alpha!, in canonical sparse form.
/// This is input data, not a differentiability certificate.
#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CoefficientRequest {
    /// Original output row.
    pub output: usize,
    /// Strictly increasing bound symbol identifiers and nonzero powers.
    /// An empty list requests the original output (degree zero).
    pub powers: Vec<(SymbolId, usize)>,
}

/// Invalid Taylor/coefficient input or an original graph/scalar failure.
#[derive(Clone, Debug)]
pub enum TaylorError {
    /// A requested output row does not exist.
    InvalidOutput { request: usize, output: usize },
    /// Sparse powers contain a zero exponent or non-increasing identifiers.
    InvalidPowers { request: usize, power: usize },
    /// Center dimension differs from the source binding dimension.
    CenterCount { expected: usize, actual: usize },
    /// A center coordinate is empty or not an exact singleton.
    NonPointCenter { slot: usize },
    /// A center coordinate lies outside the nonempty model domain.
    CenterOutsideDomain { slot: usize },
    /// Reuse was requested outside the certified model domain.
    OutsideDomain { slot: usize },
    /// The requested next derivative order cannot be represented by usize.
    DegreeOverflow,
    /// A native monomial image did not cover its whole operand domain.
    MonomialDomain { variable: SymbolId, exponent: usize },
    /// Two independently valid enclosures unexpectedly have empty intersection.
    InconsistentEnclosure { output: usize },
    /// Native interval resource, domain or representation failure.
    Interval(hyperinterval::IntervalError),
    /// Original validation, resource, cancellation or scalar diagnostic.
    Expression(ExpressionError),
}
impl fmt::Display for TaylorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Taylor construction: {self:?}")
    }
}
impl std::error::Error for TaylorError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Expression(error) => Some(error),
            Self::Interval(error) => Some(error),
            _ => None,
        }
    }
}
impl From<ExpressionError> for TaylorError {
    fn from(error: ExpressionError) -> Self {
        Self::Expression(error)
    }
}

/// Immutable association of an exact source and requested normalized partials.
///
/// The executable retains every original output first, then one coefficient per
/// request. Original domain checks cannot disappear under a zero derivative.
/// Partial formulas alone do not establish source smoothness. Replay the original
/// definition and requests through `ExpressionProgram::coefficients`; generated
/// nodes and serialized flags are not proof of their association.
#[derive(Clone, Debug)]
pub struct CoefficientProgram {
    source: ExpressionProgram,
    requests: Arc<[CoefficientRequest]>,
    program: ExpressionProgram,
}
impl CoefficientProgram {
    /// Original exact source and ordered bindings.
    pub fn source(&self) -> &ExpressionProgram {
        &self.source
    }
    /// Canonical sparse requests, preserving caller order and repetitions.
    pub fn requests(&self) -> &[CoefficientRequest] {
        &self.requests
    }
    /// Original outputs followed by the requested normalized coefficient values.
    pub fn program(&self) -> &ExpressionProgram {
        &self.program
    }
    /// Output position of one requested coefficient.
    pub fn coefficient_output(&self, request: usize) -> Option<usize> {
        (request < self.requests.len()).then(|| self.source.definition().outputs.len() + request)
    }
}
impl ExpressionProgram {
    /// Generate only requested mixed partial paths into a shared flat graph.
    ///
    /// Repeated node/variable derivatives are memoized within this construction;
    /// common prefix paths and repeated requests reuse those nodes. Local rules
    /// are the same authority as `jacobian`, with the same left-before-right
    /// derivative order. No source expression tree is reconstructed. The powers
    /// must be canonical and bound; formulas follow increasing identifier order.
    ///
    /// Graph and temporary derivative-workspace entries are each bounded by the
    /// shared entry limit; all visits, copies and new records spend graph work.
    /// Exact factorial/coefficient arithmetic and scalar payloads retain the
    /// existing accounting exclusions. These are not CPU or heap-byte bounds.
    /// Cancellation is checked before publication. Failed attempts retain no
    /// derivative cache, so a larger-budget retry is independent.
    pub fn coefficients(
        &self,
        requests: &[CoefficientRequest],
        budget: &mut ExpressionBudget,
    ) -> Result<CoefficientProgram, TaylorError> {
        budget.charge(0)?;
        let source = self.definition();
        let outputs = source
            .outputs
            .len()
            .checked_add(requests.len())
            .ok_or(ExpressionError::SizeExhausted)?;
        budget.check_entries(&[source.variables.len(), source.nodes.len(), outputs])?;
        let mut powers = 0usize;
        for (index, request) in requests.iter().enumerate() {
            budget.charge(1)?;
            powers = powers
                .checked_add(request.powers.len())
                .ok_or(ExpressionError::SizeExhausted)?;
            budget.check_entries(&[requests.len(), powers])?;
            if request.output >= source.outputs.len() {
                return Err(TaylorError::InvalidOutput {
                    request: index,
                    output: request.output,
                });
            }
            let mut previous = None;
            for (power, &(variable, exponent)) in request.powers.iter().enumerate() {
                budget.charge(1)?;
                if exponent == 0 || previous.is_some_and(|p| p >= variable) {
                    return Err(TaylorError::InvalidPowers {
                        request: index,
                        power,
                    });
                }
                budget.charge(source.variables.len())?;
                if !source.variables.contains(&variable) {
                    return Err(ExpressionError::UnboundVariable(variable).into());
                }
                previous = Some(variable);
            }
        }
        budget.charge(
            requests
                .len()
                .checked_add(powers)
                .ok_or(ExpressionError::SizeExhausted)?,
        )?;
        let requests: Arc<[CoefficientRequest]> = requests.into();
        if requests.is_empty() {
            budget.charge(0)?;
            return Ok(CoefficientProgram {
                source: self.clone(),
                requests,
                program: self.clone(),
            });
        }
        budget.charge(source.variables.len() + source.outputs.len())?;
        let mut definition = ExpressionDefinition {
            variables: source.variables.clone(),
            nodes: Vec::new(),
            outputs: source.outputs.clone(),
        };
        for node in &source.nodes {
            budget.charge(1)?;
            definition.nodes.push(node.clone());
        }
        let mut builder = Builder {
            definition,
            outputs,
            budget,
        };
        let zero = builder.push(ExpressionNode::Constant(Real::zero()))?;
        let one = builder.push(ExpressionNode::Constant(Real::one()))?;
        let mut memo = BTreeMap::new();
        let mut normalized = BTreeMap::new();
        for request in requests.iter() {
            builder.budget.charge(1)?;
            let value = if let Some(&value) = normalized.get(request) {
                value
            } else {
                let mut root = source.outputs[request.output];
                for &(variable, exponent) in &request.powers {
                    for _ in 0..exponent {
                        builder.budget.charge(1)?;
                        root = derive(
                            &mut builder,
                            root,
                            variable,
                            zero,
                            one,
                            &mut memo,
                            requests
                                .len()
                                .checked_add(powers)
                                .and_then(|n| n.checked_add(normalized.len()))
                                .ok_or(ExpressionError::SizeExhausted)?,
                        )?;
                    }
                }
                let value = if root == zero {
                    zero
                } else {
                    let mut factorial = BigInt::from(1);
                    for &(_, exponent) in &request.powers {
                        for k in 2..=exponent {
                            builder.budget.charge(1)?;
                            factorial *= BigInt::from(k);
                        }
                    }
                    if factorial == BigInt::from(1) {
                        root
                    } else {
                        let factor = builder.push(ExpressionNode::Constant(Real::from(
                            Rational::from(1) / Rational::from_bigint(factorial),
                        )))?;
                        builder.push(ExpressionNode::Binary {
                            operation: super::BinaryOperation::Mul,
                            left: root,
                            right: factor,
                        })?
                    }
                };
                builder.budget.check_entries(&[
                    requests.len(),
                    powers,
                    normalized.len(),
                    1,
                    memo.len(),
                ])?;
                builder.budget.charge(1)?;
                normalized.insert(request, value);
                value
            };
            builder.budget.charge(1)?;
            builder.definition.outputs.push(value);
        }
        let program = Self::from_definition(builder.definition, builder.budget)?;
        budget.charge(0)?;
        Ok(CoefficientProgram {
            source: self.clone(),
            requests,
            program,
        })
    }
}
fn derive(
    builder: &mut Builder<'_>,
    root: usize,
    variable: SymbolId,
    zero: usize,
    one: usize,
    memo: &mut BTreeMap<(usize, SymbolId), usize>,
    retained: usize,
) -> Result<usize, ExpressionError> {
    builder.budget.check_entries(&[retained, memo.len(), 1])?;
    builder.budget.charge(1)?;
    let mut tasks = vec![(root, false)];
    while let Some((index, ready)) = tasks.pop() {
        builder.budget.charge(1)?;
        if memo.contains_key(&(index, variable)) {
            continue;
        }
        let node = builder.definition.nodes[index].clone();
        if !ready {
            let children = node.inputs();
            let count = 1 + children.iter().flatten().count();
            builder
                .budget
                .check_entries(&[retained, memo.len(), tasks.len(), count])?;
            builder.budget.charge(count)?;
            tasks.push((index, true));
            if let Some(right) = children[1] {
                tasks.push((right, false));
            }
            if let Some(left) = children[0] {
                tasks.push((left, false));
            }
            continue;
        }
        // Demand scheduling differs from the complete-column Jacobian loop.
        // Reuse its construction primitives and Expr's local derivative rules;
        // combine the nonzero operand contributions in original input order.
        let value = match &node {
            ExpressionNode::Constant(_) => zero,
            ExpressionNode::Variable(reference) => {
                if reference.id == variable {
                    one
                } else {
                    zero
                }
            }
            ExpressionNode::Unary { .. } | ExpressionNode::Binary { .. } => {
                let operands = node.inputs();
                let first = operands[0].expect("nonleaf has an operand");
                let inputs = [first, operands[1].unwrap_or(first)];
                let arity = if operands[1].is_some() { 2 } else { 1 };
                let mut terms = [zero; 2];
                for column in 0..arity {
                    let derivative = memo[&(inputs[column], variable)];
                    if derivative == zero {
                        continue;
                    }
                    let template = match &node {
                        ExpressionNode::Unary { operation, .. } => super::jacobian::unary(
                            *operation,
                            crate::Expr::anonymous_symbol(SymbolId(0)),
                        ),
                        ExpressionNode::Binary { operation, .. } => super::jacobian::binary(
                            *operation,
                            crate::Expr::anonymous_symbol(SymbolId(0)),
                            crate::Expr::anonymous_symbol(SymbolId(1)),
                        ),
                        _ => unreachable!("nonleaf already selected"),
                    };
                    let partial =
                        builder.partial(template, column as u32, &inputs[..arity], zero, one)?;
                    terms[column] = builder.chain(partial, derivative, zero, one)?;
                }
                match terms {
                    [a, b] if a == zero => b,
                    [a, b] if b == zero => a,
                    [left, right] => builder.push(ExpressionNode::Binary {
                        operation: super::BinaryOperation::Add,
                        left,
                        right,
                    })?,
                }
            }
        };
        builder
            .budget
            .check_entries(&[retained, memo.len(), tasks.len(), 1])?;
        builder.budget.charge(1)?;
        memo.insert((index, variable), value);
    }
    Ok(memo[&(root, variable)])
}

impl From<hyperinterval::IntervalError> for TaylorError {
    fn from(error: hyperinterval::IntervalError) -> Self {
        Self::Interval(error)
    }
}

#[cfg(feature = "taylor-affine")]
pub use model::{
    TaylorAffineError, TaylorAffineMap, TaylorAffineModel, TaylorAffineRange, TaylorAffineRow,
    TaylorAffineTerm,
};
