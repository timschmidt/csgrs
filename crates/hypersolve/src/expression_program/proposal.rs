//! Prepared primitive estimates; no operation here certifies the exact source.
use super::*;

/// Failure of a primitive proposal, not a domain or nonexistence proof for its source.
#[derive(Clone, Debug)]
pub enum FloatProposalError {
    /// Graph validation, resource or cancellation failure.
    Expression(ExpressionError),
    /// A supplied primitive argument is NaN or infinite.
    NonFiniteArgument {
        /// Original ordered argument slot.
        slot: usize,
    },
    /// An exact constant has no finite lossy primitive view.
    ConstantUnavailable {
        /// Original source node.
        node: usize,
    },
    /// Rounded operands fail a native operation's domain guard.
    NativeDomain {
        /// Original source node; this does not classify its exact domain.
        node: usize,
    },
    /// A native operation produced NaN or infinity.
    NonFiniteResult {
        /// Original source node.
        node: usize,
    },
}
impl fmt::Display for FloatProposalError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "primitive proposal: {self:?}")
    }
}
impl std::error::Error for FloatProposalError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Expression(error) => Some(error),
            _ => None,
        }
    }
}
impl From<ExpressionError> for FloatProposalError {
    fn from(error: ExpressionError) -> Self {
        Self::Expression(error)
    }
}

#[derive(Debug)]
struct ProposalData {
    source: ExpressionProgram,
    order: Vec<usize>,
    constants: Vec<Option<f64>>,
}

/// An exact owned source and a prepared, explicitly lossy primitive view.
///
/// Native rounding, underflow and platform math-library differences are allowed.
/// Values are proposals only: they cannot establish containment, exclusion, signs,
/// regularity or existence. Recompute any consumed proof from `source()` using an
/// authoritative evaluator. Preparation projects demanded constants once and
/// retains the original graph; clones share this association. Replay the original
/// definition and prepare again, never deserialize a projected graph as evidence.
#[derive(Clone, Debug)]
pub struct FloatProposalProgram {
    data: Arc<ProposalData>,
}

impl ExpressionProgram {
    /// Prepare primitive estimates of the demanded source DAG, in left-first order.
    ///
    /// Domain guards and sharing remain in the schedule; unused nodes are not
    /// evaluated. Constant projection uses Hyperreal's explicit lossy edge and can
    /// refine or populate primitive caches outside the graph work/heap contract.
    /// Cancellation is checked between stages, not inside scalar projection.
    pub fn float_proposal(
        &self,
        budget: &mut ExpressionBudget,
    ) -> Result<FloatProposalProgram, FloatProposalError> {
        budget.charge(0)?;
        let source = self.definition();
        let nodes = source.nodes.len();
        budget.check_entries(&[
            source.variables.len(),
            source.outputs.len(),
            nodes.checked_mul(5).ok_or(ExpressionError::SizeExhausted)?,
        ])?;
        budget.charge(nodes.checked_mul(2).ok_or(ExpressionError::SizeExhausted)?)?;
        let mut seen = vec![false; nodes];
        let mut constants = vec![None; nodes];
        let mut order = Vec::new();
        for &root in &source.outputs {
            budget.charge(1)?;
            let mut tasks = vec![(root, false)];
            while let Some((node, ready)) = tasks.pop() {
                budget.charge(1)?;
                if seen[node] {
                    continue;
                }
                if !ready {
                    tasks.push((node, true));
                    let [left, right] = source.nodes[node].inputs();
                    if let Some(right) = right {
                        tasks.push((right, false));
                    }
                    if let Some(left) = left {
                        tasks.push((left, false));
                    }
                    continue;
                }
                budget.charge(1)?;
                if let ExpressionNode::Constant(value) = &source.nodes[node] {
                    constants[node] = Some(
                        value
                            .to_f64_lossy()
                            .filter(|value| value.is_finite())
                            .ok_or(FloatProposalError::ConstantUnavailable { node })?,
                    );
                }
                order.push(node);
                seen[node] = true;
            }
        }
        budget.charge(1)?;
        let result = FloatProposalProgram {
            data: Arc::new(ProposalData {
                source: self.clone(),
                order,
                constants,
            }),
        };
        budget.charge(0)?;
        Ok(result)
    }
}

impl FloatProposalProgram {
    /// Original exact source, including all bindings and unused definitions.
    pub fn source(&self) -> &ExpressionProgram {
        &self.data.source
    }

    /// Estimate outputs from finite native arguments; no enclosure is established.
    ///
    /// Each demanded node executes once, preserving left-first source guards.
    /// All native intermediates must be finite; underflow to zero is permitted.
    /// Full-width integer powers use bounded binary exponentiation, with graph
    /// admission per iteration. This is not a CPU-instruction or total-heap bound.
    pub fn estimate(
        &self,
        arguments: &[f64],
        budget: &mut ExpressionBudget,
    ) -> Result<Vec<f64>, FloatProposalError> {
        budget.charge(0)?;
        let source = self.source().definition();
        if arguments.len() != source.variables.len() {
            return Err(ExpressionError::BindingCount {
                expected: source.variables.len(),
                actual: arguments.len(),
            }
            .into());
        }
        budget.check_entries(&[
            arguments.len(),
            source.nodes.len(),
            self.data.order.len(),
            source.outputs.len(),
        ])?;
        for (slot, value) in arguments.iter().enumerate() {
            budget.charge(1)?;
            if !value.is_finite() {
                return Err(FloatProposalError::NonFiniteArgument { slot });
            }
        }
        budget.charge(source.nodes.len())?;
        let mut values = vec![0.0; source.nodes.len()];
        for &node in &self.data.order {
            budget.charge(1)?;
            let value = match source.nodes[node] {
                ExpressionNode::Constant(_) => {
                    self.data.constants[node].expect("prepared constant")
                }
                ExpressionNode::Variable(_) => {
                    arguments[self.source().slots[node].expect("validated binding")]
                }
                ExpressionNode::Unary { operation, input } => {
                    estimate_unary(operation, values[input], node, budget)?
                }
                ExpressionNode::Binary {
                    operation,
                    left,
                    right,
                } => match operation {
                    BinaryOperation::Add => values[left] + values[right],
                    BinaryOperation::Sub => values[left] - values[right],
                    BinaryOperation::Mul => values[left] * values[right],
                    BinaryOperation::Div => {
                        if values[right] == 0.0 {
                            return Err(FloatProposalError::NativeDomain { node });
                        }
                        values[left] / values[right]
                    }
                },
            };
            values[node] = finite(value, node)?;
        }
        budget.charge(source.outputs.len())?;
        let result = source.outputs.iter().map(|&node| values[node]).collect();
        budget.charge(0)?;
        Ok(result)
    }
}

fn finite(value: f64, node: usize) -> Result<f64, FloatProposalError> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(FloatProposalError::NonFiniteResult { node })
    }
}

fn estimate_unary(
    operation: UnaryOperation,
    value: f64,
    node: usize,
    budget: &mut ExpressionBudget,
) -> Result<f64, FloatProposalError> {
    let domain = match operation {
        UnaryOperation::PowI(exponent) => exponent > 0 || value != 0.0,
        UnaryOperation::Sqrt => value >= 0.0,
        UnaryOperation::Ln | UnaryOperation::Log10 => value > 0.0,
        UnaryOperation::Asin | UnaryOperation::Acos => value.abs() <= 1.0,
        UnaryOperation::Acosh => value >= 1.0,
        UnaryOperation::Atanh => value.abs() < 1.0,
        _ => true,
    };
    if !domain {
        return Err(FloatProposalError::NativeDomain { node });
    }
    Ok(match operation {
        UnaryOperation::Neg => -value,
        UnaryOperation::PowI(exponent) => {
            let mut n = exponent.unsigned_abs();
            let mut power = if exponent < 0 {
                finite(1.0 / value, node)?
            } else {
                value
            };
            let mut result = 1.0;
            while n != 0 {
                budget.charge(2)?;
                if n & 1 != 0 {
                    result = finite(result * power, node)?;
                }
                n >>= 1;
                if n != 0 {
                    power = finite(power * power, node)?;
                }
            }
            result
        }
        UnaryOperation::Sqrt => value.sqrt(),
        UnaryOperation::Sin => value.sin(),
        UnaryOperation::Cos => value.cos(),
        UnaryOperation::Ln => value.ln(),
        UnaryOperation::Log10 => value.log10(),
        UnaryOperation::Asin => value.asin(),
        UnaryOperation::Acos => value.acos(),
        UnaryOperation::Acosh => value.acosh(),
        UnaryOperation::Atanh => value.atanh(),
    })
}
