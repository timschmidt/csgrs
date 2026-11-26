use hyperinterval::{DomainCoverage, DyadicBudget, DyadicInterval, IntervalError, RangeEnclosure};
use hyperreal::Real;

use super::{
    BinaryOperation, ExpressionBudget, ExpressionError, ExpressionNode, ExpressionProgram,
    UnaryOperation,
};

mod regularity;
pub use regularity::RegularJacobianEnclosure;
#[cfg(feature = "interval-lattice")]
mod matrix;
#[cfg(feature = "interval-lattice")]
pub use matrix::JacobianMatrix;

impl ExpressionProgram {
    /// Enclose every output over the Cartesian product of the input intervals.
    ///
    /// Success certifies that each reached operation covers its whole operand
    /// enclosure. Partial-domain images are refused, including when dependency
    /// overestimation prevents a proof for otherwise valid exact inputs. Original
    /// domain guards remain active under zero factors or zero powers. An empty
    /// input product maps to canonical empty outputs after binding/graph checks;
    /// zero input dimensions instead form the nonempty unit product.
    ///
    /// The exact owned definition stays authoritative. Graph traversal uses
    /// `graph_budget`; interval arithmetic uses the separate `arithmetic_budget`.
    /// `precision` is spacing for rounded native operations and constant
    /// enclosures, not a final-width guarantee. Exact sums/products keep their
    /// native grids. Structurally rational constants use native budgeted
    /// conversion before coefficient copies or normalization. Other Hyperreal
    /// constant enclosure refinement is outside both
    /// budgets' total-work accounting; this API is not a wholly bounded field
    /// evaluator. Graph cancellation cannot interrupt that scalar refinement or
    /// a native interval operation. No interval result is retained between calls.
    pub fn eval_dyadic(
        &self,
        values: &[DyadicInterval],
        precision: i32,
        graph_budget: &mut ExpressionBudget,
        arithmetic_budget: &mut DyadicBudget,
    ) -> Result<Vec<DyadicInterval>, ExpressionError> {
        graph_budget.charge(0)?;
        let source = &self.definition;
        if values.len() != source.variables.len() {
            return Err(ExpressionError::BindingCount {
                expected: source.variables.len(),
                actual: values.len(),
            });
        }
        graph_budget.entries(&[
            source.variables.len(),
            source.nodes.len(),
            source.outputs.len(),
        ])?;
        graph_budget.charge(values.len())?;
        if values.iter().any(DyadicInterval::is_empty) {
            graph_budget.charge(source.outputs.len())?;
            let outputs = vec![DyadicInterval::empty(); source.outputs.len()];
            graph_budget.charge(0)?;
            return Ok(outputs);
        }
        self.evaluate_dyadic(
            values,
            graph_budget,
            &mut DyadicArithmetic {
                precision,
                budget: arithmetic_budget,
            },
            None,
        )
    }

    // Keep interval effects specialized: copying and each operation can fail
    // under the arithmetic budget. The shared generic traversal candidate grew
    // the accepted point evaluator's artifacts and failed its retention gate.
    // Graph references and use counts still share the source's validation.
    fn evaluate_dyadic(
        &self,
        values: &[DyadicInterval],
        budget: &mut ExpressionBudget,
        arithmetic: &mut DyadicArithmetic<'_>,
        regularity: Option<&[bool]>,
    ) -> Result<Vec<DyadicInterval>, ExpressionError> {
        budget.charge(0)?;
        let source = &self.definition;
        if values.len() != source.variables.len() {
            return Err(ExpressionError::BindingCount {
                expected: source.variables.len(),
                actual: values.len(),
            });
        }
        budget.entries(&[
            source.variables.len(),
            source.nodes.len(),
            source.outputs.len(),
        ])?;
        let mut uses = self.operand_uses(budget)?;
        budget.charge(source.nodes.len())?;
        let mut cache: Vec<Option<DyadicInterval>> = vec![None; source.nodes.len()];
        let mut outputs = Vec::new();
        for &root in &source.outputs {
            budget.charge(1)?;
            let mut tasks = vec![(root, false)];
            while let Some((index, ready)) = tasks.pop() {
                budget.charge(1)?;
                if cache[index].is_some() {
                    continue;
                }
                let node = &source.nodes[index];
                if !ready {
                    tasks.push((index, true));
                    let [left, right] = node.inputs();
                    if let Some(right) = right {
                        tasks.push((right, false));
                    }
                    if let Some(left) = left {
                        tasks.push((left, false));
                    }
                    continue;
                }
                let value = match node {
                    ExpressionNode::Constant(value) => arithmetic
                        .constant(value)
                        .map_err(|error| DyadicArithmetic::copy_error(index, error))?,
                    ExpressionNode::Variable(_) => arithmetic
                        .copy(&values[self.slots[index].expect("validated binding")])
                        .map_err(|error| DyadicArithmetic::copy_error(index, error))?,
                    ExpressionNode::Unary { operation, input } => {
                        let value =
                            consume_dyadic_operand(&mut cache, &mut uses, *input, arithmetic)
                                .map_err(|error| DyadicArithmetic::copy_error(*input, error))?;
                        let regular = if regularity
                            .and_then(|active| active.get(index))
                            .copied()
                            .unwrap_or(false)
                        {
                            arithmetic
                                .regular_input(*operation, &value)
                                .map_err(|error| DyadicArithmetic::copy_error(index, error))?
                        } else {
                            true
                        };
                        let result = arithmetic
                            .unary(*operation, value)
                            .map_err(|error| DyadicArithmetic::operation_error(index, error))?;
                        // Preserve original domain failures before reporting a
                        // stronger differentiability proof as unavailable.
                        if !regular {
                            return Err(ExpressionError::RegularityNotCertified { node: index });
                        }
                        result
                    }
                    ExpressionNode::Binary {
                        operation,
                        left,
                        right,
                    } => {
                        let left = consume_dyadic_operand(&mut cache, &mut uses, *left, arithmetic)
                            .map_err(|error| DyadicArithmetic::copy_error(*left, error))?;
                        let right =
                            consume_dyadic_operand(&mut cache, &mut uses, *right, arithmetic)
                                .map_err(|error| DyadicArithmetic::copy_error(*right, error))?;
                        arithmetic
                            .binary(*operation, left, right)
                            .map_err(|error| DyadicArithmetic::operation_error(index, error))?
                    }
                };
                cache[index] = Some(value);
            }
            outputs.push(
                consume_dyadic_operand(&mut cache, &mut uses, root, arithmetic)
                    .map_err(|error| DyadicArithmetic::copy_error(root, error))?,
            );
        }
        budget.charge(0)?;
        Ok(outputs)
    }
}

struct DyadicArithmetic<'a> {
    precision: i32,
    budget: &'a mut DyadicBudget,
}

enum DyadicFailure {
    Native(IntervalError),
    DomainNotCertified,
}

impl From<IntervalError> for DyadicFailure {
    fn from(error: IntervalError) -> Self {
        Self::Native(error)
    }
}

/// Grid-aligned range of `value^power` for a positive integer power, or `None`
/// when the general rational-power path must be used.
///
/// Such a power is defined on every real, and its exact image of a closed
/// interval is a closed interval, so the tight outward grid image of the
/// general rational power equals the exact interval power enclosed outward on
/// the same grid. This skips the branch, pole and extended-interval machinery.
/// When the exact endpoint powers could exceed the per-stage bit limit, the
/// general path, which first removes each endpoint's own trailing zeros, keeps
/// its original size behavior.
pub(crate) fn positive_power(
    value: &DyadicInterval,
    power: u32,
    precision: i32,
    budget: &mut DyadicBudget,
) -> Result<Option<DyadicInterval>, IntervalError> {
    debug_assert!(power >= 1);
    let Some((lower, upper, _)) = value.bounds() else {
        return Ok(None);
    };
    let bits = lower.bits().max(upper.bits()).checked_mul(u64::from(power));
    if bits.is_none_or(|bits| bits > budget.max_bits()) {
        return Ok(None);
    }
    Ok(Some(
        value
            .checked_pow(power, budget)?
            .regrid(precision, budget)?,
    ))
}
/// Image of one unary primitive over a whole interval, with the same domain
/// and guard contract as [`ExpressionProgram::eval_dyadic`]; `None` when the
/// operand is not certified to lie inside the primitive's domain.
#[cfg(feature = "taylor-model")]
pub(crate) fn unary_image(
    operation: UnaryOperation,
    value: DyadicInterval,
    precision: i32,
    budget: &mut DyadicBudget,
) -> Result<Option<DyadicInterval>, IntervalError> {
    match (DyadicArithmetic { precision, budget }).unary(operation, value) {
        Ok(value) => Ok(Some(value)),
        Err(DyadicFailure::DomainNotCertified) => Ok(None),
        Err(DyadicFailure::Native(error)) => Err(error),
    }
}
fn complete<T>(image: RangeEnclosure<T>) -> Result<T, DyadicFailure> {
    if image.domain == DomainCoverage::Complete {
        Ok(image.range)
    } else {
        Err(DyadicFailure::DomainNotCertified)
    }
}

impl DyadicArithmetic<'_> {
    fn copy_error(node: usize, source: IntervalError) -> ExpressionError {
        ExpressionError::Enclosure { node, source }
    }
    fn operation_error(node: usize, error: DyadicFailure) -> ExpressionError {
        match error {
            DyadicFailure::Native(source) => ExpressionError::Enclosure { node, source },
            DyadicFailure::DomainNotCertified => ExpressionError::DomainNotCertified { node },
        }
    }
    fn constant(&mut self, value: &Real) -> Result<DyadicInterval, IntervalError> {
        DyadicInterval::from_real(value, self.precision, self.budget)
    }
    fn copy(&mut self, value: &DyadicInterval) -> Result<DyadicInterval, IntervalError> {
        // Native exponent one copies on the original grid and charges its
        // mantissas. Calling Clone here would escape arithmetic accounting.
        value.checked_pow(1, self.budget)
    }
    fn unary(
        &mut self,
        operation: UnaryOperation,
        value: DyadicInterval,
    ) -> Result<DyadicInterval, DyadicFailure> {
        let p = self.precision;
        let b = &mut *self.budget;
        match operation {
            UnaryOperation::Neg => Ok(value.neg(b)?),
            UnaryOperation::PowI(exponent) => {
                // Hyperinterval's set power deliberately defines x^0 at zero;
                // the original Expr/Real grammar does not. Do not erase it.
                if exponent == 0 && value.signs().contains_zero() {
                    return Err(if value.is_singleton() {
                        IntervalError::EmptyDomain.into()
                    } else {
                        DyadicFailure::DomainNotCertified
                    });
                }
                if let Ok(power @ 1..) = u32::try_from(exponent)
                    && let Some(range) = positive_power(&value, power, p, b)?
                {
                    return Ok(range);
                }
                let image = value.pow_rational(&exponent.into(), &1.into(), p, b)?;
                Ok(complete(image)?.hull(b)?.finite_range(b)?)
            }
            UnaryOperation::Sqrt => complete(value.sqrt(p, b)?),
            UnaryOperation::Sin => complete(value.sin(p, b)?),
            UnaryOperation::Cos => complete(value.cos(p, b)?),
            UnaryOperation::Ln | UnaryOperation::Log10 => {
                let range = complete(value.log(p, b)?)?.finite_range(b)?;
                if operation == UnaryOperation::Log10 {
                    // Keep the constant denominator positive even when the
                    // requested output grid is coarser than log(10).
                    let ten = DyadicInterval::point(10.into(), 0)?;
                    let denominator = complete(ten.log(p.min(0), b)?)?.finite_range(b)?;
                    Ok(range.div(&denominator, p, b)?)
                } else {
                    Ok(range)
                }
            }
            UnaryOperation::Asin => complete(value.asin(p, b)?),
            UnaryOperation::Acos => complete(value.acos(p, b)?),
            UnaryOperation::Acosh => complete(value.acosh(p, b)?),
            UnaryOperation::Atanh => Ok(complete(value.atanh(p, b)?)?.finite_range(b)?),
        }
    }
    fn binary(
        &mut self,
        operation: BinaryOperation,
        left: DyadicInterval,
        right: DyadicInterval,
    ) -> Result<DyadicInterval, DyadicFailure> {
        Ok(match operation {
            BinaryOperation::Add => left.add(&right, self.budget)?,
            BinaryOperation::Sub => left.sub(&right, self.budget)?,
            BinaryOperation::Mul => left.mul(&right, self.budget)?,
            BinaryOperation::Div => left.div(&right, self.precision, self.budget)?,
        })
    }
}

fn consume_dyadic_operand(
    cache: &mut [Option<DyadicInterval>],
    uses: &mut [usize],
    index: usize,
    arithmetic: &mut DyadicArithmetic<'_>,
) -> Result<DyadicInterval, IntervalError> {
    uses[index] -= 1;
    if uses[index] == 0 {
        Ok(cache[index].take().expect("last use of evaluated operand"))
    } else {
        arithmetic.copy(cache[index].as_ref().expect("shared evaluated operand"))
    }
}
