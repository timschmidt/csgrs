//! Owned, flat symbolic definitions with explicit ordered bindings.
//!
//! This experimental prerequisite for enclosed vector fields preserves every
//! [`Expr`] operation and its domain guards. The `interval-program` feature adds
//! certified dyadic images with complete-domain checks. Source-bound Jacobian
//! construction has graph limits; trajectory certification remains separate.
//! Graph limits are distinct from scalar arithmetic limits.
//!
//! Definitions are data, not evidence. After deserialization, call
//! [`ExpressionProgram::from_definition`] to validate references and bindings.
//! Scalar deserialization uses Hyperreal's validation; it is outside the graph
//! budget, as are scalar cloning, arithmetic, equality and destruction.

use std::{collections::BTreeMap, fmt, sync::Arc};

use hyperreal::{Problem, Real};

use crate::{Expr, SymbolId, SymbolRef, work_budget::CancellationToken};

#[cfg(feature = "interval-program")]
mod dyadic;
#[cfg(feature = "interval-lattice")]
pub use dyadic::JacobianMatrix;
#[cfg(feature = "interval-program")]
pub use dyadic::RegularJacobianEnclosure;
mod affine;
pub use affine::{AffineParameterization, AffineProgram};
mod jacobian;
pub use jacobian::JacobianProgram;

/// An operation on two expressions, preserving left-to-right evaluation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum BinaryOperation {
    /// Addition.
    Add,
    /// Subtraction.
    Sub,
    /// Multiplication.
    Mul,
    /// Division, including its nonzero-denominator obligation.
    Div,
}

/// An operation on one expression, including the original domain obligation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum UnaryOperation {
    /// Negation.
    Neg,
    /// Integer power; even exponent zero retains evaluation of its operand.
    PowI(i64),
    /// Principal square root.
    Sqrt,
    /// Sine.
    Sin,
    /// Cosine.
    Cos,
    /// Natural logarithm.
    Ln,
    /// Base-ten logarithm.
    Log10,
    /// Principal inverse sine.
    Asin,
    /// Principal inverse cosine.
    Acos,
    /// Principal inverse hyperbolic cosine.
    Acosh,
    /// Inverse hyperbolic tangent.
    Atanh,
}

/// A flat expression node. Child indices must be strictly less than its index.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ExpressionNode {
    /// An authoritative exact scalar definition.
    Constant(Real),
    /// A variable whose identifier occurs in the ordered binding table.
    Variable(SymbolRef),
    /// A unary operation on an earlier node.
    Unary {
        /// Operation to evaluate.
        operation: UnaryOperation,
        /// Earlier operand index.
        input: usize,
    },
    /// A binary operation on earlier nodes.
    Binary {
        /// Operation to evaluate.
        operation: BinaryOperation,
        /// Earlier left operand index.
        left: usize,
        /// Earlier right operand index.
        right: usize,
    },
}

/// Unvalidated replay data, including the full source and binding order.
///
/// Equality is structural, using Hyperreal's structural scalar equality. It
/// includes node order, sharing, names and output order; it neither proves
/// equality of functions nor assigns an allocation-based identity. The serde
/// form is experimental and not yet a versioned interchange format.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ExpressionDefinition {
    /// Distinct variable identifiers, in evaluation argument order.
    pub variables: Vec<SymbolId>,
    /// Flat source graph, in dependency order. Unused nodes are permitted.
    pub nodes: Vec<ExpressionNode>,
    /// Output node indices, in result order. Repeated outputs are permitted.
    pub outputs: Vec<usize>,
}

/// Structural limits shared across compilation, validation and evaluation.
///
/// One work unit admits one visited input, emitted node, validated record,
/// traversal task, or initialized cache entry. `max_entries` bounds the sum of
/// definition variables, nodes and outputs; temporary graph storage is linear
/// in that sum. Neither limit bounds heap bytes, scalar arithmetic, coefficient
/// growth, scalar cloning/destruction, serialization, or elapsed time. Map
/// lookups have logarithmic cost; a unit is not a CPU instruction.
///
/// Cancellation is checked before each charge and before publication.
/// It cannot interrupt scalar operations. Failed charges spend no work; work
/// already done is not refunded. There is no global cache to poison on retry.
#[derive(Clone, Debug)]
pub struct ExpressionBudget {
    max_entries: usize,
    remaining_work: usize,
    /// Optional shared cooperative cancellation request.
    pub cancellation: Option<CancellationToken>,
}

impl ExpressionBudget {
    /// Configure graph entry and work limits; cancellation is initially absent.
    pub const fn new(max_entries: usize, work: usize) -> Self {
        Self {
            max_entries,
            remaining_work: work,
            cancellation: None,
        }
    }

    /// Work units not yet consumed.
    pub const fn remaining_work(&self) -> usize {
        self.remaining_work
    }

    /// Admit structural work, including a consumer adapter's own graph work.
    /// Checks cancellation before spending; a failed charge spends no units.
    /// This does not account for scalar arithmetic or heap bytes.
    pub fn charge(&mut self, amount: usize) -> Result<(), ExpressionError> {
        if self
            .cancellation
            .as_ref()
            .is_some_and(CancellationToken::is_cancelled)
        {
            return Err(ExpressionError::Cancelled);
        }
        if amount > self.remaining_work {
            return Err(ExpressionError::WorkExhausted);
        }
        self.remaining_work -= amount;
        Ok(())
    }

    /// Check a consumer's aggregate structural entry count without allocating.
    /// Counts use the same configured limit and checked sum as program builds.
    /// This is a stage size check, not a retained-memory reservation.
    pub fn check_entries(&self, counts: &[usize]) -> Result<(), ExpressionError> {
        self.entries(counts)
    }

    fn entries(&self, counts: &[usize]) -> Result<(), ExpressionError> {
        let total = counts
            .iter()
            .try_fold(0_usize, |sum, count| sum.checked_add(*count));
        if total.is_none_or(|total| total > self.max_entries) {
            return Err(ExpressionError::SizeExhausted);
        }
        Ok(())
    }
}

/// Validation, graph resource, or scalar evaluation failure.
#[derive(Clone, Debug)]
pub enum ExpressionError {
    /// Two binding slots name the same variable.
    DuplicateVariable(SymbolId),
    /// A source variable has no binding slot.
    UnboundVariable(SymbolId),
    /// An operand is not an earlier node (including cycles and forward edges).
    InvalidReference { node: usize, input: usize },
    /// An output index is outside the node table.
    InvalidOutput { output: usize, node: usize },
    /// Evaluation arguments do not match the explicit binding table.
    BindingCount { expected: usize, actual: usize },
    /// Requested matrix rows differ from the source output count.
    #[cfg(feature = "interval-lattice")]
    JacobianRows { expected: usize, actual: usize },
    /// Requested matrix columns differ from the selected variable count.
    #[cfg(feature = "interval-lattice")]
    JacobianColumns { expected: usize, actual: usize },
    /// Native matrix construction failure, preserving the lattice diagnostic.
    #[cfg(feature = "interval-lattice")]
    Lattice(hyperlattice::IntervalLatticeError),
    /// The configured definition-entry limit was exceeded.
    SizeExhausted,
    /// The next graph work charge could not be admitted.
    WorkExhausted,
    /// Cooperative cancellation was observed.
    Cancelled,
    /// Scalar failure, retaining its source node and original diagnostic.
    Scalar { node: usize, source: Problem },
    /// The operand enclosure does not certify the operation's whole domain.
    /// Dependency overestimation can cause this even for valid exact inputs.
    #[cfg(feature = "interval-program")]
    DomainNotCertified { node: usize },
    /// A state-dependent primitive enclosure touches a differentiability boundary.
    /// This is a failed sufficient proof, not a claim of nondifferentiability.
    #[cfg(feature = "interval-program")]
    RegularityNotCertified { node: usize },
    /// Native failure while retaining an argument for a domain certificate.
    #[cfg(feature = "interval-program")]
    ArgumentEnclosure {
        /// Original binding slot.
        slot: usize,
        /// Native resource/domain cause.
        source: hyperinterval::IntervalError,
    },
    /// Interval failure, preserving the node and native resource/domain cause.
    #[cfg(feature = "interval-program")]
    Enclosure {
        node: usize,
        source: hyperinterval::IntervalError,
    },
}

impl fmt::Display for ExpressionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "expression program: {self:?}")
    }
}

impl std::error::Error for ExpressionError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            #[cfg(feature = "interval-lattice")]
            Self::Lattice(source) => Some(source),
            Self::Scalar { source, .. } => Some(source),
            #[cfg(feature = "interval-program")]
            Self::Enclosure { source, .. } => Some(source),
            #[cfg(feature = "interval-program")]
            Self::ArgumentEnclosure { source, .. } => Some(source),
            _ => None,
        }
    }
}

/// Immutable validated definition, cheaply shared without cloning its scalars.
///
/// The definition itself is the replayable source identity. No externally
/// mutable evaluation context or serialized validity flag is trusted.
#[derive(Clone, Debug)]
pub struct ExpressionProgram {
    definition: Arc<ExpressionDefinition>,
    slots: Arc<[Option<usize>]>,
}

impl ExpressionProgram {
    /// Validate a flat definition before owning it as an executable program.
    /// Unused nodes are validated too, but are not evaluated.
    pub fn from_definition(
        definition: ExpressionDefinition,
        budget: &mut ExpressionBudget,
    ) -> Result<Self, ExpressionError> {
        budget.charge(0)?;
        budget.entries(&[
            definition.variables.len(),
            definition.nodes.len(),
            definition.outputs.len(),
        ])?;
        let mut variables = BTreeMap::new();
        for (slot, id) in definition.variables.iter().enumerate() {
            budget.charge(1)?;
            if variables.insert(*id, slot).is_some() {
                return Err(ExpressionError::DuplicateVariable(*id));
            }
        }
        let mut slots = Vec::new();
        for (index, node) in definition.nodes.iter().enumerate() {
            budget.charge(1)?;
            let slot = match node {
                ExpressionNode::Variable(reference) => Some(
                    *variables
                        .get(&reference.id)
                        .ok_or(ExpressionError::UnboundVariable(reference.id))?,
                ),
                _ => None,
            };
            for input in node.inputs().into_iter().flatten() {
                if input >= index {
                    return Err(ExpressionError::InvalidReference { node: index, input });
                }
            }
            slots.push(slot);
        }
        for (output, &node) in definition.outputs.iter().enumerate() {
            budget.charge(1)?;
            if node >= definition.nodes.len() {
                return Err(ExpressionError::InvalidOutput { output, node });
            }
        }
        budget.charge(0)?;
        Ok(Self {
            definition: Arc::new(definition),
            slots: slots.into(),
        })
    }

    /// Import borrowed trees without recursive traversal or simplification.
    ///
    /// Every tree occurrence is retained; this import does not infer common
    /// subexpressions. Constants and symbol names are cloned once. Their byte
    /// sizes and cloning costs are outside the structural graph budget.
    pub fn compile(
        sources: &[Expr],
        variables: &[SymbolId],
        budget: &mut ExpressionBudget,
    ) -> Result<Self, ExpressionError> {
        budget.charge(0)?;
        budget.entries(&[variables.len(), sources.len()])?;
        budget.charge(variables.len())?;
        let mut definition = ExpressionDefinition {
            variables: variables.to_vec(),
            nodes: Vec::new(),
            outputs: Vec::new(),
        };
        let mut visited = 0_usize;
        for source in sources {
            budget.charge(1)?;
            let mut tasks = vec![(source, false)];
            let mut results = Vec::new();
            while let Some((expression, ready)) = tasks.pop() {
                budget.charge(1)?;
                if !ready {
                    // Bound before growing stacks or cloning source payloads.
                    budget.entries(&[variables.len(), sources.len(), visited, 1])?;
                    visited += 1;
                    tasks.push((expression, true));
                    let (left, right) = expression_inputs(expression);
                    if let Some(right) = right {
                        tasks.push((right, false));
                    }
                    if let Some(left) = left {
                        tasks.push((left, false));
                    }
                } else {
                    let node = import_node(expression, &mut results);
                    results.push(definition.nodes.len());
                    definition.nodes.push(node);
                }
            }
            definition
                .outputs
                .push(results.pop().expect("one root per imported tree"));
        }
        Self::from_definition(definition, budget)
    }

    /// Full immutable source identity; cheap program clones share this data.
    pub fn definition(&self) -> &ExpressionDefinition {
        &self.definition
    }

    /// Evaluate selected outputs in order, with left operands before right.
    ///
    /// Each reached DAG node is evaluated at most once per call. The program
    /// keeps no additional evaluation cache between calls; Hyperreal constants
    /// retain their own scalar caches. Unreached nodes cannot cause domain
    /// errors. Argument values correspond exactly to `variables`.
    /// Successful values are exact Hyperreal definitions, not interval proofs.
    pub fn eval_real(
        &self,
        values: &[Real],
        budget: &mut ExpressionBudget,
    ) -> Result<Vec<Real>, ExpressionError> {
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
        let mut cache: Vec<Option<Real>> = vec![None; source.nodes.len()];
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
                    ExpressionNode::Constant(value) => value.clone(),
                    ExpressionNode::Variable(_) => {
                        values[self.slots[index].expect("validated binding")].clone()
                    }
                    ExpressionNode::Unary { operation, input } => operation
                        .eval(consume_operand(&mut cache, &mut uses, *input))
                        .map_err(|source| ExpressionError::Scalar {
                            node: index,
                            source,
                        })?,
                    ExpressionNode::Binary {
                        operation,
                        left,
                        right,
                    } => operation
                        .eval(
                            consume_operand(&mut cache, &mut uses, *left),
                            consume_operand(&mut cache, &mut uses, *right),
                        )
                        .map_err(|source| ExpressionError::Scalar {
                            node: index,
                            source,
                        })?,
                };
                cache[index] = Some(value);
            }
            outputs.push(consume_operand(&mut cache, &mut uses, root));
        }
        budget.charge(0)?;
        Ok(outputs)
    }

    // Count only demanded graph edges. Moving an operand at its final use
    // avoids retaining an unnecessary scalar clone across an operation, which
    // can change Hyperreal's unique-ownership dispatch even on a plain tree.
    fn operand_uses(&self, budget: &mut ExpressionBudget) -> Result<Vec<usize>, ExpressionError> {
        let source = &self.definition;
        budget.charge(source.nodes.len())?;
        let mut uses = vec![0_usize; source.nodes.len()];
        budget.charge(source.nodes.len())?;
        let mut expanded = vec![false; source.nodes.len()];
        for &root in &source.outputs {
            budget.charge(1)?;
            uses[root] = uses[root]
                .checked_add(1)
                .ok_or(ExpressionError::SizeExhausted)?;
            let mut tasks = vec![root];
            while let Some(index) = tasks.pop() {
                budget.charge(1)?;
                if expanded[index] {
                    continue;
                }
                expanded[index] = true;
                for input in source.nodes[index].inputs().into_iter().flatten() {
                    uses[input] = uses[input]
                        .checked_add(1)
                        .ok_or(ExpressionError::SizeExhausted)?;
                    tasks.push(input);
                }
            }
        }
        Ok(uses)
    }
}

fn consume_operand(cache: &mut [Option<Real>], uses: &mut [usize], index: usize) -> Real {
    uses[index] -= 1;
    if uses[index] == 0 {
        cache[index].take().expect("last use of evaluated operand")
    } else {
        cache[index]
            .as_ref()
            .expect("shared evaluated operand")
            .clone()
    }
}

impl ExpressionNode {
    fn inputs(&self) -> [Option<usize>; 2] {
        match self {
            Self::Constant(_) | Self::Variable(_) => [None, None],
            Self::Unary { input, .. } => [Some(*input), None],
            Self::Binary { left, right, .. } => [Some(*left), Some(*right)],
        }
    }
}

fn expression_inputs(expression: &Expr) -> (Option<&Expr>, Option<&Expr>) {
    match expression {
        Expr::Constant(_) | Expr::Symbol(_) => (None, None),
        Expr::Add(a, b) | Expr::Sub(a, b) | Expr::Mul(a, b) | Expr::Div(a, b) => (Some(a), Some(b)),
        Expr::Neg(a)
        | Expr::PowI(a, _)
        | Expr::Sqrt(a)
        | Expr::Sin(a)
        | Expr::Cos(a)
        | Expr::Ln(a)
        | Expr::Log10(a)
        | Expr::Asin(a)
        | Expr::Acos(a)
        | Expr::Acosh(a)
        | Expr::Atanh(a) => (Some(a), None),
    }
}

fn import_node(expression: &Expr, results: &mut Vec<usize>) -> ExpressionNode {
    match expression {
        Expr::Constant(value) => return ExpressionNode::Constant(value.clone()),
        Expr::Symbol(reference) => return ExpressionNode::Variable(reference.clone()),
        _ => {}
    }
    let last = results.pop().expect("postorder operand");
    let binary = match expression {
        Expr::Add(..) => Some(BinaryOperation::Add),
        Expr::Sub(..) => Some(BinaryOperation::Sub),
        Expr::Mul(..) => Some(BinaryOperation::Mul),
        Expr::Div(..) => Some(BinaryOperation::Div),
        _ => None,
    };
    if let Some(operation) = binary {
        return ExpressionNode::Binary {
            operation,
            left: results.pop().expect("left postorder operand"),
            right: last,
        };
    }
    let operation = match expression {
        Expr::Neg(_) => UnaryOperation::Neg,
        Expr::PowI(_, exponent) => UnaryOperation::PowI(*exponent),
        Expr::Sqrt(_) => UnaryOperation::Sqrt,
        Expr::Sin(_) => UnaryOperation::Sin,
        Expr::Cos(_) => UnaryOperation::Cos,
        Expr::Ln(_) => UnaryOperation::Ln,
        Expr::Log10(_) => UnaryOperation::Log10,
        Expr::Asin(_) => UnaryOperation::Asin,
        Expr::Acos(_) => UnaryOperation::Acos,
        Expr::Acosh(_) => UnaryOperation::Acosh,
        Expr::Atanh(_) => UnaryOperation::Atanh,
        _ => unreachable!("leaves and binary operators handled above"),
    };
    ExpressionNode::Unary {
        operation,
        input: last,
    }
}

impl BinaryOperation {
    fn eval(self, left: Real, right: Real) -> Result<Real, Problem> {
        match self {
            Self::Add => Ok(left + right),
            Self::Sub => Ok(left - right),
            Self::Mul => Ok(left * right),
            Self::Div => left / right,
        }
    }
}

impl UnaryOperation {
    fn eval(self, value: Real) -> Result<Real, Problem> {
        match self {
            Self::Neg => Ok(-value),
            Self::PowI(exponent) => value.powi(exponent.into()),
            Self::Sqrt => value.sqrt(),
            Self::Sin => Ok(value.sin()),
            Self::Cos => Ok(value.cos()),
            Self::Ln => value.ln(),
            Self::Log10 => value.log10(),
            Self::Asin => value.asin(),
            Self::Acos => value.acos(),
            Self::Acosh => value.acosh(),
            Self::Atanh => value.atanh(),
        }
    }
}

#[cfg(feature = "float-proposal-program")]
mod proposal;
#[cfg(feature = "float-proposal-program")]
pub use proposal::{FloatProposalError, FloatProposalProgram};

#[cfg(feature = "taylor-model")]
mod taylor;
#[cfg(feature = "taylor-model")]
pub use taylor::{
    CoefficientProgram, CoefficientRequest, TaylorError, TaylorModel, TaylorRemainderTerm,
    TaylorRequest, TaylorTerm,
};

#[cfg(feature = "taylor-affine")]
pub use taylor::{
    TaylorAffineError, TaylorAffineMap, TaylorAffineModel, TaylorAffineRange, TaylorAffineRow,
    TaylorAffineTerm,
};

#[cfg(feature = "coordinate-program")]
mod coordinates;
#[cfg(feature = "coordinate-program")]
pub use coordinates::{CoordinateChange, CoordinateError, CoordinateProgram};

mod composition;
pub use composition::{CompositionError, CompositionProgram};

#[cfg(feature = "taylor-polynomial")]
pub use taylor::{
    TaylorPolynomialError, TaylorPolynomialModel, TaylorPolynomialProduct, TaylorPolynomialTerms,
    TaylorRange,
};

#[cfg(feature = "coordinate-transition")]
pub use coordinates::{CoordinateTransition, CoordinateTransitionError};
