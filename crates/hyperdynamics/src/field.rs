//! Owned dyadic fields. Scalar refinement and payload costs retain the
//! Hypersolve accounting exclusions; these are not yet whole-work-bounded ODEs.

/// Exact rational physical time through a source-bound affine clock.
pub mod clock;
/// Bounded native proposals and certified finite-horizon composition.
pub mod evolution;
/// Source-bound Picard self-map and contraction verification for dyadic steps.
pub mod picard;
use std::{collections::BTreeSet, fmt, sync::Arc};

use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};
use hyperlattice_interval::{BoxN, IntervalLatticeError, IntervalMatrix, IntervalVector};
use hypersolve::{
    SymbolId,
    expression_program::{
        ExpressionBudget, ExpressionDefinition, ExpressionError, ExpressionProgram, JacobianProgram,
    },
    work_budget::CancellationToken,
};

/// Explicit precision and resource configuration, shared by nested field calls.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct EnclosurePolicy {
    /// Requested dyadic grid exponent, not a final-width guarantee.
    pub precision: i32,
    /// Maximum aggregate structural entries in one checked stage.
    pub graph_entries: usize,
    /// Cumulative structural work, including adapter bookkeeping.
    pub graph_work: usize,
    /// Maximum native scalar bit reservation per arithmetic stage.
    pub scalar_bits: u64,
    /// Cumulative native dyadic arithmetic work.
    pub scalar_work: u64,
}
impl Default for EnclosurePolicy {
    fn default() -> Self {
        Self {
            precision: -64,
            graph_entries: 1_000_000,
            graph_work: 100_000_000,
            scalar_bits: 16_384,
            scalar_work: 1_000_000_000_000,
        }
    }
}

/// Mutable work remaining under one explicit enclosure policy.
///
/// Pass the same budget through construction and subsequent/nested calls to
/// aggregate work. Source ingestion/serde, scalar payload cloning and Hyperreal
/// constant refinement are outside total-work accounting. Heap bytes are not
/// bounded. Cancellation cannot interrupt one native scalar/refinement stage.
#[derive(Clone, Debug)]
pub struct EnclosureBudget {
    policy: EnclosurePolicy,
    graph: ExpressionBudget,
    arithmetic: DyadicBudget,
}
impl EnclosureBudget {
    /// Start fresh counters; constructing another budget explicitly starts a retry.
    pub fn new(policy: EnclosurePolicy) -> Self {
        Self {
            graph: ExpressionBudget::new(policy.graph_entries, policy.graph_work),
            arithmetic: DyadicBudget::new(policy.scalar_bits, policy.scalar_work),
            policy,
        }
    }
    /// The immutable configuration governing these counters.
    pub fn policy(&self) -> EnclosurePolicy {
        self.policy
    }
    /// Remaining structural work units.
    pub fn remaining_graph_work(&self) -> usize {
        self.graph.remaining_work()
    }
    /// Remaining native scalar arithmetic units.
    pub fn remaining_scalar_work(&self) -> u64 {
        self.arithmetic.remaining_work()
    }
    /// Set or remove cooperative cancellation for subsequent charged stages.
    pub fn set_cancellation(&mut self, token: Option<CancellationToken>) {
        self.graph.cancellation = token;
    }
}

/// Replay data, not a certificate. Load with `ExpressionField::from_definition`.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FieldDefinition {
    /// Full exact source with ordered bindings and outputs.
    pub source: ExpressionDefinition,
    /// Source variable denoting time.
    pub time: SymbolId,
    /// State variables in coordinate order; time must not occur here.
    pub state: Vec<SymbolId>,
}

#[derive(Debug)]
struct IdentityData {
    source: ExpressionProgram,
    time: SymbolId,
    state: Vec<SymbolId>,
}

/// Shared immutable structural identity, including time/state interpretation.
///
/// Equality is structural, not equality of functions. Serialization emits only
/// `FieldDefinition`; validation and derivative regeneration are required on load.
#[derive(Clone, Debug)]
pub struct FieldIdentity(Arc<IdentityData>);
impl FieldIdentity {
    /// Exact source definition.
    pub fn source(&self) -> &ExpressionDefinition {
        self.0.source.definition()
    }
    /// Time variable.
    pub fn time(&self) -> SymbolId {
        self.0.time
    }
    /// State variables in coordinate order.
    pub fn state(&self) -> &[SymbolId] {
        &self.0.state
    }
}
impl PartialEq for FieldIdentity {
    fn eq(&self, other: &Self) -> bool {
        self.time() == other.time()
            && self.state() == other.state()
            && self.source() == other.source()
    }
}
#[cfg(feature = "serde")]
impl serde::Serialize for FieldIdentity {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        #[derive(serde::Serialize)]
        struct DefinitionRef<'a> {
            source: &'a ExpressionDefinition,
            time: SymbolId,
            state: &'a [SymbolId],
        }
        DefinitionRef {
            source: self.source(),
            time: self.time(),
            state: self.state(),
        }
        .serialize(serializer)
    }
}

/// Shape, binding, expression or native enclosure failure, retaining its source.
#[derive(Clone, Debug)]
pub enum FieldError {
    /// The replay state list has a different length from the field dimension.
    StateCount {
        /// Required count.
        expected: usize,
        /// Supplied count.
        actual: usize,
    },
    /// Source outputs do not match the required shape.
    OutputCount {
        /// Required count.
        expected: usize,
        /// Supplied count.
        actual: usize,
    },
    /// The source must bind exactly one time and N state variables.
    BindingCount {
        /// Required count.
        expected: usize,
        /// Supplied count.
        actual: usize,
    },
    /// Two state coordinates name the same variable.
    DuplicateState(SymbolId),
    /// Time also occurs in the state coordinate list.
    TimeIsState(SymbolId),
    /// A source binding is not the declared time or any declared state variable.
    UnexpectedBinding(SymbolId),
    /// Validation, graph resource, derivative or expression-image failure.
    Expression(ExpressionError),
    /// Native scalar failure while preparing field inputs.
    Scalar(IntervalError),
    /// Native Cartesian carrier failure.
    Lattice(IntervalLatticeError),
}
impl fmt::Display for FieldError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "field enclosure: {self:?}")
    }
}
impl std::error::Error for FieldError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Expression(e) => Some(e),
            Self::Scalar(e) => Some(e),
            Self::Lattice(e) => Some(e),
            _ => None,
        }
    }
}
impl From<ExpressionError> for FieldError {
    fn from(e: ExpressionError) -> Self {
        Self::Expression(e)
    }
}
impl From<IntervalError> for FieldError {
    fn from(e: IntervalError) -> Self {
        Self::Scalar(e)
    }
}
impl From<IntervalLatticeError> for FieldError {
    fn from(e: IntervalLatticeError) -> Self {
        Self::Lattice(e)
    }
}

mod sealed {
    pub trait Sealed {}
}
/// A field with an owned validated definition, rather than an unchecked closure.
///
/// This initial sealed interface uses dyadic boxes. Images retain full-domain
/// checks. Jacobian success is a source-bound formula enclosure, not a general
/// continuity/existence/uniqueness certificate. Dependency overestimation and
/// removable chain-rule singularities can prevent a Jacobian proof even when
/// the field's value image succeeds.
pub trait EnclosedVectorField<const N: usize>: sealed::Sealed {
    /// Shared complete source/time/state identity.
    fn identity(&self) -> FieldIdentity;
    /// Enclose values on the Cartesian time/state product.
    fn image(
        &self,
        time: &DyadicInterval,
        state: &BoxN<DyadicInterval, N>,
        budget: &mut EnclosureBudget,
    ) -> Result<IntervalVector<DyadicInterval, N>, FieldError>;
    /// Enclose state partials as a row-major native interval matrix.
    fn jacobian(
        &self,
        time: &DyadicInterval,
        state: &BoxN<DyadicInterval, N>,
        budget: &mut EnclosureBudget,
    ) -> Result<IntervalMatrix<DyadicInterval, N, N>, FieldError>;
}

#[derive(Clone, Copy, Debug)]
enum Input {
    Time,
    State(usize),
}
#[derive(Debug)]
struct FieldData {
    identity: FieldIdentity,
    jacobian: JacobianProgram,
    inputs: Vec<Input>,
}

/// A validated Hypersolve expression field with exactly N outputs and state lanes.
///
/// The source may bind variables in any order. Clones share immutable source,
/// derivative and binding-map storage. No executable field is deserialized.
#[derive(Clone, Debug)]
pub struct ExpressionField<const N: usize> {
    data: Arc<FieldData>,
}
impl<const N: usize> ExpressionField<N> {
    /// Validate shape, all source references and bindings, then regenerate partials.
    /// Scalar ingress/payload and graph-budget exclusions remain explicit.
    pub fn from_definition(
        definition: FieldDefinition,
        budget: &mut EnclosureBudget,
    ) -> Result<Self, FieldError> {
        budget.graph.charge(0)?;
        let inputs = N.checked_add(1).ok_or(ExpressionError::SizeExhausted)?;
        if definition.state.len() != N {
            return Err(FieldError::StateCount {
                expected: N,
                actual: definition.state.len(),
            });
        }
        if definition.source.outputs.len() != N {
            return Err(FieldError::OutputCount {
                expected: N,
                actual: definition.source.outputs.len(),
            });
        }
        if definition.source.variables.len() != inputs {
            return Err(FieldError::BindingCount {
                expected: inputs,
                actual: definition.source.variables.len(),
            });
        }
        budget.graph.check_entries(&[
            N,
            definition.source.variables.len(),
            definition.source.nodes.len(),
            definition.source.outputs.len(),
        ])?;
        let mut seen = BTreeSet::new();
        for id in &definition.state {
            budget.graph.charge(1)?;
            if *id == definition.time {
                return Err(FieldError::TimeIsState(*id));
            }
            if !seen.insert(*id) {
                return Err(FieldError::DuplicateState(*id));
            }
        }
        let source = ExpressionProgram::from_definition(definition.source, &mut budget.graph)?;
        Self::from_validated_source(source, definition.time, definition.state, inputs, budget)
    }

    // Shape, time/state separation and the source graph are already validated.
    // Reparameterization preserves those invariants and can share the owned DAG.
    // Keep the same constructor effects/order when called from from_definition.
    #[inline(always)]
    fn from_validated_source(
        source: ExpressionProgram,
        time: SymbolId,
        state: Vec<SymbolId>,
        inputs: usize,
        budget: &mut EnclosureBudget,
    ) -> Result<Self, FieldError> {
        let mut bindings = Vec::new();
        for id in &source.definition().variables {
            budget.graph.charge(inputs)?;
            bindings.push(if *id == time {
                Input::Time
            } else {
                Input::State(
                    state
                        .iter()
                        .position(|state| state == id)
                        .ok_or(FieldError::UnexpectedBinding(*id))?,
                )
            });
        }
        let jacobian = source.jacobian(&state, &mut budget.graph)?;
        budget.graph.charge(2)?;
        let identity = FieldIdentity(Arc::new(IdentityData {
            source,
            time,
            state,
        }));
        let field = Self {
            data: Arc::new(FieldData {
                identity,
                jacobian,
                inputs: bindings,
            }),
        };
        budget.graph.charge(0)?;
        Ok(field)
    }

    fn arguments(
        &self,
        program: &ExpressionProgram,
        time: &DyadicInterval,
        state: &BoxN<DyadicInterval, N>,
        budget: &mut EnclosureBudget,
    ) -> Result<Option<Vec<DyadicInterval>>, FieldError> {
        budget.graph.charge(0)?;
        let source = program.definition();
        budget.graph.check_entries(&[
            source.variables.len(),
            source.nodes.len(),
            source.outputs.len(),
            self.data.inputs.len(),
        ])?;
        budget.graph.charge(1)?;
        if time.is_empty() || state.is_empty() {
            return Ok(None);
        }
        let lanes = state.lanes().expect("nonempty state");
        let mut values = Vec::new();
        for input in &self.data.inputs {
            budget.graph.charge(1)?;
            let value = match input {
                Input::Time => time,
                Input::State(i) => &lanes[*i],
            };
            values.push(value.checked_pow(1, &mut budget.arithmetic)?);
        }
        Ok(Some(values))
    }
}
impl<const N: usize> sealed::Sealed for ExpressionField<N> {}
impl<const N: usize> EnclosedVectorField<N> for ExpressionField<N> {
    fn identity(&self) -> FieldIdentity {
        self.data.identity.clone()
    }
    fn image(
        &self,
        time: &DyadicInterval,
        state: &BoxN<DyadicInterval, N>,
        budget: &mut EnclosureBudget,
    ) -> Result<IntervalVector<DyadicInterval, N>, FieldError> {
        let source = self.data.jacobian.source();
        let Some(values) = self.arguments(source, time, state, budget)? else {
            let output = IntervalVector::empty()?;
            budget.graph.charge(0)?;
            return Ok(output);
        };
        let output = source.eval_dyadic(
            &values,
            budget.policy.precision,
            &mut budget.graph,
            &mut budget.arithmetic,
        )?;
        budget.graph.charge(N)?;
        let lanes = output
            .try_into()
            .map_err(|v: Vec<_>| FieldError::OutputCount {
                expected: N,
                actual: v.len(),
            })?;
        let image = IntervalVector::new(lanes)?;
        budget.graph.charge(0)?;
        Ok(image)
    }
    fn jacobian(
        &self,
        time: &DyadicInterval,
        state: &BoxN<DyadicInterval, N>,
        budget: &mut EnclosureBudget,
    ) -> Result<IntervalMatrix<DyadicInterval, N, N>, FieldError> {
        let source = self.data.jacobian.program();
        let Some(values) = self.arguments(source, time, state, budget)? else {
            let output = IntervalMatrix::empty()?;
            budget.graph.charge(0)?;
            return Ok(output);
        };
        let output = source.eval_dyadic(
            &values,
            budget.policy.precision,
            &mut budget.graph,
            &mut budget.arithmetic,
        )?;
        let entries = N
            .checked_mul(N)
            .and_then(|v| v.checked_add(N))
            .ok_or(ExpressionError::SizeExhausted)?;
        if output.len() != entries {
            return Err(FieldError::OutputCount {
                expected: entries,
                actual: output.len(),
            });
        }
        budget.graph.charge(entries)?;
        let mut values: Vec<_> = output.into_iter().map(Some).collect();
        budget.graph.charge(entries)?;
        let rows = core::array::from_fn(|row| {
            core::array::from_fn(|column| {
                let index = self
                    .data
                    .jacobian
                    .partial_output(row, column)
                    .expect("validated matrix shape");
                values[index].take().expect("distinct Jacobian coordinate")
            })
        });
        let image = IntervalMatrix::new(rows)?;
        budget.graph.charge(0)?;
        Ok(image)
    }
}
