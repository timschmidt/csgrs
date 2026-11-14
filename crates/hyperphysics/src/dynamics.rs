//! Source-bound Newtonian motion with native trajectory certificates.
//!
//! State order is `[x, y, z, vx, vy, vz]`. Hyperphysics retains physical meaning
//! and force provenance; Hyperdynamics owns all ODE verification and evolution.
use std::{fmt, sync::Arc};

use crate::{ForceAccumulator3, SourceSpec};
use hyperdynamics::field::{
    EnclosureBudget, ExpressionField, FieldDefinition, FieldError,
    evolution::{EvolutionOptions, EvolutionReport, EvolutionRequest},
    picard::{PicardProposal, PicardReport},
};
use hyperlimit::{Certainty, PredicateOutcome, PredicatePolicy, Sign};
use hyperreal::Real;
use hypersolve::{
    SymbolId, SymbolRef,
    expression_program::{
        BinaryOperation, ExpressionDefinition, ExpressionError, ExpressionNode, ExpressionProgram,
    },
};

/// Replayable force source and its physical coordinate interpretation.
///
/// Exactly three outputs denote net Cartesian force, with seven distinct inputs
/// denoting time and three position/velocity components. Binding order is arbitrary.
/// Units/frame consistency and the physical force law are authored assumptions.
/// Nonlinear/time/velocity dependence uses the existing Hypersolve grammar; source
/// guards remain active. No scalar/vector expression is approximated on import.
#[derive(Clone, Debug)]
pub struct ForceFieldDefinition3 {
    /// Exact force graph, including all ordered bindings and original guards.
    pub source: ExpressionDefinition,
    /// Time symbol.
    pub time: SymbolId,
    /// Cartesian position symbols, in x/y/z order.
    pub position: [SymbolId; 3],
    /// Cartesian velocity symbols, in x/y/z order.
    pub velocity: [SymbolId; 3],
    /// Authored source references; not independent verification of a force law.
    pub provenance: Vec<SourceSpec>,
}

/// Physical validation or original lower-layer construction failure.
#[derive(Clone, Debug)]
pub enum DynamicsError {
    /// Force output count differs from three.
    ForceOutputCount { actual: usize },
    /// Mass was not strictly certified positive. Retains a negative/zero result
    /// separately from an unresolved predicate and its refinement requirement.
    MassSign(PredicateOutcome<Sign>),
    /// Original source/reference/graph resource or cancellation failure.
    Expression(ExpressionError),
    /// Original field shape/binding/derivative/native failure.
    Field(FieldError),
}
impl fmt::Display for DynamicsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "physical dynamics: {self:?}")
    }
}
impl std::error::Error for DynamicsError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Expression(e) => Some(e),
            Self::Field(e) => Some(e),
            _ => None,
        }
    }
}
impl From<ExpressionError> for DynamicsError {
    fn from(e: ExpressionError) -> Self {
        Self::Expression(e)
    }
}
impl From<FieldError> for DynamicsError {
    fn from(e: FieldError) -> Self {
        Self::Field(e)
    }
}
#[derive(Debug)]
struct DynamicsData {
    mass: Real,
    mass_sign: PredicateOutcome<Sign>,
    force_source: ExpressionProgram,
    provenance: Vec<SourceSpec>,
    constant_forces: Option<ForceAccumulator3>,
    field: ExpressionField<6>,
}

/// Fixed-mass Newtonian model x'=v, v'=F(t,x,v)/m with immutable source evidence.
///
/// This is continuous dynamics, distinct from a discrete Euler replay. Constructing
/// a field proves no trajectory. Step/evolution certificates depend on the supplied
/// initial/candidate sets and retain native qualified failures. Variable/uncertain
/// mass, contact impulses, rotations and coupled bodies require separate contracts.
#[derive(Clone, Debug)]
pub struct PointMassDynamics3(Arc<DynamicsData>);
impl PointMassDynamics3 {
    /// Validate an exact mass and force definition, then build its six-state field.
    ///
    /// Strict mass positivity retains its full predicate receipt. One shared graph
    /// counter covers adapter work, original source validation and field/derivative
    /// construction. Scalar positivity refinement/arithmetic, metadata byte copies,
    /// total heap and cancellation within scalar operations retain explicit exclusions.
    /// Failed construction leaves no published model or mutable source cache.
    pub fn new(
        mass: Real,
        forces: ForceFieldDefinition3,
        budget: &mut EnclosureBudget,
    ) -> Result<Self, DynamicsError> {
        Self::build(mass, forces, None, budget)
    }
    fn build(
        mass: Real,
        forces: ForceFieldDefinition3,
        constant_forces: Option<ForceAccumulator3>,
        budget: &mut EnclosureBudget,
    ) -> Result<Self, DynamicsError> {
        budget.graph_budget().charge(0)?;
        if forces.source.outputs.len() != 3 {
            return Err(DynamicsError::ForceOutputCount {
                actual: forces.source.outputs.len(),
            });
        }
        budget.graph_budget().check_entries(&[
            forces.source.variables.len(),
            forces.source.nodes.len(),
            forces.source.outputs.len(),
            forces.provenance.len(),
            7,
            6,
        ])?;
        budget.graph_budget().charge(1)?;
        let mass_sign = hyperlimit::classify_real_sign(&mass, PredicatePolicy::STRICT);
        if !matches!(
            mass_sign,
            PredicateOutcome::Decided {
                value: Sign::Positive,
                certainty: Certainty::Exact | Certainty::Filtered,
                ..
            }
        ) {
            return Err(DynamicsError::MassSign(mass_sign));
        }
        // Validate BEFORE appending nodes: an invalid original reference must not
        // accidentally become a valid reference to a newly generated mass/velocity.
        let force_source = budget.graph_budget().validate(forces.source)?;
        let original = force_source.definition();
        budget.graph_budget().charge(original.variables.len())?;
        budget.graph_budget().charge(original.nodes.len())?;
        budget.graph_budget().charge(original.outputs.len())?;
        let mut source = original.clone();
        let roots = [source.outputs[0], source.outputs[1], source.outputs[2]];
        let mass_node = source.nodes.len();
        budget.graph_budget().charge(1)?;
        source.nodes.push(ExpressionNode::Constant(mass.clone()));
        source.outputs.clear();
        for id in forces.velocity {
            budget.graph_budget().charge(2)?;
            source.outputs.push(source.nodes.len());
            source
                .nodes
                .push(ExpressionNode::Variable(SymbolRef::new(id, None)));
        }
        for root in roots {
            budget.graph_budget().charge(2)?;
            source.outputs.push(source.nodes.len());
            source.nodes.push(ExpressionNode::Binary {
                operation: BinaryOperation::Div,
                left: root,
                right: mass_node,
            });
        }
        budget.graph_budget().charge(6)?;
        let state = forces.position.into_iter().chain(forces.velocity).collect();
        let field = ExpressionField::from_definition(
            FieldDefinition {
                source,
                time: forces.time,
                state,
            },
            budget,
        )?;
        budget.graph_budget().charge(1)?;
        let result = Self(Arc::new(DynamicsData {
            mass,
            mass_sign,
            force_source,
            provenance: forces.provenance,
            constant_forces,
            field,
        }));
        budget.graph_budget().charge(0)?;
        Ok(result)
    }
    /// Exact retained fixed mass.
    pub fn mass(&self) -> &Real {
        &self.0.mass
    }
    /// Original strict positive-mass predicate, including certainty and stage.
    pub fn mass_sign(&self) -> PredicateOutcome<Sign> {
        self.0.mass_sign
    }
    /// Original force graph, preserving its input order and domain guards.
    pub fn force_source(&self) -> &ExpressionProgram {
        &self.0.force_source
    }
    /// Authored force provenance.
    pub fn provenance(&self) -> &[SourceSpec] {
        &self.0.provenance
    }
    /// Every original contribution when built from a constant-force accumulator.
    pub fn constant_forces(&self) -> Option<&ForceAccumulator3> {
        self.0.constant_forces.as_ref()
    }
    /// Generated field and its exact time/state interpretation. Its state order is
    /// `[x,y,z,vx,vy,vz]`. Lower-layer coordinate/Taylor consumers remain available.
    pub fn field(&self) -> &ExpressionField<6> {
        &self.0.field
    }
    /// Verify a proposed tube, retaining this model alongside all native evidence.
    /// Rejected native attempts publish no certified tube/endpoint. Metadata sharing
    /// does not create a new budget or change the native policy/status.
    pub fn certify_step(
        &self,
        proposal: PicardProposal<6>,
        budget: &mut EnclosureBudget,
    ) -> PointMassStep3 {
        PointMassStep3 {
            model: self.clone(),
            native: self.0.field.certify_step(proposal, budget),
        }
    }
    /// Evolve across a finite horizon with shared native budgets and attempt history.
    /// A certified prefix remains distinct from whole-request completion. Generated
    /// end boxes feed subsequent steps without relabeling them as exact reachable sets.
    pub fn integrate(
        &self,
        request: EvolutionRequest<6>,
        options: EvolutionOptions,
        budget: &mut EnclosureBudget,
    ) -> PointMassEvolution3 {
        PointMassEvolution3 {
            model: self.clone(),
            native: self.0.field.integrate(request, options, budget),
        }
    }
}

/// Attempted physical step, including its model and native proof or typed rejection.
#[derive(Clone, Debug)]
pub struct PointMassStep3 {
    model: PointMassDynamics3,
    native: PicardReport<6>,
}
impl PointMassStep3 {
    /// Mass, original force sources and exact equations used for this attempt.
    pub fn model(&self) -> &PointMassDynamics3 {
        &self.model
    }
    /// Native source/proposal/policy, statuses and certified tube/end accessors.
    pub fn native(&self) -> &PicardReport<6> {
        &self.native
    }
}
/// Whole-horizon physical evolution with retained model and qualified native prefix.
#[derive(Clone, Debug)]
pub struct PointMassEvolution3 {
    model: PointMassDynamics3,
    native: EvolutionReport<6>,
}
impl PointMassEvolution3 {
    /// Original physical model and provenance.
    pub fn model(&self) -> &PointMassDynamics3 {
        &self.model
    }
    /// Native whole-request status, attempts and separate prefix/end accessors.
    pub fn native(&self) -> &EvolutionReport<6> {
        &self.native
    }
}
impl ForceAccumulator3 {
    /// Snapshot constant forces into continuous fixed-mass Newtonian dynamics.
    ///
    /// Uses existing exact force accumulation and retains every source contribution.
    /// Labels are attributed to their original accumulator entries, not promoted to
    /// external validation. Scalar summation and source-byte costs are outside the
    /// graph budget; cancellation is checked before/after this scalar operation.
    pub fn dynamics(
        &self,
        mass: Real,
        budget: &mut EnclosureBudget,
    ) -> Result<PointMassDynamics3, DynamicsError> {
        let count = self.contributions().len();
        budget.graph_budget().charge(0)?;
        budget.graph_budget().check_entries(&[count, 7, 3, 3])?;
        budget.graph_budget().charge(count)?;
        let total = self.total_force();
        budget.graph_budget().charge(count)?;
        let provenance = self
            .contributions()
            .iter()
            .map(|c| SourceSpec::new("ForceAccumulator3", c.source.clone()))
            .collect();
        budget.graph_budget().charge(count)?;
        let retained = self.clone();
        let forces = ForceFieldDefinition3 {
            source: ExpressionDefinition {
                variables: (0..7).map(SymbolId).collect(),
                nodes: vec![
                    ExpressionNode::Constant(total[0].clone()),
                    ExpressionNode::Constant(total[1].clone()),
                    ExpressionNode::Constant(total[2].clone()),
                ],
                outputs: vec![0, 1, 2],
            },
            time: SymbolId(0),
            position: [SymbolId(1), SymbolId(2), SymbolId(3)],
            velocity: [SymbolId(4), SymbolId(5), SymbolId(6)],
            provenance,
        };
        PointMassDynamics3::build(mass, forces, Some(retained), budget)
    }
}
