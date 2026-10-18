use std::{cmp::Ordering, sync::Arc};

use hyperlimit::{Certainty, PredicateOutcome};
use hyperreal::Real;

/// Physical/input accuracy, independent of arithmetic certainty.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum InputAccuracy {
    /// No accuracy information was provided; this does not mean zero error.
    Unspecified,
    /// Values define an exact mathematical model, without a physical claim.
    ExactModel,
    /// A retained qualification supplied by the input producer.
    Qualified(Arc<str>),
}

/// Source description supplied at an explicit coordinate ingress boundary.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct InputProvenance {
    source: Arc<str>,
    accuracy: InputAccuracy,
}

impl InputProvenance {
    /// Names the source and explicitly states what is known about its accuracy.
    pub fn new(source: impl Into<Arc<str>>, accuracy: InputAccuracy) -> Self {
        Self {
            source: source.into(),
            accuracy,
        }
    }

    /// Source identity/description retained unchanged by transformations.
    pub fn source(&self) -> &str {
        &self.source
    }

    /// Physical/model qualification of this input.
    pub fn accuracy(&self) -> &InputAccuracy {
        &self.accuracy
    }
}

/// An actual predicate result bound to the operands and claim it decided.
#[derive(Clone, Debug)]
pub struct Decision {
    pub(crate) claim: &'static str,
    pub(crate) left: Real,
    pub(crate) right: Real,
    pub(crate) outcome: PredicateOutcome<Ordering>,
}

impl Decision {
    /// Domain/branch claim examined by the predicate.
    pub fn claim(&self) -> &'static str {
        self.claim
    }
    /// Exact represented left operand.
    pub fn left(&self) -> &Real {
        &self.left
    }
    /// Exact represented right operand.
    pub fn right(&self) -> &Real {
        &self.right
    }
    /// Decision, certainty and escalation stage supplied by Hyperlimit.
    pub fn outcome(&self) -> PredicateOutcome<Ordering> {
        self.outcome
    }
}

#[derive(Debug)]
struct Node {
    operation: &'static str,
    parents: Vec<Receipt>,
    decisions: Vec<Decision>,
    input: Option<InputProvenance>,
    certainty: Certainty,
    depth: usize,
}

/// Immutable shared evidence graph; cloning a receipt does not copy its history.
///
/// Certainty concerns arithmetic and consumed predicates. Inspect input leaves
/// separately for model/measurement qualification. A later strict operation
/// retains any earlier approximate decision in this graph.
#[derive(Clone, Debug)]
pub struct Receipt(Arc<Node>);

impl Receipt {
    pub(crate) fn build(
        operation: &'static str,
        parents: Vec<Self>,
        decisions: Vec<Decision>,
        input: Option<InputProvenance>,
    ) -> Self {
        let depth = parents.iter().map(Self::depth).max().unwrap_or(0) + 1;
        let certainty = parents
            .iter()
            .map(Self::certainty)
            .chain(
                decisions
                    .iter()
                    .filter_map(|decision| match decision.outcome {
                        PredicateOutcome::Decided { certainty, .. } => Some(certainty),
                        PredicateOutcome::Unknown { .. } => None,
                    }),
            )
            .fold(Certainty::Exact, weaker);
        Self(Arc::new(Node {
            operation,
            parents,
            decisions,
            input,
            certainty,
            depth,
        }))
    }

    /// Operation that produced this node.
    pub fn operation(&self) -> &'static str {
        self.0.operation
    }
    /// Retained source operations, including their inherited qualification.
    pub fn parents(&self) -> &[Self] {
        &self.0.parents
    }
    /// Domain decisions made locally by this operation.
    pub fn decisions(&self) -> &[Decision] {
        &self.0.decisions
    }
    /// Source qualification at an ingress node, if this is one.
    pub fn input(&self) -> Option<&InputProvenance> {
        self.0.input.as_ref()
    }
    /// Weakest numerical certainty consumed anywhere in this ancestry.
    pub fn certainty(&self) -> Certainty {
        self.0.certainty
    }
    /// Maximum path length through the retained evidence graph.
    pub fn depth(&self) -> usize {
        self.0.depth
    }
}

fn weaker(left: Certainty, right: Certainty) -> Certainty {
    match (left, right) {
        (Certainty::Approximate, _) | (_, Certainty::Approximate) => Certainty::Approximate,
        (Certainty::Filtered, _) | (_, Certainty::Filtered) => Certainty::Filtered,
        _ => Certainty::Exact,
    }
}
