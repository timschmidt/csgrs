//! Circuit-owned residual metadata with source-bound interval/Jacobian evidence.
//!
//! Native intervals describe a Cartesian input domain. Repeated use of a parameter
//! shares its symbol; correlated parameters must be expressed through shared latent
//! inputs. Zero-containing residual ranges alone prove neither a root nor uniqueness.
use std::{collections::BTreeSet, fmt, sync::Arc};

use hyperinterval::{DyadicBudget, DyadicInterval};
use hypersolve::{
    SymbolId, SymbolRef,
    expression_program::{
        BinaryOperation, ExpressionBudget, ExpressionDefinition, ExpressionError, ExpressionNode,
        ExpressionProgram, JacobianProgram, RegularJacobianEnclosure,
    },
};

use crate::{CircuitError, CircuitParameter, ComponentId, LinearMnaSystem, MnaUnknown};

/// Circuit meaning of one independently varying source binding.
#[derive(Clone, Debug, PartialEq)]
pub enum ResidualInputKind {
    /// Net voltage or branch current, retaining the original MNA identity.
    Unknown(MnaUnknown),
    /// Component parameter with nominal value, units and authored provenance.
    /// Its admitted interval is supplied on each enclosure query; a restricted
    /// operating region need not contain the nominal value.
    Parameter {
        /// Owning component.
        component: ComponentId,
        /// Original parameter metadata; `name` must be nonempty.
        parameter: CircuitParameter,
    },
}

/// Ordered source binding and its circuit meaning.
#[derive(Clone, Debug, PartialEq)]
pub struct ResidualInput {
    /// Exact source symbol; must match this slot in the source binding table.
    pub symbol: SymbolId,
    /// Circuit-domain identity and provenance.
    pub kind: ResidualInputKind,
}

/// Invalid circuit association or original lower-layer construction failure.
#[derive(Clone, Debug)]
pub enum ResidualError {
    /// Original circuit validation failure (including malformed dense MNA shape).
    Circuit(CircuitError),
    /// Original graph, domain, regularity, native arithmetic or resource failure.
    Expression(ExpressionError),
    /// Metadata count differs from the source input or output count.
    MetadataCount {
        /// True for input metadata; false for equation labels.
        inputs: bool,
        /// Count required by the authoritative source.
        expected: usize,
        /// Supplied metadata count.
        actual: usize,
    },
    /// Input symbol does not match its source slot.
    InputSymbol {
        /// First mismatching input slot.
        slot: usize,
    },
    /// Two inputs name the same unknown or the same component/parameter.
    DuplicateInput {
        /// Second occurrence in input order.
        slot: usize,
    },
    /// A parameter name is empty.
    EmptyParameterName {
        /// Input containing the invalid name.
        slot: usize,
    },
}
impl fmt::Display for ResidualError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "circuit residual: {self:?}")
    }
}
impl std::error::Error for ResidualError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Circuit(error) => Some(error),
            Self::Expression(error) => Some(error),
            _ => None,
        }
    }
}
impl From<ExpressionError> for ResidualError {
    fn from(error: ExpressionError) -> Self {
        Self::Expression(error)
    }
}

#[derive(Debug)]
struct ResidualData {
    jacobian: JacobianProgram,
    inputs: Vec<ResidualInput>,
    equations: Vec<String>,
}

/// Immutable circuit association and demanded derivatives of its exact residuals.
///
/// All inputs, including uncertain component parameters, have derivative columns.
/// Construction does not certify an operating region. Replay the exact source,
/// ordered metadata and query domain through the constructors; serialized flags
/// and cached derivatives are not proof. Metadata is an authored association, not
/// certification that external circuit equations or units were authored correctly.
#[derive(Clone, Debug)]
pub struct CircuitResidualProgram(Arc<ResidualData>);
impl CircuitResidualProgram {
    /// Associate a validated source with ordered circuit input and equation metadata.
    ///
    /// Distinct circuit identities and exact symbol order are checked before any
    /// derivative generation. Graph work and structural size share `graph`.
    /// Metadata strings, scalar payload/refinement, total heap bytes and cancellation
    /// within scalar/native operations remain outside that contract.
    pub fn new(
        source: ExpressionProgram,
        inputs: Vec<ResidualInput>,
        equations: Vec<String>,
        graph: &mut ExpressionBudget,
    ) -> Result<Self, ResidualError> {
        graph.charge(0)?;
        let definition = source.definition();
        for (input_metadata, expected, actual) in [
            (true, definition.variables.len(), inputs.len()),
            (false, definition.outputs.len(), equations.len()),
        ] {
            if expected != actual {
                return Err(ResidualError::MetadataCount {
                    inputs: input_metadata,
                    expected,
                    actual,
                });
            }
        }
        graph.check_entries(&[
            definition.variables.len(),
            definition.nodes.len(),
            definition.outputs.len(),
            inputs.len(),
            equations.len(),
        ])?;
        let mut unknowns = BTreeSet::new();
        let mut parameters = BTreeSet::new();
        for (slot, input) in inputs.iter().enumerate() {
            graph.charge(1)?;
            if input.symbol != definition.variables[slot] {
                return Err(ResidualError::InputSymbol { slot });
            }
            let unique = match &input.kind {
                ResidualInputKind::Unknown(unknown) => unknowns.insert(unknown),
                ResidualInputKind::Parameter {
                    component,
                    parameter,
                } => {
                    if parameter.name.is_empty() {
                        return Err(ResidualError::EmptyParameterName { slot });
                    }
                    parameters.insert((component, &parameter.name))
                }
            };
            if !unique {
                return Err(ResidualError::DuplicateInput { slot });
            }
        }
        let jacobian = source.jacobian(&definition.variables, graph)?;
        graph.charge(1)?;
        let result = Self(Arc::new(ResidualData {
            jacobian,
            inputs,
            equations,
        }));
        graph.charge(0)?;
        Ok(result)
    }
    /// Original exact residual definitions and binding order.
    pub fn source(&self) -> &ExpressionProgram {
        self.0.jacobian.source()
    }
    /// Circuit inputs in source binding and Jacobian column order.
    pub fn inputs(&self) -> &[ResidualInput] {
        &self.0.inputs
    }
    /// Equation labels in source output and Jacobian row order.
    pub fn equations(&self) -> &[String] {
        &self.0.equations
    }
    /// Source-bound generated derivatives, reusable by lower-layer consumers.
    pub fn jacobian(&self) -> &JacobianProgram {
        &self.0.jacobian
    }
    /// Enclose the entire input product and all derivatives, preserving evidence.
    ///
    /// Uses Hypersolve's sufficient full-domain continuity/C1 checks. Conservative
    /// guard failure does not prove the actual function undefined. `precision` is
    /// grid spacing, not a final-width target. Native arithmetic uses `arithmetic`;
    /// graph and adapter traversal share `graph`. No partial certificate is published
    /// on failure, and failed attempts do not change this source or cache a decision.
    pub fn enclose(
        &self,
        arguments: &[DyadicInterval],
        precision: i32,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<CircuitResidualEnclosure, ResidualError> {
        let evidence = self
            .0
            .jacobian
            .enclose_regular(arguments, precision, graph, arithmetic)?;
        graph.charge(arguments.len())?;
        let disposition = if arguments.iter().any(DyadicInterval::is_empty) {
            ResidualDisposition::Empty
        } else {
            let mut disposition = ResidualDisposition::NotExcluded;
            for (equation, value) in evidence.values()[..self.equations().len()]
                .iter()
                .enumerate()
            {
                graph.charge(1)?;
                if !value.signs().contains_zero() {
                    disposition = ResidualDisposition::Excluded { equation };
                    break;
                }
            }
            disposition
        };
        graph.charge(1)?;
        let result = CircuitResidualEnclosure {
            circuit: self.clone(),
            evidence,
            disposition,
        };
        graph.charge(0)?;
        Ok(result)
    }
}

/// What these residual bounds establish about simultaneous roots.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ResidualDisposition {
    /// The argument Cartesian product is empty (not a domain or resource failure).
    Empty,
    /// This residual excludes zero throughout the product, proving no common root.
    Excluded {
        /// Equation index in the retained circuit metadata.
        equation: usize,
    },
    /// No residual excludes zero. This does not prove a root exists or is unique.
    NotExcluded,
}

/// Circuit metadata and lower-layer full-domain residual/Jacobian proof.
#[derive(Clone, Debug)]
pub struct CircuitResidualEnclosure {
    circuit: CircuitResidualProgram,
    evidence: RegularJacobianEnclosure,
    disposition: ResidualDisposition,
}
impl CircuitResidualEnclosure {
    /// Immutable circuit definitions and metadata used by this query.
    pub fn circuit(&self) -> &CircuitResidualProgram {
        &self.circuit
    }
    /// Full source, arguments, precision, residuals and derivative evidence.
    /// `matrix` projects derivatives for subsequent Hyperlattice operations.
    pub fn evidence(&self) -> &RegularJacobianEnclosure {
        &self.evidence
    }
    /// Residual ranges in retained equation order.
    pub fn residuals(&self) -> &[DyadicInterval] {
        &self.evidence.values()[..self.circuit.equations().len()]
    }
    /// Root exclusion, empty domain or an unresolved zero-containing result.
    pub fn disposition(&self) -> ResidualDisposition {
        self.disposition
    }
}

impl LinearMnaSystem {
    /// Snapshot A*x-b into the same circuit residual carrier used for nonlinear
    /// and component-tolerance sources. Preserves exact coefficients and unknown
    /// order; it neither restamps nor solves the circuit. The dense MNA carrier no
    /// longer contains original stamp provenance, so this method does not invent it.
    /// Symbols are local sequential indices. Later mutation of the public matrix,
    /// RHS or unknowns cannot alter the returned source or any issued evidence.
    pub fn residual_program(
        &self,
        graph: &mut ExpressionBudget,
    ) -> Result<CircuitResidualProgram, ResidualError> {
        graph.charge(0)?;
        let n = self.unknowns.len();
        graph.charge(self.matrix.len())?;
        if self.matrix.len() != n || self.rhs.len() != n || self.matrix.iter().any(|r| r.len() != n)
        {
            return Err(ResidualError::Circuit(
                CircuitError::CandidateLengthMismatch,
            ));
        }
        // n shared variable nodes; each row has zero + 3*n product/sum nodes +
        // RHS + subtraction. Check size before cloning any scalar or metadata.
        let nodes_count = n
            .checked_mul(3)
            .and_then(|v| v.checked_add(3))
            .and_then(|v| n.checked_mul(v))
            .and_then(|v| v.checked_add(n))
            .ok_or(ExpressionError::SizeExhausted)?;
        graph.check_entries(&[nodes_count, n, n, n, n])?;
        if n > u32::MAX as usize {
            return Err(ExpressionError::SizeExhausted.into());
        }
        let mut nodes = Vec::new();
        let mut variables = Vec::new();
        let mut inputs = Vec::new();
        let mut equations = Vec::new();
        let mut outputs = Vec::new();
        for (slot, unknown) in self.unknowns.iter().enumerate() {
            graph.charge(4)?;
            let symbol = SymbolId(slot as u32);
            variables.push(symbol);
            nodes.push(ExpressionNode::Variable(SymbolRef::new(symbol, None)));
            inputs.push(ResidualInput {
                symbol,
                kind: ResidualInputKind::Unknown(unknown.clone()),
            });
            equations.push(format!("MNA residual {slot}"));
        }
        for (row, rhs) in self.matrix.iter().zip(&self.rhs) {
            graph.charge(1)?;
            let mut sum = nodes.len();
            nodes.push(ExpressionNode::Constant(hyperreal::Real::zero()));
            for (column, coefficient) in row.iter().enumerate() {
                graph.charge(3)?;
                let constant = nodes.len();
                nodes.push(ExpressionNode::Constant(coefficient.clone()));
                nodes.push(ExpressionNode::Binary {
                    operation: BinaryOperation::Mul,
                    left: constant,
                    right: column,
                });
                nodes.push(ExpressionNode::Binary {
                    operation: BinaryOperation::Add,
                    left: sum,
                    right: constant + 1,
                });
                sum = constant + 2;
            }
            graph.charge(3)?;
            let constant = nodes.len();
            nodes.push(ExpressionNode::Constant(rhs.clone()));
            nodes.push(ExpressionNode::Binary {
                operation: BinaryOperation::Sub,
                left: sum,
                right: constant,
            });
            outputs.push(constant + 1);
        }
        let source = ExpressionProgram::from_definition(
            ExpressionDefinition {
                variables,
                nodes,
                outputs,
            },
            graph,
        )?;
        CircuitResidualProgram::new(source, inputs, equations, graph)
    }
}
