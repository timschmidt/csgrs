//! Source-bound composition of two already certified exact charts.
use super::*;

/// Incompatible chart associations or original arithmetic/resource failure.
#[derive(Clone, Debug)]
pub enum CoordinateTransitionError {
    /// Charts refer to different structural source definitions.
    SourceMismatch,
    /// Charts reinterpret different variables or a different variable order.
    VariableMismatch,
    /// Original coordinate arithmetic or graph diagnostic.
    Coordinate(CoordinateError),
}
impl fmt::Display for CoordinateTransitionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "coordinate transition: {self:?}")
    }
}
impl std::error::Error for CoordinateTransitionError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Coordinate(e) => Some(e),
            _ => None,
        }
    }
}
impl From<ExpressionError> for CoordinateTransitionError {
    fn from(e: ExpressionError) -> Self {
        Self::Coordinate(e.into())
    }
}
impl From<Problem> for CoordinateTransitionError {
    fn from(e: Problem) -> Self {
        Self::Coordinate(CoordinateError::Scalar(e))
    }
}

#[derive(Debug)]
struct TransitionData {
    from: CoordinateProgram,
    to: CoordinateProgram,
    matrix: Vec<Vec<Real>>,
    origin: Vec<Real>,
    program: ExpressionProgram,
}

/// Exact affine relation between two charts of the same structural source.
///
/// For x=c0+A0*y=c1+A1*z, this retains z=B1*A0*y+B1*(c0-c1).
/// The target chart already proves B1*A1=I under STRICT. Composing exact
/// coefficients before enclosure avoids an intermediate Cartesian projection.
/// Evaluation gives an outer enclosure of this retained exact affine image;
/// it does not assert that the image is Cartesian or that projections are equal.
///
/// Replay source and both coordinate changes, then regenerate this relation.
/// It cannot be deserialized as a certificate. Scalar arithmetic/refinement and
/// payload lifecycle, predicate internals, total heap bytes and cancellation
/// inside scalar work retain the coordinate program's accounting exclusions.
#[derive(Clone, Debug)]
pub struct CoordinateTransition(Arc<TransitionData>);
impl CoordinateTransition {
    /// Original chart, including its source and strict inverse proof.
    pub fn from(&self) -> &CoordinateProgram {
        &self.0.from
    }
    /// Destination chart with the same structural source and ordered variables.
    pub fn to(&self) -> &CoordinateProgram {
        &self.0.to
    }
    /// Exact row-major B1*A0, computed before native enclosure.
    pub fn matrix(&self) -> &[Vec<Real>] {
        &self.0.matrix
    }
    /// Exact B1*(c0-c1).
    pub fn origin(&self) -> &[Real] {
        &self.0.origin
    }
    /// Executable affine relation with inputs in `from().change().variables` order.
    /// Other source bindings, such as time, are not inputs to this state map.
    /// This map alone makes no source-domain or ODE existence claim.
    pub fn program(&self) -> &ExpressionProgram {
        &self.0.program
    }
}
impl CoordinateProgram {
    /// Compose exact chart coefficients, preserving both validated associations.
    ///
    /// Structural source equality permits independently reconstructed definitions;
    /// allocation identity is not required. Equality of arbitrary mathematical
    /// functions is not inferred. No chart is selected or declared optimal here.
    pub fn transition_to(
        &self,
        target: &Self,
        budget: &mut ExpressionBudget,
    ) -> Result<CoordinateTransition, CoordinateTransitionError> {
        budget.charge(0)?;
        let source = self.source().definition();
        let other = target.source().definition();
        // Fixed-arity node scans are charged; exact scalar payload comparison
        // retains the original ExpressionBudget exclusions.
        for definition in [source, other] {
            budget.charge(definition.variables.len())?;
            budget.charge(definition.nodes.len())?;
            budget.charge(definition.outputs.len())?;
        }
        if source != other {
            return Err(CoordinateTransitionError::SourceMismatch);
        }
        let n = self.change().variables.len();
        budget.charge(n)?;
        budget.charge(target.change().variables.len())?;
        if self.change().variables != target.change().variables {
            return Err(CoordinateTransitionError::VariableMismatch);
        }
        let square = n.checked_mul(n).ok_or(ExpressionError::SizeExhausted)?;
        budget.check_entries(&[square, n, n, n, n])?;
        let a = self.change();
        let b = target.change();
        let mut delta = Vec::with_capacity(n);
        for (old, new) in a.origin.iter().zip(&b.origin) {
            budget.charge(1)?;
            delta.push(old - new);
        }
        let mut matrix = Vec::with_capacity(n);
        let mut origin = Vec::with_capacity(n);
        for row in &b.from_source {
            budget.charge(n)?;
            origin.push(Real::sum_products(row, &delta)?);
            budget.charge(1)?;
            let mut values = Vec::with_capacity(n);
            for column in 0..n {
                budget.charge(n)?;
                let axis: Vec<_> = a.to_source.iter().map(|r| r[column].clone()).collect();
                budget.charge(n)?;
                values.push(Real::sum_products(row, &axis)?);
            }
            matrix.push(values);
        }
        budget.charge(n)?;
        let mut builder = Builder {
            definition: ExpressionDefinition {
                variables: a.variables.clone(),
                nodes: Vec::new(),
                outputs: Vec::new(),
            },
            outputs: n,
            budget,
        };
        let mut inputs = Vec::with_capacity(n);
        for &variable in &a.variables {
            inputs.push(builder.push(ExpressionNode::Variable(SymbolRef::new(variable, None)))?);
        }
        for (row, offset) in matrix.iter().zip(&origin) {
            let root = affine_row(&mut builder, row, &inputs, offset)?;
            builder.budget.charge(1)?;
            builder.definition.outputs.push(root);
        }
        let program = ExpressionProgram::from_definition(builder.definition, builder.budget)?;
        budget.charge(1)?;
        let transition = CoordinateTransition(Arc::new(TransitionData {
            from: self.clone(),
            to: target.clone(),
            matrix,
            origin,
            program,
        }));
        budget.charge(0)?;
        Ok(transition)
    }
}
