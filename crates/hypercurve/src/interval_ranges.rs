//! Native curve sources with shared dyadic range and regularity evidence.
use std::fmt;

use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};
use hyperreal::Real;
use hypersolve::{
    SymbolId, SymbolRef,
    expression_program::{
        BinaryOperation, ExpressionBudget, ExpressionDefinition, ExpressionError, ExpressionNode,
        ExpressionProgram, JacobianProgram, RegularJacobianEnclosure,
    },
};

use crate::{Aabb2, CurveError, Point2, RationalBezier2};

/// Native source conversion, invalid local domain, or original resource diagnostic.
#[derive(Debug)]
pub enum CurveRangeError {
    /// Native homogeneous source conversion failed.
    Source(CurveError),
    /// A nonempty query extends outside this source's local unit chart.
    OutsideUnitDomain,
    /// Original graph, differentiation, domain or resource diagnostic.
    Expression(ExpressionError),
    /// Original interval validation, arithmetic or resource diagnostic.
    Interval(IntervalError),
}
impl fmt::Display for CurveRangeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "rational curve range: {self:?}")
    }
}
impl std::error::Error for CurveRangeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Source(error) => Some(error),
            Self::Expression(error) => Some(error),
            Self::Interval(error) => Some(error),
            Self::OutsideUnitDomain => None,
        }
    }
}
impl From<ExpressionError> for CurveRangeError {
    fn from(error: ExpressionError) -> Self {
        Self::Expression(error)
    }
}
impl From<IntervalError> for CurveRangeError {
    fn from(error: IntervalError) -> Self {
        Self::Interval(error)
    }
}

/// Exact native source and a reusable coordinate/derivative expression program.
///
/// This is preparation, not a curve-range certificate. The original homogeneous
/// controls and lineage remain shared through the native immutable carrier.
/// The graph represents X(t)/W(t), Y(t)/W(t), retaining denominator guards.
#[derive(Clone, Debug)]
pub struct PreparedRationalBezierRanges {
    source: RationalBezier2,
    jacobian: JacobianProgram,
}

/// Whole-domain coordinate and tangent enclosures with their native source.
///
/// Coordinates enclose every point of the source at the retained parameters;
/// tangents enclose derivatives with respect to that source's local parameter.
/// This establishes neither an exact range hull nor topology or source fidelity
/// beyond the supplied native curve. Empty domains yield empty numerical ranges.
/// No serialized certificate or cached flag is accepted as replay authority.
#[derive(Clone, Debug)]
pub struct RationalBezierRangeEnclosure {
    source: RationalBezier2,
    evidence: RegularJacobianEnclosure,
}

/// A native broad-phase box borrowing the range evidence that justifies it.
///
/// Empty parameter sets have no finite box. For a nonempty set, non-overlap with
/// another box excludes this curve image; overlap does not prove intersection.
/// Existing Aabb2 predicates retain their own certainty and failure semantics.
#[derive(Debug)]
pub struct CurveRangeBox<'a> {
    evidence: &'a RationalBezierRangeEnclosure,
    bounds: Option<Aabb2>,
}
impl CurveRangeBox<'_> {
    /// Original native curve, selected parameter domain and regularity evidence.
    pub fn evidence(&self) -> &RationalBezierRangeEnclosure {
        self.evidence
    }
    /// Native ordered box, or None for an empty parameter set.
    pub fn bounds(&self) -> Option<&Aabb2> {
        self.bounds.as_ref()
    }
}

/// Prepare repeated range queries for an existing exact native rational span.
///
/// Arbitrary finite degree, algebraic controls and generated native spans retain
/// their exact definitions. Existing native homogeneous basis conversion is
/// reused; the adapter builds no independent geometry or arithmetic kernel.
/// The graph budget reserves `6*n*n + 6*n` structural conversion units, where n
/// is the number of homogeneous controls, even if the native basis is cached.
/// It also charges emitted nodes, validation and differentiation. Entry checks
/// precede conversion and allocation. Scalar arithmetic, coefficient growth,
/// payload cloning/destruction and total heap bytes remain outside this budget.
/// Cancellation cannot interrupt native scalar work or basis conversion; it is
/// checked before and after conversion and before publishing the prepared result.
/// A failed attempt may warm the native exact basis cache, never cache failure.
pub fn prepare_rational_bezier_ranges(
    source: &RationalBezier2,
    graph: &mut ExpressionBudget,
) -> Result<PreparedRationalBezierRanges, CurveRangeError> {
    graph.charge(0)?;
    let n = source.homogeneous_controls().len();
    let nodes = n.checked_mul(9).ok_or(ExpressionError::SizeExhausted)?;
    let conversion_entries = n.checked_mul(6).ok_or(ExpressionError::SizeExhausted)?;
    graph.check_entries(&[1, nodes, 2, conversion_entries])?;
    let conversion_work = n
        .checked_mul(n)
        .and_then(|v| v.checked_mul(6))
        .and_then(|v| v.checked_add(conversion_entries))
        .ok_or(ExpressionError::SizeExhausted)?;
    graph.charge(conversion_work)?;
    let basis = source
        .homogeneous_power_basis()
        .map_err(CurveRangeError::Source)?;
    graph.charge(1)?;
    let parameter = SymbolId(0);
    let mut definition = ExpressionDefinition {
        variables: vec![parameter],
        nodes: vec![ExpressionNode::Variable(SymbolRef::new(parameter, None))],
        outputs: Vec::new(),
    };
    let x = append_polynomial(&basis.x_numerator, &mut definition.nodes, graph)?;
    let y = append_polynomial(&basis.y_numerator, &mut definition.nodes, graph)?;
    let weight = append_polynomial(&basis.weight, &mut definition.nodes, graph)?;
    for numerator in [x, y] {
        graph.charge(2)?;
        definition.outputs.push(definition.nodes.len());
        definition.nodes.push(ExpressionNode::Binary {
            operation: BinaryOperation::Div,
            left: numerator,
            right: weight,
        });
    }
    let program = ExpressionProgram::from_definition(definition, graph)?;
    let jacobian = program.jacobian(&[parameter], graph)?;
    graph.charge(1)?;
    let prepared = PreparedRationalBezierRanges {
        source: source.clone(),
        jacobian,
    };
    graph.charge(0)?;
    Ok(prepared)
}

// Encode the native ascending power coefficients. Numerical evaluation and
// differentiation remain in the shared expression program, not this adapter.
fn append_polynomial(
    coefficients: &[Real],
    nodes: &mut Vec<ExpressionNode>,
    graph: &mut ExpressionBudget,
) -> Result<usize, ExpressionError> {
    let (leading, rest) = coefficients
        .split_last()
        .expect("native homogeneous power basis is nonempty");
    graph.charge(1)?;
    let mut value = nodes.len();
    nodes.push(ExpressionNode::Constant(leading.clone()));
    for coefficient in rest.iter().rev() {
        graph.charge(3)?;
        let product = nodes.len();
        nodes.push(ExpressionNode::Binary {
            operation: BinaryOperation::Mul,
            left: value,
            right: 0,
        });
        let constant = nodes.len();
        nodes.push(ExpressionNode::Constant(coefficient.clone()));
        value = nodes.len();
        nodes.push(ExpressionNode::Binary {
            operation: BinaryOperation::Add,
            left: product,
            right: constant,
        });
    }
    Ok(value)
}

impl PreparedRationalBezierRanges {
    /// Native exact source, including retained subdivision lineage.
    pub fn source(&self) -> &RationalBezier2 {
        &self.source
    }
    /// Original coordinate graph and generated derivative association.
    pub fn program(&self) -> &JacobianProgram {
        &self.jacobian
    }
    /// Enclose coordinates and local tangents on a dyadic subset of [0,1].
    ///
    /// Precision selects an enclosure grid, not a final-width guarantee. All
    /// nested graph/native work shares these budgets. Exact scalar refinement
    /// retains the expression evaluator's accounting exclusions. A denominator
    /// enclosure containing zero refuses certification even if the actual curve
    /// is regular; it does not assert a pole. Subdivision or a larger arithmetic
    /// budget can be retried without mutating this prepared object. Source and
    /// full-domain regularity evidence survive as part of every successful result.
    pub fn enclose(
        &self,
        parameters: &DyadicInterval,
        precision: i32,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<RationalBezierRangeEnclosure, CurveRangeError> {
        graph.charge(0)?;
        let unit = DyadicInterval::new(0.into(), 1.into(), 0)?;
        if !parameters.subset_of(&unit, arithmetic)? {
            return Err(CurveRangeError::OutsideUnitDomain);
        }
        let evidence = self.jacobian.enclose_regular(
            std::slice::from_ref(parameters),
            precision,
            graph,
            arithmetic,
        )?;
        graph.charge(1)?;
        let result = RationalBezierRangeEnclosure {
            source: self.source.clone(),
            evidence,
        };
        graph.charge(0)?;
        Ok(result)
    }
}
impl RationalBezierRangeEnclosure {
    /// Project into the existing native broad phase, retaining borrowed evidence.
    ///
    /// Dyadic bounds convert exactly to rational Real endpoints. Ordered interval
    /// invariants justify native box ordering without a new predicate kernel.
    /// Copies/conversions share the supplied budgets; Real payload construction
    /// retains the scalar accounting exclusions. Cancellation and resource failure
    /// publish no partial box and do not mutate this report. No approximate point
    /// or fitted curve is constructed from these enclosure endpoints.
    pub fn native_bounds(
        &self,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<CurveRangeBox<'_>, CurveRangeError> {
        graph.charge(0)?;
        graph.check_entries(&[4])?;
        graph.charge(1)?;
        let bounds = if self.coordinates()[0].is_empty() {
            None
        } else {
            graph.charge(2)?;
            let [x, y] = self.coordinates();
            let [(x_min, x_max), (y_min, y_max)] =
                [x.to_rational(arithmetic)?, y.to_rational(arithmetic)?].map(|coordinate| {
                    coordinate
                        .into_bounds()
                        .expect("nonempty regular coordinate enclosure")
                });
            graph.charge(4)?;
            Some(Aabb2::new_unchecked(
                Point2::new(Real::new(x_min), Real::new(y_min)),
                Point2::new(Real::new(x_max), Real::new(y_max)),
            ))
        };
        graph.charge(0)?;
        Ok(CurveRangeBox {
            evidence: self,
            bounds,
        })
    }

    /// Original native source; numerical enclosures never replace it.
    pub fn source(&self) -> &RationalBezier2 {
        &self.source
    }
    /// Exact dyadic local parameter set certified by this report.
    pub fn parameters(&self) -> &DyadicInterval {
        &self.evidence.arguments()[0]
    }
    /// Coordinate enclosures in x,y order.
    pub fn coordinates(&self) -> [&DyadicInterval; 2] {
        [&self.evidence.values()[0], &self.evidence.values()[1]]
    }
    /// Local parameter derivative enclosures in x,y order.
    pub fn tangents(&self) -> [&DyadicInterval; 2] {
        std::array::from_fn(|row| {
            &self.evidence.values()[self
                .evidence
                .program()
                .partial_output(row, 0)
                .expect("prepared two-output one-parameter Jacobian")]
        })
    }
    /// Original source-domain continuity and C1 certificate, including precision.
    pub fn evidence(&self) -> &RegularJacobianEnclosure {
        &self.evidence
    }
}

#[cfg(test)]
mod tests;
