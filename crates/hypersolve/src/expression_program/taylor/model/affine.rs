//! Exact output maps over a shared Taylor source and its original residual coordinates.
use super::super::*;
use hyperinterval::{DyadicBudget, DyadicInterval};

/// One exact affine output row. Replay data, not an enclosure certificate.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TaylorAffineRow {
    /// Weights in the immediate input's output order.
    pub coefficients: Vec<Real>,
    /// Exact additive constant.
    pub offset: Real,
}
/// Finite exact output transformation, including rectangular and singular maps.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TaylorAffineMap {
    /// Explicit input dimension, also when there are no output rows.
    pub input_count: usize,
    /// Output rows in order.
    pub rows: Vec<TaylorAffineRow>,
}
/// Invalid map shape or an original scalar, Taylor or resource failure.
#[derive(Clone, Debug)]
pub enum TaylorAffineError {
    /// Map input dimension differs from its source output dimension.
    InputCount { expected: usize, actual: usize },
    /// A row does not have one coefficient per input.
    RowWidth {
        row: usize,
        expected: usize,
        actual: usize,
    },
    /// Original Taylor/domain/resource diagnostic.
    Taylor(TaylorError),
    /// Original exact aggregate diagnostic.
    Scalar(hyperreal::Problem),
}
impl fmt::Display for TaylorAffineError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "affine Taylor form: {self:?}")
    }
}
impl std::error::Error for TaylorAffineError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Taylor(e) => Some(e),
            Self::Scalar(e) => Some(e),
            _ => None,
        }
    }
}
impl From<TaylorError> for TaylorAffineError {
    fn from(e: TaylorError) -> Self {
        Self::Taylor(e)
    }
}
impl From<ExpressionError> for TaylorAffineError {
    fn from(e: ExpressionError) -> Self {
        Self::Taylor(e.into())
    }
}
impl From<hyperinterval::IntervalError> for TaylorAffineError {
    fn from(e: hyperinterval::IntervalError) -> Self {
        Self::Taylor(e.into())
    }
}
impl TaylorAffineMap {
    fn validate(
        &self,
        expected: usize,
        graph: &mut ExpressionBudget,
    ) -> Result<usize, TaylorAffineError> {
        graph.charge(0)?;
        if self.input_count != expected {
            return Err(TaylorAffineError::InputCount {
                expected,
                actual: self.input_count,
            });
        }
        let mut entries = 0usize;
        for (row, value) in self.rows.iter().enumerate() {
            graph.charge(1)?;
            if value.coefficients.len() != expected {
                return Err(TaylorAffineError::RowWidth {
                    row,
                    expected,
                    actual: value.coefficients.len(),
                });
            }
            entries = entries
                .checked_add(expected)
                .and_then(|n| n.checked_add(1))
                .ok_or(ExpressionError::SizeExhausted)?;
        }
        graph.check_entries(&[entries])?;
        Ok(entries)
    }
}
/// A shared centered monomial and its exact coefficients in transformed output order.
#[derive(Clone, Debug)]
pub struct TaylorAffineTerm {
    monomial: CoefficientRequest,
    coefficients: Vec<Real>,
}
impl TaylorAffineTerm {
    /// Sparse powers of original source variables; empty denotes the constant.
    pub fn powers(&self) -> &[(SymbolId, usize)] {
        &self.monomial.powers
    }
    /// Exact coefficients, including the affine translation in the constant term.
    pub fn coefficients(&self) -> &[Real] {
        &self.coefficients
    }
}
/// Certified Cartesian outer enclosure, distinguishing empty from the zero-dimensional unit.
#[derive(Clone, Debug)]
pub struct TaylorAffineRange {
    bounds: Vec<DyadicInterval>,
    empty: bool,
}
impl TaylorAffineRange {
    /// Coordinate bounds; an explicit projection, not a replacement source.
    pub fn bounds(&self) -> &[DyadicInterval] {
        &self.bounds
    }
    /// Whether the input parameter domain was empty, even with zero outputs.
    pub fn is_empty(&self) -> bool {
        self.empty
    }
}
/// Source-bound exact affine image of a Taylor-certified function.
///
/// Represents `A*f(x)+b` on the original certified domain. Exact polynomial
/// coefficients are combined before enclosure. Original residual coordinates
/// remain distinct; their interval bounds do not imply independence or equality.
/// Repeated transforms compose exact weights against the original source before
/// projecting either polynomial or residual. Intermediate boxes are never adopted.
///
/// Replay the source definition/request and the composed `map()`. This retains
/// the final exact transformation, not a history of authored transformation steps.
/// No nonlinear model composition or ODE coordinate certificate is implied.
/// Structural budgets cover new coefficient/map/scratch entries and loops;
/// shared input storage is not copied. Exact Real arithmetic/refinement and
/// payload/heap-byte lifecycle keep TaylorModel's documented exclusions.
/// Entry checks bound individual structural stages, not total live entries or
/// bytes across the retained source and nested enclosure routines.
#[derive(Clone, Debug)]
pub struct TaylorAffineModel {
    source: Arc<TaylorModel>,
    map: TaylorAffineMap,
    terms: Vec<TaylorAffineTerm>,
    remainder: Vec<DyadicInterval>,
    range: TaylorAffineRange,
}
impl TaylorAffineModel {
    /// Validate an exact output map and transport an existing Taylor certificate.
    /// Shape is validated even for empty sources; all work shares the supplied budgets.
    pub fn new(
        source: Arc<TaylorModel>,
        map: TaylorAffineMap,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<Self, TaylorAffineError> {
        let inputs = source.source().definition().outputs.len();
        let map_entries = map.validate(inputs, graph)?;
        graph.check_entries(&[map_entries, map.rows.len(), map.rows.len()])?;
        let mut result = Self {
            source,
            map,
            terms: Vec::new(),
            remainder: Vec::new(),
            range: TaylorAffineRange {
                bounds: Vec::new(),
                empty: false,
            },
        };
        if !result.source.is_empty() {
            // Group by the existing canonical sparse powers, retaining shared
            // monomials across output rows rather than boxing each output first.
            graph.check_entries(&[map_entries, inputs, 1])?;
            graph.charge(inputs + 1)?;
            let mut groups = BTreeMap::from([(Vec::new(), vec![Real::zero(); inputs])]);
            let mut group_entries = inputs + 1;
            for term in result.source.terms() {
                let request = &result.source.coefficient_program().requests()[term.request_index()];
                graph.charge(request.powers.len())?;
                graph.charge(1)?;
                if !groups.contains_key(&request.powers) {
                    group_entries = group_entries
                        .checked_add(inputs)
                        .and_then(|n| n.checked_add(request.powers.len()))
                        .and_then(|n| n.checked_add(1))
                        .ok_or(ExpressionError::SizeExhausted)?;
                    graph.check_entries(&[map_entries, group_entries])?;
                    graph.charge(inputs)?;
                    groups.insert(request.powers.clone(), vec![Real::zero(); inputs]);
                }
                groups.get_mut(&request.powers).expect("inserted group")[request.output] =
                    term.coefficient().clone();
            }
            let output_entries = groups
                .len()
                .checked_mul(result.map.rows.len())
                .ok_or(ExpressionError::SizeExhausted)?;
            graph.check_entries(&[
                map_entries,
                group_entries,
                output_entries,
                result.map.rows.len(),
                result.map.rows.len(),
            ])?;
            for (powers, values) in groups {
                graph.charge(1)?;
                let mut coefficients = Vec::new();
                for row in &result.map.rows {
                    graph.charge(inputs)?;
                    graph.charge(1)?;
                    let coefficient = Real::sum_products(&row.coefficients, &values)
                        .map_err(TaylorAffineError::Scalar)?;
                    let coefficient = if powers.is_empty() {
                        coefficient + &row.offset
                    } else {
                        coefficient
                    };
                    coefficients.push(coefficient);
                }
                result.terms.push(TaylorAffineTerm {
                    monomial: CoefficientRequest { output: 0, powers },
                    coefficients,
                });
            }
            for row in &result.map.rows {
                graph.charge(1)?;
                result.remainder.push(bound_row(
                    row,
                    result.source.remainder(),
                    false,
                    result.source.request().precision,
                    graph,
                    arithmetic,
                )?);
            }
        }
        result.range = result.enclose_on(&result.source.request().domain, graph, arithmetic)?;
        graph.charge(0)?;
        Ok(result)
    }
    /// Original shared certificate, unchanged by every subsequent output map.
    pub fn source(&self) -> &Arc<TaylorModel> {
        &self.source
    }
    /// Exact composed map acting on the original source output coordinates.
    pub fn map(&self) -> &TaylorAffineMap {
        &self.map
    }
    /// Shared centered monomials with exact mapped coefficients.
    pub fn terms(&self) -> &[TaylorAffineTerm] {
        &self.terms
    }
    /// Projection of the original residual coordinates, with no dropped contribution.
    /// Empty sources carry no residual claim.
    pub fn remainder(&self) -> &[DyadicInterval] {
        &self.remainder
    }
    /// Certified image on the original parameter domain.
    pub fn range(&self) -> &TaylorAffineRange {
        &self.range
    }
    /// Apply another exact output map. Compose before projecting original residuals.
    /// Failure leaves this form and its shared source unchanged for retry.
    pub fn transform(
        &self,
        next: TaylorAffineMap,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<Self, TaylorAffineError> {
        let entries = next.validate(self.map.rows.len(), graph)?;
        let inputs = self.map.input_count;
        let outputs = next.rows.len();
        let composed = inputs
            .checked_add(1)
            .and_then(|n| n.checked_mul(outputs))
            .ok_or(ExpressionError::SizeExhausted)?;
        graph.check_entries(&[entries, composed, self.map.rows.len(), self.map.rows.len()])?;
        graph.charge(self.map.rows.len())?;
        let offsets: Vec<_> = self.map.rows.iter().map(|row| row.offset.clone()).collect();
        let mut rows = Vec::new();
        for row in next.rows {
            graph.charge(1)?;
            let mut coefficients = Vec::new();
            for column in 0..inputs {
                graph.charge(self.map.rows.len())?;
                graph.charge(1)?;
                let values: Vec<_> = self
                    .map
                    .rows
                    .iter()
                    .map(|old| old.coefficients[column].clone())
                    .collect();
                coefficients.push(
                    Real::sum_products(&row.coefficients, &values)
                        .map_err(TaylorAffineError::Scalar)?,
                );
            }
            graph.charge(self.map.rows.len())?;
            let offset = Real::sum_products(&row.coefficients, &offsets)
                .map_err(TaylorAffineError::Scalar)?
                + row.offset;
            rows.push(TaylorAffineRow {
                coefficients,
                offset,
            });
        }
        drop(offsets);
        Self::new(
            self.source.clone(),
            TaylorAffineMap {
                input_count: inputs,
                rows,
            },
            graph,
            arithmetic,
        )
    }
    /// Reuse on a subdomain, preserving the original global residual and grid.
    /// Even a zero output map cannot bypass the original domain restriction.
    pub fn enclose_on(
        &self,
        domain: &[DyadicInterval],
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<TaylorAffineRange, TaylorAffineError> {
        // The original owner validates containment, dimensions and empty products.
        // Its image also supplies an independent projected enclosure below.
        let original = self.source.enclose_on(domain, graph, arithmetic)?;
        graph.check_entries(&[self.map.rows.len(), self.map.rows.len(), domain.len()])?;
        graph.charge(self.map.rows.len())?;
        graph.charge(domain.len())?;
        if domain.iter().any(DyadicInterval::is_empty) {
            graph.charge(0)?;
            return Ok(TaylorAffineRange {
                bounds: vec![DyadicInterval::empty(); self.map.rows.len()],
                empty: true,
            });
        }
        let shifts = model::shifts(domain, &self.source.request().center, graph, arithmetic)?;
        let precision = self.source.request().precision;
        let mut bounds = Vec::new();
        for (output, row) in self.map.rows.iter().enumerate() {
            graph.charge(1)?;
            let mut polynomial = model::zero()?;
            for term in &self.terms {
                graph.charge(1)?;
                let coefficient =
                    DyadicInterval::from_real(&term.coefficients[output], precision, arithmetic)?;
                let (_, contribution) = model::contribution(
                    &term.monomial,
                    &coefficient,
                    &shifts,
                    self.source.source().definition(),
                    precision,
                    graph,
                    arithmetic,
                )?;
                polynomial = polynomial.add(&contribution, arithmetic)?;
            }
            let independent = bound_row(row, &original, true, precision, graph, arithmetic)?;
            let value = polynomial
                .add(&self.remainder[output], arithmetic)?
                .intersection(&independent, arithmetic)?;
            if value.is_empty() {
                return Err(TaylorError::InconsistentEnclosure { output }.into());
            }
            bounds.push(value);
        }
        graph.charge(0)?;
        Ok(TaylorAffineRange {
            bounds,
            empty: false,
        })
    }
}
// One projection routine for source image bounds and original residual bounds.
// Exact point/interval arithmetic remains owned by Hyperreal/Hyperinterval.
fn bound_row(
    row: &TaylorAffineRow,
    values: &[DyadicInterval],
    translated: bool,
    precision: i32,
    graph: &mut ExpressionBudget,
    arithmetic: &mut DyadicBudget,
) -> Result<DyadicInterval, TaylorAffineError> {
    assert_eq!(
        row.coefficients.len(),
        values.len(),
        "validated source output shape"
    );
    let mut result = if translated {
        DyadicInterval::from_real(&row.offset, precision, arithmetic)?
    } else {
        model::zero()?
    };
    for (coefficient, value) in row.coefficients.iter().zip(values) {
        graph.charge(1)?;
        let weight = DyadicInterval::from_real(coefficient, precision, arithmetic)?;
        result = result.add(&weight.mul(value, arithmetic)?, arithmetic)?;
    }
    Ok(result)
}

#[cfg(feature = "taylor-polynomial")]
impl TaylorAffineRange {
    // Callers retain the source-domain proof, including empty/unit identity.
    pub(super) fn new(bounds: Vec<DyadicInterval>, empty: bool) -> Self {
        Self { bounds, empty }
    }
    pub(super) fn into_bounds(self) -> Vec<DyadicInterval> {
        self.bounds
    }
}
