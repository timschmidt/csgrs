//! Independent outer noise coordinates, selected by certified contribution widths.
use super::*;
use core::cmp::Ordering;

/// A retained shared monomial, treated as a bounded independent parameter only
/// when projecting an enclosure. This does not assert functional independence.
#[derive(Clone, Debug)]
pub struct TaylorNoiseColumn {
    term: usize,
    domain: DyadicInterval,
    variation: DyadicInterval,
}
impl TaylorNoiseColumn {
    /// Index into the retained source's `terms()`. Identity is source-relative;
    /// equal bounds never establish equality of distinct monomials or residuals.
    pub fn term_index(&self) -> usize {
        self.term
    }
    /// Certified full-domain range of the original centered monomial.
    pub fn domain(&self) -> &DyadicInterval {
        &self.domain
    }
    /// Maximum width of its coefficient-times-monomial enclosure over outputs.
    /// A certified variation upper bound, not an exact derivative or importance.
    pub fn variation(&self) -> &DyadicInterval {
        &self.variation
    }
}

/// A bounded-column outer representation of an unchanged exact Taylor source.
///
/// Constants remain exact. Selected monomials retain their shared exact output
/// coefficient vectors. For enclosure, each monomial ranges independently over
/// its certified interval; original functional dependencies remain in `source`.
/// Every omitted contribution is added to its original output's residual bound.
/// Omission residuals remain distinct, even when their bounds agree. The original
/// Taylor residual coordinates retain their exact affine weights separately, so
/// known duplication/cancellation of an original residual is not lost.
///
/// Selection uses descending certified variation bounds, with original canonical
/// term order breaking ties. `maximum_noise` limits retained nonconstant columns,
/// not the original source or residual count. Zero retains only the constants
/// and full residual boxes. Queries combine exact weights before projecting the
/// selected columns, and intersect with the independently projected source image.
/// No intermediate box is adopted as a replacement exact source.
///
/// Replay the original Taylor definition/request, affine map and noise capacity.
/// This private proof is not deserialized. The fixed original domain cannot be
/// extrapolated. Construction uses O(T*K) bounded selection work and O(T+K+N+D)
/// structural scratch for T terms, K retained columns, N outputs and D inputs.
/// Graph/native budgets cover loops, entries, comparisons and interval arithmetic;
/// inherited exact-Real payload/refinement, heap-byte and in-scalar cancellation
/// exclusions remain. Shared source storage is retained, not counted as a copy.
/// Explicit clones are caller-owned work, outside construction/query budgets.
#[derive(Clone, Debug)]
pub struct TaylorNoiseModel {
    source: Arc<TaylorAffineModel>,
    maximum_noise: usize,
    constant: usize,
    columns: Vec<TaylorNoiseColumn>,
    discarded: Vec<usize>,
    discarded_remainder: Vec<DyadicInterval>,
    remainder: Vec<DyadicInterval>,
}
impl TaylorNoiseModel {
    /// Prepare a certified outer representation without changing the exact source.
    /// Failure leaves the shared source unchanged and available for a fresh retry.
    pub fn new(
        source: Arc<TaylorAffineModel>,
        maximum_noise: usize,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<Self, TaylorAffineError> {
        graph.charge(0)?;
        let terms = source.terms.len();
        let outputs = source.map.rows.len();
        let capacity = maximum_noise.min(terms);
        let request = source.source.request();
        graph.check_entries(&[
            terms,
            terms,
            terms,
            terms,
            terms,
            capacity,
            capacity,
            capacity,
            capacity,
            outputs,
            outputs,
            request.domain.len(),
            1,
        ])?;
        if source.range.is_empty() {
            graph.charge(0)?;
            return Ok(Self {
                source,
                maximum_noise,
                constant: 0,
                columns: Vec::new(),
                discarded: Vec::new(),
                discarded_remainder: Vec::new(),
                remainder: Vec::new(),
            });
        }
        let shifts = model::shifts(&request.domain, &request.center, graph, arithmetic)?;
        let one = DyadicInterval::point(1.into(), 0)?;
        let mut constant = 0;
        let mut candidates = Vec::new();
        for (index, term) in source.terms.iter().enumerate() {
            graph.charge(1)?;
            if term.powers().is_empty() {
                constant = index;
                continue;
            }
            let (_, domain) = model::contribution(
                &term.monomial,
                &one,
                &shifts,
                source.source.source().definition(),
                request.precision,
                graph,
                arithmetic,
            )?;
            let mut variation = model::zero()?;
            for coefficient in &term.coefficients {
                graph.charge(1)?;
                let width = DyadicInterval::from_real(coefficient, request.precision, arithmetic)?
                    .mul(&domain, arithmetic)?
                    .width(arithmetic)?;
                if width.compare_singleton(&variation, arithmetic)? == Ordering::Greater {
                    variation = width;
                }
            }
            candidates.push(Some(TaylorNoiseColumn {
                term: index,
                domain,
                variation,
            }));
        }
        // Fallible exact comparisons share both budgets. Never use a guessed
        // ordering or an infallible sort closure when comparison can exhaust.
        let mut selected: Vec<usize> = Vec::new();
        if capacity != 0 {
            for (index, candidate) in candidates.iter().enumerate() {
                graph.charge(1)?;
                let candidate = candidate.as_ref().expect("not yet moved");
                let mut position = selected.len();
                for (slot, previous) in selected.iter().enumerate() {
                    graph.charge(1)?;
                    let previous = candidates[*previous].as_ref().expect("not yet moved");
                    if candidate
                        .variation
                        .compare_singleton(&previous.variation, arithmetic)?
                        == Ordering::Greater
                    {
                        position = slot;
                        break;
                    }
                }
                if position < capacity {
                    // Include element moves in structural work before mutation.
                    graph.charge(selected.len() - position)?;
                    graph.charge(1)?;
                    selected.insert(position, index);
                    selected.truncate(capacity);
                }
            }
        }
        let mut columns = Vec::new();
        for index in selected {
            graph.charge(1)?;
            columns.push(candidates[index].take().expect("selected once"));
        }
        let mut discarded_remainder = Vec::new();
        for _ in &source.remainder {
            graph.charge(1)?;
            discarded_remainder.push(model::zero()?);
        }
        let mut discarded = Vec::new();
        for candidate in candidates {
            graph.charge(1)?;
            let Some(candidate) = candidate else { continue };
            for (bound, coefficient) in discarded_remainder
                .iter_mut()
                .zip(&source.terms[candidate.term].coefficients)
            {
                graph.charge(1)?;
                let contribution =
                    DyadicInterval::from_real(coefficient, request.precision, arithmetic)?
                        .mul(&candidate.domain, arithmetic)?;
                *bound = bound.add(&contribution, arithmetic)?;
            }
            discarded.push(candidate.term);
        }
        let mut remainder = Vec::new();
        for (original, omitted) in source.remainder.iter().zip(&discarded_remainder) {
            graph.charge(1)?;
            remainder.push(original.add(omitted, arithmetic)?);
        }
        graph.charge(0)?;
        Ok(Self {
            source,
            maximum_noise,
            constant,
            columns,
            discarded,
            discarded_remainder,
            remainder,
        })
    }
    /// Original exact source, including all coefficients, guards and residual proof.
    pub fn source(&self) -> &Arc<TaylorAffineModel> {
        &self.source
    }
    /// Requested capacity; the actual retained count may be smaller.
    pub fn maximum_noise(&self) -> usize {
        self.maximum_noise
    }
    /// Selected shared columns in descending variation order, with stable ties.
    pub fn columns(&self) -> &[TaylorNoiseColumn] {
        &self.columns
    }
    /// Original term indices whose full contributions were added to the residual.
    /// This omits constants and preserves original source order.
    pub fn discarded_terms(&self) -> &[usize] {
        &self.discarded
    }
    /// Every omitted monomial contribution, in affine source output order.
    /// These bounds do not include the separately retained original residuals.
    pub fn discarded_remainder(&self) -> &[DyadicInterval] {
        &self.discarded_remainder
    }
    /// Cartesian projection of original residuals plus every discarded contribution.
    /// Queries retain original residual identities through the exact source map;
    /// this projection does not replace that map. Empty sources have no claim.
    pub fn remainder(&self) -> &[DyadicInterval] {
        &self.remainder
    }
    /// Certify an affine observation over the original domain. The map's inputs
    /// refer to source outputs. This query does not mutate or reselect the model.
    /// Invalid shapes fail before empty shortcuts; no partial result is published.
    pub fn enclose(
        &self,
        map: &TaylorAffineMap,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<TaylorAffineRange, TaylorAffineError> {
        let entries = map.validate(self.source.map.rows.len(), graph)?;
        graph.check_entries(&[entries, map.rows.len(), self.source.map.rows.len()])?;
        graph.charge(map.rows.len())?;
        if self.source.range.is_empty() {
            graph.charge(0)?;
            return Ok(TaylorAffineRange {
                bounds: vec![DyadicInterval::empty(); map.rows.len()],
                empty: true,
            });
        }
        let precision = self.source.source.request().precision;
        let mut bounds = Vec::new();
        for (output, row) in map.rows.iter().enumerate() {
            graph.charge(row.coefficients.len())?;
            let constant = Real::sum_products(
                &row.coefficients,
                &self.source.terms[self.constant].coefficients,
            )
            .map_err(TaylorAffineError::Scalar)?
                + &row.offset;
            let mut value = DyadicInterval::from_real(&constant, precision, arithmetic)?;
            for column in &self.columns {
                graph.charge(row.coefficients.len())?;
                graph.charge(1)?;
                let coefficient = Real::sum_products(
                    &row.coefficients,
                    &self.source.terms[column.term].coefficients,
                )
                .map_err(TaylorAffineError::Scalar)?;
                let weight = DyadicInterval::from_real(&coefficient, precision, arithmetic)?;
                value = value.add(&weight.mul(&column.domain, arithmetic)?, arithmetic)?;
            }
            for (column, residual) in self.source.source.remainder().iter().enumerate() {
                graph.charge(1)?;
                if residual.signs() == hyperinterval::SignSet::Zero {
                    continue;
                }
                graph.charge(row.coefficients.len())?;
                let weights: Vec<_> = self
                    .source
                    .map
                    .rows
                    .iter()
                    .map(|source_row| source_row.coefficients[column].clone())
                    .collect();
                let coefficient = Real::sum_products(&row.coefficients, &weights)
                    .map_err(TaylorAffineError::Scalar)?;
                let weight = DyadicInterval::from_real(&coefficient, precision, arithmetic)?;
                value = value.add(&weight.mul(residual, arithmetic)?, arithmetic)?;
            }
            value = value.add(
                &bound_row(
                    row,
                    &self.discarded_remainder,
                    false,
                    precision,
                    graph,
                    arithmetic,
                )?,
                arithmetic,
            )?;
            let independent = bound_row(
                row,
                self.source.range.bounds(),
                true,
                precision,
                graph,
                arithmetic,
            )?;
            value = value.intersection(&independent, arithmetic)?;
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
