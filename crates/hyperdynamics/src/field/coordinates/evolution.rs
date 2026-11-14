//! Whole-horizon evolution in one retained exact state chart.
use super::{CoordinateField, CoordinateFieldError};
use crate::field::{
    EnclosureBudget,
    evolution::{
        EvolutionFailure, EvolutionOptions, EvolutionReport, EvolutionRequest, EvolutionStatus,
    },
    picard::PicardReport,
};
use hyperinterval::DyadicInterval;
use hyperlattice_interval::BoxN;
use hypersolve::expression_program::ExpressionError;

impl<const N: usize> CoordinateField<N> {
    /// Integrate entirely in this fixed chart, then transport every accepted tube/end.
    ///
    /// The original initial set is exactly `c + A * request.initial`. Its Cartesian
    /// projection is only an outer enclosure, never the input to the next step.
    /// Native interval overestimation still accumulates between steps; this is not
    /// an exact reachable-set representation or an adaptive change of frame.
    ///
    /// The same budget covers native integration and all output transport. A
    /// stopped native run keeps its qualified prefix in `native()`. Original-
    /// coordinate output is published atomically, only on whole-horizon success.
    pub fn integrate(
        &self,
        request: EvolutionRequest<N>,
        options: EvolutionOptions,
        budget: &mut EnclosureBudget,
    ) -> CoordinateEvolution<N> {
        let native = self.transformed_field().integrate(request, options, budget);
        let status = match native.status() {
            EvolutionStatus::Completed => CoordinateEvolutionStatus::Completed,
            EvolutionStatus::Identity => CoordinateEvolutionStatus::Identity,
            EvolutionStatus::Empty => CoordinateEvolutionStatus::Empty,
            EvolutionStatus::Stopped(reason) => CoordinateEvolutionStatus::Stopped(reason.clone()),
        };
        let mut result = CoordinateEvolution {
            chart: self.clone(),
            native,
            status,
            source: None,
        };
        if !matches!(result.status, CoordinateEvolutionStatus::Stopped(_)) {
            match result.materialize(budget) {
                Ok(source) => result.source = Some(source),
                Err(error) => result.status = CoordinateEvolutionStatus::TransportFailed(error),
            }
        }
        result
    }
}

/// Whole-horizon outcome, separate from the native integration stage.
#[derive(Clone, Debug)]
pub enum CoordinateEvolutionStatus {
    /// Native integration and all original-coordinate projections completed.
    Completed,
    /// Zero-duration identity with transported initial set; no ODE existence claim.
    Identity,
    /// Validated successful empty evolution.
    Empty,
    /// Original native failure, without a transported partial-output claim.
    Stopped(EvolutionFailure),
    /// Native completion succeeded but original-coordinate materialization failed.
    TransportFailed(CoordinateFieldError),
}

#[derive(Clone, Debug)]
struct SourceEvolution<const N: usize> {
    initial: BoxN<DyadicInterval, N>,
    segments: Vec<(BoxN<DyadicInterval, N>, BoxN<DyadicInterval, N>)>,
}

/// A native certified segment with its exact chart and original-coordinate envelopes.
/// Time is unchanged by the chart; endpoints are in `native().proposal()`.
#[derive(Clone, Copy, Debug)]
pub struct CoordinateSegment<'a, const N: usize> {
    chart: &'a CoordinateField<N>,
    native: &'a PicardReport<N>,
    tube: &'a BoxN<DyadicInterval, N>,
    end: &'a BoxN<DyadicInterval, N>,
}
impl<'a, const N: usize> CoordinateSegment<'a, N> {
    /// Exact source/transformed-field association and inverse evidence.
    pub fn chart(&self) -> &'a CoordinateField<N> {
        self.chart
    }
    /// Retained source-bound native proof, including exact time and state enclosures.
    pub fn native(&self) -> &'a PicardReport<N> {
        self.native
    }
    /// Cartesian outer enclosure of the original trajectory over this segment.
    pub fn source_tube(&self) -> &'a BoxN<DyadicInterval, N> {
        self.tube
    }
    /// Cartesian outer enclosure at this segment's exact endpoint.
    pub fn source_end_enclosure(&self) -> &'a BoxN<DyadicInterval, N> {
        self.end
    }
}

/// Immutable evolution retaining the chart, all native attempts and transported output.
///
/// Replay the coordinate field definition and native request/options to regenerate
/// proofs. No serialized status or projection is trusted as a certificate.
/// Output allocation checks count native attempt/index records, source segment
/// records, projected box lanes and transform temporaries together. These are
/// structural entry bounds, not heap-byte limits; scalar payloads, vector capacity
/// and refinement retain the exclusions of [`EnclosureBudget`].
#[derive(Clone, Debug)]
pub struct CoordinateEvolution<const N: usize> {
    chart: CoordinateField<N>,
    native: EvolutionReport<N>,
    status: CoordinateEvolutionStatus,
    source: Option<SourceEvolution<N>>,
}
impl<const N: usize> CoordinateEvolution<N> {
    /// Retained original/transformed field association and exact chart.
    pub fn chart(&self) -> &CoordinateField<N> {
        &self.chart
    }
    /// All native attempts, policy, search options, status and qualified prefix.
    pub fn native(&self) -> &EvolutionReport<N> {
        &self.native
    }
    /// Whole-horizon status; native completion alone is insufficient.
    pub fn status(&self) -> &CoordinateEvolutionStatus {
        &self.status
    }
    /// Cartesian envelope of the correlated original initial set, after all transport.
    pub fn source_initial_enclosure(&self) -> Option<&BoxN<DyadicInterval, N>> {
        self.source.as_ref().map(|source| &source.initial)
    }
    /// Original-coordinate endpoint after whole-request success; absent on failure.
    pub fn source_end_enclosure(&self) -> Option<&BoxN<DyadicInterval, N>> {
        self.source.as_ref().map(|source| {
            source
                .segments
                .last()
                .map_or(&source.initial, |(_, end)| end)
        })
    }
    /// All transported segments on completion, otherwise an empty iterator.
    /// The separately qualified native prefix remains available through `native()`.
    pub fn segments(&self) -> impl ExactSizeIterator<Item = CoordinateSegment<'_, N>> {
        let source = self.source.as_ref().map_or(&[][..], |s| &s.segments);
        self.native
            .segments()
            .zip(source)
            .map(|(native, (tube, end))| CoordinateSegment {
                chart: &self.chart,
                native,
                tube,
                end,
            })
    }
    fn materialize(
        &self,
        budget: &mut EnclosureBudget,
    ) -> Result<SourceEvolution<N>, CoordinateFieldError> {
        budget.graph.charge(0)?;
        let count = self.native.segments().len();
        let lanes = count
            .checked_mul(2)
            .and_then(|n| n.checked_add(1))
            .and_then(|n| n.checked_mul(N))
            .ok_or(ExpressionError::SizeExhausted)?;
        let square = N.checked_mul(N).ok_or(ExpressionError::SizeExhausted)?;
        budget.graph.check_entries(&[
            self.native.attempts().len(),
            count,
            count,
            lanes,
            square,
            N,
            N,
            N,
        ])?;
        budget.graph.charge(1)?;
        let initial = self
            .chart
            .source_enclosure(&self.native.request().initial, budget)?;
        let mut segments = Vec::with_capacity(count);
        for native in self.native.segments() {
            budget.graph.charge(1)?;
            let tube = self
                .chart
                .source_enclosure(native.tube().expect("accepted native segment"), budget)?;
            let end = self.chart.source_enclosure(
                native.end_enclosure().expect("accepted native segment"),
                budget,
            )?;
            segments.push((tube, end));
        }
        budget.graph.charge(1)?;
        Ok(SourceEvolution { initial, segments })
    }
}
