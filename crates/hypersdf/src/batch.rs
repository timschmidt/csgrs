//! Report-bearing prepared-batch classification metadata.
//!
//! Batch APIs in this crate are scheduling surfaces, not alternate geometry
//! semantics. They therefore carry explicit dispatch and cache-payoff metadata
//! while every topology decision remains the same exact predicate replay used
//! by scalar classification. This follows Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7.1-2 (1997): performance packages
//! may organize arithmetic work, but certified decisions remain report facts.

use crate::status::{SdfCellClassificationReport, SdfFreshness, SdfPointClassificationReport};

/// Dispatch path used by a prepared batch query.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfBatchDispatch {
    /// Scalar replay over one retained prepared expression.
    ScalarReplay,
    /// A future vectorized path may fill this label without changing results.
    VectorizedReplay,
    /// A future parallel path may fill this label without changing results.
    ParallelReplay,
}

/// Cache/payoff metadata for a prepared batch query.
///
/// These counters are scheduling evidence only. They let callers decide whether
/// retained facts amortize preparation costs, but they are not predicate
/// certificates and must not be used to accept topology.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SdfCachePayoffReport {
    /// Number of query records evaluated by the batch call.
    pub query_count: usize,
    /// Retained expression nodes reused from the prepared handle.
    pub retained_node_count: usize,
    /// Retained analytic primitive nodes reused from the prepared handle.
    pub retained_primitive_count: usize,
    /// Retained transform nodes reused from the prepared handle.
    pub retained_transform_count: usize,
    /// Number of structural fact builds avoided by reusing the prepared handle.
    pub avoided_fact_rebuild_count: usize,
}

impl SdfCachePayoffReport {
    /// Build payoff metadata from retained structural counts and query count.
    pub const fn new(
        query_count: usize,
        retained_node_count: usize,
        retained_primitive_count: usize,
        retained_transform_count: usize,
    ) -> Self {
        Self {
            query_count,
            retained_node_count,
            retained_primitive_count,
            retained_transform_count,
            avoided_fact_rebuild_count: query_count.saturating_sub(1),
        }
    }

    /// Validate summary fields without making a topology claim.
    pub const fn is_self_consistent(&self) -> bool {
        self.avoided_fact_rebuild_count == self.query_count.saturating_sub(1)
    }
}

/// Report returned by prepared point-batch classification.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfPointBatchClassificationReport {
    /// Dispatch path selected for this batch.
    pub dispatch: SdfBatchDispatch,
    /// Prepared-handle cache/payoff metadata.
    pub cache_payoff: SdfCachePayoffReport,
    /// Prepared-source freshness shared by all item reports.
    pub freshness: SdfFreshness,
    /// Per-point scalar-replay reports.
    pub reports: Vec<SdfPointClassificationReport>,
}

impl SdfPointBatchClassificationReport {
    /// Validate internal counts and item freshness.
    pub fn is_self_consistent(&self) -> bool {
        self.cache_payoff.is_self_consistent()
            && self.cache_payoff.query_count == self.reports.len()
            && self
                .reports
                .iter()
                .all(|report| report.freshness == self.freshness && report.is_self_consistent())
    }
}

/// Report returned by prepared cell-batch classification.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfCellBatchClassificationReport {
    /// Dispatch path selected for this batch.
    pub dispatch: SdfBatchDispatch,
    /// Prepared-handle cache/payoff metadata.
    pub cache_payoff: SdfCachePayoffReport,
    /// Prepared-source freshness shared by all item reports.
    pub freshness: SdfFreshness,
    /// Per-cell scalar-replay reports.
    pub reports: Vec<SdfCellClassificationReport>,
}

impl SdfCellBatchClassificationReport {
    /// Validate internal counts and item freshness.
    pub fn is_self_consistent(&self) -> bool {
        self.cache_payoff.is_self_consistent()
            && self.cache_payoff.query_count == self.reports.len()
            && self
                .reports
                .iter()
                .all(|report| report.freshness == self.freshness && report.is_self_consistent())
    }
}
