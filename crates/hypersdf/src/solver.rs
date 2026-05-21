//! Solver and projection proposal replay reports.
//!
//! Iterative closest-point projection, ray/level-set intersection, fitting, and
//! calibration are proposal mechanisms. `hypersdf` records those proposals and
//! replays their candidates through exact/certified field classification
//! instead of hiding numeric iteration inside topology decisions. This follows
//! Yap, "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997), and matches the Hyper stack split where `hypersolve` owns residual
//! construction and nonlinear iteration while geometry crates own replayable
//! evidence.

use hyperlimit::Point3;
use hyperreal::Real;

use crate::status::{
    SdfFreshness, SdfMetricStatus, SdfPointClassificationReport, SdfPointLocation,
};

/// Kind of external proposal being replayed through `hypersdf`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfProjectionProposalKind {
    /// Closest-point projection candidate.
    ClosestPoint,
    /// Ray or segment level-set intersection candidate.
    LevelSetIntersection,
    /// Field fitting or calibration candidate.
    Fitting,
    /// Caller-defined proposal kind.
    External,
}

/// Acceptance status after exact/certified replay.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfProjectionReplayStatus {
    /// Candidate was replayed on the zero level set.
    BoundaryCertified,
    /// Candidate replayed strictly inside or outside, so it is not accepted as
    /// a zero-level projection.
    RejectedByClassification,
    /// Candidate classification was unknown under the current exact evidence.
    Unknown,
}

/// Caller-supplied projection/intersection/fitting candidate.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfProjectionProposal {
    /// Human-readable adapter or solver source.
    pub source: String,
    /// Proposal kind.
    pub kind: SdfProjectionProposalKind,
    /// Original query point.
    pub query: Point3,
    /// Candidate returned by an external proposal route.
    pub candidate: Point3,
}

impl SdfProjectionProposal {
    /// Construct a proposal from an external adapter or solver.
    pub fn new(
        source: impl Into<String>,
        kind: SdfProjectionProposalKind,
        query: Point3,
        candidate: Point3,
    ) -> Self {
        Self {
            source: source.into(),
            kind,
            query,
            candidate,
        }
    }
}

/// Exact replay report for a projection candidate.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfProjectionReplayReport {
    /// Original proposal.
    pub proposal: SdfProjectionProposal,
    /// Classification of the candidate point.
    pub candidate_report: SdfPointClassificationReport,
    /// Squared displacement from query to candidate.
    pub displacement_squared: Real,
    /// Metric claim of the source field.
    pub metric_status: SdfMetricStatus,
    /// Prepared-source freshness.
    pub freshness: SdfFreshness,
    /// Acceptance status after replay.
    pub status: SdfProjectionReplayStatus,
}

impl SdfProjectionReplayReport {
    /// Build a replay report from a proposal and exact candidate
    /// classification.
    pub fn from_candidate_report(
        proposal: SdfProjectionProposal,
        candidate_report: SdfPointClassificationReport,
        metric_status: SdfMetricStatus,
        freshness: SdfFreshness,
    ) -> Self {
        let displacement_squared = squared_distance3(&proposal.query, &proposal.candidate);
        let status = match candidate_report.location {
            SdfPointLocation::Boundary => SdfProjectionReplayStatus::BoundaryCertified,
            SdfPointLocation::Inside | SdfPointLocation::Outside => {
                SdfProjectionReplayStatus::RejectedByClassification
            }
            SdfPointLocation::Unknown => SdfProjectionReplayStatus::Unknown,
        };
        Self {
            proposal,
            candidate_report,
            displacement_squared,
            metric_status,
            freshness,
            status,
        }
    }

    /// Validate replay status against the contained candidate classification.
    pub fn is_self_consistent(&self) -> bool {
        self.candidate_report.is_self_consistent()
            && self.status
                == match self.candidate_report.location {
                    SdfPointLocation::Boundary => SdfProjectionReplayStatus::BoundaryCertified,
                    SdfPointLocation::Inside | SdfPointLocation::Outside => {
                        SdfProjectionReplayStatus::RejectedByClassification
                    }
                    SdfPointLocation::Unknown => SdfProjectionReplayStatus::Unknown,
                }
    }
}

fn squared_distance3(a: &Point3, b: &Point3) -> Real {
    let dx = &a.x - &b.x;
    let dy = &a.y - &b.y;
    let dz = &a.z - &b.z;
    let dx2 = &dx * &dx;
    let dy2 = &dy * &dy;
    let dz2 = &dz * &dz;
    &(&dx2 + &dy2) + &dz2
}
