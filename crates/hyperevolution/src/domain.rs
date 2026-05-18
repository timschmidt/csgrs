//! Domain replay integration manifests.
//!
//! Memetic search combines global proposal generation with local/domain replay.
//! `hyperevolution` should record which domain owns certification without
//! taking over that domain's semantics. These manifests name replay owners such
//! as `hypersolve`, `hyperpath`, `hypercurve`, `hypervoxel`, `hyperpack`,
//! `hyperdrc`, `hyperphysics`, and `hypercircuit`, then carry the resulting
//! exact/certified/unknown/lossy replay status back into search reports. This
//! preserves Yap's exact-geometric-computation layering: search proposes;
//! domain predicates, residuals, or simulators certify.

use crate::{CandidateId, ConstructionDependency, EvaluationCacheKey, ReplayHook, ReplayStatus};

/// Hyper domain that owns replay for a candidate.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DomainReplayTarget {
    /// `hypersolve` residual/candidate certification.
    Hypersolve,
    /// `hyperpath` route/path certification.
    Hyperpath,
    /// `hypercurve` curve-event/error certification.
    Hypercurve,
    /// `hypermesh` or `csgrs` topology certification.
    MeshOrCsg,
    /// `hypervoxel` voxel/process artifact replay.
    Hypervoxel,
    /// `hyperpack` packing feasibility replay.
    Hyperpack,
    /// `hyperdrc` rule-deck check replay.
    Hyperdrc,
    /// `hyperphysics` physical-property or simulation replay.
    Hyperphysics,
    /// `hypercircuit` circuit residual replay.
    Hypercircuit,
    /// Another named domain owner.
    Other(String),
}

/// Replay manifest for one candidate/domain handoff.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DomainReplayManifest {
    /// Candidate being handed off.
    pub candidate: CandidateId,
    /// Domain owner expected to certify the candidate.
    pub target: DomainReplayTarget,
    /// Stable replay hook.
    pub hook: ReplayHook,
    /// Construction dependencies that must still be fresh for replay.
    pub dependencies: Vec<ConstructionDependency>,
    /// Optional cache key for replay/evaluation reuse.
    pub cache_key: Option<EvaluationCacheKey>,
}

/// Report returned after domain replay.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DomainReplayReport {
    /// Manifest that was replayed.
    pub manifest: DomainReplayManifest,
    /// Domain replay status.
    pub status: ReplayStatus,
    /// Human-readable evidence handles from the domain owner.
    pub evidence: Vec<String>,
}

impl DomainReplayReport {
    /// Returns whether this replay result can promote a candidate into accepted state.
    pub fn is_accepted(&self) -> bool {
        self.status == ReplayStatus::Accepted
    }

    /// Returns whether replay still needs more exact/certified evidence.
    pub fn needs_followup(&self) -> bool {
        self.status == ReplayStatus::Unknown
    }
}

/// Builds a stable replay manifest for a Hyper domain owner.
pub fn domain_replay_manifest(
    candidate: CandidateId,
    target: DomainReplayTarget,
    target_id: impl Into<String>,
) -> DomainReplayManifest {
    let target_text = target_id.into();
    let domain = match &target {
        DomainReplayTarget::Hypersolve => "hypersolve",
        DomainReplayTarget::Hyperpath => "hyperpath",
        DomainReplayTarget::Hypercurve => "hypercurve",
        DomainReplayTarget::MeshOrCsg => "mesh-or-csg",
        DomainReplayTarget::Hypervoxel => "hypervoxel",
        DomainReplayTarget::Hyperpack => "hyperpack",
        DomainReplayTarget::Hyperdrc => "hyperdrc",
        DomainReplayTarget::Hyperphysics => "hyperphysics",
        DomainReplayTarget::Hypercircuit => "hypercircuit",
        DomainReplayTarget::Other(name) => name.as_str(),
    }
    .to_string();
    DomainReplayManifest {
        candidate,
        target,
        hook: ReplayHook {
            domain,
            target: target_text.clone(),
        },
        dependencies: Vec::new(),
        cache_key: Some(EvaluationCacheKey(format!("domain-replay:{target_text}"))),
    }
}
