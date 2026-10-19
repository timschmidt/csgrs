//! Bounded qualifications of a mesh's construction dependencies.

use crate::MeshCertainty;

/// Monotone summary of construction dependencies, separate from local predicate
/// receipts, topology proofs and geometric error bounds.
///
/// This one-byte value retains qualifications, not a replayable proof or a
/// boundary-to-source mapping. `Some(Certified)` from [`Self::predicate_certainty`]
/// concerns decisions since the declared coordinate source only. Sampling and
/// coordinate projection remain separate fidelity boundaries even in that case.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct ConstructionEvidence(u8);

impl Default for ConstructionEvidence {
    fn default() -> Self {
        Self::UNTRACKED
    }
}

impl ConstructionEvidence {
    const MISSING: u8 = 1;
    const APPROXIMATE: u8 = 2;
    const PROJECTED: u8 = 4;
    const SAMPLED: u8 = 8;
    const MUTATED: u8 = 16;

    /// Start a new claim at explicitly declared coordinates. This does not prove
    /// their relationship to any earlier model or intended analytic surface.
    pub const DECLARED_COORDINATES: Self = Self(0);
    /// The preceding construction history is unavailable.
    pub const UNTRACKED: Self = Self(Self::MISSING);

    /// Retain every qualification from both dependency chains.
    pub const fn combine(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    /// Record actual predicate consumption. A later certified operation cannot
    /// clear an earlier approximate decision or fill missing source history.
    pub const fn with_predicate_certainty(self, certainty: MeshCertainty) -> Self {
        match certainty {
            MeshCertainty::Certified => self,
            MeshCertainty::Approximate512Consumed => Self(self.0 | Self::APPROXIMATE),
        }
    }

    /// Record a coordinate projection, independently of predicate certainty.
    /// The boundary API supplies its format and loss model.
    pub const fn with_coordinate_projection(self) -> Self {
        Self(self.0 | Self::PROJECTED)
    }

    /// Record construction from samples; this supplies no continuous-source
    /// fidelity or error-bound claim.
    pub const fn with_sampling(self) -> Self {
        Self(self.0 | Self::SAMPLED)
    }

    pub(crate) const fn after_buffer_mutation(self) -> Self {
        Self(self.0 | Self::MISSING | Self::MUTATED)
    }

    /// Whether any preceding construction history is unavailable.
    pub const fn source_history_missing(self) -> bool {
        self.0 & Self::MISSING != 0
    }

    /// Whether retained evidence records an actual approximate predicate.
    /// False does not certify missing history.
    pub const fn approximate_512_consumed(self) -> bool {
        self.0 & Self::APPROXIMATE != 0
    }

    /// Whether the dependency chain contains a coordinate projection.
    pub const fn coordinates_projected(self) -> bool {
        self.0 & Self::PROJECTED != 0
    }

    /// Whether the dependency chain contains construction from samples.
    pub const fn source_sampled(self) -> bool {
        self.0 & Self::SAMPLED != 0
    }

    /// Whether buffers ceased to match a retained construction identity.
    pub const fn buffers_changed(self) -> bool {
        self.0 & Self::MUTATED != 0
    }

    /// Aggregate predicate certainty when source history is available. This
    /// never certifies topology, geometric fidelity or preceding source intent.
    pub const fn predicate_certainty(self) -> Option<MeshCertainty> {
        if self.source_history_missing() {
            None
        } else if self.approximate_512_consumed() {
            Some(MeshCertainty::Approximate512Consumed)
        } else {
            Some(MeshCertainty::Certified)
        }
    }
}
