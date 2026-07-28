//! Retained BREP surface carriers.
//!
//! Surface records retain analytic geometry needed by faces.

use hyperlimit::{
    Escalation, Plane3, Plane3Evidence, PlaneSide, Point3, PredicateOutcome,
    classify_point_plane_with_evidence, plane3_evidence,
};

/// Stable identifier for a retained surface.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct BrepSurfaceId(pub u64);

impl BrepSurfaceId {
    /// Construct a stable retained surface identifier.
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Return the stored identifier value.
    pub const fn get(self) -> u64 {
        self.0
    }
}

/// Analytic surface family retained by `hyperbrep`.
#[derive(Clone, Debug, PartialEq)]
pub enum BrepSurfaceKind {
    /// Exact plane represented by `normal . point + offset = 0`.
    Plane(Box<Plane3>),
    /// Named unsupported surface family retained for adapter diagnostics.
    Unsupported {
        /// Source family name, for example `"nurbs-surface"`.
        family: String,
    },
}

/// Retained surface evidence for a BREP face.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepSurface {
    /// Stable surface identifier.
    pub id: BrepSurfaceId,
    /// Analytic or explicit unsupported surface kind.
    pub kind: BrepSurfaceKind,
}

/// Structural facts for a retained BREP surface.
///
/// This is scheduling and readiness evidence, not a face/trim validity claim.
/// The first exact surface family is planar because `hyperlimit::Plane3`
/// already owns point-plane predicates and coefficient facts. Additional
/// analytic families should add explicit unsupported or unknown reports until
/// their parameter frames and intersection predicates exist.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepSurfaceFacts {
    /// Retained surface id.
    pub surface: BrepSurfaceId,
    /// Whether this surface family has current exact-core support.
    pub supported_family: bool,
    /// Whether the surface is a supported exact plane with nonzero normal.
    pub supported_exact_plane: bool,
    /// Whether an exact dyadic coefficient schedule is available.
    pub dyadic_schedule: bool,
    /// Whether a shared-denominator coefficient schedule is available.
    pub shared_denominator_schedule: bool,
    /// Whether the normal is structurally known to be zero.
    pub normal_known_zero: bool,
    /// Whether this surface is ready for exact predicate replay.
    pub exact_replay_ready: bool,
}

/// Explicit blocker for surface evidence or evaluation.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepSurfaceBlocker {
    /// Surface family is unsupported by the current exact core.
    UnsupportedFamily,
    /// Plane normal is structurally zero.
    ZeroNormal,
}

/// Reusable exact-query evidence for a retained surface.
#[derive(Clone, Debug)]
pub enum BrepSurfaceEvidence {
    /// Evidence for an exact plane surface.
    Plane {
        /// Retained surface id.
        surface: BrepSurfaceId,
        /// Retained surface facts.
        facts: BrepSurfaceFacts,
        /// Retained `hyperlimit` plane evidence.
        plane: Box<Plane3Evidence>,
    },
    /// Unsupported or blocked surface.
    Blocked {
        /// Retained surface id.
        surface: BrepSurfaceId,
        /// Cached surface facts.
        facts: BrepSurfaceFacts,
        /// Explicit blockers.
        blockers: Vec<BrepSurfaceBlocker>,
    },
}

/// Point/surface classification report.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepSurfacePointReport {
    /// Retained surface id.
    pub surface: BrepSurfaceId,
    /// Decided plane side for supported planes.
    pub side: Option<PlaneSide>,
    /// Predicate escalation stage when available.
    pub stage: Option<Escalation>,
    /// Whether the point was classified by exact/certified surface replay.
    pub exact_replay: bool,
    /// Whether the point lies on the surface.
    pub on_surface: bool,
    /// Explicit blockers.
    pub blockers: Vec<BrepSurfaceBlocker>,
}

impl BrepSurface {
    /// Construct an exact planar surface record.
    pub fn plane(id: BrepSurfaceId, plane: Plane3) -> Self {
        Self {
            id,
            kind: BrepSurfaceKind::Plane(Box::new(plane)),
        }
    }

    /// Construct a named unsupported surface record.
    pub fn unsupported(id: BrepSurfaceId, family: impl Into<String>) -> Self {
        Self {
            id,
            kind: BrepSurfaceKind::Unsupported {
                family: family.into(),
            },
        }
    }

    /// Returns whether the surface is a plane with a structurally nonzero normal.
    pub fn is_supported_exact_plane(&self) -> bool {
        match &self.kind {
            BrepSurfaceKind::Plane(plane) => !plane.structural_facts().normal_known_zero(),
            BrepSurfaceKind::Unsupported { .. } => false,
        }
    }

    /// Return exact-core surface facts without evaluating any trim topology.
    pub fn facts(&self) -> BrepSurfaceFacts {
        let mut supported_family = false;
        let mut supported_exact_plane = false;
        let mut dyadic_schedule = false;
        let mut shared_denominator_schedule = false;
        let mut normal_known_zero = false;
        if let BrepSurfaceKind::Plane(plane) = &self.kind {
            supported_family = true;
            let plane_facts = plane.structural_facts();
            normal_known_zero = plane_facts.normal_known_zero();
            dyadic_schedule = plane_facts.has_dyadic_schedule();
            shared_denominator_schedule = plane_facts.has_shared_denominator_schedule();
            supported_exact_plane = !normal_known_zero;
        }
        BrepSurfaceFacts {
            surface: self.id,
            supported_family,
            supported_exact_plane,
            dyadic_schedule,
            shared_denominator_schedule,
            normal_known_zero,
            exact_replay_ready: supported_exact_plane,
        }
    }

    /// Derive evidence for repeated exact point classification.
    ///
    /// Plane evidence retains coefficient facts and a certified filter.
    /// Unsupported surfaces retain explicit blockers. Query evidence does not
    /// imply face-topology or trim validity.
    pub fn evidence(&self) -> BrepSurfaceEvidence {
        let facts = self.facts();
        let blockers = self.blockers_from_facts(&facts);
        if blockers.is_empty()
            && let BrepSurfaceKind::Plane(plane) = &self.kind
        {
            return BrepSurfaceEvidence::Plane {
                surface: self.id,
                facts,
                plane: Box::new(plane3_evidence(plane)),
            };
        }
        BrepSurfaceEvidence::Blocked {
            surface: self.id,
            facts,
            blockers,
        }
    }

    fn blockers_from_facts(&self, facts: &BrepSurfaceFacts) -> Vec<BrepSurfaceBlocker> {
        let mut blockers = Vec::new();
        if !facts.supported_family {
            blockers.push(BrepSurfaceBlocker::UnsupportedFamily);
        }
        if facts.normal_known_zero {
            blockers.push(BrepSurfaceBlocker::ZeroNormal);
        }
        blockers
    }
}

impl BrepSurfaceEvidence {
    /// Return retained surface facts.
    pub const fn facts(&self) -> &BrepSurfaceFacts {
        match self {
            Self::Plane { facts, .. } | Self::Blocked { facts, .. } => facts,
        }
    }

    /// Return whether this evidence can replay exact point predicates.
    pub const fn exact_replay_ready(&self) -> bool {
        self.facts().exact_replay_ready
    }

    /// Return retained plane evidence for a supported planar surface.
    pub fn plane_evidence(&self) -> Option<&Plane3Evidence> {
        match self {
            Self::Plane { plane, .. } => Some(plane),
            Self::Blocked { .. } => None,
        }
    }
}

/// Classify a point against a known planar surface using retained evidence.
///
/// `evidence` must have been derived from `plane` as part of the surface's
/// [`BrepSurfaceEvidence`].
#[inline]
pub fn classify_plane_surface_point_with_evidence(
    surface: BrepSurfaceId,
    plane: &Plane3,
    point: &Point3,
    evidence: &Plane3Evidence,
) -> BrepSurfacePointReport {
    match classify_point_plane_with_evidence(point, plane, evidence) {
        PredicateOutcome::Decided { value, stage, .. } => BrepSurfacePointReport {
            surface,
            side: Some(value),
            stage: Some(stage),
            exact_replay: true,
            on_surface: value == PlaneSide::On,
            blockers: Vec::new(),
        },
        PredicateOutcome::Unknown { stage, .. } => BrepSurfacePointReport {
            surface,
            side: None,
            stage: Some(stage),
            exact_replay: false,
            on_surface: false,
            blockers: Vec::new(),
        },
    }
}

/// Classify a point against a retained surface using derived evidence.
///
/// `evidence` must have been derived from `surface` with
/// [`BrepSurface::evidence`].
pub fn classify_surface_point_with_evidence(
    surface: &BrepSurface,
    point: &Point3,
    evidence: &BrepSurfaceEvidence,
) -> BrepSurfacePointReport {
    match evidence {
        BrepSurfaceEvidence::Plane {
            surface: surface_id,
            plane: plane_evidence,
            ..
        } => {
            let BrepSurfaceKind::Plane(plane) = &surface.kind else {
                return BrepSurfacePointReport {
                    surface: *surface_id,
                    side: None,
                    stage: None,
                    exact_replay: false,
                    on_surface: false,
                    blockers: vec![BrepSurfaceBlocker::UnsupportedFamily],
                };
            };
            classify_plane_surface_point_with_evidence(*surface_id, plane, point, plane_evidence)
        }
        BrepSurfaceEvidence::Blocked {
            surface, blockers, ..
        } => BrepSurfacePointReport {
            surface: *surface,
            side: None,
            stage: None,
            exact_replay: false,
            on_surface: false,
            blockers: blockers.clone(),
        },
    }
}
