//! Retained BREP surface carriers.
//!
//! A surface record is source evidence for faces, not an implicit tolerance
//! adapter. Unsupported and lossy imports remain named surface kinds so callers
//! cannot accidentally treat them as exact BREP topology.

use hyperlimit::{
    Plane3, PlaneSide, Point3, PredicateOutcome, PreparedPlane3, predicate::Escalation,
};

/// Stable identifier for a retained surface.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct BrepSurfaceId(pub u64);

/// Provenance class for a retained surface.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum BrepSurfaceSource {
    /// Constructed exactly inside the Hyper stack.
    ExactConstruction,
    /// Imported with exact scalar parameters and retained topology evidence.
    ExactImport,
    /// Imported through a lossy or tolerance-bearing adapter.
    LossyImport,
    /// Source/provenance is not known.
    Unknown,
}

/// Analytic surface family retained by `hyperbrep`.
#[derive(Clone, Debug, PartialEq)]
pub enum BrepSurfaceKind {
    /// Exact plane represented by `normal . point + offset = 0`.
    Plane(Plane3),
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
    /// Surface provenance class.
    pub source: BrepSurfaceSource,
}

/// Prepared facts for a retained BREP surface.
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
    /// Whether the source/provenance is exact Hyper evidence.
    pub exact_source: bool,
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

/// Explicit blocker for surface preparation or evaluation.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepSurfaceBlocker {
    /// Surface provenance is lossy or unknown.
    NonExactSource,
    /// Surface family is unsupported by the current exact core.
    UnsupportedFamily,
    /// Plane normal is structurally zero.
    ZeroNormal,
}

/// Prepared retained surface.
#[derive(Clone, Debug)]
pub enum PreparedBrepSurface<'a> {
    /// Prepared exact plane surface.
    Plane {
        /// Retained surface id.
        surface: BrepSurfaceId,
        /// Cached surface facts.
        facts: BrepSurfaceFacts,
        /// Prepared `hyperlimit` plane classifier.
        prepared: PreparedPlane3<'a>,
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
    pub fn plane(id: BrepSurfaceId, plane: Plane3, source: BrepSurfaceSource) -> Self {
        Self {
            id,
            kind: BrepSurfaceKind::Plane(plane),
            source,
        }
    }

    /// Construct a named unsupported surface record.
    pub fn unsupported(
        id: BrepSurfaceId,
        family: impl Into<String>,
        source: BrepSurfaceSource,
    ) -> Self {
        Self {
            id,
            kind: BrepSurfaceKind::Unsupported {
                family: family.into(),
            },
            source,
        }
    }

    /// Returns whether this surface is exact Hyper-owned evidence.
    pub const fn has_exact_source(&self) -> bool {
        matches!(
            self.source,
            BrepSurfaceSource::ExactConstruction | BrepSurfaceSource::ExactImport
        )
    }

    /// Returns whether the surface is a plane with a structurally nonzero normal.
    pub fn is_supported_exact_plane(&self) -> bool {
        match &self.kind {
            BrepSurfaceKind::Plane(plane) => {
                self.has_exact_source() && !plane.structural_facts().normal_known_zero()
            }
            BrepSurfaceKind::Unsupported { .. } => false,
        }
    }

    /// Return exact-core surface facts without evaluating any trim topology.
    pub fn facts(&self) -> BrepSurfaceFacts {
        let exact_source = self.has_exact_source();
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
            supported_exact_plane = exact_source && !normal_known_zero;
        }
        BrepSurfaceFacts {
            surface: self.id,
            exact_source,
            supported_family,
            supported_exact_plane,
            dyadic_schedule,
            shared_denominator_schedule,
            normal_known_zero,
            exact_replay_ready: supported_exact_plane,
        }
    }

    /// Prepare this surface for repeated exact point classification.
    ///
    /// This wraps `hyperlimit::PreparedPlane3` for planes and returns explicit
    /// blockers for unsupported or lossy surfaces. It mirrors Yap's exact
    /// geometric computation split: retain object-level facts and prepared
    /// predicate state near the surface, but do not infer face topology or trim
    /// validity from a point-plane query alone. See Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997).
    pub fn prepare(&self) -> PreparedBrepSurface<'_> {
        let facts = self.facts();
        let blockers = self.blockers_from_facts(&facts);
        if blockers.is_empty()
            && let BrepSurfaceKind::Plane(plane) = &self.kind
        {
            return PreparedBrepSurface::Plane {
                surface: self.id,
                facts,
                prepared: plane.prepare(),
            };
        }
        PreparedBrepSurface::Blocked {
            surface: self.id,
            facts,
            blockers,
        }
    }

    fn blockers_from_facts(&self, facts: &BrepSurfaceFacts) -> Vec<BrepSurfaceBlocker> {
        let mut blockers = Vec::new();
        if !facts.exact_source {
            blockers.push(BrepSurfaceBlocker::NonExactSource);
        }
        if !facts.supported_family {
            blockers.push(BrepSurfaceBlocker::UnsupportedFamily);
        }
        if facts.normal_known_zero {
            blockers.push(BrepSurfaceBlocker::ZeroNormal);
        }
        blockers
    }
}

impl<'a> PreparedBrepSurface<'a> {
    /// Return cached surface facts.
    pub const fn facts(&self) -> &BrepSurfaceFacts {
        match self {
            Self::Plane { facts, .. } | Self::Blocked { facts, .. } => facts,
        }
    }

    /// Return whether this prepared surface can replay exact point predicates.
    pub const fn exact_replay_ready(&self) -> bool {
        self.facts().exact_replay_ready
    }

    /// Classify a point against this surface.
    pub fn classify_point(&self, point: &Point3) -> BrepSurfacePointReport {
        match self {
            Self::Plane {
                surface, prepared, ..
            } => match prepared.classify_point(point) {
                PredicateOutcome::Decided { value, stage, .. } => BrepSurfacePointReport {
                    surface: *surface,
                    side: Some(value),
                    stage: Some(stage),
                    exact_replay: true,
                    on_surface: value == PlaneSide::On,
                    blockers: Vec::new(),
                },
                PredicateOutcome::Unknown { stage, .. } => BrepSurfacePointReport {
                    surface: *surface,
                    side: None,
                    stage: Some(stage),
                    exact_replay: false,
                    on_surface: false,
                    blockers: Vec::new(),
                },
            },
            Self::Blocked {
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
}
