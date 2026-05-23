//! Construction provenance and freshness reports.
//!
//! BREP construction reports are only valid for the source topology they
//! replayed. This module keeps feature ids, source versions, selected
//! references, adapter diagnostics, and topology snapshots beside reports so a
//! copied or stale result is rejected before mesh, voxel, physics, or export
//! consumers trust it.

use std::collections::BTreeSet;

use crate::report::BrepTopologyCounts;
use crate::topology::{BrepEdgeId, BrepFaceId, BrepShell, BrepVertexId};

const FNV_OFFSET: u64 = 0xcbf29ce484222325;
const FNV_PRIME: u64 = 0x00000100000001b3;

fn fingerprint_bytes(mut state: u64, bytes: &[u8]) -> u64 {
    for byte in bytes {
        state ^= u64::from(*byte);
        state = state.wrapping_mul(FNV_PRIME);
    }
    state
}

fn fingerprint_debug<T: core::fmt::Debug>(state: u64, label: &str, value: &T) -> u64 {
    let state = fingerprint_bytes(state, label.as_bytes());
    fingerprint_bytes(state, format!("{value:?}").as_bytes())
}

/// Stable identifier for a construction feature or adapter operation.
#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct BrepFeatureId(pub String);

impl BrepFeatureId {
    /// Construct a non-empty feature id.
    pub fn new(id: impl Into<String>) -> Option<Self> {
        let id = id.into();
        if id.trim().is_empty() {
            None
        } else {
            Some(Self(id))
        }
    }
}

/// Stable source object/version used to build or replay BREP evidence.
#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct BrepSourceVersion {
    /// Source object id or namespace path.
    pub object: String,
    /// Monotonic source construction version.
    pub version: u64,
}

impl BrepSourceVersion {
    /// Construct a non-empty source/version pair.
    pub fn new(object: impl Into<String>, version: u64) -> Option<Self> {
        let object = object.into();
        if object.trim().is_empty() {
            None
        } else {
            Some(Self { object, version })
        }
    }
}

/// Construction operation family.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum BrepConstructionKind {
    /// Exact planar face construction.
    PlanarFace,
    /// Exact extrusion construction.
    Extrusion,
    /// Exact analytic shell construction.
    AnalyticShell,
    /// Imported through an external adapter.
    AdapterImport,
    /// Producer did not declare the construction kind.
    Unknown,
}

/// Replay status for construction evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum BrepConstructionReplayStatus {
    /// Exact/certified replay accepted the construction.
    Accepted,
    /// Replay rejected the construction.
    Rejected,
    /// Replay was not available.
    Missing,
    /// Replay outcome was unknown.
    Unknown,
}

/// Reference selected by a feature or adapter operation.
#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepSelectedReference {
    /// Selected source vertex.
    Vertex(BrepVertexId),
    /// Selected source edge.
    Edge(BrepEdgeId),
    /// Selected source face.
    Face(BrepFaceId),
    /// External object path or selector.
    External(String),
}

/// Compact topology snapshot captured by a construction manifest.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BrepTopologySnapshot {
    /// Number of vertices.
    pub vertex_count: usize,
    /// Number of edges.
    pub edge_count: usize,
    /// Number of surfaces.
    pub surface_count: usize,
    /// Number of faces.
    pub face_count: usize,
    /// Number of loops.
    pub loop_count: usize,
    /// Number of coedges.
    pub coedge_count: usize,
}

impl From<BrepTopologyCounts> for BrepTopologySnapshot {
    fn from(counts: BrepTopologyCounts) -> Self {
        Self {
            vertex_count: counts.vertex_count,
            edge_count: counts.edge_count,
            surface_count: counts.surface_count,
            face_count: counts.face_count,
            loop_count: counts.loop_count,
            coedge_count: counts.coedge_count,
        }
    }
}

impl BrepTopologySnapshot {
    /// Capture current topology counts from a shell.
    pub fn from_shell(shell: &BrepShell) -> Self {
        shell.audit_closure().counts.into()
    }
}

/// Deterministic retained-shell fingerprint used to reject stale evidence.
///
/// This fingerprint is intentionally an audit token, not a geometric hash API.
/// It records ordered retained object identity, incidence, surface records, and
/// exact vertex payloads so count-preserving mutations cannot reuse an old
/// construction manifest. The gate follows Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997): exact combinatorial
/// decisions are valid only for the object representation that was replayed.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct BrepTopologyFingerprint {
    /// Deterministic FNV-1a-style digest over retained BREP evidence.
    pub value: u64,
}

impl BrepTopologyFingerprint {
    /// Capture a deterministic fingerprint from a retained shell.
    pub fn from_shell(shell: &BrepShell) -> Self {
        let mut state = fingerprint_bytes(FNV_OFFSET, b"hyperbrep:v1");
        state = fingerprint_debug(state, "vertices", &shell.vertices);
        state = fingerprint_debug(state, "edges", &shell.edges);
        state = fingerprint_debug(state, "surfaces", &shell.surfaces);
        state = fingerprint_debug(state, "faces", &shell.faces);
        Self { value: state }
    }
}

/// Construction provenance manifest.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepConstructionManifest {
    /// Feature or adapter operation id.
    pub feature: Option<BrepFeatureId>,
    /// Construction operation family.
    pub kind: BrepConstructionKind,
    /// Source object versions replayed by this construction.
    pub sources: Vec<BrepSourceVersion>,
    /// Selected source references.
    pub selected_references: Vec<BrepSelectedReference>,
    /// Typed parameter payload summaries.
    pub parameter_payloads: Vec<String>,
    /// Adapter diagnostics retained as source evidence.
    pub adapter_diagnostics: Vec<String>,
    /// Exact/certified replay status.
    pub replay_status: BrepConstructionReplayStatus,
    /// Topology snapshot captured when the report was built.
    pub topology_snapshot: BrepTopologySnapshot,
    /// Retained-shell fingerprint captured when the report was built.
    pub topology_fingerprint: BrepTopologyFingerprint,
}

impl BrepConstructionManifest {
    /// Build an exact construction manifest from a shell snapshot.
    pub fn exact(
        feature: BrepFeatureId,
        kind: BrepConstructionKind,
        sources: Vec<BrepSourceVersion>,
        shell: &BrepShell,
    ) -> Self {
        Self {
            feature: Some(feature),
            kind,
            sources,
            selected_references: Vec::new(),
            parameter_payloads: Vec::new(),
            adapter_diagnostics: Vec::new(),
            replay_status: BrepConstructionReplayStatus::Accepted,
            topology_snapshot: BrepTopologySnapshot::from_shell(shell),
            topology_fingerprint: BrepTopologyFingerprint::from_shell(shell),
        }
    }

    /// Validate manifest freshness and replay readiness against the current shell.
    pub fn report(&self, shell: &BrepShell) -> BrepConstructionProvenanceReport {
        BrepConstructionProvenanceReport::from_manifest(self, shell)
    }
}

/// Explicit blocker for construction provenance readiness.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepConstructionBlocker {
    /// Feature id is absent.
    MissingFeatureId,
    /// Construction kind is unknown.
    UnknownConstructionKind,
    /// No source versions were declared.
    MissingSourceVersions,
    /// A selected reference is missing from the current shell.
    MissingSelectedReference,
    /// Exact/certified replay did not accept the construction.
    ReplayNotAccepted,
    /// Manifest topology snapshot differs from the current shell.
    StaleTopologySnapshot,
    /// Manifest retained-shell fingerprint differs from the current shell.
    StaleTopologyFingerprint,
    /// Adapter diagnostics are present.
    AdapterDiagnosticsPresent,
}

/// Freshness and replay report for construction provenance.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepConstructionProvenanceReport {
    /// Feature or adapter operation id.
    pub feature: Option<BrepFeatureId>,
    /// Construction operation family.
    pub kind: BrepConstructionKind,
    /// Source count.
    pub source_count: usize,
    /// Selected reference count.
    pub selected_reference_count: usize,
    /// Parameter payload count.
    pub parameter_payload_count: usize,
    /// Adapter diagnostic count.
    pub adapter_diagnostic_count: usize,
    /// Whether the topology snapshot still matches the current shell.
    pub topology_snapshot_current: bool,
    /// Whether the retained-shell fingerprint still matches the current shell.
    pub topology_fingerprint_current: bool,
    /// Whether exact/certified replay accepted this construction.
    pub replay_accepted: bool,
    /// Explicit blockers.
    pub blockers: Vec<BrepConstructionBlocker>,
    /// Whether construction provenance is ready for downstream report reuse.
    pub construction_fresh: bool,
}

impl BrepConstructionProvenanceReport {
    /// Validate a construction manifest against the current shell.
    ///
    /// This is a Yap-style source-replay gate: exact decisions stay attached to
    /// the source object versions and topology snapshot that justified them.
    /// See Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7.1-2 (1997). The feature-history shape is intentionally similar to
    /// product CAD kernels and BREP.io-style authoring records, but this layer
    /// stores evidence only; editing policy belongs above `hyperbrep`.
    pub fn from_manifest(manifest: &BrepConstructionManifest, shell: &BrepShell) -> Self {
        let mut blockers = BTreeSet::new();
        if manifest.feature.is_none() {
            blockers.insert(BrepConstructionBlocker::MissingFeatureId);
        }
        if manifest.kind == BrepConstructionKind::Unknown {
            blockers.insert(BrepConstructionBlocker::UnknownConstructionKind);
        }
        if manifest.sources.is_empty() {
            blockers.insert(BrepConstructionBlocker::MissingSourceVersions);
        }
        if manifest.replay_status != BrepConstructionReplayStatus::Accepted {
            blockers.insert(BrepConstructionBlocker::ReplayNotAccepted);
        }
        if !manifest.adapter_diagnostics.is_empty() {
            blockers.insert(BrepConstructionBlocker::AdapterDiagnosticsPresent);
        }

        let topology_snapshot_current =
            manifest.topology_snapshot == BrepTopologySnapshot::from_shell(shell);
        if !topology_snapshot_current {
            blockers.insert(BrepConstructionBlocker::StaleTopologySnapshot);
        }
        let topology_fingerprint_current =
            manifest.topology_fingerprint == BrepTopologyFingerprint::from_shell(shell);
        if !topology_fingerprint_current {
            blockers.insert(BrepConstructionBlocker::StaleTopologyFingerprint);
        }

        let vertex_ids = shell
            .vertices
            .iter()
            .map(|vertex| vertex.id)
            .collect::<BTreeSet<_>>();
        let edge_ids = shell
            .edges
            .iter()
            .map(|edge| edge.id)
            .collect::<BTreeSet<_>>();
        let face_ids = shell
            .faces
            .iter()
            .map(|face| face.id)
            .collect::<BTreeSet<_>>();
        for reference in &manifest.selected_references {
            let present = match reference {
                BrepSelectedReference::Vertex(id) => vertex_ids.contains(id),
                BrepSelectedReference::Edge(id) => edge_ids.contains(id),
                BrepSelectedReference::Face(id) => face_ids.contains(id),
                BrepSelectedReference::External(path) => !path.trim().is_empty(),
            };
            if !present {
                blockers.insert(BrepConstructionBlocker::MissingSelectedReference);
            }
        }

        let blockers = blockers.into_iter().collect::<Vec<_>>();
        Self {
            feature: manifest.feature.clone(),
            kind: manifest.kind,
            source_count: manifest.sources.len(),
            selected_reference_count: manifest.selected_references.len(),
            parameter_payload_count: manifest.parameter_payloads.len(),
            adapter_diagnostic_count: manifest.adapter_diagnostics.len(),
            topology_snapshot_current,
            topology_fingerprint_current,
            replay_accepted: manifest.replay_status == BrepConstructionReplayStatus::Accepted,
            construction_fresh: blockers.is_empty(),
            blockers,
        }
    }
}
