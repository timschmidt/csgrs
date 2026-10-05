//! Metadata attached to CSG objects that represent parts or subassemblies.
//!
//! NopSCADlib popularized the practical split between parametric part geometry,
//! vitamin metadata, and assembly documentation. Coordinates on these records
//! are `hyperlattice` points and vectors of `hyperreal::Real`.

use hyperlattice::{Point3, Vector3};

use super::scalar::vector_is_zero;

/// Index of a node in an [`super::AssemblyDocument`].
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct NodeId(pub u32);

/// Source evidence for a part-interface fact.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PartSource {
    /// Source family, such as `nopscadlib`, `datasheet`, `kicad`, or `manual`.
    pub family: String,
    /// Source revision, commit, page, or local fixture label.
    pub revision: String,
}

/// Certainty of a source field before it is consumed by another Hyper crate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SourceCertainty {
    /// Native exact value or symbolic construction.
    Exact,
    /// Reviewed/certified imported value.
    Certified,
    /// Present but lossy, usually from a mesh or preview artifact.
    Lossy,
    /// Display-only data that must not become topology or compatibility proof.
    DisplayOnly,
    /// Source was inspected but did not provide the value.
    Missing,
}

/// Geometry status for the CSG object associated with a part interface.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GeometryCertainty {
    /// Native CSG/curve-region construction over exact Hyper scalar carriers.
    NativeExactCsg,
    /// Imported geometry certified by a domain-specific report.
    CertifiedImported,
    /// Lossy preview mesh.
    LossyPreviewMesh,
    /// Display-only artifact.
    DisplayOnly,
    /// No usable geometry was supplied.
    Missing,
    /// Geometry handle exists but no longer replays against the source.
    Stale,
}

/// High-level aspect carried by a part interface.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct InterfaceAspect {
    /// Stable aspect handle.
    pub handle: String,
    /// Aspect family.
    pub kind: InterfaceKind,
}

/// Aspect family for CSG-authored part metadata.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum InterfaceKind {
    /// Physical body or sub-body.
    Body,
    /// Thermal/contact region.
    Thermal,
    /// Mechanical mating/mounting interface.
    Mechanical,
    /// Manufacturing or tool-process interface.
    Process,
    /// Source-specific aspect.
    Custom(String),
}

/// Geometry-attached mechanical, thermal, or process attachment point.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PartTerminal {
    /// Stable attachment handle.
    pub handle: String,
    /// Human-readable attachment name.
    pub name: String,
    /// Geometry-side role such as `mounting`, `mating`, `thermal`, or `tooling`.
    pub role: String,
}

/// Named port frame used by mating, callouts, and downstream handoff.
#[derive(Clone, Debug, PartialEq)]
pub struct PortFrame {
    /// Stable port handle.
    pub handle: String,
    /// Terminal handles exposed by this port.
    pub terminals: Vec<String>,
    /// Origin in the part's local frame.
    pub origin: Point3,
    /// Outward or insertion direction.
    pub direction: Vector3,
}

/// Named anchor frame for attachment.
#[derive(Clone, Debug, PartialEq)]
pub struct AnchorFrame {
    /// Stable anchor handle.
    pub handle: String,
    /// Origin in the part's local frame.
    pub origin: Point3,
    /// Outward normal or mate direction.
    pub normal: Vector3,
    /// Roll vector when the source specifies orientation.
    pub up: Option<Vector3>,
}

/// Material region handle attached to geometry metadata.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MaterialRegion {
    /// Stable region handle.
    pub handle: String,
    /// Material label or external material-model handle.
    pub material: String,
}

/// NopSCADlib-style installation and exploded-view vector.
///
/// `install_direction` is the direction of insertion/removal in the part's
/// local frame. `explode_offset` is the vector used for exploded diagrams.
/// Keeping both vectors explicit prevents a display transform from becoming an
/// unreviewable assembly fact.
#[derive(Clone, Debug, PartialEq)]
pub struct InstallationVector {
    /// Direction used for insertion/removal.
    pub install_direction: Vector3,
    /// Offset used in exploded documentation.
    pub explode_offset: Vector3,
    /// Whether guide lines should be drawn from assembled to exploded pose.
    pub show_guide: bool,
    /// Whether children inherit the parent's exploded offset.
    pub propagate_to_children: bool,
}

/// Documentation flags for a part or subassembly.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AssemblyFlag {
    /// Do not include this part in BOM output.
    HiddenFromBom,
    /// Do not move this part in exploded views.
    NoExplode,
    /// Render only as a guide/reference object.
    ReferenceOnly,
}

/// Documentation metadata used for assembly instructions and diagrams.
#[derive(Clone, Debug, PartialEq)]
pub struct AssemblyDocumentation {
    /// Optional installation vector.
    pub installation: Option<InstallationVector>,
    /// Optional camera/pose label for docs.
    pub pose_hint: Option<String>,
    /// Documentation flags.
    pub flags: Vec<AssemblyFlag>,
}

/// Full CSG-side part interface.
#[derive(Clone, Debug, PartialEq)]
pub struct CsgPartInterface {
    /// Part family id.
    pub family_id: String,
    /// Part variant id.
    pub variant_id: String,
    /// Source evidence.
    pub source: PartSource,
    /// Geometry certainty for the associated CSG object.
    pub geometry: GeometryCertainty,
    /// Named aspects.
    pub aspects: Vec<InterfaceAspect>,
    /// Terminals.
    pub terminals: Vec<PartTerminal>,
    /// Ports.
    pub ports: Vec<PortFrame>,
    /// Anchors.
    pub anchors: Vec<AnchorFrame>,
    /// Material regions.
    pub materials: Vec<MaterialRegion>,
    /// Manufacturing/process notes.
    pub process_notes: Vec<String>,
    /// Assembly documentation metadata.
    pub documentation: AssemblyDocumentation,
}

/// Metadata wrapper intended for `AttributedMesh<PartMetadata>`.
///
/// Frames live on the assembly document. This sidecar keeps the handle and an
/// optional [`NodeId`] so a triangle can point at that document without
/// copying the frame onto every face.
#[derive(Clone, Debug, PartialEq)]
pub struct PartMetadata {
    /// Stable local handle for this CSG object.
    pub handle: String,
    /// Part-interface facts.
    pub interface: CsgPartInterface,
    /// Assembly node this face belongs to, when the mesh was built from a document.
    pub assembly_node: Option<NodeId>,
}

impl InstallationVector {
    /// Creates an installation vector after rejecting zero directions.
    ///
    /// The validation uses exact sign refinement over the squared vector length
    /// rather than an epsilon. This follows Yap's exact-computation boundary:
    /// a zero installation vector is rejected as source data, not accepted
    /// because it is "small enough."
    pub fn new(
        install_direction: Vector3,
        explode_offset: Vector3,
        show_guide: bool,
        propagate_to_children: bool,
    ) -> Option<Self> {
        if vector_is_zero(&install_direction) != Some(false) {
            return None;
        }
        Some(Self {
            install_direction,
            explode_offset,
            show_guide,
            propagate_to_children,
        })
    }
}

impl CsgPartInterface {
    /// Creates a minimal interface with exact native CSG geometry status.
    pub fn exact_csg(
        family_id: impl Into<String>,
        variant_id: impl Into<String>,
        source: PartSource,
    ) -> Self {
        Self {
            family_id: family_id.into(),
            variant_id: variant_id.into(),
            source,
            geometry: GeometryCertainty::NativeExactCsg,
            aspects: Vec::new(),
            terminals: Vec::new(),
            ports: Vec::new(),
            anchors: Vec::new(),
            materials: Vec::new(),
            process_notes: Vec::new(),
            documentation: AssemblyDocumentation {
                installation: None,
                pose_hint: None,
                flags: Vec::new(),
            },
        }
    }

    /// Returns true when a part has a replayable installation vector.
    pub const fn has_installation_vector(&self) -> bool {
        self.documentation.installation.is_some()
    }
}

impl PartMetadata {
    /// Creates metadata for a CSG object handle.
    pub fn new(handle: impl Into<String>, interface: CsgPartInterface) -> Self {
        Self {
            handle: handle.into(),
            interface,
            assembly_node: None,
        }
    }

    /// Records the assembly node that owns this face.
    pub const fn with_assembly_node(mut self, node: NodeId) -> Self {
        self.assembly_node = Some(node);
        self
    }
}
