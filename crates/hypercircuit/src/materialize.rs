//! Exact-aware lowering of declarative PCB intent into `csgrs` geometry.
//!
//! The retained layout remains authoritative. Materialized profiles are output
//! products with source ids and net identities, so a boolean union never
//! destroys the information needed by `hyperdrc` or an interchange adapter.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::{Display, Formatter};
use std::io::{BufReader, Cursor};

use csgrs::{csg::CSG, mesh::Mesh, sketch::Profile};
use hypercurve::{
    Classification, CurvePolicy, CurveRegion2, CurveString2, LineArcRegion2, LineSeg2, OffsetCap,
    Point2 as CurvePoint2, Segment2,
};
use hyperlattice::Point2;
use hyperpath::TraceLayer;
use hyperreal::Real;
use sha2::{Digest, Sha256};

use crate::layout::{
    BoardSide, CopperZone, CopperZoneConnection, CopperZoneFill, DrillShape, KeepoutScope,
    LandPatternGraphic, LandPatternGraphicPrimitive, LandPatternPad, LayerRole, PadShape,
    Pcb3dModelFormat, Pcb3dModelReference, PcbLayout, PcbPlacement, Plating, ViaMaskDisposition,
};
use crate::{
    Circuit, CircuitInstanceId, LandPatternGraphicId, LandPatternId, NetId, PadId, PinRef, RouteId,
    ViaId, ZoneId,
};

/// Options controlling finite display approximations at materialization boundaries.
#[derive(Clone, Debug, PartialEq)]
pub struct MaterializationOptions {
    /// Segment count used for circles and rounded pad display geometry.
    pub circular_segments: usize,
    /// Default exact expansion applied to pad solder-mask openings when the pad delegates policy.
    pub default_solder_mask_margin: Real,
    /// Default exact expansion/reduction applied to SMD paste apertures when delegated by the pad.
    pub default_paste_margin: Real,
    /// Caller-selected font bytes and stable identity for production text lowering.
    pub production_text: Option<ProductionTextPolicy>,
    /// Maximum finite chord error used only when curved routes cross into polygonal manufacturing geometry.
    pub route_arc_chord_error: f64,
    /// Maximum finite flatness error for cubic-Bezier manufacturing projection.
    pub route_bezier_chord_error: f64,
}

impl Default for MaterializationOptions {
    fn default() -> Self {
        Self {
            circular_segments: 64,
            default_solder_mask_margin: Real::zero(),
            default_paste_margin: Real::zero(),
            production_text: None,
            route_arc_chord_error: 1.0e-3,
            route_bezier_chord_error: 1.0e-3,
        }
    }
}

/// Reproducible caller-selected font policy for production artwork.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProductionTextPolicy {
    /// Stable human-readable font identity recorded in release evidence.
    pub font_name: String,
    /// Complete OpenType/TrueType font bytes used by csgrs's outline importer.
    pub font_data: Vec<u8>,
}

/// Digest evidence for the exact font bytes used during materialization.
#[derive(Clone, Debug, Eq, PartialEq, serde::Deserialize, serde::Serialize)]
pub struct ProductionTextEvidence {
    /// Caller-declared font identity.
    pub font_name: String,
    /// Lowercase SHA-256 of the exact font bytes.
    pub sha256: String,
}

/// Audited finite projection used at an explicit geometry/output boundary.
#[derive(Clone, Debug, Eq, PartialEq, serde::Deserialize, serde::Serialize)]
pub enum MaterializationProjection {
    /// Exact hyperpath arc projected to a polyline before stroke materialization.
    CircularRoutePolyline {
        /// Stable semantic route source.
        source: String,
        /// Requested maximum chord error, retained as a deterministic decimal string.
        chord_error: String,
    },
    /// Exact hyperpath cubic Bezier projected to a polyline before stroke materialization.
    CubicBezierRoutePolyline {
        /// Stable semantic route source.
        source: String,
        /// Requested maximum flatness/chord error as a deterministic decimal string.
        chord_error: String,
    },
    /// Exact board-contour cubic Bezier projected to bounded line segments for CAM.
    CubicBezierBoardContourPolyline {
        /// Stable semantic contour source (`board.exterior` or `board.cutout[n]`).
        source: String,
        /// Zero-based segment index within the retained contour.
        segment: usize,
        /// Requested maximum flatness/chord error as a deterministic decimal string.
        chord_error: String,
        /// Number of emitted linear interpolation segments.
        generated_segments: usize,
    },
}

/// Audited realization summary for one retained copper zone.
#[derive(Clone, Debug, Eq, PartialEq, serde::Deserialize, serde::Serialize)]
pub struct ZoneMaterializationEvidence {
    /// Stable semantic zone source.
    pub source: String,
    /// Retained zone priority.
    pub priority: i32,
    /// `solid` or `hatched`.
    pub fill: String,
    /// `solid`, `isolated`, or `thermal-relief`.
    pub connection: String,
    /// Exact foreign-net clearance as retained display text.
    pub clearance: String,
    /// Foreign-net copper features cleared from this zone.
    pub cleared_foreign_features: usize,
    /// Same-net pads/vias receiving isolated or thermal treatment.
    pub treated_same_net_lands: usize,
    /// Thermal lands whose spoke termination used the profile bounding box.
    pub thermal_bounding_box_projections: usize,
    /// Authored keepouts subtracted from this zone.
    pub applied_keepouts: usize,
    /// Islands present after fill, clearance, and connection realization.
    pub initial_islands: usize,
    /// Islands retained after applying the authored cleanup policy.
    pub retained_islands: usize,
    /// Islands rejected because they did not intersect same-net copper.
    pub pruned_unconnected_islands: usize,
    /// Unconnected islands rejected because their exact filled area was below the threshold.
    pub pruned_below_area_islands: usize,
}

/// Source kind retained for one materialized copper feature.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CopperFeatureKind {
    /// Component land-pattern pad.
    Pad,
    /// Routed trace chain.
    Route,
    /// Via land.
    Via,
    /// Copper-zone source boundary.
    Zone,
    /// Copper artwork retained by a reusable land pattern.
    Artwork,
}

/// Stable semantic owner of one materialized copper feature.
///
/// Geometry consumers may use [`MaterializedCopperFeature::source`] for
/// diagnostics, while circuit, DRC, and manufacturing adapters should use this
/// typed identity rather than parsing that display string.
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum MaterializedCopperIdentity {
    /// One physical pad belonging to a placed logical instance.
    Pad {
        /// Placed circuit instance.
        instance: CircuitInstanceId,
        /// Reusable land pattern.
        land_pattern: LandPatternId,
        /// Physical pad identity.
        pad: PadId,
        /// Logical pin mapped to the pad, when one is retained.
        pin: Option<PinRef>,
    },
    /// Authored route identity.
    Route(RouteId),
    /// Authored or deterministically generated via identity.
    Via(ViaId),
    /// Authored copper-zone identity.
    Zone(ZoneId),
    /// Copper artwork attached to a placed land pattern.
    Artwork {
        /// Placed circuit instance.
        instance: CircuitInstanceId,
        /// Reusable land pattern.
        land_pattern: LandPatternId,
        /// Land-pattern graphic identity.
        graphic: LandPatternGraphicId,
    },
}

/// One source-addressable copper feature before per-layer union.
#[derive(Clone, Debug)]
pub struct MaterializedCopperFeature {
    /// Stable source handle, prefixed by feature kind.
    pub source: String,
    /// Logical net when the source retained one.
    pub net: Option<NetId>,
    /// Copper layer receiving this feature.
    pub layer: TraceLayer,
    /// Semantic source kind.
    pub kind: CopperFeatureKind,
    /// Stable typed semantic owner.
    pub identity: MaterializedCopperIdentity,
    /// Exact board-space source anchor for diagnostics and spatial policies.
    pub anchor: Point2,
    /// Exact-aware materialized copper profile.
    pub profile: Profile,
}

/// Unioned copper image for one routing layer.
#[derive(Clone, Debug)]
pub struct LayerImage {
    /// Copper routing layer.
    pub layer: TraceLayer,
    /// Union of every materialized feature when the exact boolean was decided.
    pub copper: Option<Profile>,
    /// Number of retained source features represented by this layer image.
    pub source_feature_count: usize,
    /// Explicit exact-boolean blocker when no unioned image could be certified.
    pub blocker: Option<String>,
}

/// Side-aware non-copper production image role.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum ProcessLayerRole {
    /// Front solder-mask opening image.
    FrontSolderMask,
    /// Back solder-mask opening image.
    BackSolderMask,
    /// Front stencil-paste aperture image.
    FrontPaste,
    /// Back stencil-paste aperture image.
    BackPaste,
    /// Front legend/silkscreen image.
    FrontSilkscreen,
    /// Back legend/silkscreen image.
    BackSilkscreen,
}

/// Semantic origin of one non-copper production feature.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProcessFeatureKind {
    /// Pad-derived solder-mask opening.
    PadMaskOpening,
    /// Pad-derived stencil-paste aperture.
    PadPasteAperture,
    /// Explicit via solder-mask opening.
    ViaMaskOpening,
    /// Explicit land-pattern artwork.
    Artwork,
}

/// One independently source-addressable non-copper production feature.
#[derive(Clone, Debug)]
pub struct MaterializedProcessFeature {
    /// Stable source handle.
    pub source: String,
    /// Side/process image receiving the feature.
    pub role: ProcessLayerRole,
    /// Semantic source kind.
    pub kind: ProcessFeatureKind,
    /// Exact-aware board-space profile.
    pub profile: Profile,
}

/// Certified union image for one side/process combination.
#[derive(Clone, Debug)]
pub struct ProcessLayerImage {
    /// Side/process role.
    pub role: ProcessLayerRole,
    /// Union of every source feature when the exact boolean was decided.
    pub image: Option<Profile>,
    /// Number of retained source features represented by this image.
    pub source_feature_count: usize,
    /// Explicit exact-boolean blocker when no image could be certified.
    pub blocker: Option<String>,
}

/// Retained artwork or process intent not materialized into production geometry.
#[derive(Clone, Debug, Eq, PartialEq, serde::Deserialize, serde::Serialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
pub enum ProcessMaterializationOmission {
    /// A stroked primitive omitted the required exact stroke width.
    MissingArtworkStroke { source: String },
    /// Text remains retained because no production-font policy was selected.
    TextArtwork { source: String },
    /// The selected production font could not produce geometry for non-whitespace text.
    TextFontRejected { source: String },
    /// Package-local edge-cut artwork was not merged with the authoritative board contour.
    PackageEdgeCuts { source: String },
    /// A custom layer has no declared production-file role.
    CustomArtworkLayer { source: String, layer: String },
    /// One or more vias have no retained mask-tenting/opening policy on an applicable surface.
    ViaMaskIntentUnavailable { count: usize },
    /// Legend geometry has not been clipped away from mask openings.
    SilkscreenNotClippedToMask,
}

/// Board-space drill or routed-slot handoff record.
#[derive(Clone, Debug, PartialEq)]
pub struct DrillHit {
    /// Stable source handle.
    pub source: String,
    /// Board-space center for round drills and placement origin for slots.
    pub center: Point2,
    /// Board-space drill/slot geometry.
    pub shape: DrillShape,
    /// Retained plating intent.
    pub plating: Plating,
}

/// Output of declarative PCB geometry materialization.
#[derive(Clone, Debug)]
pub struct PcbMaterializationReport {
    /// Substrate region after applying authored cutouts.
    pub substrate: Profile,
    /// Individually addressable copper features with source and net identity.
    pub copper_features: Vec<MaterializedCopperFeature>,
    /// Per-layer union images suitable for Gerber or 3D lowering.
    pub copper_layers: Vec<LayerImage>,
    /// Individually source-addressable mask, paste, and legend features.
    pub process_features: Vec<MaterializedProcessFeature>,
    /// Certified per-side non-copper production images.
    pub process_layers: Vec<ProcessLayerImage>,
    /// Explicitly retained production details not materialized.
    pub process_omissions: Vec<ProcessMaterializationOmission>,
    /// Exact font-byte identity when a production text policy was selected.
    pub production_text: Option<ProductionTextEvidence>,
    /// Every lossy finite geometry projection used to produce this report.
    pub projections: Vec<MaterializationProjection>,
    /// Audited copper-zone fill/clearance/connection realizations.
    pub zone_realizations: Vec<ZoneMaterializationEvidence>,
    /// Deterministic generated stitching-via evidence.
    pub stitching_realizations: Vec<crate::ZoneStitchingEvidence>,
    /// Drill and routed-slot handoff records.
    pub drills: Vec<DrillHit>,
    /// Finite segment policy retained for circular review-tool realization.
    pub preview_circular_segments: usize,
    /// Number of keepouts retained for downstream realization/DRC.
    pub retained_keepout_count: usize,
}

/// Physical role of one extruded stackup preview layer.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Pcb3dLayerKind {
    /// Conductive image from a decided per-layer copper union.
    Copper { routing_layer: TraceLayer },
    /// Uniform dielectric board planform.
    Dielectric,
    /// Solder-mask material with decided side-specific openings subtracted.
    SolderMask,
    /// Source-specific uniform board-planform layer.
    Custom(String),
}

/// Metadata attached to every triangle of one stackup preview solid.
#[derive(Clone, Debug, PartialEq)]
pub struct Pcb3dLayerMetadata {
    /// Authored stackup layer name.
    pub name: String,
    /// Physical layer role.
    pub kind: Pcb3dLayerKind,
    /// Exact lower Z coordinate in stack order.
    pub z_start: Real,
    /// Exact layer thickness.
    pub thickness: Real,
}

/// One independently source-addressable extruded stackup layer.
#[derive(Clone, Debug)]
pub struct Pcb3dLayer {
    /// Layer metadata also cloned onto the mesh polygons.
    pub metadata: Pcb3dLayerMetadata,
    /// Exact-aware triangulated preview solid.
    pub solid: Mesh<Pcb3dLayerMetadata>,
}

/// Metadata attached to a native package-body envelope preview.
#[derive(Clone, Debug, PartialEq)]
pub struct Pcb3dComponentBodyMetadata {
    /// Logical placed instance.
    pub instance: crate::CircuitInstanceId,
    /// Reusable land pattern supplying the body.
    pub land_pattern: crate::LandPatternId,
    /// Exact lower Z coordinate of the body envelope.
    pub z_start: Real,
    /// Exact body height.
    pub height: Real,
}

/// Independently inspectable native package-body envelope solid.
#[derive(Clone, Debug)]
pub struct Pcb3dComponentBody {
    /// Source identity and exact Z/model intent.
    pub metadata: Pcb3dComponentBodyMetadata,
    /// Exact-aware triangulated body envelope.
    pub solid: Mesh<Pcb3dComponentBodyMetadata>,
}

/// Caller-controlled byte resolver for package-model URIs.
///
/// Filesystem, package registry, embedded asset, and authenticated fetch policy
/// remain outside HyperCircuit. The resolver returns the exact source bytes.
pub trait Pcb3dModelResolver {
    /// Resolves one retained reference or returns an audited diagnostic.
    fn resolve(&mut self, reference: &Pcb3dModelReference) -> Result<Vec<u8>, String>;
}

impl<F> Pcb3dModelResolver for F
where
    F: FnMut(&Pcb3dModelReference) -> Result<Vec<u8>, String>,
{
    fn resolve(&mut self, reference: &Pcb3dModelReference) -> Result<Vec<u8>, String> {
        self(reference)
    }
}

/// Semantic metadata attached to one successfully loaded component mesh.
#[derive(Clone, Debug, PartialEq)]
pub struct Pcb3dComponentModelMetadata {
    /// Logical placed instance.
    pub instance: CircuitInstanceId,
    /// Reusable land pattern supplying the model reference.
    pub land_pattern: LandPatternId,
    /// Zero-based authored model record within the land pattern.
    pub model_index: usize,
    /// Exact retained URI, format, and local transform.
    pub reference: Pcb3dModelReference,
    /// Lowercase SHA-256 of the resolved source bytes.
    pub source_sha256: String,
}

/// Placed external package mesh after exact retained transforms.
#[derive(Clone, Debug)]
pub struct Pcb3dComponentModel {
    /// Source identity, transform, and digest evidence.
    pub metadata: Pcb3dComponentModelMetadata,
    /// Parsed and board-placed exact-aware triangle mesh.
    pub mesh: Mesh<Pcb3dComponentModelMetadata>,
}

/// Successful external-model resolution and parse evidence.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Pcb3dModelResolutionEvidence {
    /// Logical placed instance.
    pub instance: CircuitInstanceId,
    /// Reusable land pattern supplying the reference.
    pub land_pattern: LandPatternId,
    /// Zero-based authored model record within the land pattern.
    pub model_index: usize,
    /// Stable source URI.
    pub uri: String,
    /// Explicit parsed source container.
    pub format: Pcb3dModelFormat,
    /// Lowercase SHA-256 of the exact resolved bytes.
    pub source_sha256: String,
    /// Triangle count after source-container triangulation.
    pub triangle_count: usize,
    /// Selected glTF scene index, or `None` for other source containers.
    pub source_scene_index: Option<usize>,
    /// Source mesh-node instances (or VRML shapes) visited.
    pub source_mesh_node_count: usize,
    /// Source triangle primitives (or VRML indexed face sets) flattened.
    pub source_primitive_count: usize,
    /// VRML line/point geometry omitted from the triangle mesh.
    pub ignored_non_mesh_geometry_count: usize,
    /// VRML polygons that could not form a surface.
    pub ignored_degenerate_polygon_count: usize,
    /// VRML triangles rejected by exact degeneracy checks.
    pub ignored_degenerate_triangle_count: usize,
}

/// Successfully applied semantic subtraction in one 3D preview layer.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Pcb3dSubtractionKind {
    /// One retained round drill or routed slot.
    Drill {
        /// Stable pad/via drill source.
        source: String,
        /// Retained fabrication plating intent.
        plating: Plating,
    },
    /// One certified side-specific union of solder-mask openings.
    SolderMaskOpenings {
        /// Surface process image that supplied the opening geometry.
        role: ProcessLayerRole,
        /// Independently source-addressable features represented by the union.
        source_feature_count: usize,
    },
}

/// Evidence that a semantic tool image was subtracted from one physical layer.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Pcb3dSubtractionEvidence {
    /// Authored stackup layer receiving the subtraction.
    pub layer: String,
    /// Exact semantic tool family and source.
    pub kind: Pcb3dSubtractionKind,
}

/// Deliberately unmodeled detail in a stackup preview assembly.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Pcb3dAssemblyOmission {
    /// A copper union was unresolved, so no authoritative layer solid was made.
    BlockedCopperLayer { layer: u16, blocker: String },
    /// A declared conductor had no retained copper image.
    EmptyCopperLayer { layer: u16 },
    /// A retained drill/slot could not produce a certified planar cutter.
    InvalidDrillGeometry { source: String, detail: String },
    /// A valid drill/slot cutter could not be subtracted from one physical layer.
    DrillSubtractionFailed {
        source: String,
        layer: String,
        detail: String,
    },
    /// An ordered solder-mask layer was not outside the conductor stack.
    SolderMaskSideIndeterminate(String),
    /// The required side-specific mask-opening union was already blocked.
    SolderMaskImageBlocked { layer: String, blocker: String },
    /// A decided mask-opening image could not be subtracted from its physical layer.
    SolderMaskSubtractionFailed { layer: String, detail: String },
    /// A custom physical layer used the board planform because no shape policy exists.
    CustomLayerUsesBoardPlanform(String),
    /// A declared package body outline could not produce material topology.
    InvalidComponentBody(String),
    /// An external package model was retained, but no resolver was supplied.
    ExternalPackageModelNotLoaded {
        /// Logical placed instance.
        instance: String,
        /// Zero-based authored model record.
        model_index: usize,
        /// Stable unresolved source URI.
        uri: String,
    },
    /// The resolver rejected or could not find the retained model URI.
    ExternalPackageModelResolutionFailed {
        /// Logical placed instance.
        instance: String,
        /// Zero-based authored model record.
        model_index: usize,
        /// Stable unresolved source URI.
        uri: String,
        /// Resolver-provided diagnostic.
        detail: String,
    },
    /// The source format is retained/exportable but has no native mesh loader.
    ExternalPackageModelFormatUnsupported {
        /// Logical placed instance.
        instance: String,
        /// Zero-based authored model record.
        model_index: usize,
        /// Stable unsupported source URI.
        uri: String,
        /// Explicit unsupported source container.
        format: Pcb3dModelFormat,
    },
    /// Resolved bytes were malformed for the declared source format.
    ExternalPackageModelParseFailed {
        /// Logical placed instance.
        instance: String,
        /// Zero-based authored model record.
        model_index: usize,
        /// Stable malformed source URI.
        uri: String,
        /// Native parser diagnostic.
        detail: String,
    },
}

/// Exact-Z 3D review assembly plus explicit unrealized detail.
#[derive(Clone, Debug)]
pub struct Pcb3dAssemblyReport {
    /// Independently inspectable physical layers in front-to-back order.
    pub layers: Vec<Pcb3dLayer>,
    /// Placed component body envelopes in logical-instance order.
    pub component_bodies: Vec<Pcb3dComponentBody>,
    /// Successfully resolved and placed external component meshes.
    pub component_models: Vec<Pcb3dComponentModel>,
    /// Digest, format, and triangle evidence for resolved component models.
    pub model_resolutions: Vec<Pcb3dModelResolutionEvidence>,
    /// Exact total stackup thickness.
    pub total_thickness: Real,
    /// Every successful drill or process-image subtraction by physical layer.
    pub subtractions: Vec<Pcb3dSubtractionEvidence>,
    /// Details not represented by the preview solids.
    pub omissions: Vec<Pcb3dAssemblyOmission>,
}

/// Semantic identity for one named object in a glTF review scene.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Pcb3dSceneObjectKind {
    /// One physical stackup layer.
    StackupLayer {
        /// Authored layer name.
        layer: String,
        /// Retained physical role.
        kind: Pcb3dLayerKind,
    },
    /// One native placed package-body envelope.
    ComponentBody {
        /// Logical circuit instance.
        instance: CircuitInstanceId,
        /// Reusable land pattern supplying the envelope.
        land_pattern: LandPatternId,
    },
    /// One resolved external component model.
    ComponentModel {
        /// Logical circuit instance.
        instance: CircuitInstanceId,
        /// Reusable land pattern supplying the reference.
        land_pattern: LandPatternId,
        /// Zero-based authored model record within the land pattern.
        model_index: usize,
        /// Stable resolved source URI.
        uri: String,
        /// SHA-256 of the exact resolved bytes.
        source_sha256: String,
    },
}

/// Audited named object emitted into a 3D review scene.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Pcb3dSceneObject {
    /// Collision-free glTF node and mesh name.
    pub name: String,
    /// Retained HyperCircuit semantic identity.
    pub kind: Pcb3dSceneObjectKind,
    /// Number of csgrs triangles serialized for this object.
    pub triangle_count: usize,
}

/// Finite coordinate encoding used at the glTF review boundary.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Pcb3dCoordinateEncoding {
    /// glTF 2.0 floating-point vertex attributes.
    Ieee754Binary32,
}

/// One complete named glTF scene plus semantic object and omission evidence.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Pcb3dGltfReport {
    /// Self-contained glTF 2.0 JSON with an embedded binary buffer.
    pub gltf: String,
    /// Stable semantic identity for every emitted node/mesh.
    pub objects: Vec<Pcb3dSceneObject>,
    /// Explicit finite-coordinate projection selected by the format.
    pub coordinate_encoding: Pcb3dCoordinateEncoding,
    /// Assembly details intentionally absent from the scene.
    pub assembly_omissions: Vec<Pcb3dAssemblyOmission>,
}

/// Failure to serialize an otherwise inspectable 3D review assembly.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Pcb3dGltfError {
    /// No layer or component solid was available.
    EmptyAssembly,
    /// The geometry-format adapter rejected a named object or finite coordinate.
    Geometry(String),
}

impl Display for Pcb3dGltfError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EmptyAssembly => formatter.write_str("3D review assembly has no scene objects"),
            Self::Geometry(detail) => {
                write!(formatter, "glTF scene serialization failed: {detail}")
            }
        }
    }
}

impl std::error::Error for Pcb3dGltfError {}

/// Failure while lowering retained PCB intent into geometry.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum GeometryMaterializationError {
    /// Structural circuit or layout validation failed.
    InvalidSourceModel,
    /// Exact division required by a feature could not be represented.
    Arithmetic,
    /// A polygon failed to produce material topology.
    InvalidPolygon(String),
    /// Route centerline construction failed.
    InvalidRoute(String),
    /// Exact route outlining was uncertain or unsupported.
    RouteOutline(String),
    /// A profile boolean failed.
    Boolean(String),
    /// Exact zone-island classification or area comparison was indeterminate.
    ZoneIsland(String),
}

impl Display for GeometryMaterializationError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidSourceModel => formatter.write_str("circuit or PCB layout is invalid"),
            Self::Arithmetic => formatter.write_str("PCB geometry arithmetic failed"),
            Self::InvalidPolygon(source) => write!(formatter, "invalid PCB polygon: {source}"),
            Self::InvalidRoute(source) => write!(formatter, "invalid PCB route: {source}"),
            Self::RouteOutline(source) => {
                write!(
                    formatter,
                    "PCB route outline could not be certified: {source}"
                )
            }
            Self::Boolean(source) => write!(formatter, "PCB profile boolean failed: {source}"),
            Self::ZoneIsland(source) => {
                write!(
                    formatter,
                    "PCB zone island policy could not be certified: {source}"
                )
            }
        }
    }
}

impl std::error::Error for GeometryMaterializationError {}

impl PcbLayout {
    /// Materializes substrate, copper, and drill geometry without discarding sources.
    pub fn materialize(
        &self,
        circuit: &Circuit,
        options: MaterializationOptions,
    ) -> Result<PcbMaterializationReport, GeometryMaterializationError> {
        if !circuit.validate().is_valid() || !self.validate(circuit).is_valid() {
            return Err(GeometryMaterializationError::InvalidSourceModel);
        }
        if options.circular_segments < 8
            || !options.route_arc_chord_error.is_finite()
            || options.route_arc_chord_error <= 0.0
            || !options.route_bezier_chord_error.is_finite()
            || options.route_bezier_chord_error <= 0.0
        {
            return Err(GeometryMaterializationError::InvalidSourceModel);
        }

        let boundary = self.outline.boundary_geometry().map_err(|error| {
            GeometryMaterializationError::InvalidPolygon(format!("board outline: {error}"))
        })?;
        let substrate = Profile::from_curve_region(boundary.region().clone());

        let mut copper_features = Vec::new();
        let mut process_features = Vec::new();
        let mut process_omissions = Vec::new();
        let mut projections = Vec::new();
        let mut zone_realizations = Vec::new();
        let mut drills = Vec::new();
        let stitching = self.realize_stitching_vias();
        materialize_placements(
            self,
            circuit,
            &options,
            &mut copper_features,
            &mut process_features,
            &mut process_omissions,
            &mut drills,
        )?;

        for route in &self.routes {
            let (profile, route_projections) = route_profile(route, &options)?;
            projections.extend(route_projections);
            copper_features.push(MaterializedCopperFeature {
                source: format!("route:{}", route.id.as_str()),
                net: Some(route.net.clone()),
                layer: route.layer,
                kind: CopperFeatureKind::Route,
                identity: MaterializedCopperIdentity::Route(route.id.clone()),
                anchor: route.segments[0].start().clone(),
                profile,
            });
        }

        let (front_layer, back_layer) = surface_copper_layers(self)?;
        let mut vias_with_unknown_mask_intent = 0;
        for via in self.vias.iter().chain(&stitching.vias) {
            let radius = half(&via.land_diameter)?;
            for layer in via.start_layer.0..=via.end_layer.0 {
                let profile = Profile::circle(radius.clone(), options.circular_segments).translate(
                    via.center.x.clone(),
                    via.center.y.clone(),
                    Real::zero(),
                );
                copper_features.push(MaterializedCopperFeature {
                    source: format!("via:{}", via.id.as_str()),
                    net: Some(via.net.clone()),
                    layer: TraceLayer(layer),
                    kind: CopperFeatureKind::Via,
                    identity: MaterializedCopperIdentity::Via(via.id.clone()),
                    anchor: via.center.clone(),
                    profile,
                });
            }
            drills.push(DrillHit {
                source: format!("via:{}", via.id.as_str()),
                center: via.center.clone(),
                shape: DrillShape::Round {
                    diameter: via.drill_diameter.clone(),
                },
                plating: via.plating,
            });
            let front_applicable = via.start_layer <= front_layer && front_layer <= via.end_layer;
            let back_applicable = via.start_layer <= back_layer && back_layer <= via.end_layer;
            let mut unknown_mask_intent = false;
            for (side, applicable, disposition) in [
                (BoardSide::Front, front_applicable, &via.mask.front),
                (BoardSide::Back, back_applicable, &via.mask.back),
            ] {
                if !applicable {
                    continue;
                }
                match disposition {
                    ViaMaskDisposition::Unspecified => unknown_mask_intent = true,
                    ViaMaskDisposition::Tented => {}
                    ViaMaskDisposition::Open { margin } => {
                        let opening_radius = radius.clone() + margin;
                        let profile = Profile::circle(opening_radius, options.circular_segments)
                            .translate(via.center.x.clone(), via.center.y.clone(), Real::zero());
                        process_features.push(MaterializedProcessFeature {
                            source: format!(
                                "mask:via:{}:{}",
                                via.id.as_str(),
                                match side {
                                    BoardSide::Front => "front",
                                    BoardSide::Back => "back",
                                }
                            ),
                            role: side_process_role(side, true),
                            kind: ProcessFeatureKind::ViaMaskOpening,
                            profile,
                        });
                    }
                }
            }
            if unknown_mask_intent {
                vias_with_unknown_mask_intent += 1;
            }
        }
        if vias_with_unknown_mask_intent != 0 {
            process_omissions.push(ProcessMaterializationOmission::ViaMaskIntentUnavailable {
                count: vias_with_unknown_mask_intent,
            });
        }

        let mut zones = self.zones.iter().collect::<Vec<_>>();
        zones.sort_by(|left, right| {
            right
                .priority
                .cmp(&left.priority)
                .then_with(|| left.id.as_str().cmp(right.id.as_str()))
        });
        for zone in zones {
            let (profile, evidence) = materialize_zone(self, zone, &substrate, &copper_features)?;
            copper_features.push(MaterializedCopperFeature {
                source: format!("zone:{}", zone.id.as_str()),
                net: Some(zone.net.clone()),
                layer: zone.layer,
                kind: CopperFeatureKind::Zone,
                identity: MaterializedCopperIdentity::Zone(zone.id.clone()),
                anchor: zone.boundary[0].clone(),
                profile,
            });
            zone_realizations.push(evidence);
        }

        let copper_layers = union_layer_images(&copper_features);
        let process_layers = union_process_images(&process_features);
        if process_features.iter().any(|feature| {
            matches!(
                feature.role,
                ProcessLayerRole::FrontSilkscreen | ProcessLayerRole::BackSilkscreen
            )
        }) {
            process_omissions.push(ProcessMaterializationOmission::SilkscreenNotClippedToMask);
        }
        Ok(PcbMaterializationReport {
            substrate,
            copper_features,
            copper_layers,
            process_features,
            process_layers,
            process_omissions,
            production_text: options.production_text.as_ref().map(|policy| {
                ProductionTextEvidence {
                    font_name: policy.font_name.clone(),
                    sha256: format!("{:x}", Sha256::digest(&policy.font_data)),
                }
            }),
            projections,
            zone_realizations,
            stitching_realizations: stitching.evidence,
            drills,
            preview_circular_segments: options.circular_segments,
            retained_keepout_count: self.keepouts.len(),
        })
    }
}

fn materialize_zone(
    layout: &PcbLayout,
    zone: &CopperZone,
    substrate: &Profile,
    existing: &[MaterializedCopperFeature],
) -> Result<(Profile, ZoneMaterializationEvidence), GeometryMaterializationError> {
    let source = format!("zone:{}", zone.id.as_str());
    let boundary = polygon_profile(&zone.boundary, &source)?;
    let boundary = zone_boolean(
        &source,
        "clip to substrate",
        boundary.try_intersection(substrate),
    )?;
    let mut profile = match &zone.fill {
        CopperZoneFill::Solid => boundary.clone(),
        CopperZoneFill::Hatched {
            line_width,
            gap,
            angle_degrees,
        } => hatch_zone(
            &source,
            &boundary,
            &zone.boundary,
            line_width,
            gap,
            angle_degrees,
        )?,
    };
    let mut keepout_profiles = Vec::new();
    for keepout in &layout.keepouts {
        let applies = match &keepout.scope {
            KeepoutScope::All => true,
            KeepoutScope::Copper(layers) => layers.contains(&zone.layer),
            KeepoutScope::Vias | KeepoutScope::Components => false,
        };
        if !applies {
            continue;
        }
        let keepout_profile = polygon_profile(
            &keepout.boundary,
            &format!("zone {} keepout {}", zone.id.as_str(), keepout.id.as_str()),
        )?;
        profile = zone_boolean(
            &source,
            &format!("subtract keepout {}", keepout.id.as_str()),
            profile.try_difference(&keepout_profile),
        )?;
        keepout_profiles.push((keepout.id.as_str(), keepout_profile));
    }
    let mut cleared_foreign_features = 0;
    let mut treated_same_net_lands = 0;
    let mut thermal_bounding_box_projections = 0;
    for feature in existing
        .iter()
        .filter(|feature| feature.layer == zone.layer)
    {
        if feature.net.as_ref() != Some(&zone.net) {
            let clearance = zone_offset(
                &source,
                &format!("foreign clearance for {}", feature.source),
                &feature.profile,
                &zone.clearance,
            )?;
            profile = zone_boolean(
                &source,
                &format!("subtract foreign feature {}", feature.source),
                profile.try_difference(&clearance),
            )?;
            cleared_foreign_features += 1;
            continue;
        }
        if !matches!(
            feature.kind,
            CopperFeatureKind::Pad | CopperFeatureKind::Via
        ) {
            continue;
        }
        match &zone.connection {
            CopperZoneConnection::Solid => {}
            CopperZoneConnection::Isolated => {
                let clearance = zone_offset(
                    &source,
                    &format!("same-net isolation for {}", feature.source),
                    &feature.profile,
                    &zone.clearance,
                )?;
                profile = zone_boolean(
                    &source,
                    &format!("isolate same-net feature {}", feature.source),
                    profile.try_difference(&clearance),
                )?;
                treated_same_net_lands += 1;
            }
            CopperZoneConnection::ThermalRelief {
                air_gap,
                spoke_width,
                spoke_count,
            } => {
                let clearance = zone_offset(
                    &source,
                    &format!("thermal air gap for {}", feature.source),
                    &feature.profile,
                    air_gap,
                )?;
                profile = zone_boolean(
                    &source,
                    &format!("subtract thermal gap for {}", feature.source),
                    profile.try_difference(&clearance),
                )?;
                let spokes = thermal_spoke_mask(
                    &zone.boundary,
                    &feature.anchor,
                    spoke_width,
                    *spoke_count,
                    &clearance,
                )?;
                let mut spokes = zone_boolean(
                    &source,
                    &format!("clip thermal spokes to boundary for {}", feature.source),
                    spokes.try_intersection(&boundary),
                )?;
                for (keepout, keepout_profile) in &keepout_profiles {
                    spokes = zone_boolean(
                        &source,
                        &format!(
                            "subtract keepout {keepout} from thermal spokes for {}",
                            feature.source
                        ),
                        spokes.try_difference(keepout_profile),
                    )?;
                }
                profile = zone_boolean(
                    &source,
                    &format!("merge thermal spokes for {}", feature.source),
                    profile.try_union(&spokes),
                )?;
                treated_same_net_lands += 1;
                thermal_bounding_box_projections += 1;
            }
        }
    }

    let (profile, islands) = apply_zone_island_policy(&source, zone, profile, existing)?;

    Ok((
        profile,
        ZoneMaterializationEvidence {
            source,
            priority: zone.priority,
            fill: match zone.fill {
                CopperZoneFill::Solid => "solid",
                CopperZoneFill::Hatched { .. } => "hatched",
            }
            .into(),
            connection: match zone.connection {
                CopperZoneConnection::Solid => "solid",
                CopperZoneConnection::Isolated => "isolated",
                CopperZoneConnection::ThermalRelief { .. } => "thermal-relief",
            }
            .into(),
            clearance: format!("{}", zone.clearance),
            cleared_foreign_features,
            treated_same_net_lands,
            thermal_bounding_box_projections,
            applied_keepouts: keepout_profiles.len(),
            initial_islands: islands.initial,
            retained_islands: islands.retained,
            pruned_unconnected_islands: islands.pruned_unconnected,
            pruned_below_area_islands: islands.pruned_below_area,
        },
    ))
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct ZoneIslandRealization {
    initial: usize,
    retained: usize,
    pruned_unconnected: usize,
    pruned_below_area: usize,
}

fn apply_zone_island_policy(
    source: &str,
    zone: &CopperZone,
    profile: Profile,
    existing: &[MaterializedCopperFeature],
) -> Result<(Profile, ZoneIslandRealization), GeometryMaterializationError> {
    let policy = CurvePolicy::certified();
    let native = match profile
        .region_geometry()
        .native_contours_fast_path(&policy)
        .map_err(|error| {
            GeometryMaterializationError::ZoneIsland(format!(
                "{source} native contour extraction: {error}"
            ))
        })? {
        Classification::Decided(native) => native,
        Classification::Uncertain(reason) => {
            return Err(GeometryMaterializationError::ZoneIsland(format!(
                "{source} native contour extraction: {reason:?}"
            )));
        }
    };
    let native_region = LineArcRegion2::new(
        native.material_contours().to_vec(),
        native.hole_contours().to_vec(),
    );
    let components = match native_region.contour_profiles(&policy) {
        Classification::Decided(components) => components,
        Classification::Uncertain(reason) => {
            return Err(GeometryMaterializationError::ZoneIsland(format!(
                "{source} hole ownership: {reason:?}"
            )));
        }
    };
    let mut realization = ZoneIslandRealization {
        initial: components.len(),
        ..ZoneIslandRealization::default()
    };
    if !zone.islands.remove_unconnected && zone.islands.minimum_area.is_none() {
        realization.retained = components.len();
        return Ok((profile, realization));
    }

    let mut retained = None::<Profile>;
    for (index, component) in components.iter().enumerate() {
        let region = CurveRegion2::try_from_native_contours(
            vec![(*component.material).clone()],
            component.holes.iter().map(|hole| (*hole).clone()).collect(),
            &policy,
        )
        .map_err(|error| {
            GeometryMaterializationError::ZoneIsland(format!(
                "{source} component {index} reconstruction: {error}"
            ))
        })?;
        let area = match region.filled_area(&policy).map_err(|error| {
            GeometryMaterializationError::ZoneIsland(format!(
                "{source} component {index} area: {error}"
            ))
        })? {
            Classification::Decided(Some(area)) => area,
            Classification::Decided(None) => {
                return Err(GeometryMaterializationError::ZoneIsland(format!(
                    "{source} component {index} area unsupported"
                )));
            }
            Classification::Uncertain(reason) => {
                return Err(GeometryMaterializationError::ZoneIsland(format!(
                    "{source} component {index} area: {reason:?}"
                )));
            }
        };
        let below_area = match &zone.islands.minimum_area {
            Some(minimum) => {
                area.partial_cmp(minimum).ok_or_else(|| {
                    GeometryMaterializationError::ZoneIsland(format!(
                        "{source} component {index} area ordering"
                    ))
                })? == std::cmp::Ordering::Less
            }
            None => false,
        };
        let component_profile = Profile::from_curve_region(region);
        let connected = if zone.islands.remove_unconnected {
            let mut connected = false;
            for feature in existing.iter().filter(|feature| {
                feature.layer == zone.layer && feature.net.as_ref() == Some(&zone.net)
            }) {
                let intersection = zone_boolean(
                    source,
                    &format!("classify island {index} connection to {}", feature.source),
                    component_profile.try_intersection(&feature.profile),
                )?;
                if !intersection.is_empty() {
                    connected = true;
                    break;
                }
            }
            connected
        } else {
            true
        };

        let remove = !connected
            && zone
                .islands
                .minimum_area
                .as_ref()
                .is_none_or(|_| below_area);
        if remove {
            realization.pruned_unconnected += 1;
            if zone.islands.minimum_area.is_some() {
                realization.pruned_below_area += 1;
            }
            continue;
        }
        retained = Some(match retained {
            Some(current) => zone_boolean(
                source,
                &format!("merge retained island {index}"),
                current.try_union(&component_profile),
            )?,
            None => component_profile,
        });
        realization.retained += 1;
    }

    Ok((retained.unwrap_or_else(Profile::empty), realization))
}

fn hatch_zone(
    source: &str,
    boundary: &Profile,
    points: &[Point2],
    line_width: &Real,
    gap: &Real,
    angle_degrees: &Real,
) -> Result<Profile, GeometryMaterializationError> {
    let (center, extent) = zone_center_extent(points)?;
    let pitch = line_width.clone() + gap.clone();
    let mut offset = -extent.clone();
    let mut stripes: Option<Profile> = None;
    while offset <= extent {
        let stripe = Profile::rectangle(extent.clone() + extent.clone(), line_width.clone())
            .translate(Real::zero(), offset.clone(), Real::zero())
            .rotate(Real::zero(), Real::zero(), angle_degrees.clone())
            .translate(center.x.clone(), center.y.clone(), Real::zero());
        stripes = Some(match stripes {
            None => stripe,
            Some(existing) => {
                zone_boolean(source, "merge hatch stripes", existing.try_union(&stripe))?
            }
        });
        offset += pitch.clone();
    }
    let stripes = stripes.ok_or_else(|| {
        GeometryMaterializationError::InvalidPolygon(format!("{source} hatch produced no stripes"))
    })?;
    zone_boolean(
        source,
        "clip hatch to boundary",
        stripes.try_intersection(boundary),
    )
}

fn thermal_spoke_mask(
    zone_boundary: &[Point2],
    center: &Point2,
    width: &Real,
    count: u8,
    clearance: &Profile,
) -> Result<Profile, GeometryMaterializationError> {
    let (_, extent) = zone_center_extent(zone_boundary)?;
    let step = (Real::from(360) / Real::from(count))
        .map_err(|_| GeometryMaterializationError::Arithmetic)?;
    let mut spokes: Option<Profile> = None;
    for index in 0..count {
        let angle = step.clone() * Real::from(index);
        let spoke = Profile::rectangle(extent.clone() + extent.clone(), width.clone())
            .rotate(Real::zero(), Real::zero(), angle)
            .translate(center.x.clone(), center.y.clone(), Real::zero());
        spokes = Some(match spokes {
            None => spoke,
            Some(existing) => zone_boolean(
                "thermal-spokes",
                "merge spoke masks",
                existing.try_union(&spoke),
            )?,
        });
    }
    let spokes = spokes.ok_or(GeometryMaterializationError::Arithmetic)?;
    let bounds = clearance.bounding_box();
    let width = bounds.maxs.x.clone() - bounds.mins.x.clone();
    let height = bounds.maxs.y.clone() - bounds.mins.y.clone();
    let two = Real::from(2);
    let clip_center = Point2::new(
        ((bounds.mins.x + bounds.maxs.x) / two.clone())
            .map_err(|_| GeometryMaterializationError::Arithmetic)?,
        ((bounds.mins.y + bounds.maxs.y) / two)
            .map_err(|_| GeometryMaterializationError::Arithmetic)?,
    );
    let clip =
        Profile::rectangle(width, height).translate(clip_center.x, clip_center.y, Real::zero());
    zone_boolean(
        "thermal-spokes",
        "clip spoke masks to land clearance bounds",
        spokes.try_intersection(&clip),
    )
}

fn zone_center_extent(points: &[Point2]) -> Result<(Point2, Real), GeometryMaterializationError> {
    let first = points
        .first()
        .ok_or_else(|| GeometryMaterializationError::InvalidPolygon("empty zone".into()))?;
    let mut min_x = first.x.clone();
    let mut min_y = first.y.clone();
    let mut max_x = first.x.clone();
    let mut max_y = first.y.clone();
    for point in points.iter().skip(1) {
        if point.x < min_x {
            min_x = point.x.clone();
        }
        if point.y < min_y {
            min_y = point.y.clone();
        }
        if point.x > max_x {
            max_x = point.x.clone();
        }
        if point.y > max_y {
            max_y = point.y.clone();
        }
    }
    let two = Real::from(2);
    let center = Point2::new(
        ((min_x.clone() + max_x.clone()) / two.clone())
            .map_err(|_| GeometryMaterializationError::Arithmetic)?,
        ((min_y.clone() + max_y.clone()) / two)
            .map_err(|_| GeometryMaterializationError::Arithmetic)?,
    );
    let extent = (max_x - min_x) + (max_y - min_y);
    Ok((center, extent))
}

fn zone_offset(
    source: &str,
    operation: &str,
    profile: &Profile,
    distance: &Real,
) -> Result<Profile, GeometryMaterializationError> {
    profile.try_offset(distance.clone()).map_err(|error| {
        GeometryMaterializationError::Boolean(format!("{source} {operation}: {error}"))
    })
}

fn zone_boolean(
    source: &str,
    operation: &str,
    result: Result<Profile, csgrs::errors::ProfileBooleanError>,
) -> Result<Profile, GeometryMaterializationError> {
    result.map_err(|error| {
        GeometryMaterializationError::Boolean(format!("{source} {operation}: {error:?}"))
    })
}

impl PcbMaterializationReport {
    /// Extrudes decided materialized images into an exact-Z stackup preview.
    ///
    /// Layer solids remain separate so metadata and unresolved copper never
    /// disappear into a boolean assembly. Retained drills, routed slots, and
    /// decided side-specific solder-mask openings are subtracted before
    /// extrusion. This remains a review artifact, not STEP or manufacturing
    /// evidence; any exact-boolean failure is retained in
    /// [`Pcb3dAssemblyReport::omissions`].
    pub fn stackup_3d(&self, layout: &PcbLayout) -> Pcb3dAssemblyReport {
        self.stackup_3d_internal(layout, None)
    }

    /// Extrudes the stackup and resolves supported external package models.
    ///
    /// Explicit Wavefront OBJ, VRML/WRL indexed-face scenes, and self-contained
    /// glTF/GLB triangle scenes are parsed today. STEP references remain typed
    /// omissions rather than being relabeled as mesh support.
    pub fn stackup_3d_with_model_resolver<R: Pcb3dModelResolver>(
        &self,
        layout: &PcbLayout,
        resolver: &mut R,
    ) -> Pcb3dAssemblyReport {
        self.stackup_3d_internal(layout, Some(resolver))
    }

    fn stackup_3d_internal(
        &self,
        layout: &PcbLayout,
        mut resolver: Option<&mut dyn Pcb3dModelResolver>,
    ) -> Pcb3dAssemblyReport {
        let mut layers = Vec::new();
        let mut omissions = Vec::new();
        let mut subtractions = Vec::new();
        let mut z_start = Real::zero();
        let first_conductor = layout
            .stackup
            .layers
            .iter()
            .position(|layer| matches!(layer.kind, crate::StackupLayerKind::Conductor(_)));
        let last_conductor = layout
            .stackup
            .layers
            .iter()
            .rposition(|layer| matches!(layer.kind, crate::StackupLayerKind::Conductor(_)));
        let mut drill_profiles = Vec::new();
        for drill in &self.drills {
            match preview_drill_profile(drill, self.preview_circular_segments) {
                Ok(profile) => drill_profiles.push((drill, profile)),
                Err(error) => omissions.push(Pcb3dAssemblyOmission::InvalidDrillGeometry {
                    source: drill.source.clone(),
                    detail: error.to_string(),
                }),
            }
        }

        for (layer_index, layer) in layout.stackup.layers.iter().enumerate() {
            let kind = match &layer.kind {
                crate::StackupLayerKind::Conductor(routing_layer) => Pcb3dLayerKind::Copper {
                    routing_layer: *routing_layer,
                },
                crate::StackupLayerKind::Dielectric => Pcb3dLayerKind::Dielectric,
                crate::StackupLayerKind::SolderMask => Pcb3dLayerKind::SolderMask,
                crate::StackupLayerKind::Custom(name) => {
                    omissions.push(Pcb3dAssemblyOmission::CustomLayerUsesBoardPlanform(
                        layer.name.clone(),
                    ));
                    Pcb3dLayerKind::Custom(name.clone())
                }
            };
            let profile = match &kind {
                Pcb3dLayerKind::Copper { routing_layer } => {
                    match self
                        .copper_layers
                        .iter()
                        .find(|image| image.layer == *routing_layer)
                    {
                        Some(image) => {
                            if let Some(blocker) = &image.blocker {
                                omissions.push(Pcb3dAssemblyOmission::BlockedCopperLayer {
                                    layer: routing_layer.0,
                                    blocker: blocker.clone(),
                                });
                                None
                            } else if let Some(copper) = &image.copper {
                                Some(copper.clone())
                            } else {
                                omissions.push(Pcb3dAssemblyOmission::EmptyCopperLayer {
                                    layer: routing_layer.0,
                                });
                                None
                            }
                        }
                        None => {
                            omissions.push(Pcb3dAssemblyOmission::EmptyCopperLayer {
                                layer: routing_layer.0,
                            });
                            None
                        }
                    }
                }
                Pcb3dLayerKind::Dielectric
                | Pcb3dLayerKind::SolderMask
                | Pcb3dLayerKind::Custom(_) => Some(self.substrate.clone()),
            };
            if let Some(mut profile) = profile {
                if matches!(kind, Pcb3dLayerKind::SolderMask) {
                    let role = match (first_conductor, last_conductor) {
                        (Some(first), _) if layer_index < first => {
                            Some(ProcessLayerRole::FrontSolderMask)
                        }
                        (_, Some(last)) if layer_index > last => {
                            Some(ProcessLayerRole::BackSolderMask)
                        }
                        _ => {
                            omissions.push(Pcb3dAssemblyOmission::SolderMaskSideIndeterminate(
                                layer.name.clone(),
                            ));
                            None
                        }
                    };
                    if let Some(role) = role
                        && let Some(image) =
                            self.process_layers.iter().find(|image| image.role == role)
                    {
                        if let Some(blocker) = &image.blocker {
                            omissions.push(Pcb3dAssemblyOmission::SolderMaskImageBlocked {
                                layer: layer.name.clone(),
                                blocker: blocker.clone(),
                            });
                        } else if let Some(openings) = &image.image {
                            match profile.try_difference(openings) {
                                Ok(realized) => {
                                    profile = realized;
                                    subtractions.push(Pcb3dSubtractionEvidence {
                                        layer: layer.name.clone(),
                                        kind: Pcb3dSubtractionKind::SolderMaskOpenings {
                                            role,
                                            source_feature_count: image.source_feature_count,
                                        },
                                    });
                                }
                                Err(error) => omissions.push(
                                    Pcb3dAssemblyOmission::SolderMaskSubtractionFailed {
                                        layer: layer.name.clone(),
                                        detail: format!("{error:?}"),
                                    },
                                ),
                            }
                        }
                    }
                }
                for (drill, cutter) in &drill_profiles {
                    match profile.try_difference(cutter) {
                        Ok(realized) => {
                            profile = realized;
                            subtractions.push(Pcb3dSubtractionEvidence {
                                layer: layer.name.clone(),
                                kind: Pcb3dSubtractionKind::Drill {
                                    source: drill.source.clone(),
                                    plating: drill.plating,
                                },
                            });
                        }
                        Err(error) => {
                            omissions.push(Pcb3dAssemblyOmission::DrillSubtractionFailed {
                                source: drill.source.clone(),
                                layer: layer.name.clone(),
                                detail: format!("{error:?}"),
                            });
                        }
                    }
                }
                if profile.is_empty() {
                    z_start += layer.thickness.clone();
                    continue;
                }
                let metadata = Pcb3dLayerMetadata {
                    name: layer.name.clone(),
                    kind,
                    z_start: z_start.clone(),
                    thickness: layer.thickness.clone(),
                };
                let solid = profile
                    .extrude(layer.thickness.clone(), metadata.clone())
                    .translate(Real::zero(), Real::zero(), z_start.clone());
                layers.push(Pcb3dLayer { metadata, solid });
            }
            z_start += layer.thickness.clone();
        }
        let mut component_bodies = Vec::new();
        let mut component_models = Vec::new();
        let mut model_resolutions = Vec::new();
        for placement in &layout.placements {
            let pattern = layout
                .land_patterns
                .iter()
                .find(|pattern| pattern.id == placement.land_pattern)
                .expect("validated placement pattern exists");
            if let Some(body) = &pattern.body {
                let outline = body
                    .outline
                    .iter()
                    .map(|point| placement.transform_point(point))
                    .collect::<Vec<_>>();
                match polygon_profile(&outline, placement.instance.as_str()) {
                    Ok(profile) => {
                        let body_z = match placement.side {
                            BoardSide::Front => z_start.clone() + body.standoff.clone(),
                            BoardSide::Back => -body.standoff.clone() - body.height.clone(),
                        };
                        let metadata = Pcb3dComponentBodyMetadata {
                            instance: placement.instance.clone(),
                            land_pattern: placement.land_pattern.clone(),
                            z_start: body_z.clone(),
                            height: body.height.clone(),
                        };
                        let solid = profile
                            .extrude(body.height.clone(), metadata.clone())
                            .translate(Real::zero(), Real::zero(), body_z);
                        component_bodies.push(Pcb3dComponentBody { metadata, solid });
                    }
                    Err(_) => omissions.push(Pcb3dAssemblyOmission::InvalidComponentBody(
                        placement.instance.as_str().into(),
                    )),
                }
            }
            for (model_index, reference) in pattern.models.iter().enumerate() {
                let Some(model_resolver) = resolver.as_deref_mut() else {
                    omissions.push(Pcb3dAssemblyOmission::ExternalPackageModelNotLoaded {
                        instance: placement.instance.as_str().into(),
                        model_index,
                        uri: reference.uri.clone(),
                    });
                    continue;
                };
                match resolve_component_model(
                    model_resolver,
                    reference,
                    placement,
                    &pattern.id,
                    model_index,
                    &z_start,
                ) {
                    Ok((model, evidence)) => {
                        component_models.push(model);
                        model_resolutions.push(evidence);
                    }
                    Err(omission) => omissions.push(omission),
                }
            }
        }
        Pcb3dAssemblyReport {
            layers,
            component_bodies,
            component_models,
            model_resolutions,
            total_thickness: z_start,
            subtractions,
            omissions,
        }
    }
}

fn resolve_component_model(
    resolver: &mut dyn Pcb3dModelResolver,
    reference: &Pcb3dModelReference,
    placement: &PcbPlacement,
    land_pattern: &LandPatternId,
    model_index: usize,
    board_thickness: &Real,
) -> Result<(Pcb3dComponentModel, Pcb3dModelResolutionEvidence), Pcb3dAssemblyOmission> {
    if !matches!(
        reference.format,
        Pcb3dModelFormat::WavefrontObj | Pcb3dModelFormat::Vrml | Pcb3dModelFormat::Gltf
    ) {
        return Err(
            Pcb3dAssemblyOmission::ExternalPackageModelFormatUnsupported {
                instance: placement.instance.as_str().into(),
                model_index,
                uri: reference.uri.clone(),
                format: reference.format,
            },
        );
    }
    let bytes = resolver.resolve(reference).map_err(|detail| {
        Pcb3dAssemblyOmission::ExternalPackageModelResolutionFailed {
            instance: placement.instance.as_str().into(),
            model_index,
            uri: reference.uri.clone(),
            detail,
        }
    })?;
    let source_sha256 = format!("{:x}", Sha256::digest(&bytes));
    let metadata = Pcb3dComponentModelMetadata {
        instance: placement.instance.clone(),
        land_pattern: land_pattern.clone(),
        model_index,
        reference: reference.clone(),
        source_sha256: source_sha256.clone(),
    };
    let parse_failure = |error: String| Pcb3dAssemblyOmission::ExternalPackageModelParseFailed {
        instance: placement.instance.as_str().into(),
        model_index,
        uri: reference.uri.clone(),
        detail: error,
    };
    let (
        source_mesh,
        source_scene_index,
        source_mesh_node_count,
        source_primitive_count,
        ignored_non_mesh_geometry_count,
        ignored_degenerate_polygon_count,
        ignored_degenerate_triangle_count,
    ) = match reference.format {
        Pcb3dModelFormat::WavefrontObj => (
            Mesh::from_obj(
                BufReader::new(Cursor::new(bytes.as_slice())),
                metadata.clone(),
            )
            .map_err(|error| parse_failure(error.to_string()))?,
            None,
            1,
            1,
            0,
            0,
            0,
        ),
        Pcb3dModelFormat::Vrml => {
            let imported = csgrs::io::vrml::from_vrml(&bytes, metadata.clone())
                .map_err(|error| parse_failure(error.to_string()))?;
            (
                imported.mesh,
                None,
                imported.shape_count,
                imported.indexed_face_set_count,
                imported.ignored_non_mesh_geometry_count,
                imported.ignored_degenerate_polygon_count,
                imported.ignored_degenerate_triangle_count,
            )
        }
        Pcb3dModelFormat::Gltf => {
            let imported = csgrs::io::gltf::from_gltf(&bytes, metadata.clone())
                .map_err(|error| parse_failure(error.to_string()))?;
            (
                imported.mesh,
                Some(imported.scene_index),
                imported.mesh_node_count,
                imported.primitive_count,
                0,
                0,
                0,
            )
        }
        _ => unreachable!("unsupported formats return before resolution"),
    };
    if source_mesh.triangles().is_empty() {
        return Err(Pcb3dAssemblyOmission::ExternalPackageModelParseFailed {
            instance: placement.instance.as_str().into(),
            model_index,
            uri: reference.uri.clone(),
            detail: "source container contains no triangles".into(),
        });
    }
    let source_triangle_count = source_mesh.triangles().len();
    let transform = &reference.transform;
    let mut mesh = source_mesh
        .scale(
            transform.scale_x.clone(),
            transform.scale_y.clone(),
            transform.scale_z.clone(),
        )
        .rotate(
            transform.rotate_x_degrees.clone(),
            transform.rotate_y_degrees.clone(),
            transform.rotate_z_degrees.clone(),
        )
        .translate(
            transform.offset_x.clone(),
            transform.offset_y.clone(),
            transform.offset_z.clone(),
        );
    let mounting_z = match placement.side {
        BoardSide::Front => board_thickness.clone(),
        BoardSide::Back => {
            mesh = mesh.scale(-Real::one(), Real::one(), -Real::one());
            Real::zero()
        }
    };
    mesh = mesh
        .rotate(
            Real::zero(),
            Real::zero(),
            placement.rotation_degrees.clone(),
        )
        .translate(
            placement.position.x.clone(),
            placement.position.y.clone(),
            mounting_z,
        );
    let evidence = Pcb3dModelResolutionEvidence {
        instance: placement.instance.clone(),
        land_pattern: land_pattern.clone(),
        model_index,
        uri: reference.uri.clone(),
        format: reference.format,
        source_sha256,
        triangle_count: source_triangle_count,
        source_scene_index,
        source_mesh_node_count,
        source_primitive_count,
        ignored_non_mesh_geometry_count,
        ignored_degenerate_polygon_count,
        ignored_degenerate_triangle_count,
    };
    Ok((Pcb3dComponentModel { metadata, mesh }, evidence))
}

impl Pcb3dAssemblyReport {
    /// Serializes every independently named layer and component body into one
    /// self-contained glTF 2.0 review scene.
    ///
    /// Exact source geometry remains in this report. The returned artifact
    /// explicitly records glTF's finite binary32 coordinate projection and
    /// carries the assembly omissions alongside stable semantic node identity.
    pub fn to_gltf(&self, scene_name: &str) -> Result<Pcb3dGltfReport, Pcb3dGltfError> {
        let mut geometry = Vec::with_capacity(
            self.layers.len() + self.component_bodies.len() + self.component_models.len(),
        );
        let mut objects = Vec::with_capacity(geometry.capacity());
        for (index, layer) in self.layers.iter().enumerate() {
            let name = format!("layer:{index}:{}", layer.metadata.name);
            geometry.push(csgrs::io::gltf::GltfSceneObject::new(
                name.clone(),
                &layer.solid,
            ));
            objects.push(Pcb3dSceneObject {
                name,
                kind: Pcb3dSceneObjectKind::StackupLayer {
                    layer: layer.metadata.name.clone(),
                    kind: layer.metadata.kind.clone(),
                },
                triangle_count: layer.solid.triangles().len(),
            });
        }
        let resolved_instances = self
            .component_models
            .iter()
            .map(|model| &model.metadata.instance)
            .collect::<BTreeSet<_>>();
        for body in &self.component_bodies {
            if resolved_instances.contains(&body.metadata.instance) {
                continue;
            }
            let name = format!("component:{}", body.metadata.instance.as_str());
            geometry.push(csgrs::io::gltf::GltfSceneObject::new(
                name.clone(),
                &body.solid,
            ));
            objects.push(Pcb3dSceneObject {
                name,
                kind: Pcb3dSceneObjectKind::ComponentBody {
                    instance: body.metadata.instance.clone(),
                    land_pattern: body.metadata.land_pattern.clone(),
                },
                triangle_count: body.solid.triangles().len(),
            });
        }
        for model in &self.component_models {
            let name = format!(
                "component-model:{}:{}",
                model.metadata.instance.as_str(),
                model.metadata.model_index
            );
            geometry.push(csgrs::io::gltf::GltfSceneObject::new(
                name.clone(),
                &model.mesh,
            ));
            objects.push(Pcb3dSceneObject {
                name,
                kind: Pcb3dSceneObjectKind::ComponentModel {
                    instance: model.metadata.instance.clone(),
                    land_pattern: model.metadata.land_pattern.clone(),
                    model_index: model.metadata.model_index,
                    uri: model.metadata.reference.uri.clone(),
                    source_sha256: model.metadata.source_sha256.clone(),
                },
                triangle_count: model.mesh.triangles().len(),
            });
        }
        if geometry.is_empty() {
            return Err(Pcb3dGltfError::EmptyAssembly);
        }
        let gltf = csgrs::io::gltf::to_gltf_scene(scene_name, &geometry)
            .map_err(|error| Pcb3dGltfError::Geometry(error.to_string()))?;
        Ok(Pcb3dGltfReport {
            gltf,
            objects,
            coordinate_encoding: Pcb3dCoordinateEncoding::Ieee754Binary32,
            assembly_omissions: self.omissions.clone(),
        })
    }
}

fn preview_drill_profile(
    drill: &DrillHit,
    circular_segments: usize,
) -> Result<Profile, GeometryMaterializationError> {
    match &drill.shape {
        DrillShape::Round { diameter } => Ok(Profile::circle(half(diameter)?, circular_segments)
            .translate(drill.center.x.clone(), drill.center.y.clone(), Real::zero())),
        DrillShape::Slot { start, end, width } => stroked_path_profile(
            &[start.clone(), end.clone()],
            false,
            width,
            circular_segments,
            &drill.source,
        ),
    }
}

fn materialize_placements(
    layout: &PcbLayout,
    circuit: &Circuit,
    options: &MaterializationOptions,
    copper_features: &mut Vec<MaterializedCopperFeature>,
    process_features: &mut Vec<MaterializedProcessFeature>,
    process_omissions: &mut Vec<ProcessMaterializationOmission>,
    drills: &mut Vec<DrillHit>,
) -> Result<(), GeometryMaterializationError> {
    let (front_layer, back_layer) = surface_copper_layers(layout)?;
    for placement in &layout.placements {
        let pattern = layout
            .land_patterns
            .iter()
            .find(|pattern| pattern.id == placement.land_pattern)
            .ok_or(GeometryMaterializationError::InvalidSourceModel)?;
        let instance = circuit
            .instances
            .iter()
            .find(|instance| instance.id == placement.instance)
            .ok_or(GeometryMaterializationError::InvalidSourceModel)?;
        for pad in &pattern.pads {
            let pin = pattern
                .pin_map
                .iter()
                .find(|mapping| mapping.pad == pad.id)
                .map(|mapping| mapping.pin.clone());
            let net = pin
                .as_ref()
                .and_then(|pin| instance.pins.iter().find(|binding| binding.pin == *pin))
                .map(|binding| binding.net.clone());
            let local_profile = pad_profile(pad, options)?;
            let profile = transform_pad_profile(local_profile.clone(), pad, placement);
            let source = format!(
                "pad:{}:{}:{}",
                placement.instance.as_str(),
                pattern.id.as_str(),
                pad.id.as_str()
            );
            let mut surface_roles = Vec::new();
            for layer in &pad.copper_layers {
                let placed = placed_layer(layout, *layer, placement.side);
                copper_features.push(MaterializedCopperFeature {
                    source: source.clone(),
                    net: net.clone(),
                    layer: placed,
                    kind: CopperFeatureKind::Pad,
                    identity: MaterializedCopperIdentity::Pad {
                        instance: placement.instance.clone(),
                        land_pattern: pattern.id.clone(),
                        pad: pad.id.clone(),
                        pin: pin.clone(),
                    },
                    anchor: transform_point(&pad.center, placement),
                    profile: profile.clone(),
                });
                if placed == front_layer && !surface_roles.contains(&BoardSide::Front) {
                    surface_roles.push(BoardSide::Front);
                }
                if placed == back_layer
                    && back_layer != front_layer
                    && !surface_roles.contains(&BoardSide::Back)
                {
                    surface_roles.push(BoardSide::Back);
                }
            }
            for side in surface_roles {
                let mask_margin = pad
                    .solder_mask_margin
                    .as_ref()
                    .unwrap_or(&options.default_solder_mask_margin);
                let mask = local_profile
                    .try_offset(mask_margin.clone())
                    .map_err(|error| {
                        GeometryMaterializationError::Boolean(format!(
                            "{source} solder-mask offset: {error}"
                        ))
                    })?;
                if !mask.is_empty() {
                    process_features.push(MaterializedProcessFeature {
                        source: format!("mask:{source}"),
                        role: side_process_role(side, true),
                        kind: ProcessFeatureKind::PadMaskOpening,
                        profile: transform_pad_profile(mask, pad, placement),
                    });
                }
                if pad.drill.is_none() {
                    let paste_margin = pad
                        .paste_margin
                        .as_ref()
                        .unwrap_or(&options.default_paste_margin);
                    let paste =
                        local_profile
                            .try_offset(paste_margin.clone())
                            .map_err(|error| {
                                GeometryMaterializationError::Boolean(format!(
                                    "{source} paste offset: {error}"
                                ))
                            })?;
                    if !paste.is_empty() {
                        process_features.push(MaterializedProcessFeature {
                            source: format!("paste:{source}"),
                            role: side_process_role(side, false),
                            kind: ProcessFeatureKind::PadPasteAperture,
                            profile: transform_pad_profile(paste, pad, placement),
                        });
                    }
                }
            }
            if let Some(drill) = &pad.drill {
                drills.push(transform_drill(pad, drill, placement));
            }
        }
        materialize_pattern_graphics(
            layout,
            pattern,
            placement,
            options,
            copper_features,
            process_features,
            process_omissions,
        )?;
    }
    Ok(())
}

fn surface_copper_layers(
    layout: &PcbLayout,
) -> Result<(TraceLayer, TraceLayer), GeometryMaterializationError> {
    let layers = layout
        .stackup
        .layers
        .iter()
        .filter_map(|layer| match layer.kind {
            crate::StackupLayerKind::Conductor(layer) => Some(layer),
            _ => None,
        })
        .collect::<Vec<_>>();
    let Some(front) = layers.first().copied() else {
        return Err(GeometryMaterializationError::InvalidSourceModel);
    };
    Ok((front, layers.last().copied().unwrap_or(front)))
}

fn side_process_role(side: BoardSide, mask: bool) -> ProcessLayerRole {
    match (side, mask) {
        (BoardSide::Front, true) => ProcessLayerRole::FrontSolderMask,
        (BoardSide::Back, true) => ProcessLayerRole::BackSolderMask,
        (BoardSide::Front, false) => ProcessLayerRole::FrontPaste,
        (BoardSide::Back, false) => ProcessLayerRole::BackPaste,
    }
}

fn materialize_pattern_graphics(
    layout: &PcbLayout,
    pattern: &crate::LandPattern,
    placement: &PcbPlacement,
    options: &MaterializationOptions,
    copper_features: &mut Vec<MaterializedCopperFeature>,
    process_features: &mut Vec<MaterializedProcessFeature>,
    omissions: &mut Vec<ProcessMaterializationOmission>,
) -> Result<(), GeometryMaterializationError> {
    for graphic in &pattern.graphics {
        let source = format!(
            "graphic:{}:{}:{}",
            placement.instance.as_str(),
            pattern.id.as_str(),
            graphic.id.as_str()
        );
        match &graphic.layer {
            LayerRole::EdgeCuts => {
                omissions.push(ProcessMaterializationOmission::PackageEdgeCuts { source });
                continue;
            }
            LayerRole::Custom(layer) => {
                omissions.push(ProcessMaterializationOmission::CustomArtworkLayer {
                    source,
                    layer: layer.clone(),
                });
                continue;
            }
            LayerRole::Fabrication | LayerRole::Courtyard => continue,
            _ => {}
        }
        let Some(profile) = graphic_profile(graphic, options, &source, omissions)? else {
            continue;
        };
        let profile = transform_local_profile(profile, placement);
        match &graphic.layer {
            LayerRole::Copper(layer) => copper_features.push(MaterializedCopperFeature {
                source,
                net: None,
                layer: placed_layer(layout, *layer, placement.side),
                kind: CopperFeatureKind::Artwork,
                identity: MaterializedCopperIdentity::Artwork {
                    instance: placement.instance.clone(),
                    land_pattern: pattern.id.clone(),
                    graphic: graphic.id.clone(),
                },
                anchor: placement.position.clone(),
                profile,
            }),
            LayerRole::FrontSolderMask
            | LayerRole::BackSolderMask
            | LayerRole::FrontPaste
            | LayerRole::BackPaste
            | LayerRole::FrontSilkscreen
            | LayerRole::BackSilkscreen => {
                let role = placed_process_role(&graphic.layer, placement.side)
                    .expect("matched production artwork role");
                process_features.push(MaterializedProcessFeature {
                    source,
                    role,
                    kind: ProcessFeatureKind::Artwork,
                    profile,
                });
            }
            LayerRole::EdgeCuts
            | LayerRole::Custom(_)
            | LayerRole::Fabrication
            | LayerRole::Courtyard => unreachable!("handled before geometry materialization"),
        }
    }
    Ok(())
}

fn placed_process_role(role: &LayerRole, side: BoardSide) -> Option<ProcessLayerRole> {
    let role = match role {
        LayerRole::FrontSolderMask => ProcessLayerRole::FrontSolderMask,
        LayerRole::BackSolderMask => ProcessLayerRole::BackSolderMask,
        LayerRole::FrontPaste => ProcessLayerRole::FrontPaste,
        LayerRole::BackPaste => ProcessLayerRole::BackPaste,
        LayerRole::FrontSilkscreen => ProcessLayerRole::FrontSilkscreen,
        LayerRole::BackSilkscreen => ProcessLayerRole::BackSilkscreen,
        _ => return None,
    };
    if side == BoardSide::Front {
        return Some(role);
    }
    Some(match role {
        ProcessLayerRole::FrontSolderMask => ProcessLayerRole::BackSolderMask,
        ProcessLayerRole::BackSolderMask => ProcessLayerRole::FrontSolderMask,
        ProcessLayerRole::FrontPaste => ProcessLayerRole::BackPaste,
        ProcessLayerRole::BackPaste => ProcessLayerRole::FrontPaste,
        ProcessLayerRole::FrontSilkscreen => ProcessLayerRole::BackSilkscreen,
        ProcessLayerRole::BackSilkscreen => ProcessLayerRole::FrontSilkscreen,
    })
}

fn graphic_profile(
    graphic: &LandPatternGraphic,
    options: &MaterializationOptions,
    source: &str,
    omissions: &mut Vec<ProcessMaterializationOmission>,
) -> Result<Option<Profile>, GeometryMaterializationError> {
    match &graphic.primitive {
        LandPatternGraphicPrimitive::Line { start, end } => {
            let Some(width) = &graphic.stroke_width else {
                omissions.push(ProcessMaterializationOmission::MissingArtworkStroke {
                    source: source.to_owned(),
                });
                return Ok(None);
            };
            stroked_path_profile(
                &[start.clone(), end.clone()],
                false,
                width,
                options.circular_segments,
                source,
            )
            .map(Some)
        }
        LandPatternGraphicPrimitive::Circle { center, radius } => {
            let Some(width) = &graphic.stroke_width else {
                omissions.push(ProcessMaterializationOmission::MissingArtworkStroke {
                    source: source.to_owned(),
                });
                return Ok(None);
            };
            let half_width = half(width)?;
            let outer = Profile::circle(
                radius.clone() + half_width.clone(),
                options.circular_segments,
            );
            let inner_radius = radius.clone() - half_width;
            let ring = if inner_radius <= Real::zero() {
                outer
            } else {
                let inner = Profile::circle(inner_radius, options.circular_segments);
                outer.try_difference(&inner).map_err(|error| {
                    GeometryMaterializationError::Boolean(format!("{source}: {error:?}"))
                })?
            };
            Ok(Some(ring.translate(
                center.x.clone(),
                center.y.clone(),
                Real::zero(),
            )))
        }
        LandPatternGraphicPrimitive::Polygon { vertices, filled } => {
            if *filled {
                let profile = polygon_profile(vertices, source)?;
                if let Some(width) = &graphic.stroke_width {
                    profile.try_offset(half(width)?).map(Some).map_err(|error| {
                        GeometryMaterializationError::Boolean(format!("{source}: {error}"))
                    })
                } else {
                    Ok(Some(profile))
                }
            } else {
                let Some(width) = &graphic.stroke_width else {
                    omissions.push(ProcessMaterializationOmission::MissingArtworkStroke {
                        source: source.to_owned(),
                    });
                    return Ok(None);
                };
                stroked_polygon_profile(vertices, width, options.circular_segments, source)
                    .map(Some)
            }
        }
        LandPatternGraphicPrimitive::Text {
            text,
            position,
            height,
            rotation_degrees,
        } => {
            let Some(policy) = &options.production_text else {
                omissions.push(ProcessMaterializationOmission::TextArtwork {
                    source: source.to_owned(),
                });
                return Ok(None);
            };
            if text.trim().is_empty() {
                return Ok(None);
            }
            // csgrs accepts typographic points per em. One point is exactly
            // 127/360 mm, so this converts the authored em height without a
            // floating policy at the hypercircuit boundary.
            let point_height = (height.clone() * Real::from(360) / Real::from(127))
                .map_err(|_| GeometryMaterializationError::Arithmetic)?;
            let profile = Profile::text(text, &policy.font_data, point_height)
                .rotate(Real::zero(), Real::zero(), rotation_degrees.clone())
                .translate(position.x.clone(), position.y.clone(), Real::zero());
            if profile.is_empty() {
                omissions.push(ProcessMaterializationOmission::TextFontRejected {
                    source: source.to_owned(),
                });
                Ok(None)
            } else {
                Ok(Some(profile))
            }
        }
    }
}

fn stroked_path_profile(
    points: &[Point2],
    closed: bool,
    width: &Real,
    circular_segments: usize,
    source: &str,
) -> Result<Profile, GeometryMaterializationError> {
    if !closed && points.len() == 2 && points[0].x != points[1].x && points[0].y != points[1].y {
        return straight_capsule_profile(&points[0], &points[1], width, circular_segments, source);
    }
    let mut segments = points
        .windows(2)
        .map(|pair| {
            LineSeg2::try_new(curve_point(&pair[0]), curve_point(&pair[1])).map(Segment2::Line)
        })
        .collect::<Result<Vec<_>, _>>()
        .map_err(|error| {
            GeometryMaterializationError::InvalidRoute(format!("{source}: {error}"))
        })?;
    if closed {
        let first = points
            .first()
            .ok_or_else(|| GeometryMaterializationError::InvalidPolygon(source.to_owned()))?;
        let last = points
            .last()
            .ok_or_else(|| GeometryMaterializationError::InvalidPolygon(source.to_owned()))?;
        segments.push(
            LineSeg2::try_new(curve_point(last), curve_point(first))
                .map(Segment2::Line)
                .map_err(|error| {
                    GeometryMaterializationError::InvalidRoute(format!("{source}: {error}"))
                })?,
        );
    }
    let centerline = CurveString2::try_new(segments).map_err(|error| {
        GeometryMaterializationError::InvalidRoute(format!("{source}: {error}"))
    })?;
    match centerline
        .offset_outline(half(width)?, OffsetCap::Round, &CurvePolicy::certified())
        .map_err(|error| GeometryMaterializationError::RouteOutline(format!("{source}: {error}")))?
    {
        Classification::Decided(contour) => Ok(Profile::from_contour(contour)),
        Classification::Uncertain(reason) => Err(GeometryMaterializationError::RouteOutline(
            format!("{source}: {reason:?}"),
        )),
    }
}

fn straight_capsule_profile(
    start: &Point2,
    end: &Point2,
    width: &Real,
    circular_segments: usize,
    source: &str,
) -> Result<Profile, GeometryMaterializationError> {
    let half_width = half(width)?;
    let dx = end.x.clone() - start.x.clone();
    let dy = end.y.clone() - start.y.clone();
    let length = (dx.clone() * dx.clone() + dy.clone() * dy.clone())
        .sqrt()
        .map_err(|error| {
            GeometryMaterializationError::RouteOutline(format!("{source}: {error}"))
        })?;
    if length <= Real::zero() {
        return Err(GeometryMaterializationError::InvalidRoute(
            source.to_owned(),
        ));
    }
    let tangent_x = ((dx.clone() * half_width.clone()) / length.clone())
        .map_err(|_| GeometryMaterializationError::Arithmetic)?;
    let tangent_y = ((dy.clone() * half_width.clone()) / length.clone())
        .map_err(|_| GeometryMaterializationError::Arithmetic)?;
    let normal_x = ((-dy * half_width.clone()) / length.clone())
        .map_err(|_| GeometryMaterializationError::Arithmetic)?;
    let normal_y = ((dx * half_width.clone()) / length)
        .map_err(|_| GeometryMaterializationError::Arithmetic)?;
    let cap_steps = circular_segments.max(4) / 2;
    let mut boundary = Vec::with_capacity(cap_steps.saturating_mul(2).saturating_add(3));
    boundary.push(curve_point(&Point2::new(
        start.x.clone() + normal_x.clone(),
        start.y.clone() + normal_y.clone(),
    )));
    boundary.push(curve_point(&Point2::new(
        end.x.clone() + normal_x.clone(),
        end.y.clone() + normal_y.clone(),
    )));
    for index in 1..=cap_steps {
        let angle = (Real::pi() * Real::from(index as u128) / Real::from(cap_steps as u128))
            .map_err(|_| GeometryMaterializationError::Arithmetic)?;
        let sin = angle.clone().sin();
        let cos = angle.cos();
        boundary.push(curve_point(&Point2::new(
            end.x.clone() + normal_x.clone() * cos.clone() + tangent_x.clone() * sin.clone(),
            end.y.clone() + normal_y.clone() * cos + tangent_y.clone() * sin,
        )));
    }
    boundary.push(curve_point(&Point2::new(
        start.x.clone() - normal_x.clone(),
        start.y.clone() - normal_y.clone(),
    )));
    for index in 1..cap_steps {
        let angle = (Real::pi() * Real::from(index as u128) / Real::from(cap_steps as u128))
            .map_err(|_| GeometryMaterializationError::Arithmetic)?;
        let sin = angle.clone().sin();
        let cos = angle.cos();
        boundary.push(curve_point(&Point2::new(
            start.x.clone() - normal_x.clone() * cos.clone() - tangent_x.clone() * sin.clone(),
            start.y.clone() - normal_y.clone() * cos - tangent_y.clone() * sin,
        )));
    }
    let capsule = Profile::polygon_points(&boundary);
    if capsule.is_empty() {
        return Err(GeometryMaterializationError::InvalidRoute(
            source.to_owned(),
        ));
    }
    Ok(capsule)
}

fn stroked_polygon_profile(
    points: &[Point2],
    width: &Real,
    circular_segments: usize,
    source: &str,
) -> Result<Profile, GeometryMaterializationError> {
    if points.len() < 3 {
        return Err(GeometryMaterializationError::InvalidPolygon(
            source.to_owned(),
        ));
    }
    let mut outline: Option<Profile> = None;
    for index in 0..points.len() {
        let edge = stroked_path_profile(
            &[
                points[index].clone(),
                points[(index + 1) % points.len()].clone(),
            ],
            false,
            width,
            circular_segments,
            source,
        )?;
        outline = Some(match outline {
            Some(existing) => existing.try_union(&edge).map_err(|error| {
                GeometryMaterializationError::Boolean(format!("{source}: {error:?}"))
            })?,
            None => edge,
        });
    }
    outline.ok_or_else(|| GeometryMaterializationError::InvalidPolygon(source.to_owned()))
}

fn pad_profile(
    pad: &LandPatternPad,
    options: &MaterializationOptions,
) -> Result<Profile, GeometryMaterializationError> {
    match &pad.shape {
        PadShape::Circle { diameter } => {
            Ok(Profile::circle(half(diameter)?, options.circular_segments))
        }
        PadShape::Rectangle { width, height } => Ok(center_pad_profile(
            Profile::rectangle(width.clone(), height.clone()),
            width,
            height,
        )?),
        PadShape::RoundedRectangle {
            width,
            height,
            corner_radius,
        } => Ok(center_pad_profile(
            Profile::rounded_rectangle(
                width.clone(),
                height.clone(),
                corner_radius.clone(),
                options.circular_segments / 4,
            ),
            width,
            height,
        )?),
        PadShape::Obround { width, height } => {
            let radius = half(if width <= height { width } else { height })?;
            Ok(center_pad_profile(
                Profile::rounded_rectangle(
                    width.clone(),
                    height.clone(),
                    radius,
                    options.circular_segments / 4,
                ),
                width,
                height,
            )?)
        }
        PadShape::Polygon { vertices } => polygon_profile(vertices, pad.id.as_str()),
    }
}

fn center_pad_profile(
    profile: Profile,
    width: &Real,
    height: &Real,
) -> Result<Profile, GeometryMaterializationError> {
    Ok(profile.translate(-half(width)?, -half(height)?, Real::zero()))
}

fn transform_pad_profile(
    profile: Profile,
    pad: &LandPatternPad,
    placement: &PcbPlacement,
) -> Profile {
    let local = profile
        .rotate(Real::zero(), Real::zero(), pad.rotation_degrees.clone())
        .translate(pad.center.x.clone(), pad.center.y.clone(), Real::zero());
    transform_local_profile(local, placement)
}

fn transform_local_profile(profile: Profile, placement: &PcbPlacement) -> Profile {
    let sided = match placement.side {
        BoardSide::Front => profile,
        BoardSide::Back => profile.scale(-Real::one(), Real::one(), Real::one()),
    };
    sided
        .rotate(
            Real::zero(),
            Real::zero(),
            placement.rotation_degrees.clone(),
        )
        .translate(
            placement.position.x.clone(),
            placement.position.y.clone(),
            Real::zero(),
        )
}

fn transform_drill(pad: &LandPatternPad, drill: &DrillShape, placement: &PcbPlacement) -> DrillHit {
    let center = transform_point(&pad.center, placement);
    let shape = match drill {
        DrillShape::Round { diameter } => DrillShape::Round {
            diameter: diameter.clone(),
        },
        DrillShape::Slot { start, end, width } => DrillShape::Slot {
            start: transform_point(
                &translate_pad_point(rotate_pad_point(start, pad), pad),
                placement,
            ),
            end: transform_point(
                &translate_pad_point(rotate_pad_point(end, pad), pad),
                placement,
            ),
            width: width.clone(),
        },
    };
    DrillHit {
        source: format!(
            "pad-drill:{}:{}",
            placement.instance.as_str(),
            pad.id.as_str()
        ),
        center,
        shape,
        plating: pad.plating,
    }
}

fn rotate_pad_point(point: &Point2, pad: &LandPatternPad) -> Point2 {
    let radians = pad.rotation_degrees.clone().to_radians();
    let sin = radians.clone().sin();
    let cos = radians.cos();
    Point2::new(
        point.x.clone() * cos.clone() - point.y.clone() * sin.clone(),
        point.x.clone() * sin + point.y.clone() * cos,
    )
}

fn translate_pad_point(point: Point2, pad: &LandPatternPad) -> Point2 {
    Point2::new(
        point.x + pad.center.x.clone(),
        point.y + pad.center.y.clone(),
    )
}

fn transform_point(point: &Point2, placement: &PcbPlacement) -> Point2 {
    placement.transform_point(point)
}

fn placed_layer(layout: &PcbLayout, layer: TraceLayer, side: BoardSide) -> TraceLayer {
    if side == BoardSide::Front {
        return layer;
    }
    let last = layout
        .stackup
        .layers
        .iter()
        .filter_map(|candidate| match candidate.kind {
            crate::layout::StackupLayerKind::Conductor(index) => Some(index.0),
            _ => None,
        })
        .max()
        .unwrap_or(layer.0);
    TraceLayer(last.saturating_sub(layer.0))
}

fn route_profile(
    route: &crate::layout::PcbRoute,
    options: &MaterializationOptions,
) -> Result<(Profile, Vec<MaterializationProjection>), GeometryMaterializationError> {
    let source = route.id.as_str();
    let has_curve = route.segments.iter().any(|segment| {
        matches!(
            segment,
            crate::PcbRouteSegment::CircularArc(_) | crate::PcbRouteSegment::CubicBezier(_)
        )
    });
    if !has_curve {
        let mut profile = None::<Profile>;
        for (index, segment) in route.segments.iter().enumerate() {
            let crate::PcbRouteSegment::Line(line) = segment else {
                unreachable!("curve-free routes contain only line segments");
            };
            let swept = stroked_path_profile(
                &[line.start().clone(), line.end().clone()],
                false,
                &route.width,
                options.circular_segments,
                &format!("{source} segment {index}"),
            )?;
            profile = Some(match profile {
                Some(existing) => existing.try_union(&swept).map_err(|error| {
                    GeometryMaterializationError::Boolean(format!(
                        "{source} merge segment {index}: {error:?}"
                    ))
                })?,
                None => swept,
            });
        }
        return profile
            .ok_or_else(|| GeometryMaterializationError::InvalidRoute(source.to_owned()))
            .map(|profile| (profile, Vec::new()));
    }
    let has_arc = route
        .segments
        .iter()
        .any(|segment| matches!(segment, crate::PcbRouteSegment::CircularArc(_)));
    let has_bezier = route
        .segments
        .iter()
        .any(|segment| matches!(segment, crate::PcbRouteSegment::CubicBezier(_)));
    let mut points = vec![route.segments[0].start().clone()];
    for segment in &route.segments {
        match segment {
            crate::PcbRouteSegment::Line(segment) => points.push(segment.end().clone()),
            crate::PcbRouteSegment::CircularArc(arc) => {
                let finite = |value: &Real| {
                    value
                        .to_f64_lossy()
                        .filter(|value| value.is_finite())
                        .ok_or(GeometryMaterializationError::Arithmetic)
                };
                let cx = finite(&arc.center().x)?;
                let cy = finite(&arc.center().y)?;
                let radius = finite(arc.radius())?;
                let sx = finite(&arc.start().x)?;
                let sy = finite(&arc.start().y)?;
                let ex = finite(&arc.end().x)?;
                let ey = finite(&arc.end().y)?;
                let start_angle = (sy - cy).atan2(sx - cx);
                let end_angle = (ey - cy).atan2(ex - cx);
                let mut sweep = if arc.direction() == hyperpath::ArcDirection::Ccw {
                    (end_angle - start_angle).rem_euclid(std::f64::consts::TAU)
                } else {
                    (start_angle - end_angle).rem_euclid(std::f64::consts::TAU)
                };
                if arc.start() == arc.end() {
                    sweep = std::f64::consts::TAU;
                }
                let chord_error = options.route_arc_chord_error.min(radius);
                let maximum_step = 2.0 * (1.0 - chord_error / radius).clamp(-1.0, 1.0).acos();
                let steps = if maximum_step.is_finite() && maximum_step > 0.0 {
                    (sweep / maximum_step).ceil().max(1.0) as usize
                } else {
                    1
                };
                for step in 1..=steps {
                    let fraction = step as f64 / steps as f64;
                    let angle = if arc.direction() == hyperpath::ArcDirection::Ccw {
                        start_angle + sweep * fraction
                    } else {
                        start_angle - sweep * fraction
                    };
                    points.push(Point2::new(
                        Real::try_from(cx + radius * angle.cos())
                            .map_err(|_| GeometryMaterializationError::Arithmetic)?,
                        Real::try_from(cy + radius * angle.sin())
                            .map_err(|_| GeometryMaterializationError::Arithmetic)?,
                    ));
                }
                if let Some(last) = points.last_mut() {
                    *last = arc.end().clone();
                }
            }
            crate::PcbRouteSegment::CubicBezier(bezier) => {
                sample_cubic_bezier(
                    bezier,
                    options.route_bezier_chord_error,
                    source,
                    &mut points,
                )?;
            }
        }
    }
    let profile = finite_stroked_polyline_profile(&points, &route.width, source)?;
    let mut projections = Vec::new();
    if has_arc {
        projections.push(MaterializationProjection::CircularRoutePolyline {
            source: source.to_owned(),
            chord_error: options.route_arc_chord_error.to_string(),
        });
    }
    if has_bezier {
        projections.push(MaterializationProjection::CubicBezierRoutePolyline {
            source: source.to_owned(),
            chord_error: options.route_bezier_chord_error.to_string(),
        });
    }
    Ok((profile, projections))
}

fn sample_cubic_bezier(
    bezier: &hyperpath::CubicBezier,
    chord_error: f64,
    source: &str,
    points: &mut Vec<Point2>,
) -> Result<(), GeometryMaterializationError> {
    points.extend(project_cubic_bezier(bezier, chord_error, source)?);
    Ok(())
}

pub(crate) fn project_cubic_bezier(
    bezier: &hyperpath::CubicBezier,
    chord_error: f64,
    source: &str,
) -> Result<Vec<Point2>, GeometryMaterializationError> {
    if !chord_error.is_finite() || chord_error <= 0.0 {
        return Err(GeometryMaterializationError::RouteOutline(format!(
            "{source} cubic Bezier projection requires a finite positive chord error"
        )));
    }
    let finite_point = |point: &Point2| {
        Ok([
            point
                .x
                .to_f64_lossy()
                .filter(|value| value.is_finite())
                .ok_or(GeometryMaterializationError::Arithmetic)?,
            point
                .y
                .to_f64_lossy()
                .filter(|value| value.is_finite())
                .ok_or(GeometryMaterializationError::Arithmetic)?,
        ])
    };
    let start = finite_point(bezier.start())?;
    let control0 = finite_point(bezier.control0())?;
    let control1 = finite_point(bezier.control1())?;
    let end = finite_point(bezier.end())?;
    let mut sampled = Vec::new();
    flatten_cubic(
        start,
        control0,
        control1,
        end,
        chord_error,
        0,
        source,
        &mut sampled,
    )?;
    let mut points = Vec::with_capacity(sampled.len());
    for point in sampled {
        points.push(Point2::new(
            Real::try_from(point[0]).map_err(|_| GeometryMaterializationError::Arithmetic)?,
            Real::try_from(point[1]).map_err(|_| GeometryMaterializationError::Arithmetic)?,
        ));
    }
    if let Some(last) = points.last_mut() {
        *last = bezier.end().clone();
    }
    Ok(points)
}

#[allow(clippy::too_many_arguments)]
fn flatten_cubic(
    start: [f64; 2],
    control0: [f64; 2],
    control1: [f64; 2],
    end: [f64; 2],
    chord_error: f64,
    depth: u8,
    source: &str,
    output: &mut Vec<[f64; 2]>,
) -> Result<(), GeometryMaterializationError> {
    if cubic_flatness(start, control0, control1, end) <= chord_error {
        output.push(end);
        return Ok(());
    }
    if depth == 32 {
        return Err(GeometryMaterializationError::RouteOutline(format!(
            "{source} cubic Bezier exceeded adaptive projection depth"
        )));
    }
    let midpoint =
        |left: [f64; 2], right: [f64; 2]| [(left[0] + right[0]) / 2.0, (left[1] + right[1]) / 2.0];
    let a = midpoint(start, control0);
    let b = midpoint(control0, control1);
    let c = midpoint(control1, end);
    let d = midpoint(a, b);
    let e = midpoint(b, c);
    let middle = midpoint(d, e);
    flatten_cubic(start, a, d, middle, chord_error, depth + 1, source, output)?;
    flatten_cubic(middle, e, c, end, chord_error, depth + 1, source, output)
}

fn cubic_flatness(start: [f64; 2], control0: [f64; 2], control1: [f64; 2], end: [f64; 2]) -> f64 {
    let dx = end[0] - start[0];
    let dy = end[1] - start[1];
    let chord = dx.hypot(dy);
    if chord == 0.0 {
        return (control0[0] - start[0])
            .hypot(control0[1] - start[1])
            .max((control1[0] - start[0]).hypot(control1[1] - start[1]));
    }
    let distance = |point: [f64; 2]| {
        (dy * point[0] - dx * point[1] + end[0] * start[1] - end[1] * start[0]).abs() / chord
    };
    distance(control0).max(distance(control1))
}

fn finite_stroked_polyline_profile(
    points: &[Point2],
    width: &Real,
    source: &str,
) -> Result<Profile, GeometryMaterializationError> {
    let finite = |value: &Real| {
        value
            .to_f64_lossy()
            .filter(|value| value.is_finite())
            .ok_or(GeometryMaterializationError::Arithmetic)
    };
    let points = points
        .iter()
        .map(|point| Ok([finite(&point.x)?, finite(&point.y)?]))
        .collect::<Result<Vec<_>, GeometryMaterializationError>>()?;
    if points.len() < 2 {
        return Err(GeometryMaterializationError::InvalidRoute(
            source.to_owned(),
        ));
    }
    let half_width = finite(width)? / 2.0;
    let direction = |a: [f64; 2], b: [f64; 2]| -> Result<[f64; 2], GeometryMaterializationError> {
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let length = dx.hypot(dy);
        if !length.is_finite() || length == 0.0 {
            return Err(GeometryMaterializationError::InvalidRoute(
                source.to_owned(),
            ));
        }
        Ok([dx / length, dy / length])
    };
    let mut directions = Vec::with_capacity(points.len() - 1);
    for pair in points.windows(2) {
        directions.push(direction(pair[0], pair[1])?);
    }
    let offset = |index: usize, side: f64| -> [f64; 2] {
        let previous = directions[index.saturating_sub(1)];
        let next = directions[index.min(directions.len() - 1)];
        let previous_normal = [-previous[1] * side, previous[0] * side];
        let next_normal = [-next[1] * side, next[0] * side];
        let sum = [
            previous_normal[0] + next_normal[0],
            previous_normal[1] + next_normal[1],
        ];
        let sum_length = sum[0].hypot(sum[1]);
        let miter = if sum_length > 1.0e-12 {
            [sum[0] / sum_length, sum[1] / sum_length]
        } else {
            next_normal
        };
        let denominator = (miter[0] * next_normal[0] + miter[1] * next_normal[1])
            .abs()
            .max(0.25);
        [
            points[index][0] + miter[0] * half_width / denominator,
            points[index][1] + miter[1] * half_width / denominator,
        ]
    };
    let mut outline = (0..points.len())
        .map(|index| offset(index, 1.0))
        .collect::<Vec<_>>();
    let cap_steps = 12;
    let end_direction = directions[directions.len() - 1];
    let end_angle = end_direction[1].atan2(end_direction[0]);
    for step in 1..=cap_steps {
        let angle = end_angle + std::f64::consts::FRAC_PI_2
            - std::f64::consts::PI * step as f64 / cap_steps as f64;
        outline.push([
            points[points.len() - 1][0] + half_width * angle.cos(),
            points[points.len() - 1][1] + half_width * angle.sin(),
        ]);
    }
    outline.extend((0..points.len()).rev().map(|index| offset(index, -1.0)));
    let start_direction = directions[0];
    let start_angle = start_direction[1].atan2(start_direction[0]);
    for step in 1..=cap_steps {
        let angle = start_angle
            - std::f64::consts::FRAC_PI_2
            - std::f64::consts::PI * step as f64 / cap_steps as f64;
        outline.push([
            points[0][0] + half_width * angle.cos(),
            points[0][1] + half_width * angle.sin(),
        ]);
    }
    let outline = outline
        .into_iter()
        .map(finite_profile_point)
        .collect::<Result<Vec<_>, GeometryMaterializationError>>()?;
    match polygon_profile(&outline, source) {
        Ok(profile) => Ok(profile),
        Err(GeometryMaterializationError::InvalidPolygon(_)) => {
            finite_swept_polyline_profile(&points, half_width, source)
        }
        Err(error) => Err(error),
    }
}

fn finite_swept_polyline_profile(
    points: &[[f64; 2]],
    half_width: f64,
    source: &str,
) -> Result<Profile, GeometryMaterializationError> {
    let mut swept = None::<Profile>;
    let mut add_profile =
        |profile: Profile, description: &str| -> Result<(), GeometryMaterializationError> {
            swept = Some(match swept.take() {
                Some(existing) => existing.try_union(&profile).map_err(|error| {
                    GeometryMaterializationError::Boolean(format!(
                        "{source} swept-polyline {description}: {error:?}"
                    ))
                })?,
                None => profile,
            });
            Ok(())
        };

    for (index, pair) in points.windows(2).enumerate() {
        let dx = pair[1][0] - pair[0][0];
        let dy = pair[1][1] - pair[0][1];
        let length = dx.hypot(dy);
        if !length.is_finite() || length == 0.0 {
            return Err(GeometryMaterializationError::InvalidRoute(
                source.to_owned(),
            ));
        }
        let normal = [-dy / length * half_width, dx / length * half_width];
        let rectangle = [
            [pair[0][0] + normal[0], pair[0][1] + normal[1]],
            [pair[1][0] + normal[0], pair[1][1] + normal[1]],
            [pair[1][0] - normal[0], pair[1][1] - normal[1]],
            [pair[0][0] - normal[0], pair[0][1] - normal[1]],
        ]
        .into_iter()
        .map(finite_profile_point)
        .collect::<Result<Vec<_>, GeometryMaterializationError>>()?;
        add_profile(
            polygon_profile(&rectangle, source)?,
            &format!("segment {index}"),
        )?;
    }

    const ROUND_STEPS: usize = 16;
    for (index, point) in points.iter().enumerate() {
        let joint = (0..ROUND_STEPS)
            .map(|step| {
                let angle = std::f64::consts::TAU * step as f64 / ROUND_STEPS as f64;
                finite_profile_point([
                    point[0] + half_width * angle.cos(),
                    point[1] + half_width * angle.sin(),
                ])
            })
            .collect::<Result<Vec<_>, GeometryMaterializationError>>()?;
        add_profile(polygon_profile(&joint, source)?, &format!("joint {index}"))?;
    }

    swept.ok_or_else(|| GeometryMaterializationError::InvalidRoute(source.to_owned()))
}

fn finite_profile_point(point: [f64; 2]) -> Result<Point2, GeometryMaterializationError> {
    Ok(Point2::new(
        Real::try_from(point[0]).map_err(|_| GeometryMaterializationError::Arithmetic)?,
        Real::try_from(point[1]).map_err(|_| GeometryMaterializationError::Arithmetic)?,
    ))
}

fn polygon_profile(
    vertices: &[Point2],
    source: &str,
) -> Result<Profile, GeometryMaterializationError> {
    if vertices.len() < 3 {
        return Err(GeometryMaterializationError::InvalidPolygon(
            source.to_owned(),
        ));
    }
    let vertices = vertices.iter().map(curve_point).collect::<Vec<_>>();
    let profile = Profile::polygon_points(&vertices);
    if profile.is_empty() {
        Err(GeometryMaterializationError::InvalidPolygon(
            source.to_owned(),
        ))
    } else {
        Ok(profile)
    }
}

fn curve_point(point: &Point2) -> CurvePoint2 {
    CurvePoint2::new(point.x.clone(), point.y.clone())
}

fn union_layer_images(features: &[MaterializedCopperFeature]) -> Vec<LayerImage> {
    let mut images = BTreeMap::<TraceLayer, (usize, Option<Profile>, Option<String>)>::new();
    for feature in features {
        let entry = images
            .entry(feature.layer)
            .or_insert_with(|| (0, None, None));
        entry.0 += 1;
        if entry.2.is_some() {
            continue;
        }
        match entry.1.take() {
            Some(existing) => match existing.try_union(&feature.profile) {
                Ok(union) => entry.1 = Some(union),
                Err(error) => entry.2 = Some(format!("{error:?}")),
            },
            None => {
                entry.1 = Some(feature.profile.clone());
            }
        }
    }
    images
        .into_iter()
        .map(
            |(layer, (source_feature_count, copper, blocker))| LayerImage {
                layer,
                copper,
                source_feature_count,
                blocker,
            },
        )
        .collect()
}

fn union_process_images(features: &[MaterializedProcessFeature]) -> Vec<ProcessLayerImage> {
    let mut images = BTreeMap::<ProcessLayerRole, (usize, Option<Profile>, Option<String>)>::new();
    for feature in features {
        let entry = images
            .entry(feature.role)
            .or_insert_with(|| (0, None, None));
        entry.0 += 1;
        if entry.2.is_some() {
            continue;
        }
        match entry.1.take() {
            Some(existing) => match existing.try_union(&feature.profile) {
                Ok(union) => entry.1 = Some(union),
                Err(error) => entry.2 = Some(format!("{error:?}")),
            },
            None => entry.1 = Some(feature.profile.clone()),
        }
    }
    images
        .into_iter()
        .map(
            |(role, (source_feature_count, image, blocker))| ProcessLayerImage {
                role,
                image,
                source_feature_count,
                blocker,
            },
        )
        .collect()
}

fn half(value: &Real) -> Result<Real, GeometryMaterializationError> {
    (value.clone() / Real::from(2_u8)).map_err(|_| GeometryMaterializationError::Arithmetic)
}

#[cfg(test)]
mod tests {
    use super::{MaterializationOptions, pad_profile, transform_drill, transform_pad_profile};
    use crate::{
        BoardSide, CircuitInstanceId, DrillShape, LandPatternId, LandPatternPad, PadId, PadShape,
        PcbPlacement, Plating,
    };
    use hyperlattice::Point2;
    use hyperpath::TraceLayer;
    use hyperreal::Real;

    #[test]
    fn pad_local_rotation_precedes_translation_and_board_placement() {
        let pad = LandPatternPad {
            id: PadId::new("1").unwrap(),
            center: Point2::new(Real::from(10), Real::from(10)),
            rotation_degrees: Real::from(90),
            copper_layers: vec![TraceLayer(0)],
            shape: PadShape::Rectangle {
                width: Real::from(4),
                height: Real::from(2),
            },
            drill: Some(DrillShape::Slot {
                start: Point2::new(Real::zero(), Real::from(-2)),
                end: Point2::new(Real::zero(), Real::from(2)),
                width: Real::one(),
            }),
            plating: Plating::Plated,
            solder_mask_margin: None,
            paste_margin: None,
        };
        let placement = PcbPlacement {
            instance: CircuitInstanceId::new("U1").unwrap(),
            land_pattern: LandPatternId::new("rotated").unwrap(),
            position: Point2::origin(),
            rotation_degrees: Real::zero(),
            side: BoardSide::Front,
        };

        let profile = transform_pad_profile(
            pad_profile(&pad, &MaterializationOptions::default()).unwrap(),
            &pad,
            &placement,
        );
        let profiles = profile.region_profiles();
        let points = profiles[0].material().points();
        let bounds = points.iter().fold(
            [
                f64::INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
            ],
            |[min_x, min_y, max_x, max_y], [x, y]| {
                [min_x.min(*x), min_y.min(*y), max_x.max(*x), max_y.max(*y)]
            },
        );
        assert_eq!(bounds, [9.0, 8.0, 11.0, 12.0]);

        let drill = transform_drill(
            &pad,
            pad.drill.as_ref().expect("fixture has a slot"),
            &placement,
        );
        let DrillShape::Slot { start, end, .. } = drill.shape else {
            panic!("rotated slot must remain a slot");
        };
        assert_eq!(start, Point2::new(Real::from(12), Real::from(10)));
        assert_eq!(end, Point2::new(Real::from(8), Real::from(10)));
    }
}
