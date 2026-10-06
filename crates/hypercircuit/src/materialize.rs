//! Exact-aware lowering of declarative PCB intent into `csgrs` geometry.
//!
//! The retained layout remains authoritative. Materialized profiles are output
//! products with source ids and net identities, so a boolean union never
//! destroys the information needed by `hyperdrc` or an interchange adapter.

use std::cell::Cell;
use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};
use std::fmt::{Display, Formatter};
use std::io::{BufReader, Cursor};

use csgrs::curve;
use csgrs::solid::{self, SolidExt};
use csgrs::{AttributedMesh, GeometryCertainty, GeometryContext, GeometryOutcome};
use hypercurve::{
    CircularArc2, Contour2, Curve2, CurveCertainty, CurvePath2, CurveRegion2, CurveRegionLoopRole,
    ExactCurveError, FillRule, LineSeg2, OffsetCap, OffsetCornerStyle2, Point2 as CurvePoint2,
};
use hyperlattice::Point2;
use hyperlimit::{Certainty, PredicateOutcome, PredicatePolicy};
use hyperpath::TraceLayer;
use hyperreal::Real;
use sha2::{Digest, Sha256};

use crate::layout::{
    BoardBoundaryGeometryError, BoardSide, CopperZone, CopperZoneConnection, CopperZoneFill,
    DrillShape, KeepoutScope, LandPatternGraphic, LandPatternGraphicPrimitive, LandPatternPad,
    LayerRole, PadShape, Pcb3dModelFormat, Pcb3dModelReference, PcbLayout, PcbPlacement, Plating,
    ViaMaskDisposition,
};
use crate::{
    Circuit, CircuitInstanceId, LandPatternGraphicId, LandPatternId, NetId, PadId, PinRef, RouteId,
    ViaId, ZoneId,
};

/// Immutable predicate policy selected for one PCB materialization operation.
#[repr(transparent)]
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct MaterializationContext {
    predicates: PredicatePolicy,
}

impl MaterializationContext {
    /// Strict materialization: every topology decision must be certified.
    pub const STRICT: Self = Self::new(PredicatePolicy::STRICT);

    /// Materialization may consume Hyperlimit's terminal 512-bit interpretation.
    pub const APPROXIMATE_512: Self = Self::new(PredicatePolicy::APPROXIMATE_512);

    /// Construct a materialization context with the selected predicate policy.
    pub const fn new(predicates: PredicatePolicy) -> Self {
        Self { predicates }
    }

    /// Return the selected Hyperlimit predicate policy.
    pub const fn predicate_policy(self) -> PredicatePolicy {
        self.predicates
    }

    /// Derive the matching CSG curve/mesh operation context.
    pub const fn geometry_context(self) -> GeometryContext {
        GeometryContext::new(self.predicates)
    }
}

struct MaterializationDecisions {
    predicates: PredicatePolicy,
    certainty: Cell<GeometryCertainty>,
}

impl MaterializationDecisions {
    fn new(context: &MaterializationContext) -> Self {
        Self {
            predicates: context.predicate_policy(),
            certainty: Cell::new(GeometryCertainty::Certified),
        }
    }

    fn geometry_context(&self) -> GeometryContext {
        GeometryContext::new(self.predicates)
    }

    fn certainty(&self) -> GeometryCertainty {
        self.certainty.get()
    }

    fn observe(&self, certainty: GeometryCertainty) {
        if certainty == GeometryCertainty::Approximate512Consumed {
            self.certainty
                .set(GeometryCertainty::Approximate512Consumed);
        }
    }

    fn observe_curve(&self, certainty: CurveCertainty) {
        if certainty == CurveCertainty::Approximate512Consumed {
            self.observe(GeometryCertainty::Approximate512Consumed);
        }
    }

    /// Runs principal exact Hypercurve operations under this materialization's
    /// predicate policy through [`hypercurve::evaluate_under`], recording any
    /// approximate terminal they consumed.
    fn exact_curve<T>(&self, operation: impl FnOnce() -> T) -> T {
        let provisional = hypercurve::evaluate_under(self.predicates, operation);
        self.observe_curve(provisional.certainty());
        provisional.into_unverified()
    }

    fn consume_geometry<T, E>(&self, result: Result<GeometryOutcome<T>, E>) -> Result<T, E> {
        result.map(|outcome| {
            self.observe(outcome.certainty);
            outcome.value
        })
    }

    fn boundary_operation<T>(
        &self,
        mut evaluate: impl FnMut(PredicatePolicy) -> Result<T, BoardBoundaryGeometryError>,
    ) -> Result<T, BoardBoundaryGeometryError> {
        if self.predicates != PredicatePolicy::APPROXIMATE_512 {
            return evaluate(self.predicates);
        }
        match evaluate(PredicatePolicy::STRICT) {
            Ok(value) => Ok(value),
            Err(error) if error.is_policy_blocked() => evaluate(self.predicates).inspect(|_| {
                self.observe(GeometryCertainty::Approximate512Consumed);
            }),
            Err(error) => Err(error),
        }
    }

    fn compare_reals(&self, left: &Real, right: &Real) -> Option<Ordering> {
        match hyperlimit::compare_reals(left, right, self.predicates) {
            PredicateOutcome::Decided {
                value, certainty, ..
            } => {
                if certainty == Certainty::Approximate {
                    self.certainty
                        .set(GeometryCertainty::Approximate512Consumed);
                }
                Some(value)
            }
            PredicateOutcome::Unknown { .. } => None,
        }
    }
}

/// Options controlling finite display approximations at materialization boundaries.
#[derive(Clone, Debug, PartialEq)]
pub struct MaterializationOptions {
    /// Segment count used for circles and rounded pad display geometry.
    pub circular_segments: usize,
    /// Build boolean-unioned copper and process-layer images.
    ///
    /// DRC consumers that inspect source-addressable features can disable this
    /// expensive CAM-only aggregation while retaining every individual exact
    /// pad, route, via, drill, and process feature.
    pub aggregate_layer_images: bool,
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
            aggregate_layer_images: true,
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
    pub profile: CurveRegion2,
}

/// Unioned copper image for one routing layer.
#[derive(Clone, Debug)]
pub struct LayerImage {
    /// Copper routing layer.
    pub layer: TraceLayer,
    /// Union of every materialized feature when the exact boolean was decided.
    pub copper: Option<CurveRegion2>,
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
    pub profile: CurveRegion2,
}

/// Certified union image for one side/process combination.
#[derive(Clone, Debug)]
pub struct ProcessLayerImage {
    /// Side/process role.
    pub role: ProcessLayerRole,
    /// Union of every source feature when the exact boolean was decided.
    pub image: Option<CurveRegion2>,
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
    pub substrate: CurveRegion2,
    /// Predicate policy selected for this materialization.
    pub predicate_policy: PredicatePolicy,
    /// Weakest predicate certainty consumed by completed materialization paths.
    pub predicate_certainty: GeometryCertainty,
    /// Whether CAM-oriented copper and process layer images were aggregated.
    ///
    /// Individual exact source features remain complete when this is false,
    /// but fabrication consumers must reject the report as intentionally
    /// incomplete.
    pub layer_images_aggregated: bool,
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
    pub solid: AttributedMesh<Pcb3dLayerMetadata>,
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
    pub solid: AttributedMesh<Pcb3dComponentBodyMetadata>,
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
    pub mesh: AttributedMesh<Pcb3dComponentModelMetadata>,
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
    /// A decided physical-layer profile could not be extruded without losing geometry.
    LayerExtrusionFailed {
        /// Authored stackup layer.
        layer: String,
        /// Native exact extrusion diagnostic.
        detail: String,
    },
    /// A declared package body outline could not produce material topology.
    InvalidComponentBody(String),
    /// A valid package body profile could not be extruded without losing geometry.
    ComponentBodyExtrusionFailed {
        /// Logical placed instance.
        instance: String,
        /// Native exact extrusion diagnostic.
        detail: String,
    },
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
    /// Predicate policy inherited from materialization and used by 3D realization.
    pub predicate_policy: PredicatePolicy,
    /// Weakest predicate certainty consumed by materialization and 3D realization.
    pub predicate_certainty: GeometryCertainty,
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
    /// Predicate policy inherited from the source assembly.
    pub predicate_policy: PredicatePolicy,
    /// Weakest predicate certainty consumed by the source assembly.
    pub predicate_certainty: GeometryCertainty,
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
        context: &MaterializationContext,
        options: MaterializationOptions,
    ) -> Result<PcbMaterializationReport, GeometryMaterializationError> {
        let decisions = MaterializationDecisions::new(context);
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

        let boundary = decisions
            .boundary_operation(|policy| self.outline.boundary_geometry(policy))
            .map_err(|error| {
                GeometryMaterializationError::InvalidPolygon(format!("board outline: {error}"))
            })?;
        let substrate = boundary.region().clone();

        let mut copper_features = Vec::new();
        let mut process_features = Vec::new();
        let mut process_omissions = Vec::new();
        let mut projections = Vec::new();
        let mut zone_realizations = Vec::new();
        let mut drills = Vec::new();
        let stitching = self.realize_stitching_vias();
        {
            let mut placement_materialization = PlacementMaterialization {
                decisions: &decisions,
                copper_features: &mut copper_features,
                process_features: &mut process_features,
                process_omissions: &mut process_omissions,
                drills: &mut drills,
            };
            materialize_placements(self, circuit, &options, &mut placement_materialization)?;
        }

        for route in &self.routes {
            let (profile, route_projections) = route_profile(route, &options, &decisions)?;
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
            let source = format!("via:{}", via.id.as_str());
            let profile = curve::translated(
                &exact_circle_profile(&radius, &source, &decisions)?,
                via.center.x.clone(),
                via.center.y.clone(),
            );
            for layer in via.start_layer.0..=via.end_layer.0 {
                copper_features.push(MaterializedCopperFeature {
                    source: source.clone(),
                    net: Some(via.net.clone()),
                    layer: TraceLayer(layer),
                    kind: CopperFeatureKind::Via,
                    identity: MaterializedCopperIdentity::Via(via.id.clone()),
                    anchor: via.center.clone(),
                    profile: profile.clone(),
                });
            }
            drills.push(DrillHit {
                source,
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
                        let profile = curve::translated(
                            &exact_circle_profile(
                                &opening_radius,
                                &format!("mask:via:{}", via.id.as_str()),
                                &decisions,
                            )?,
                            via.center.x.clone(),
                            via.center.y.clone(),
                        );
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
            let (profile, evidence) =
                materialize_zone(self, zone, &substrate, &copper_features, &decisions)?;
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

        let copper_layers = if options.aggregate_layer_images {
            union_layer_images(&copper_features, &decisions)
        } else {
            Vec::new()
        };
        let process_layers = if options.aggregate_layer_images {
            union_process_images(&process_features, &decisions)
        } else {
            Vec::new()
        };
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
            predicate_policy: context.predicate_policy(),
            predicate_certainty: decisions.certainty(),
            layer_images_aggregated: options.aggregate_layer_images,
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
    substrate: &CurveRegion2,
    existing: &[MaterializedCopperFeature],
    decisions: &MaterializationDecisions,
) -> Result<(CurveRegion2, ZoneMaterializationEvidence), GeometryMaterializationError> {
    let source = format!("zone:{}", zone.id.as_str());
    let boundary = polygon_profile(&zone.boundary, &source, decisions)?;
    let boundary = zone_boolean(
        &source,
        "clip to substrate",
        decisions.exact_curve(|| {
            boundary.boolean_region(substrate, hypercurve::BooleanOp::Intersection)
        }),
    )?;
    let profile = match &zone.fill {
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
            decisions,
        )?,
    };
    let mut keepout_profiles = Vec::new();
    let mut hard_negative_profiles = Vec::new();
    let mut thermal_gap_profiles = Vec::new();
    let mut positive_additions = Vec::new();
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
            decisions,
        )?;
        hard_negative_profiles.push(keepout_profile.clone());
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
                decisions,
            )?;
            hard_negative_profiles.push(clearance);
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
                    decisions,
                )?;
                hard_negative_profiles.push(clearance);
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
                    decisions,
                )?;
                thermal_gap_profiles.push(clearance.clone());
                let spokes = thermal_spoke_mask(
                    &zone.boundary,
                    &feature.anchor,
                    spoke_width,
                    *spoke_count,
                    &clearance,
                    decisions,
                )?;
                let mut spokes =
                    if axis_aligned_rectangle_contains_region(&zone.boundary, &spokes, decisions) {
                        spokes
                    } else {
                        zone_boolean(
                            &source,
                            &format!("clip thermal spokes to boundary for {}", feature.source),
                            decisions.exact_curve(|| {
                                spokes
                                    .boolean_region(&boundary, hypercurve::BooleanOp::Intersection)
                            }),
                        )?
                    };
                if let MaterializedCopperIdentity::Via(via_id) = &feature.identity
                    && let Some(via) = layout.vias.iter().find(|via| &via.id == via_id)
                {
                    let drill = DrillHit {
                        source: format!("via:{}", via.id.as_str()),
                        center: via.center.clone(),
                        shape: DrillShape::Round {
                            diameter: via.drill_diameter.clone(),
                        },
                        plating: via.plating,
                    };
                    let cutter = preview_drill_profile(&drill, 4, decisions)?;
                    spokes = subtract_drill_exact(&spokes, &drill, &cutter, decisions).map_err(
                        |error| {
                            GeometryMaterializationError::Boolean(format!(
                                "{source} exact thermal spoke drill for {}: {error}",
                                feature.source
                            ))
                        },
                    )?;
                }
                positive_additions.push(spokes);
                treated_same_net_lands += 1;
                thermal_bounding_box_projections += 1;
            }
        }
    }

    let regularize = zone.islands.remove_unconnected
        || zone.islands.minimum_area.is_some()
        || matches!(zone.fill, CopperZoneFill::Hatched { .. })
        || matches!(zone.connection, CopperZoneConnection::ThermalRelief { .. });
    let profile = if regularize {
        // Component policies and non-solid fills require regularized exact
        // topology for island evidence and downstream physical previews.
        regularized_zone_composition(
            &source,
            profile,
            &thermal_gap_profiles,
            &positive_additions,
            &hard_negative_profiles,
            decisions,
        )?
    } else {
        let mut positive_profiles = Vec::with_capacity(1 + positive_additions.len());
        positive_profiles.push(profile);
        positive_profiles.append(&mut positive_additions);
        let mut negative_profiles = thermal_gap_profiles;
        for positive in &positive_profiles {
            negative_profiles.extend(
                hard_negative_profiles
                    .iter()
                    .filter(|negative| profiles_may_intersect(positive, negative, decisions))
                    .cloned(),
            );
        }
        zone_compound(
            &source,
            "compose exact fill",
            &positive_profiles,
            &negative_profiles,
            decisions,
        )?
    };
    let (profile, islands) = apply_zone_island_policy(&source, zone, profile, existing, decisions)?;

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
    profile: CurveRegion2,
    existing: &[MaterializedCopperFeature],
    decisions: &MaterializationDecisions,
) -> Result<(CurveRegion2, ZoneIslandRealization), GeometryMaterializationError> {
    let region = &profile;
    if !zone.islands.remove_unconnected && zone.islands.minimum_area.is_none() {
        let retained = decisions
            .exact_curve(|| region.loop_roles())
            .map_err(|error| {
                GeometryMaterializationError::ZoneIsland(format!(
                    "{source} retained loop classification: {error}"
                ))
            })?
            .iter()
            .filter(|role| **role == CurveRegionLoopRole::Material)
            .count();
        return Ok((
            profile,
            ZoneIslandRealization {
                initial: retained,
                retained,
                ..ZoneIslandRealization::default()
            },
        ));
    }
    // Normalized components keep their selected boundaries and certified hole
    // ownership; no component is reconstructed or re-intersected here.
    let components = decisions
        .exact_curve(|| region.material_components())
        .map_err(|error| {
            GeometryMaterializationError::ZoneIsland(format!(
                "{source} material component extraction: {error}"
            ))
        })?;
    let mut realization = ZoneIslandRealization {
        initial: components.len(),
        ..ZoneIslandRealization::default()
    };

    let mut retained = None::<CurveRegion2>;
    for (index, region) in components.into_iter().enumerate() {
        let area = match decisions
            .exact_curve(|| region.filled_area())
            .map_err(|error| {
                GeometryMaterializationError::ZoneIsland(format!(
                    "{source} component {index} area: {error}"
                ))
            })? {
            Some(area) => area,
            None => {
                return Err(GeometryMaterializationError::ZoneIsland(format!(
                    "{source} component {index} area unsupported"
                )));
            }
        };
        let below_area = match &zone.islands.minimum_area {
            Some(minimum) => {
                decisions.compare_reals(&area, minimum).ok_or_else(|| {
                    GeometryMaterializationError::ZoneIsland(format!(
                        "{source} component {index} area ordering"
                    ))
                })? == std::cmp::Ordering::Less
            }
            None => false,
        };
        let component_profile = region;
        let connected = if zone.islands.remove_unconnected {
            let mut connected = false;
            for feature in existing.iter().filter(|feature| {
                feature.layer == zone.layer && feature.net.as_ref() == Some(&zone.net)
            }) {
                let intersection = zone_boolean(
                    source,
                    &format!("classify island {index} connection to {}", feature.source),
                    decisions.exact_curve(|| {
                        component_profile
                            .boolean_region(&feature.profile, hypercurve::BooleanOp::Intersection)
                    }),
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
                decisions.exact_curve(|| {
                    current.boolean_region(&component_profile, hypercurve::BooleanOp::Union)
                }),
            )?,
            None => component_profile,
        });
        realization.retained += 1;
    }

    Ok((retained.unwrap_or_else(CurveRegion2::empty), realization))
}

fn hatch_zone(
    source: &str,
    boundary: &CurveRegion2,
    points: &[Point2],
    line_width: &Real,
    gap: &Real,
    angle_degrees: &Real,
    decisions: &MaterializationDecisions,
) -> Result<CurveRegion2, GeometryMaterializationError> {
    let (center, extent) = zone_center_extent(points, decisions)?;
    let pitch = line_width.clone() + gap.clone();
    let mut offset = -extent.clone();
    let mut stripes: Option<CurveRegion2> = None;
    loop {
        match decisions.compare_reals(&offset, &extent) {
            Some(Ordering::Less | Ordering::Equal) => {}
            Some(Ordering::Greater) => break,
            None => return Err(GeometryMaterializationError::Arithmetic),
        }
        let stripe = curve::translated(
            &curve::rectangle(extent.clone() + extent.clone(), line_width.clone()),
            Real::zero(),
            offset.clone(),
        );
        let stripe = match decisions.compare_reals(angle_degrees, &Real::zero()) {
            Some(Ordering::Equal) => stripe,
            Some(_) => curve::rotated(&stripe, angle_degrees.clone()),
            None => return Err(GeometryMaterializationError::Arithmetic),
        };
        let stripe = curve::translated(&stripe, center.x.clone(), center.y.clone());
        stripes = Some(match stripes {
            None => stripe,
            Some(existing) => zone_boolean(
                source,
                "merge hatch stripes",
                decisions
                    .exact_curve(|| existing.boolean_region(&stripe, hypercurve::BooleanOp::Union)),
            )?,
        });
        offset += pitch.clone();
    }
    let stripes = stripes.ok_or_else(|| {
        GeometryMaterializationError::InvalidPolygon(format!("{source} hatch produced no stripes"))
    })?;
    zone_boolean(
        source,
        "clip hatch to boundary",
        decisions
            .exact_curve(|| stripes.boolean_region(boundary, hypercurve::BooleanOp::Intersection)),
    )
}

fn thermal_spoke_mask(
    zone_boundary: &[Point2],
    center: &Point2,
    width: &Real,
    count: u8,
    clearance: &CurveRegion2,
    decisions: &MaterializationDecisions,
) -> Result<CurveRegion2, GeometryMaterializationError> {
    let (_, extent) = zone_center_extent(zone_boundary, decisions)?;
    let span = extent.clone() + extent.clone();
    let spokes = if count == 4 {
        let horizontal = curve::translated(
            &curve::rectangle(span.clone(), width.clone()),
            center.x.clone(),
            center.y.clone(),
        );
        let vertical = curve::translated(
            &curve::rectangle(width.clone(), span),
            center.x.clone(),
            center.y.clone(),
        );
        zone_boolean(
            "thermal-spokes",
            "merge exact orthogonal spoke masks",
            decisions
                .exact_curve(|| horizontal.boolean_region(&vertical, hypercurve::BooleanOp::Union)),
        )?
    } else {
        let step = (Real::from(360) / Real::from(count))
            .map_err(|_| GeometryMaterializationError::Arithmetic)?;
        let mut spokes: Option<CurveRegion2> = None;
        for index in 0..count {
            let angle = step.clone() * Real::from(index);
            let spoke = curve::rotated(&curve::rectangle(span.clone(), width.clone()), angle);
            let spoke = curve::translated(&spoke, center.x.clone(), center.y.clone());
            spokes = Some(match spokes {
                None => spoke,
                Some(existing) => zone_boolean(
                    "thermal-spokes",
                    "merge spoke masks",
                    decisions.exact_curve(|| {
                        existing.boolean_region(&spoke, hypercurve::BooleanOp::Union)
                    }),
                )?,
            });
        }
        spokes.ok_or(GeometryMaterializationError::Arithmetic)?
    };
    let bounds = curve::bounding_box(clearance);
    let width = bounds.maxs.x.clone() - bounds.mins.x.clone();
    let height = bounds.maxs.y.clone() - bounds.mins.y.clone();
    let two = Real::from(2);
    let clip_center = Point2::new(
        ((bounds.mins.x + bounds.maxs.x) / two.clone())
            .map_err(|_| GeometryMaterializationError::Arithmetic)?,
        ((bounds.mins.y + bounds.maxs.y) / two)
            .map_err(|_| GeometryMaterializationError::Arithmetic)?,
    );
    let clip = curve::translated(
        &curve::rectangle(width, height),
        clip_center.x,
        clip_center.y,
    );
    zone_boolean(
        "thermal-spokes",
        "clip spoke masks to land clearance bounds",
        decisions.exact_curve(|| spokes.boolean_region(&clip, hypercurve::BooleanOp::Intersection)),
    )
}

fn zone_center_extent(
    points: &[Point2],
    decisions: &MaterializationDecisions,
) -> Result<(Point2, Real), GeometryMaterializationError> {
    let first = points
        .first()
        .ok_or_else(|| GeometryMaterializationError::InvalidPolygon("empty zone".into()))?;
    let mut min_x = first.x.clone();
    let mut min_y = first.y.clone();
    let mut max_x = first.x.clone();
    let mut max_y = first.y.clone();
    for point in points.iter().skip(1) {
        match decisions.compare_reals(&point.x, &min_x) {
            Some(Ordering::Less) => min_x = point.x.clone(),
            Some(_) => {}
            None => return Err(GeometryMaterializationError::Arithmetic),
        }
        match decisions.compare_reals(&point.y, &min_y) {
            Some(Ordering::Less) => min_y = point.y.clone(),
            Some(_) => {}
            None => return Err(GeometryMaterializationError::Arithmetic),
        }
        match decisions.compare_reals(&point.x, &max_x) {
            Some(Ordering::Greater) => max_x = point.x.clone(),
            Some(_) => {}
            None => return Err(GeometryMaterializationError::Arithmetic),
        }
        match decisions.compare_reals(&point.y, &max_y) {
            Some(Ordering::Greater) => max_y = point.y.clone(),
            Some(_) => {}
            None => return Err(GeometryMaterializationError::Arithmetic),
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

fn axis_aligned_rectangle_contains_region(
    points: &[Point2],
    region: &CurveRegion2,
    decisions: &MaterializationDecisions,
) -> bool {
    let points = if points.len() == 5 && points.first() == points.last() {
        &points[..4]
    } else {
        points
    };
    if points.len() != 4 {
        return false;
    }

    let mut xs = Vec::<&Real>::with_capacity(2);
    let mut ys = Vec::<&Real>::with_capacity(2);
    for point in points {
        if !xs.contains(&&point.x) {
            xs.push(&point.x);
        }
        if !ys.contains(&&point.y) {
            ys.push(&point.y);
        }
    }
    if xs.len() != 2 || ys.len() != 2 {
        return false;
    }
    let (min_x, max_x) = match decisions.compare_reals(xs[0], xs[1]) {
        Some(Ordering::Less) => (xs[0], xs[1]),
        Some(Ordering::Greater) => (xs[1], xs[0]),
        Some(Ordering::Equal) | None => return false,
    };
    let (min_y, max_y) = match decisions.compare_reals(ys[0], ys[1]) {
        Some(Ordering::Less) => (ys[0], ys[1]),
        Some(Ordering::Greater) => (ys[1], ys[0]),
        Some(Ordering::Equal) | None => return false,
    };
    if xs.iter().any(|x| {
        ys.iter().any(|y| {
            points
                .iter()
                .filter(|point| {
                    decisions.compare_reals(&point.x, x) == Some(Ordering::Equal)
                        && decisions.compare_reals(&point.y, y) == Some(Ordering::Equal)
                })
                .count()
                != 1
        })
    }) {
        return false;
    }

    let Ok(Some(bounds)) = decisions.exact_curve(|| region.bounds()) else {
        return false;
    };
    matches!(
        decisions.compare_reals(bounds.min_x(), min_x),
        Some(Ordering::Equal | Ordering::Greater)
    ) && matches!(
        decisions.compare_reals(bounds.max_x(), max_x),
        Some(Ordering::Equal | Ordering::Less)
    ) && matches!(
        decisions.compare_reals(bounds.min_y(), min_y),
        Some(Ordering::Equal | Ordering::Greater)
    ) && matches!(
        decisions.compare_reals(bounds.max_y(), max_y),
        Some(Ordering::Equal | Ordering::Less)
    )
}

fn profiles_may_intersect(
    left: &CurveRegion2,
    right: &CurveRegion2,
    decisions: &MaterializationDecisions,
) -> bool {
    decisions
        .exact_curve(|| left.boolean_region(right, hypercurve::BooleanOp::Intersection))
        .map(|intersection| !intersection.is_empty())
        .unwrap_or(true)
}

fn zone_offset(
    source: &str,
    operation: &str,
    profile: &CurveRegion2,
    distance: &Real,
    decisions: &MaterializationDecisions,
) -> Result<CurveRegion2, GeometryMaterializationError> {
    decisions
        .exact_curve(|| profile.offset(distance.clone(), &OffsetCornerStyle2::Round))
        .map_err(|error| {
            GeometryMaterializationError::Boolean(format!("{source} {operation}: {error}"))
        })
}

fn zone_boolean(
    source: &str,
    operation: &str,
    result: hypercurve::ExactCurveResult<CurveRegion2>,
) -> Result<CurveRegion2, GeometryMaterializationError> {
    result.map_err(|error| {
        GeometryMaterializationError::Boolean(format!("{source} {operation}: {error:?}"))
    })
}

fn zone_compound(
    source: &str,
    operation: &str,
    positive: &[CurveRegion2],
    negative: &[CurveRegion2],
    decisions: &MaterializationDecisions,
) -> Result<CurveRegion2, GeometryMaterializationError> {
    exact_compound_composition(positive, negative, decisions).map_err(|error| {
        GeometryMaterializationError::Boolean(format!("{source} {operation}: {error}"))
    })
}

fn regularized_zone_composition(
    source: &str,
    mut profile: CurveRegion2,
    thermal_gaps: &[CurveRegion2],
    positive_additions: &[CurveRegion2],
    hard_cuts: &[CurveRegion2],
    decisions: &MaterializationDecisions,
) -> Result<CurveRegion2, GeometryMaterializationError> {
    // Preserve authored CSG priority exactly: thermal gaps cut the base,
    // spokes restore selected material, and hard keepouts remain dominant.
    for (index, gap) in thermal_gaps.iter().enumerate() {
        profile = zone_boolean(
            source,
            &format!("regularize thermal gap {index}"),
            decisions
                .exact_curve(|| profile.boolean_region(gap, hypercurve::BooleanOp::Difference)),
        )?;
    }
    for (index, addition) in positive_additions.iter().enumerate() {
        profile = zone_boolean(
            source,
            &format!("regularize positive addition {index}"),
            decisions
                .exact_curve(|| profile.boolean_region(addition, hypercurve::BooleanOp::Union)),
        )?;
    }
    for (index, cut) in hard_cuts.iter().enumerate() {
        profile = zone_boolean(
            source,
            &format!("regularize hard cut {index}"),
            decisions
                .exact_curve(|| profile.boolean_region(cut, hypercurve::BooleanOp::Difference)),
        )?;
    }
    Ok(profile)
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
        let context = MaterializationContext::new(self.predicate_policy);
        let decisions = MaterializationDecisions::new(&context);
        decisions.observe(self.predicate_certainty);
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
            match preview_drill_profile(drill, self.preview_circular_segments, &decisions) {
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
                            match decisions.exact_curve(|| {
                                profile.boolean_region(openings, hypercurve::BooleanOp::Difference)
                            }) {
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
                for (drill_index, (drill, cutter)) in drill_profiles.iter().enumerate() {
                    match subtract_drill_exact(&profile, drill, cutter, &decisions) {
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
                            // Aggregated copper retains exact signed loop
                            // composition so large layers need not be
                            // regularized eagerly. If a later drill encounters
                            // coincident aggregate carriers, rebuild just this
                            // layer by distributing the reached drill cuts
                            // over its source features and unioning the exact
                            // results. This is the set identity
                            // `(A ∪ B) \ D = (A \ D) ∪ (B \ D)`; no projection
                            // or approximation is introduced.
                            let rebuilt = match kind {
                                Pcb3dLayerKind::Copper { routing_layer } => {
                                    let features = self
                                        .copper_features
                                        .iter()
                                        .filter(|feature| feature.layer == routing_layer);
                                    let mut rebuilt = Ok(None::<CurveRegion2>);
                                    for feature in features {
                                        let mut drilled = Ok(feature.profile.clone());
                                        for (replay_drill, replay_cutter) in
                                            drill_profiles.iter().take(drill_index + 1)
                                        {
                                            drilled = drilled.and_then(|current| {
                                                subtract_drill_exact(
                                                    &current,
                                                    replay_drill,
                                                    replay_cutter,
                                                    &decisions,
                                                )
                                                .map_err(|error| {
                                                    format!("feature {}: {error}", feature.source)
                                                })
                                            });
                                        }
                                        rebuilt = rebuilt.and_then(|aggregate| {
                                            drilled.and_then(|drilled| match aggregate {
                                                Some(aggregate) => decisions
                                                    .exact_curve(|| {
                                                        aggregate.boolean_region(
                                                            &drilled,
                                                            hypercurve::BooleanOp::Union,
                                                        )
                                                    })
                                                    .map(Some)
                                                    .map_err(|error| format!("{error:?}")),
                                                None => Ok(Some(drilled)),
                                            })
                                        });
                                    }
                                    match rebuilt {
                                        Ok(Some(rebuilt)) => Ok(rebuilt),
                                        Ok(None) => Err(error.clone()),
                                        Err(error) => Err(error),
                                    }
                                }
                                _ => Err(error.clone()),
                            };
                            match rebuilt {
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
                                Err(rebuild_error) => {
                                    omissions.push(Pcb3dAssemblyOmission::DrillSubtractionFailed {
                                        source: drill.source.clone(),
                                        layer: layer.name.clone(),
                                        detail: format!(
                                            "aggregate: {error}; exact layer replay: \
                                             {rebuild_error}"
                                        ),
                                    })
                                }
                            }
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
                match decisions.consume_geometry(curve::try_extrude(
                    &profile,
                    layer.thickness.clone(),
                    &decisions.geometry_context(),
                )) {
                    Ok(geometry) => {
                        let geometry =
                            geometry.translated(Real::zero(), Real::zero(), z_start.clone());
                        let solid = AttributedMesh::from_uniform(geometry, metadata.clone());
                        layers.push(Pcb3dLayer { metadata, solid });
                    }
                    Err(error) => {
                        omissions.push(Pcb3dAssemblyOmission::LayerExtrusionFailed {
                            layer: layer.name.clone(),
                            detail: error.to_string(),
                        });
                    }
                }
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
                match polygon_profile(&outline, placement.instance.as_str(), &decisions) {
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
                        match decisions.consume_geometry(curve::try_extrude(
                            &profile,
                            body.height.clone(),
                            &decisions.geometry_context(),
                        )) {
                            Ok(geometry) => {
                                let geometry =
                                    geometry.translated(Real::zero(), Real::zero(), body_z);
                                let solid =
                                    AttributedMesh::from_uniform(geometry, metadata.clone());
                                component_bodies.push(Pcb3dComponentBody { metadata, solid });
                            }
                            Err(error) => omissions.push(
                                Pcb3dAssemblyOmission::ComponentBodyExtrusionFailed {
                                    instance: placement.instance.as_str().into(),
                                    detail: error.to_string(),
                                },
                            ),
                        }
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
            predicate_policy: self.predicate_policy,
            predicate_certainty: decisions.certainty(),
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
            csgrs::io::obj::from_obj(BufReader::new(Cursor::new(bytes.as_slice())))
                .map_err(|error| parse_failure(error.to_string()))?,
            None,
            1,
            1,
            0,
            0,
            0,
        ),
        Pcb3dModelFormat::Vrml => {
            let imported = csgrs::io::vrml::from_vrml(&bytes)
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
            let imported = csgrs::io::gltf::from_gltf(&bytes)
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
    if source_mesh.triangles.is_empty() {
        return Err(Pcb3dAssemblyOmission::ExternalPackageModelParseFailed {
            instance: placement.instance.as_str().into(),
            model_index,
            uri: reference.uri.clone(),
            detail: "source container contains no triangles".into(),
        });
    }
    let source_triangle_count = source_mesh.triangles.len();
    let transform = &reference.transform;
    let mut geometry = solid::scale(
        &source_mesh,
        transform.scale_x.clone(),
        transform.scale_y.clone(),
        transform.scale_z.clone(),
    );
    geometry = solid::rotate(
        &geometry,
        transform.rotate_x_degrees.clone(),
        transform.rotate_y_degrees.clone(),
        transform.rotate_z_degrees.clone(),
    );
    geometry = geometry.translated(
        transform.offset_x.clone(),
        transform.offset_y.clone(),
        transform.offset_z.clone(),
    );
    let mounting_z = match placement.side {
        BoardSide::Front => board_thickness.clone(),
        BoardSide::Back => {
            geometry = solid::scale(&geometry, -Real::one(), Real::one(), -Real::one());
            Real::zero()
        }
    };
    geometry = solid::rotate(
        &geometry,
        Real::zero(),
        Real::zero(),
        placement.rotation_degrees.clone(),
    );
    geometry = geometry.translated(
        placement.position.x.clone(),
        placement.position.y.clone(),
        mounting_z,
    );
    let mesh = AttributedMesh::from_uniform(geometry, metadata.clone());
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
            geometry.push(
                csgrs::io::gltf::GltfSceneObject::new(name.clone(), layer.solid.geometry())
                    .map_err(|error| Pcb3dGltfError::Geometry(error.to_string()))?,
            );
            objects.push(Pcb3dSceneObject {
                name,
                kind: Pcb3dSceneObjectKind::StackupLayer {
                    layer: layer.metadata.name.clone(),
                    kind: layer.metadata.kind.clone(),
                },
                triangle_count: layer.solid.geometry().triangles.len(),
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
            geometry.push(
                csgrs::io::gltf::GltfSceneObject::new(name.clone(), body.solid.geometry())
                    .map_err(|error| Pcb3dGltfError::Geometry(error.to_string()))?,
            );
            objects.push(Pcb3dSceneObject {
                name,
                kind: Pcb3dSceneObjectKind::ComponentBody {
                    instance: body.metadata.instance.clone(),
                    land_pattern: body.metadata.land_pattern.clone(),
                },
                triangle_count: body.solid.geometry().triangles.len(),
            });
        }
        for model in &self.component_models {
            let name = format!(
                "component-model:{}:{}",
                model.metadata.instance.as_str(),
                model.metadata.model_index
            );
            geometry.push(
                csgrs::io::gltf::GltfSceneObject::new(name.clone(), model.mesh.geometry())
                    .map_err(|error| Pcb3dGltfError::Geometry(error.to_string()))?,
            );
            objects.push(Pcb3dSceneObject {
                name,
                kind: Pcb3dSceneObjectKind::ComponentModel {
                    instance: model.metadata.instance.clone(),
                    land_pattern: model.metadata.land_pattern.clone(),
                    model_index: model.metadata.model_index,
                    uri: model.metadata.reference.uri.clone(),
                    source_sha256: model.metadata.source_sha256.clone(),
                },
                triangle_count: model.mesh.geometry().triangles.len(),
            });
        }
        if geometry.is_empty() {
            return Err(Pcb3dGltfError::EmptyAssembly);
        }
        let gltf = csgrs::io::gltf::to_gltf_scene(scene_name, &geometry)
            .map_err(|error| Pcb3dGltfError::Geometry(error.to_string()))?;
        Ok(Pcb3dGltfReport {
            predicate_policy: self.predicate_policy,
            predicate_certainty: self.predicate_certainty,
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
    decisions: &MaterializationDecisions,
) -> Result<CurveRegion2, GeometryMaterializationError> {
    match &drill.shape {
        DrillShape::Round { diameter } => {
            let profile = exact_rounded_rectangle_profile(
                diameter,
                diameter,
                &half(diameter)?,
                &drill.source,
                decisions,
            )?;
            Ok(curve::translated(
                &profile,
                drill.center.x.clone(),
                drill.center.y.clone(),
            ))
        }
        DrillShape::Slot { start, end, width } => stroked_path_profile(
            &[start.clone(), end.clone()],
            false,
            width,
            circular_segments,
            &drill.source,
            decisions,
        ),
    }
}

fn subtract_drill_exact(
    profile: &CurveRegion2,
    drill: &DrillHit,
    cutter: &CurveRegion2,
    decisions: &MaterializationDecisions,
) -> Result<CurveRegion2, String> {
    match decisions
        .exact_curve(|| profile.boolean_region(cutter, hypercurve::BooleanOp::Difference))
    {
        Ok(realized) => Ok(realized),
        Err(whole_error) if matches!(drill.shape, DrillShape::Round { .. }) => {
            let sectors = exact_round_drill_sectors(drill, decisions).map_err(|sector_error| {
                format!("whole cutter: {whole_error:?}; exact sector construction: {sector_error}")
            })?;
            let mut realized = profile.clone();
            for (index, sector) in sectors.iter().enumerate() {
                realized = decisions
                    .exact_curve(|| {
                        realized.boolean_region(sector, hypercurve::BooleanOp::Difference)
                    })
                    .map_err(|sector_error| {
                        format!(
                            "whole cutter: {whole_error:?}; exact sector {index}: {sector_error:?}"
                        )
                    })?;
            }
            Ok(realized)
        }
        Err(error) => Err(format!("{error:?}")),
    }
}

fn exact_round_drill_sectors(
    drill: &DrillHit,
    decisions: &MaterializationDecisions,
) -> Result<Vec<CurveRegion2>, GeometryMaterializationError> {
    let DrillShape::Round { diameter } = &drill.shape else {
        return Err(GeometryMaterializationError::InvalidPolygon(
            drill.source.clone(),
        ));
    };
    let radius = half(diameter)?;
    let center = curve_point(&drill.center);
    let diagonal = (radius
        / Real::from(2)
            .sqrt()
            .map_err(|_| GeometryMaterializationError::InvalidPolygon(drill.source.clone()))?)
    .map_err(|_| GeometryMaterializationError::Arithmetic)?;
    let north_east = CurvePoint2::new(&drill.center.x + &diagonal, &drill.center.y + &diagonal);
    let north_west = CurvePoint2::new(&drill.center.x - &diagonal, &drill.center.y + &diagonal);
    let south_west = CurvePoint2::new(&drill.center.x - &diagonal, &drill.center.y - &diagonal);
    let south_east = CurvePoint2::new(&drill.center.x + &diagonal, &drill.center.y - &diagonal);
    let cardinals = [north_east, north_west, south_west, south_east];
    (0..4)
        .map(|index| {
            let start = cardinals[index].clone();
            let end = cardinals[(index + 1) % 4].clone();
            let path = CurvePath2::try_new(vec![
                Curve2::from(LineSeg2::try_new(center.clone(), start.clone()).map_err(
                    |error| {
                        GeometryMaterializationError::InvalidRoute(format!(
                            "{} drill sector: {error}",
                            drill.source
                        ))
                    },
                )?),
                Curve2::from(
                    CircularArc2::try_from_center(start, end.clone(), center.clone(), false)
                        .map_err(|error| {
                            GeometryMaterializationError::InvalidRoute(format!(
                                "{} drill sector: {error}",
                                drill.source
                            ))
                        })?,
                ),
                Curve2::from(LineSeg2::try_new(end, center.clone()).map_err(|error| {
                    GeometryMaterializationError::InvalidRoute(format!(
                        "{} drill sector: {error}",
                        drill.source
                    ))
                })?),
            ])
            .map_err(|error| {
                GeometryMaterializationError::InvalidRoute(format!(
                    "{} drill sector: {error}",
                    drill.source
                ))
            })?;
            decisions
                .exact_curve(|| {
                    CurveRegion2::try_from_boundary_paths(&[path], hypercurve::FillRule::EvenOdd)
                })
                .map_err(|error| {
                    GeometryMaterializationError::InvalidRoute(format!(
                        "{} drill sector: {error}",
                        drill.source
                    ))
                })
        })
        .collect()
}

struct PlacementMaterialization<'a> {
    decisions: &'a MaterializationDecisions,
    copper_features: &'a mut Vec<MaterializedCopperFeature>,
    process_features: &'a mut Vec<MaterializedProcessFeature>,
    process_omissions: &'a mut Vec<ProcessMaterializationOmission>,
    drills: &'a mut Vec<DrillHit>,
}

fn materialize_placements(
    layout: &PcbLayout,
    circuit: &Circuit,
    options: &MaterializationOptions,
    materialization: &mut PlacementMaterialization<'_>,
) -> Result<(), GeometryMaterializationError> {
    let decisions = materialization.decisions;
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
            let local_profile = pad_profile(pad, options, decisions)?;
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
                materialization
                    .copper_features
                    .push(MaterializedCopperFeature {
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
                let mask = if mask_margin.definitely_zero() {
                    local_profile.clone()
                } else {
                    decisions
                        .exact_curve(|| {
                            local_profile.offset(mask_margin.clone(), &OffsetCornerStyle2::Round)
                        })
                        .map_err(|error| {
                            GeometryMaterializationError::Boolean(format!(
                                "{source} solder-mask offset: {error}"
                            ))
                        })?
                };
                if !mask.is_empty() {
                    materialization
                        .process_features
                        .push(MaterializedProcessFeature {
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
                    let paste = if paste_margin.definitely_zero() {
                        local_profile.clone()
                    } else {
                        decisions
                            .exact_curve(|| {
                                local_profile
                                    .offset(paste_margin.clone(), &OffsetCornerStyle2::Round)
                            })
                            .map_err(|error| {
                                GeometryMaterializationError::Boolean(format!(
                                    "{source} paste offset: {error}"
                                ))
                            })?
                    };
                    if !paste.is_empty() {
                        materialization
                            .process_features
                            .push(MaterializedProcessFeature {
                                source: format!("paste:{source}"),
                                role: side_process_role(side, false),
                                kind: ProcessFeatureKind::PadPasteAperture,
                                profile: transform_pad_profile(paste, pad, placement),
                            });
                    }
                }
            }
            if let Some(drill) = &pad.drill {
                materialization
                    .drills
                    .push(transform_drill(pad, drill, placement));
            }
        }
        materialize_pattern_graphics(layout, pattern, placement, options, materialization)?;
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
    materialization: &mut PlacementMaterialization<'_>,
) -> Result<(), GeometryMaterializationError> {
    let decisions = materialization.decisions;
    for graphic in &pattern.graphics {
        let source = format!(
            "graphic:{}:{}:{}",
            placement.instance.as_str(),
            pattern.id.as_str(),
            graphic.id.as_str()
        );
        match &graphic.layer {
            LayerRole::EdgeCuts => {
                materialization
                    .process_omissions
                    .push(ProcessMaterializationOmission::PackageEdgeCuts { source });
                continue;
            }
            LayerRole::Custom(layer) => {
                materialization.process_omissions.push(
                    ProcessMaterializationOmission::CustomArtworkLayer {
                        source,
                        layer: layer.clone(),
                    },
                );
                continue;
            }
            LayerRole::Fabrication | LayerRole::Courtyard => continue,
            _ => {}
        }
        let Some(profile) = graphic_profile(
            graphic,
            options,
            &source,
            materialization.process_omissions,
            decisions,
        )?
        else {
            continue;
        };
        let profile = transform_local_profile(profile, placement);
        match &graphic.layer {
            LayerRole::Copper(layer) => {
                materialization
                    .copper_features
                    .push(MaterializedCopperFeature {
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
                    })
            }
            LayerRole::FrontSolderMask
            | LayerRole::BackSolderMask
            | LayerRole::FrontPaste
            | LayerRole::BackPaste
            | LayerRole::FrontSilkscreen
            | LayerRole::BackSilkscreen => {
                let role = placed_process_role(&graphic.layer, placement.side)
                    .expect("matched production artwork role");
                materialization
                    .process_features
                    .push(MaterializedProcessFeature {
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
    decisions: &MaterializationDecisions,
) -> Result<Option<CurveRegion2>, GeometryMaterializationError> {
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
                decisions,
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
            let outer =
                exact_circle_profile(&(radius.clone() + half_width.clone()), source, decisions)?;
            let inner_radius = radius.clone() - half_width;
            let ring = match decisions.compare_reals(&inner_radius, &Real::zero()) {
                Some(Ordering::Less | Ordering::Equal) => outer,
                Some(Ordering::Greater) => {
                    let inner = exact_circle_profile(&inner_radius, source, decisions)?;
                    decisions
                        .exact_curve(|| {
                            outer.boolean_region(&inner, hypercurve::BooleanOp::Difference)
                        })
                        .map_err(|error| {
                            GeometryMaterializationError::Boolean(format!("{source}: {error:?}"))
                        })?
                }
                None => return Err(GeometryMaterializationError::Arithmetic),
            };
            Ok(Some(curve::translated(
                &ring,
                center.x.clone(),
                center.y.clone(),
            )))
        }
        LandPatternGraphicPrimitive::Polygon { vertices, filled } => {
            if *filled {
                let profile = polygon_profile(vertices, source, decisions)?;
                if let Some(width) = &graphic.stroke_width {
                    let half_width = half(width)?;
                    decisions
                        .exact_curve(|| profile.offset(half_width, &OffsetCornerStyle2::Round))
                        .map(Some)
                        .map_err(|error| {
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
                stroked_polygon_profile(
                    vertices,
                    width,
                    options.circular_segments,
                    source,
                    decisions,
                )
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
            let profile = curve::rotated(
                &curve::truetype_text(text, &policy.font_data, point_height),
                rotation_degrees.clone(),
            );
            let profile = curve::translated(&profile, position.x.clone(), position.y.clone());
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
    _circular_segments: usize,
    source: &str,
    decisions: &MaterializationDecisions,
) -> Result<CurveRegion2, GeometryMaterializationError> {
    let mut curves = points
        .windows(2)
        .map(|pair| {
            LineSeg2::try_new(curve_point(&pair[0]), curve_point(&pair[1])).map(Curve2::from)
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
        curves.push(
            LineSeg2::try_new(curve_point(last), curve_point(first))
                .map(Curve2::from)
                .map_err(|error| {
                    GeometryMaterializationError::InvalidRoute(format!("{source}: {error}"))
                })?,
        );
    }
    let centerline = CurvePath2::try_new(curves).map_err(|error| {
        GeometryMaterializationError::InvalidRoute(format!("{source}: {error}"))
    })?;
    let half_width = half(width)?;
    if !closed && let [start, end] = points {
        match decisions.exact_curve(|| {
            CurveRegion2::stroke_path(
                &centerline,
                half_width.clone(),
                &OffsetCornerStyle2::Round,
                OffsetCap::Round,
            )
        }) {
            Ok(region) => return Ok(region),
            Err(ExactCurveError::Blocked(_)) => {
                return exact_line_stroke_profile(start, end, width, source, decisions);
            }
            Err(error) => {
                return Err(GeometryMaterializationError::RouteOutline(format!(
                    "{source}: {error}"
                )));
            }
        }
    }
    decisions
        .exact_curve(|| {
            CurveRegion2::stroke_path(
                &centerline,
                half_width.clone(),
                &OffsetCornerStyle2::Round,
                OffsetCap::Round,
            )
        })
        .map_err(|error| GeometryMaterializationError::RouteOutline(format!("{source}: {error}")))
}

fn exact_line_stroke_profile(
    start: &Point2,
    end: &Point2,
    width: &Real,
    source: &str,
    decisions: &MaterializationDecisions,
) -> Result<CurveRegion2, GeometryMaterializationError> {
    let dx = &end.x - &start.x;
    let dy = &end.y - &start.y;
    let length_squared = &dx * &dx + &dy * &dy;
    if decisions.compare_reals(&length_squared, &Real::zero()) != Some(Ordering::Greater) {
        return Err(GeometryMaterializationError::InvalidRoute(
            source.to_owned(),
        ));
    }
    let length = length_squared
        .sqrt()
        .map_err(|_| GeometryMaterializationError::Arithmetic)?;
    let half_width = half(width)?;
    let capsule_width = &length + width;
    let capsule =
        exact_rounded_rectangle_profile(&capsule_width, width, &half_width, source, decisions)?;
    let direction_x = (dx / &length).map_err(|_| GeometryMaterializationError::Arithmetic)?;
    let direction_y = (dy / &length).map_err(|_| GeometryMaterializationError::Arithmetic)?;
    let midpoint_x = ((&start.x + &end.x) / Real::from(2_u8))
        .map_err(|_| GeometryMaterializationError::Arithmetic)?;
    let midpoint_y = ((&start.y + &end.y) / Real::from(2_u8))
        .map_err(|_| GeometryMaterializationError::Arithmetic)?;
    decisions
        .exact_curve(|| {
            capsule.transform_affine(
                &direction_x,
                &(-direction_y.clone()),
                &direction_y,
                &direction_x,
                &midpoint_x,
                &midpoint_y,
            )
        })
        .map_err(|error| GeometryMaterializationError::RouteOutline(format!("{source}: {error}")))
}

fn stroked_polygon_profile(
    points: &[Point2],
    width: &Real,
    circular_segments: usize,
    source: &str,
    decisions: &MaterializationDecisions,
) -> Result<CurveRegion2, GeometryMaterializationError> {
    if points.len() < 3 {
        return Err(GeometryMaterializationError::InvalidPolygon(
            source.to_owned(),
        ));
    }
    let mut outline: Option<CurveRegion2> = None;
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
            decisions,
        )?;
        outline = Some(match outline {
            Some(existing) => decisions
                .exact_curve(|| existing.boolean_region(&edge, hypercurve::BooleanOp::Union))
                .map_err(|error| {
                    GeometryMaterializationError::Boolean(format!("{source}: {error:?}"))
                })?,
            None => edge,
        });
    }
    outline.ok_or_else(|| GeometryMaterializationError::InvalidPolygon(source.to_owned()))
}

fn pad_profile(
    pad: &LandPatternPad,
    _options: &MaterializationOptions,
    decisions: &MaterializationDecisions,
) -> Result<CurveRegion2, GeometryMaterializationError> {
    match &pad.shape {
        PadShape::Circle { diameter } => exact_rounded_rectangle_profile(
            diameter,
            diameter,
            &half(diameter)?,
            pad.id.as_str(),
            decisions,
        ),
        PadShape::Rectangle { width, height } => Ok(center_pad_profile(
            curve::rectangle(width.clone(), height.clone()),
            width,
            height,
        )?),
        PadShape::RoundedRectangle {
            width,
            height,
            corner_radius,
        } => exact_rounded_rectangle_profile(
            width,
            height,
            corner_radius,
            pad.id.as_str(),
            decisions,
        ),
        PadShape::Obround { width, height } => {
            let (profile_width, profile_height, radius) =
                match decisions.compare_reals(width, height) {
                    Some(Ordering::Less) => (width, height, half(width)?),
                    Some(Ordering::Greater) => (width, height, half(height)?),
                    Some(Ordering::Equal) => {
                        // Equality is a topology decision owned by this operation.
                        // Reuse one exact carrier after consuming that decision so
                        // downstream arc/path validation does not independently
                        // reinterpret two equivalent symbolic expressions.
                        (width, width, half(width)?)
                    }
                    None => return Err(GeometryMaterializationError::Arithmetic),
                };
            exact_rounded_rectangle_profile(
                profile_width,
                profile_height,
                &radius,
                pad.id.as_str(),
                decisions,
            )
        }
        PadShape::Polygon { vertices } => polygon_profile(vertices, pad.id.as_str(), decisions),
    }
}

fn exact_rounded_rectangle_profile(
    width: &Real,
    height: &Real,
    radius: &Real,
    source: &str,
    decisions: &MaterializationDecisions,
) -> Result<CurveRegion2, GeometryMaterializationError> {
    let right = width - radius;
    let top = height - radius;
    let bottom_left = CurvePoint2::new(radius.clone(), Real::zero());
    let bottom_right = CurvePoint2::new(right.clone(), Real::zero());
    let right_bottom = CurvePoint2::new(width.clone(), radius.clone());
    let right_top = CurvePoint2::new(width.clone(), top.clone());
    let top_right = CurvePoint2::new(right.clone(), height.clone());
    let top_left = CurvePoint2::new(radius.clone(), height.clone());
    let left_top = CurvePoint2::new(Real::zero(), top.clone());
    let left_bottom = CurvePoint2::new(Real::zero(), radius.clone());
    let mut curves = Vec::with_capacity(8);
    push_exact_line(&mut curves, &bottom_left, &bottom_right, source, decisions)?;
    curves.push(Curve2::from(
        CircularArc2::try_from_center(
            bottom_right,
            right_bottom.clone(),
            CurvePoint2::new(right.clone(), radius.clone()),
            false,
        )
        .map_err(|_| GeometryMaterializationError::InvalidPolygon(source.to_owned()))?,
    ));
    push_exact_line(&mut curves, &right_bottom, &right_top, source, decisions)?;
    curves.push(Curve2::from(
        CircularArc2::try_from_center(
            right_top,
            top_right.clone(),
            CurvePoint2::new(right, top.clone()),
            false,
        )
        .map_err(|_| GeometryMaterializationError::InvalidPolygon(source.to_owned()))?,
    ));
    push_exact_line(&mut curves, &top_right, &top_left, source, decisions)?;
    curves.push(Curve2::from(
        CircularArc2::try_from_center(
            top_left,
            left_top.clone(),
            CurvePoint2::new(radius.clone(), top),
            false,
        )
        .map_err(|_| GeometryMaterializationError::InvalidPolygon(source.to_owned()))?,
    ));
    push_exact_line(&mut curves, &left_top, &left_bottom, source, decisions)?;
    curves.push(Curve2::from(
        CircularArc2::try_from_center(
            left_bottom,
            bottom_left,
            CurvePoint2::new(radius.clone(), radius.clone()),
            false,
        )
        .map_err(|_| GeometryMaterializationError::InvalidPolygon(source.to_owned()))?,
    ));
    let path = CurvePath2::try_new(curves)
        .map_err(|_| GeometryMaterializationError::InvalidPolygon(source.to_owned()))?;
    let region = decisions
        .exact_curve(|| {
            CurveRegion2::try_from_boundary_paths(&[path], hypercurve::FillRule::EvenOdd)
        })
        .map_err(|_| GeometryMaterializationError::InvalidPolygon(source.to_owned()))?;
    center_pad_profile(region, width, height)
}

fn exact_circle_profile(
    radius: &Real,
    source: &str,
    decisions: &MaterializationDecisions,
) -> Result<CurveRegion2, GeometryMaterializationError> {
    let diameter = Real::from(2_u8) * radius;
    exact_rounded_rectangle_profile(&diameter, &diameter, radius, source, decisions)
}

fn push_exact_line(
    curves: &mut Vec<Curve2>,
    start: &CurvePoint2,
    end: &CurvePoint2,
    source: &str,
    decisions: &MaterializationDecisions,
) -> Result<(), GeometryMaterializationError> {
    let same_point = curve_points_equal(start, end, decisions)
        .ok_or(GeometryMaterializationError::Arithmetic)?;
    if !same_point {
        curves.push(Curve2::from(
            LineSeg2::try_new(start.clone(), end.clone())
                .map_err(|_| GeometryMaterializationError::InvalidPolygon(source.to_owned()))?,
        ));
    }
    Ok(())
}

fn curve_points_equal(
    first: &CurvePoint2,
    second: &CurvePoint2,
    decisions: &MaterializationDecisions,
) -> Option<bool> {
    Some(
        decisions.compare_reals(first.x(), second.x())? == Ordering::Equal
            && decisions.compare_reals(first.y(), second.y())? == Ordering::Equal,
    )
}

fn center_pad_profile(
    profile: CurveRegion2,
    width: &Real,
    height: &Real,
) -> Result<CurveRegion2, GeometryMaterializationError> {
    Ok(curve::translated(&profile, -half(width)?, -half(height)?))
}

fn transform_pad_profile(
    profile: CurveRegion2,
    pad: &LandPatternPad,
    placement: &PcbPlacement,
) -> CurveRegion2 {
    let local = curve::rotated(&profile, pad.rotation_degrees.clone());
    let local = curve::translated(&local, pad.center.x.clone(), pad.center.y.clone());
    transform_local_profile(local, placement)
}

fn transform_local_profile(profile: CurveRegion2, placement: &PcbPlacement) -> CurveRegion2 {
    let sided = match placement.side {
        BoardSide::Front => profile,
        BoardSide::Back => curve::scaled(&profile, -Real::one(), Real::one()),
    };
    let rotated = curve::rotated(&sided, placement.rotation_degrees.clone());
    curve::translated(
        &rotated,
        placement.position.x.clone(),
        placement.position.y.clone(),
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
    decisions: &MaterializationDecisions,
) -> Result<(CurveRegion2, Vec<MaterializationProjection>), GeometryMaterializationError> {
    let source = route.id.as_str();
    let has_curve = route.segments.iter().any(|segment| {
        matches!(
            segment,
            crate::PcbRouteSegment::CircularArc(_) | crate::PcbRouteSegment::CubicBezier(_)
        )
    });
    if !has_curve {
        let mut profile = None::<CurveRegion2>;
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
                decisions,
            )?;
            profile = Some(match profile {
                Some(existing) => decisions
                    .exact_curve(|| existing.boolean_region(&swept, hypercurve::BooleanOp::Union))
                    .map_err(|error| {
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
                let same_x = decisions
                    .compare_reals(&arc.start().x, &arc.end().x)
                    .ok_or(GeometryMaterializationError::Arithmetic)?
                    == Ordering::Equal;
                let same_y = decisions
                    .compare_reals(&arc.start().y, &arc.end().y)
                    .ok_or(GeometryMaterializationError::Arithmetic)?
                    == Ordering::Equal;
                if same_x && same_y {
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
    let profile = finite_stroked_polyline_profile(&points, &route.width, source, decisions)?;
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
    source: &str,
    output: &mut Vec<[f64; 2]>,
) -> Result<(), GeometryMaterializationError> {
    type Cubic = [[f64; 2]; 4];

    let midpoint = |left: [f64; 2], right: [f64; 2]| {
        [
            left[0] * 0.5 + right[0] * 0.5,
            left[1] * 0.5 + right[1] * 0.5,
        ]
    };
    let allocation_error = || {
        GeometryMaterializationError::RouteOutline(format!(
            "{source} cubic Bezier projection exceeds addressable memory"
        ))
    };
    let mut pending = Vec::<Cubic>::new();
    pending.try_reserve(1).map_err(|_| allocation_error())?;
    pending.push([start, control0, control1, end]);

    while let Some(segment @ [start, control0, control1, end]) = pending.pop() {
        if cubic_within_chord_error(start, control0, control1, end, chord_error) {
            output.try_reserve(1).map_err(|_| allocation_error())?;
            output.push(end);
            continue;
        }

        let a = midpoint(start, control0);
        let b = midpoint(control0, control1);
        let c = midpoint(control1, end);
        let d = midpoint(a, b);
        let e = midpoint(b, c);
        let middle = midpoint(d, e);
        let left = [start, a, d, middle];
        let right = [middle, e, c, end];
        if left == segment || right == segment {
            return Err(GeometryMaterializationError::RouteOutline(format!(
                "{source} cubic Bezier projection stagnated before meeting its chord error"
            )));
        }
        pending.try_reserve(2).map_err(|_| allocation_error())?;
        pending.push(right);
        pending.push(left);
    }
    Ok(())
}

fn cubic_within_chord_error(
    start: [f64; 2],
    control0: [f64; 2],
    control1: [f64; 2],
    end: [f64; 2],
    chord_error: f64,
) -> bool {
    let scale = start
        .into_iter()
        .chain(control0)
        .chain(control1)
        .chain(end)
        .map(f64::abs)
        .fold(1.0_f64, f64::max);
    let normalize = |point: [f64; 2]| [point[0] / scale, point[1] / scale];
    let start = normalize(start);
    let control0 = normalize(control0);
    let control1 = normalize(control1);
    let end = normalize(end);
    let dx = end[0] - start[0];
    let dy = end[1] - start[1];
    let chord = dx.hypot(dy);
    let flatness = if chord == 0.0 {
        (control0[0] - start[0])
            .hypot(control0[1] - start[1])
            .max((control1[0] - start[0]).hypot(control1[1] - start[1]))
    } else {
        let distance = |point: [f64; 2]| {
            (dy * (point[0] - start[0]) - dx * (point[1] - start[1])).abs() / chord
        };
        distance(control0).max(distance(control1))
    };
    flatness <= chord_error / scale
}

fn finite_stroked_polyline_profile(
    points: &[Point2],
    width: &Real,
    source: &str,
    decisions: &MaterializationDecisions,
) -> Result<CurveRegion2, GeometryMaterializationError> {
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
        let miter = if sum_length > 0.0 {
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
    match polygon_profile(&outline, source, decisions) {
        Ok(profile) => Ok(profile),
        Err(GeometryMaterializationError::InvalidPolygon(_)) => {
            finite_swept_polyline_profile(&points, half_width, source, decisions)
        }
        Err(error) => Err(error),
    }
}

fn finite_swept_polyline_profile(
    points: &[[f64; 2]],
    half_width: f64,
    source: &str,
    decisions: &MaterializationDecisions,
) -> Result<CurveRegion2, GeometryMaterializationError> {
    let mut swept = None::<CurveRegion2>;
    let mut add_profile =
        |profile: CurveRegion2, description: &str| -> Result<(), GeometryMaterializationError> {
            swept = Some(match swept.take() {
                Some(existing) => decisions
                    .exact_curve(|| existing.boolean_region(&profile, hypercurve::BooleanOp::Union))
                    .map_err(|error| {
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
            polygon_profile(&rectangle, source, decisions)?,
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
        add_profile(
            polygon_profile(&joint, source, decisions)?,
            &format!("joint {index}"),
        )?;
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
    decisions: &MaterializationDecisions,
) -> Result<CurveRegion2, GeometryMaterializationError> {
    if vertices.len() < 3 {
        return Err(GeometryMaterializationError::InvalidPolygon(
            source.to_owned(),
        ));
    }
    let coordinates = vertices
        .iter()
        .map(|point| [point.x.clone(), point.y.clone()])
        .collect::<Vec<_>>();
    let contour = Contour2::from_real_ring(&coordinates)
        .map_err(|_| GeometryMaterializationError::InvalidPolygon(source.to_owned()))?;
    decisions
        .exact_curve(|| CurveRegion2::try_from_native_material_contours(vec![contour.clone()]))
        .map_err(|_| GeometryMaterializationError::InvalidPolygon(source.to_owned()))
}

fn curve_point(point: &Point2) -> CurvePoint2 {
    CurvePoint2::new(point.x.clone(), point.y.clone())
}

fn union_layer_images(
    features: &[MaterializedCopperFeature],
    decisions: &MaterializationDecisions,
) -> Vec<LayerImage> {
    let mut images = BTreeMap::<TraceLayer, Vec<CurveRegion2>>::new();
    for feature in features {
        images
            .entry(feature.layer)
            .or_default()
            .push(feature.profile.clone());
    }
    images
        .into_iter()
        .map(|(layer, profiles)| {
            let source_feature_count = profiles.len();
            let (copper, blocker) = match exact_compound_composition(&profiles, &[], decisions) {
                Ok(profile) => (Some(profile), None),
                Err(error) => (None, Some(error)),
            };
            LayerImage {
                layer,
                copper,
                source_feature_count,
                blocker,
            }
        })
        .collect()
}

fn union_process_images(
    features: &[MaterializedProcessFeature],
    decisions: &MaterializationDecisions,
) -> Vec<ProcessLayerImage> {
    let mut images = BTreeMap::<ProcessLayerRole, Vec<CurveRegion2>>::new();
    for feature in features {
        images
            .entry(feature.role)
            .or_default()
            .push(feature.profile.clone());
    }
    images
        .into_iter()
        .map(|(role, profiles)| {
            let source_feature_count = profiles.len();
            let (image, blocker) = match exact_compound_composition(&profiles, &[], decisions) {
                Ok(profile) => (Some(profile), None),
                Err(error) => (None, Some(error)),
            };
            ProcessLayerImage {
                role,
                image,
                source_feature_count,
                blocker,
            }
        })
        .collect()
}

fn exact_compound_composition(
    positive: &[CurveRegion2],
    negative: &[CurveRegion2],
    decisions: &MaterializationDecisions,
) -> Result<CurveRegion2, String> {
    let mut paths = Vec::new();
    let mut roles = Vec::new();
    let mut rules = Vec::new();
    for (region, invert_roles) in positive
        .iter()
        .map(|profile| (profile, false))
        .chain(negative.iter().map(|profile| (profile, true)))
    {
        let mut profile_paths = decisions
            .exact_curve(|| region.boundary_paths())
            .map_err(|error| error.to_string())?;
        let profile_roles = decisions
            .exact_curve(|| region.loop_roles())
            .map_err(|error| error.to_string())?;
        if profile_paths.len() != profile_roles.len() {
            return Err("exact aggregate path/role counts differ".to_owned());
        }
        let profile_roles = profile_roles.into_iter().map(|role| {
            if invert_roles {
                match role {
                    CurveRegionLoopRole::Material => CurveRegionLoopRole::Hole,
                    CurveRegionLoopRole::Hole => CurveRegionLoopRole::Material,
                }
            } else {
                role
            }
        });
        // Normalized loops are simple, so every per-loop fill rule selects
        // the same interior; the composed winding comes from the roles.
        let profile_rules = vec![FillRule::NonZero; profile_paths.len()];
        paths.append(&mut profile_paths);
        roles.extend(profile_roles);
        rules.extend(profile_rules);
    }
    let region = decisions
        .exact_curve(|| {
            CurveRegion2::try_from_boundary_paths_with_loop_semantics(&paths, &roles, &rules)
        })
        .map_err(|error| error.to_string())?;
    Ok(region)
}

fn half(value: &Real) -> Result<Real, GeometryMaterializationError> {
    (value.clone() / Real::from(2_u8)).map_err(|_| GeometryMaterializationError::Arithmetic)
}

#[cfg(test)]
mod tests {
    use super::{
        MaterializationContext, MaterializationDecisions, MaterializationOptions,
        exact_circle_profile, exact_compound_composition, half, pad_profile, project_cubic_bezier,
        stroked_path_profile, transform_drill, transform_pad_profile,
    };
    use crate::{
        BoardSide, CircuitInstanceId, DrillShape, LandPatternId, LandPatternPad, PadId, PadShape,
        PcbPlacement, Plating,
    };
    use csgrs::GeometryCertainty;
    use csgrs::curve;
    use hypercurve::{
        BooleanOp, Curve2, CurveFamily2, CurvePath2, CurveRegion2, LineSeg2, OffsetCap,
        OffsetCornerStyle2, Point2 as CurvePoint2,
    };
    use hyperlattice::Point2;
    use hyperpath::{CubicBezier, TraceLayer};
    use hyperreal::Real;

    fn rounded_pad(id: &str, center_x: i32) -> LandPatternPad {
        LandPatternPad {
            id: PadId::new(id).unwrap(),
            center: Point2::new(Real::from(center_x), Real::zero()),
            rotation_degrees: Real::zero(),
            copper_layers: vec![TraceLayer(0)],
            shape: PadShape::RoundedRectangle {
                width: Real::from(6),
                height: Real::from(12),
                corner_radius: Real::from(2),
            },
            drill: None,
            plating: Plating::Plated,
            solder_mask_margin: None,
            paste_margin: None,
        }
    }

    #[test]
    fn cubic_projection_handles_finite_coordinates_whose_raw_chord_overflows() {
        let maximum = Real::try_from(f64::MAX).unwrap();
        let third = Real::try_from(f64::MAX / 3.0).unwrap();
        let curve = CubicBezier::new(
            Point2::new(-maximum.clone(), Real::zero()),
            Point2::new(-third.clone(), Real::zero()),
            Point2::new(third, Real::zero()),
            Point2::new(maximum, Real::zero()),
        );

        let projected = project_cubic_bezier(&curve, 1.0, "finite-overflow-chord")
            .expect("a collinear finite cubic needs no recursive refinement");
        assert_eq!(projected, vec![curve.end().clone()]);
    }

    #[test]
    fn rounded_pads_retain_exact_arcs_and_union_without_boolean_blockers() {
        let decisions = MaterializationDecisions::new(&MaterializationContext::STRICT);
        let first = pad_profile(
            &rounded_pad("1", 0),
            &MaterializationOptions::default(),
            &decisions,
        )
        .unwrap();
        let second_local = pad_profile(
            &rounded_pad("2", 1),
            &MaterializationOptions::default(),
            &decisions,
        )
        .unwrap();
        let second = curve::translated(&second_local, Real::one(), Real::zero());
        let paths = decisions.exact_curve(|| first.boundary_paths()).unwrap();
        assert!(paths.iter().flat_map(|path| path.curves()).any(|curve| {
            matches!(
                curve.family(),
                CurveFamily2::CircularArc | CurveFamily2::RationalQuadraticBezier
            )
        }));
        let union: CurveRegion2 = decisions
            .exact_curve(|| first.boolean_region(&second, hypercurve::BooleanOp::Union))
            .expect("overlapping exact rounded pads must union");
        assert!(!union.is_empty());
    }

    #[test]
    fn short_translated_line_stroke_remains_strictly_certified() {
        let decisions = MaterializationDecisions::new(&MaterializationContext::STRICT);
        let start_x = (Real::from(2509) / Real::from(20)).unwrap();
        let end_x = (Real::from(2511) / Real::from(20)).unwrap();
        let y = (Real::from(1957) / Real::from(20)).unwrap();
        let width = (Real::one() / Real::from(2)).unwrap();
        let profile = stroked_path_profile(
            &[
                Point2::new(start_x.clone(), y.clone()),
                Point2::new(end_x.clone(), y.clone()),
            ],
            false,
            &width,
            8,
            "short-rational-route",
            &decisions,
        )
        .expect("an exact capsule fast path should not require a terminal ordering decision");
        assert!(!profile.is_empty());
        assert_eq!(decisions.certainty(), GeometryCertainty::Certified);

        let path = CurvePath2::try_new(vec![Curve2::from(
            LineSeg2::try_new(
                CurvePoint2::new(start_x, y.clone()),
                CurvePoint2::new(end_x, y),
            )
            .unwrap(),
        )])
        .unwrap();
        let generic = hypercurve::provisional(|| {
            CurveRegion2::stroke_path(
                &path,
                half(&width).unwrap(),
                &OffsetCornerStyle2::Round,
                OffsetCap::Round,
            )
        })
        .into_unverified()
        .unwrap();
        let difference =
            hypercurve::provisional(|| profile.boolean_region(&generic, BooleanOp::Xor))
                .into_unverified()
                .unwrap();
        assert!(difference.is_empty());
    }

    #[test]
    fn approximate_materialization_marks_consumed_terminal_pad_ordering() {
        let mut pad = rounded_pad("terminal", 0);
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        let unresolved_zero = &sine * &sine + &cosine * &cosine - Real::one();
        pad.shape = PadShape::Obround {
            width: Real::from(6),
            height: Real::from(6) + unresolved_zero,
        };

        let strict = MaterializationDecisions::new(&MaterializationContext::STRICT);
        assert!(pad_profile(&pad, &MaterializationOptions::default(), &strict).is_err());
        assert_eq!(strict.certainty(), GeometryCertainty::Certified);

        let approximate = MaterializationDecisions::new(&MaterializationContext::APPROXIMATE_512);
        let profile = pad_profile(&pad, &MaterializationOptions::default(), &approximate)
            .expect("the authorized 512-bit terminal should decide equal obround dimensions");
        assert!(!profile.is_empty());
        assert_eq!(
            approximate.certainty(),
            GeometryCertainty::Approximate512Consumed
        );
    }

    #[test]
    fn exact_compound_regions_replay_positive_and_negative_curve_loops() {
        let decisions = MaterializationDecisions::new(&MaterializationContext::STRICT);
        let first = pad_profile(
            &rounded_pad("1", 0),
            &MaterializationOptions::default(),
            &decisions,
        )
        .unwrap();
        let second = pad_profile(
            &rounded_pad("2", 4),
            &MaterializationOptions::default(),
            &decisions,
        )
        .unwrap();
        let second = curve::translated(&second, Real::from(4), Real::zero());

        let difference = exact_compound_composition(
            std::slice::from_ref(&first),
            std::slice::from_ref(&second),
            &decisions,
        )
        .expect("exact signed difference must retain both curve-loop operands");
        assert_eq!(
            curve::contains_xy(&difference, Real::from(-2), Real::zero()),
            Some(true)
        );
        assert_eq!(
            curve::contains_xy(&difference, Real::from(2), Real::zero()),
            Some(false)
        );

        let union = exact_compound_composition(&[first.clone(), second.clone()], &[], &decisions)
            .expect("exact signed union must retain both curve-loop operands");
        assert_eq!(
            curve::contains_xy(&union, Real::from(5), Real::zero()),
            Some(true)
        );

        let hard_cut = exact_compound_composition(
            &[first, curve::rectangle(Real::from(6), Real::one())],
            &[second.clone(), second],
            &decisions,
        )
        .expect("a dominant exact cut must survive overlapping positive operands");
        let half = (Real::one() / Real::from(2_u8)).unwrap();
        assert_eq!(
            curve::contains_xy(&hard_cut, Real::from(2), half),
            Some(false)
        );
    }

    #[test]
    fn disjoint_additive_compound_does_not_invent_coincident_holes() {
        let decisions = MaterializationDecisions::new(&MaterializationContext::STRICT);
        let circle = curve::translated(
            &exact_circle_profile(&Real::from(3), "disjoint-circle", &decisions).unwrap(),
            Real::from(15),
            Real::from(3),
        );
        let rectangle = curve::translated(
            &curve::rectangle(Real::from(26), Real::from(6)),
            Real::from(15),
            Real::from(15),
        );
        let union = exact_compound_composition(&[circle, rectangle], &[], &decisions).unwrap();
        let profiles = curve::try_finite_profiles(&union, &decisions.geometry_context())
            .unwrap()
            .into_value();
        assert_eq!(profiles.len(), 2);
        assert!(profiles.iter().all(|profile| profile.holes().is_empty()));
    }

    #[test]
    fn overlapping_additive_union_does_not_reclassify_material_as_a_hole() {
        let decisions = MaterializationDecisions::new(&MaterializationContext::STRICT);
        let trace = curve::translated(
            &curve::rectangle(Real::from(20), Real::one()),
            Real::from(15),
            Real::from(3),
        );
        let radius = (Real::from(3) / Real::from(2)).unwrap();
        let via_land = curve::translated(
            &exact_circle_profile(&radius, "via-land", &decisions).unwrap(),
            Real::from(15),
            Real::from(3),
        );

        let union = exact_compound_composition(&[trace, via_land], &[], &decisions).unwrap();
        let profiles = curve::try_finite_profiles(&union, &decisions.geometry_context())
            .unwrap()
            .into_value();
        assert_eq!(profiles.len(), 1);
        assert!(profiles[0].holes().is_empty());
        assert_eq!(
            curve::contains_xy(&union, Real::from(15), Real::from(4)),
            Some(true)
        );
    }

    #[test]
    fn additive_composition_preserves_holes_islands_and_duplicate_profiles() {
        for context in [
            MaterializationContext::STRICT,
            MaterializationContext::APPROXIMATE_512,
        ] {
            let decisions = MaterializationDecisions::new(&context);
            let square = |half: i32| {
                curve::translated(
                    &curve::rectangle(Real::from(2 * half), Real::from(2 * half)),
                    Real::from(-half),
                    Real::from(-half),
                )
            };
            let exterior = square(6);
            let cut = square(4);
            let ring = decisions
                .exact_curve(|| exterior.boolean_region(&cut, BooleanOp::Difference))
                .unwrap();
            let island = square(1);
            let bridge = curve::translated(
                &curve::rectangle(Real::from(2), Real::from(4)),
                Real::from(3),
                Real::from(-2),
            );
            let operands = [ring.clone(), island, bridge, ring, CurveRegion2::empty()];
            let aggregate = exact_compound_composition(&operands, &[], &decisions).unwrap();
            let sequential = operands
                .iter()
                .try_fold(CurveRegion2::empty(), |region, operand| {
                    decisions.exact_curve(|| region.boolean_region(operand, BooleanOp::Union))
                })
                .unwrap();
            assert!(
                decisions
                    .exact_curve(|| aggregate.boolean_region(&sequential, BooleanOp::Xor))
                    .unwrap()
                    .is_empty()
            );
            for (x, y, inside) in [
                (0, 0, true),
                (2, 0, false),
                (4, 0, true),
                (5, 5, true),
                (7, 0, false),
            ] {
                assert_eq!(
                    curve::contains_xy(&aggregate, Real::from(x), Real::from(y)),
                    Some(inside)
                );
            }
            assert!(
                exact_compound_composition(&[], &[], &decisions)
                    .unwrap()
                    .is_empty()
            );
            assert_eq!(decisions.certainty(), GeometryCertainty::Certified);
        }
    }

    #[test]
    fn pad_local_rotation_precedes_translation_and_board_placement() {
        let decisions = MaterializationDecisions::new(&MaterializationContext::STRICT);
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
            pad_profile(&pad, &MaterializationOptions::default(), &decisions).unwrap(),
            &pad,
            &placement,
        );
        let profiles = curve::try_finite_profiles(&profile, &decisions.geometry_context())
            .unwrap()
            .into_value();
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
