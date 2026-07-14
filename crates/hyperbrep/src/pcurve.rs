//! Retained planar pcurve image-equality evidence.
//!
//! BREP trims are usually carried as parameter-space curves on a supporting
//! surface. For planar faces, the first exact question is not a sampled 3D
//! proximity test: it is whether two pcurves lie on the same retained planar
//! surface and replay the same UV image. This module keeps that evidence
//! explicit.

use std::cell::OnceCell;
use std::fmt;
use std::rc::Rc;

use hypercurve::{
    Classification, Contour2, ContourPointLocation, CurveError, CurveGeometry2, CurvePath2,
    CurvePolicy, ExactCurveError, Point2, PreparedRegionView2, RegionPointLocation, RegionView2,
    Segment2, UncertaintyReason,
};

use crate::BrepSurfaceId;

/// Result of planar pcurve, trim, and face-region operations.
pub type BrepPlanarResult<T> = Result<T, BrepPlanarError>;

/// Failure from planar BREP pcurve, trim, or face-region processing.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum BrepPlanarError {
    /// A face region has no material trim loop.
    MissingMaterialLoop,
    /// A trim references a different support surface than its face region.
    SurfaceMismatch {
        /// Surface required by the face region.
        expected: BrepSurfaceId,
        /// Surface retained by the offending trim.
        actual: BrepSurfaceId,
    },
    /// The same exact trim loop was supplied more than once.
    DuplicateTrimLoop,
    /// A trim loop self-intersects or self-touches.
    SelfContactingTrimLoop,
    /// Distinct material/hole trim loops intersect or touch.
    IntersectingTrimLoops,
    /// A retained trim unexpectedly has no segment geometry.
    MissingTrimGeometry,
    /// A hole is not strictly owned by any material loop.
    UnownedHole,
    /// An exact trim predicate could not decide the required topology.
    UnresolvedTrimTopology(UncertaintyReason),
    /// A retained report was constructed with contradictory evidence.
    InvalidReport(String),
    /// The underlying exact planar curve operation failed.
    Curve(CurveError),
    /// A top-level exact planar curve operation failed with retained context.
    ExactCurve(ExactCurveError),
}

impl From<CurveError> for BrepPlanarError {
    fn from(error: CurveError) -> Self {
        Self::Curve(error)
    }
}

impl From<ExactCurveError> for BrepPlanarError {
    fn from(error: ExactCurveError) -> Self {
        Self::ExactCurve(error)
    }
}

impl fmt::Display for BrepPlanarError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingMaterialLoop => {
                formatter.write_str("planar BREP face region has no material trim loop")
            }
            Self::SurfaceMismatch { expected, actual } => write!(
                formatter,
                "planar BREP trim surface {} does not match face surface {}",
                actual.get(),
                expected.get()
            ),
            Self::DuplicateTrimLoop => {
                formatter.write_str("planar BREP face region contains a duplicate trim loop")
            }
            Self::SelfContactingTrimLoop => {
                formatter.write_str("planar BREP trim loop has a self-contact")
            }
            Self::IntersectingTrimLoops => {
                formatter.write_str("planar BREP face region has intersecting trim loops")
            }
            Self::MissingTrimGeometry => {
                formatter.write_str("planar BREP trim loop has no segment geometry")
            }
            Self::UnownedHole => {
                formatter.write_str("planar BREP hole is not owned by a material loop")
            }
            Self::UnresolvedTrimTopology(reason) => {
                write!(
                    formatter,
                    "planar BREP trim topology is unresolved: {reason:?}"
                )
            }
            Self::InvalidReport(message) => {
                write!(
                    formatter,
                    "planar BREP report evidence is invalid: {message}"
                )
            }
            Self::Curve(error) => write!(formatter, "planar curve operation failed: {error}"),
            Self::ExactCurve(error) => {
                write!(
                    formatter,
                    "top-level planar curve operation failed: {error}"
                )
            }
        }
    }
}

impl std::error::Error for BrepPlanarError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Curve(error) => Some(error),
            Self::ExactCurve(error) => Some(error),
            Self::MissingMaterialLoop
            | Self::SurfaceMismatch { .. }
            | Self::DuplicateTrimLoop
            | Self::SelfContactingTrimLoop
            | Self::IntersectingTrimLoops
            | Self::MissingTrimGeometry
            | Self::UnownedHole
            | Self::UnresolvedTrimTopology(_)
            | Self::InvalidReport(_) => None,
        }
    }
}

/// Exact image relation between two retained planar pcurves.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BrepPcurveImageRelation {
    /// Both pcurves are on the same retained planar surface and have the same
    /// UV segment image with the same traversal direction.
    SameDirected,
    /// Both pcurves are on the same retained planar surface and have the same
    /// UV segment image with opposite traversal direction.
    SameReversed,
    /// The retained planar support surfaces differ, so the image equality
    /// predicate is blocked before comparing UV curves.
    SurfaceMismatch,
    /// Both pcurves are on the same retained planar surface, but their exact
    /// UV segment images differ.
    Different,
}

/// Evidence report for one planar pcurve image-equality predicate.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepPcurveImageEqualityReport {
    relation: BrepPcurveImageRelation,
    surface: Option<BrepSurfaceId>,
    curve_count: usize,
}

/// Open retained top-level pcurve on a planar support surface.
#[derive(Clone, Debug)]
pub struct BrepPcurve {
    surface: BrepSurfaceId,
    data: Rc<BrepPcurveData>,
}

#[derive(Debug)]
struct BrepPcurveData {
    path: CurvePath2,
    reversed_path: OnceCell<Result<CurvePath2, ExactCurveError>>,
    native_segments: OnceCell<Option<Vec<Segment2>>>,
}

/// Closed retained trim-loop pcurve on a planar support surface.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepPlanarTrimLoop {
    surface: BrepSurfaceId,
    contour: Contour2,
}

/// Retained planar face assembled from material and hole pcurve trim loops.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepPlanarFaceRegion {
    surface: BrepSurfaceId,
    material_loops: Vec<BrepPlanarTrimLoop>,
    hole_loops: Vec<BrepPlanarTrimLoop>,
}

/// Prepared retained planar face for repeated support-surface and UV queries.
///
/// The prepared object keeps the retained BREP support identity beside a
/// prepared borrowed UV region. Cached boxes and prepared segment predicates
/// are only broad-phase evidence: support-surface mismatch, boundary hits, and
/// inside/outside status still replay through exact classifiers.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedBrepPlanarFaceRegion<'a> {
    face: &'a BrepPlanarFaceRegion,
    region: PreparedRegionView2<'a>,
}

/// Point classification result for a retained planar face.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BrepPlanarFacePointLocation {
    /// The query was made against a different retained support surface.
    SurfaceMismatch,
    /// The UV point is outside the retained face.
    Outside,
    /// The UV point lies on a material or hole trim boundary.
    Boundary,
    /// The UV point is inside the retained face.
    Inside,
}

/// Evidence report for an exact UV point-in-face query.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepPlanarFacePointReport {
    location: BrepPlanarFacePointLocation,
    surface: Option<BrepSurfaceId>,
    material_loop_count: usize,
    hole_loop_count: usize,
}

/// Role of the retained trim loop that owns a matched pcurve edge-use.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BrepPlanarTrimLoopRole {
    /// The matched pcurve lies on a material trim loop.
    Material,
    /// The matched pcurve lies on a hole trim loop.
    Hole,
}

/// Exact edge-use agreement between a retained planar pcurve and face trims.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BrepPlanarFaceEdgeUseRelation {
    /// The query was made against a different retained support surface.
    SurfaceMismatch,
    /// The pcurve's exact UV image matches a contiguous trim subchain in the
    /// same traversal direction.
    BoundarySameDirected,
    /// The pcurve's exact UV image matches a contiguous trim subchain in the
    /// opposite traversal direction.
    BoundarySameReversed,
    /// The support surface matches, but the pcurve image is not a retained
    /// trim-boundary subchain of this face.
    NotTrimBoundary,
    /// The support surface matches, but this planar face still uses native
    /// segment trims and cannot yet replay the pcurve's top-level curve family.
    UnsupportedCurveFamily,
}

/// Evidence report for a retained planar pcurve edge-use query.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepPlanarFaceEdgeUseReport {
    relation: BrepPlanarFaceEdgeUseRelation,
    surface: Option<BrepSurfaceId>,
    trim_role: Option<BrepPlanarTrimLoopRole>,
    trim_loop_index: Option<usize>,
    trim_segment_index: Option<usize>,
    segment_count: usize,
    trim_role_loop_count: Option<usize>,
    trim_loop_segment_count: Option<usize>,
}

impl BrepPcurveImageRelation {
    /// Returns true when the reports certify equal UV images.
    pub const fn is_same_image(self) -> bool {
        matches!(self, Self::SameDirected | Self::SameReversed)
    }

    /// Returns true when equal images have opposite traversal orientation.
    pub const fn is_reversed(self) -> bool {
        matches!(self, Self::SameReversed)
    }
}

impl BrepPcurveImageEqualityReport {
    /// Constructs a planar pcurve image-equality report.
    pub fn new(
        relation: BrepPcurveImageRelation,
        surface: Option<BrepSurfaceId>,
        curve_count: usize,
    ) -> BrepPlanarResult<Self> {
        validate_planar_pcurve_image_report(relation, surface, curve_count)?;
        Ok(Self {
            relation,
            surface,
            curve_count,
        })
    }

    /// Returns the certified relation.
    pub const fn relation(&self) -> BrepPcurveImageRelation {
        self.relation
    }

    /// Returns the common retained surface when both pcurves share one.
    pub const fn surface(&self) -> Option<BrepSurfaceId> {
        self.surface
    }

    /// Returns the authored curve count in the compared UV image when it matched.
    pub const fn curve_count(&self) -> usize {
        self.curve_count
    }
}

impl BrepPcurve {
    /// Constructs an open retained planar pcurve from Hypercurve's top-level path.
    pub fn new(surface: BrepSurfaceId, path: CurvePath2) -> Self {
        Self {
            surface,
            data: Rc::new(BrepPcurveData {
                path,
                reversed_path: OnceCell::new(),
                native_segments: OnceCell::new(),
            }),
        }
    }

    /// Returns the retained planar surface identity.
    pub const fn surface(&self) -> BrepSurfaceId {
        self.surface
    }

    /// Returns the retained top-level UV curve path.
    pub fn path(&self) -> &CurvePath2 {
        &self.data.path
    }

    /// Returns whether reverse traversal has already been retained.
    pub fn is_reversed_path_cached(&self) -> bool {
        self.data.reversed_path.get().is_some()
    }

    /// Returns whether native line/arc extraction has already been retained.
    pub fn is_native_segment_view_cached(&self) -> bool {
        self.data.native_segments.get().is_some()
    }

    /// Compares two open planar pcurves by exact UV image.
    ///
    /// This is a structural exact predicate over authored top-level curves:
    /// equal images must have identical curve boundaries in UV, either in the
    /// same order or in exact reverse order. It supports every top-level
    /// Hypercurve family, including arbitrary rational Beziers and NURBS, but
    /// does not sample or merge unsplit overlaps; those remain later
    /// trim-splitting work.
    pub fn image_equality_report(
        &self,
        other: &Self,
    ) -> BrepPlanarResult<BrepPcurveImageEqualityReport> {
        if self.surface != other.surface {
            return BrepPcurveImageEqualityReport::new(
                BrepPcurveImageRelation::SurfaceMismatch,
                None,
                0,
            );
        }
        let relation = if same_directed_curve_paths(self.path(), other.path()) {
            BrepPcurveImageRelation::SameDirected
        } else if same_directed_curve_paths(self.path(), other.reversed_path()?) {
            BrepPcurveImageRelation::SameReversed
        } else {
            BrepPcurveImageRelation::Different
        };
        let curve_count = usize::from(relation.is_same_image()) * self.path().curves().len();
        BrepPcurveImageEqualityReport::new(relation, Some(self.surface), curve_count)
    }

    fn reversed_path(&self) -> BrepPlanarResult<&CurvePath2> {
        match self
            .data
            .reversed_path
            .get_or_init(|| self.path().reversed())
        {
            Ok(path) => Ok(path),
            Err(error) => Err(BrepPlanarError::ExactCurve(error.clone())),
        }
    }

    fn native_segments(&self) -> Option<&[Segment2]> {
        self.data
            .native_segments
            .get_or_init(|| {
                self.path()
                    .curves()
                    .iter()
                    .map(|curve| match curve.geometry() {
                        CurveGeometry2::Line(line) => Some(Segment2::Line(line.clone())),
                        CurveGeometry2::CircularArc(arc) => Some(Segment2::Arc(arc.clone())),
                        CurveGeometry2::QuadraticBezier(_)
                        | CurveGeometry2::CubicBezier(_)
                        | CurveGeometry2::RationalQuadraticBezier(_)
                        | CurveGeometry2::RationalBezier(_)
                        | CurveGeometry2::PolynomialBSpline(_)
                        | CurveGeometry2::Nurbs(_) => None,
                    })
                    .collect::<Option<Vec<_>>>()
            })
            .as_deref()
    }
}

impl PartialEq for BrepPcurve {
    fn eq(&self, other: &Self) -> bool {
        self.surface == other.surface && self.path() == other.path()
    }
}

impl BrepPlanarTrimLoop {
    /// Constructs a closed retained planar trim-loop pcurve.
    pub const fn new(surface: BrepSurfaceId, contour: Contour2) -> Self {
        Self { surface, contour }
    }

    /// Returns the retained planar surface identity.
    pub const fn surface(&self) -> BrepSurfaceId {
        self.surface
    }

    /// Returns the retained UV contour.
    pub const fn contour(&self) -> &Contour2 {
        &self.contour
    }

    /// Compares two closed planar trim loops by exact cyclic UV image.
    ///
    /// Closed loops may start at different trim vertices, so this accepts
    /// cyclic rotations as well as opposite traversal direction. Fill rules are
    /// not part of pcurve image equality; this is only the support-surface/UV
    /// image predicate needed before face-role policy can run.
    pub fn image_equality_report(
        &self,
        other: &Self,
    ) -> BrepPlanarResult<BrepPcurveImageEqualityReport> {
        if self.surface != other.surface {
            return BrepPcurveImageEqualityReport::new(
                BrepPcurveImageRelation::SurfaceMismatch,
                None,
                0,
            );
        }
        let relation =
            if same_directed_segment_cycle(self.contour.segments(), other.contour.segments()) {
                BrepPcurveImageRelation::SameDirected
            } else if same_reversed_segment_cycle(self.contour.segments(), other.contour.segments())
            {
                BrepPcurveImageRelation::SameReversed
            } else {
                BrepPcurveImageRelation::Different
            };
        let segment_count = usize::from(relation.is_same_image()) * self.contour.len();
        BrepPcurveImageEqualityReport::new(relation, Some(self.surface), segment_count)
    }
}

impl BrepPlanarFaceRegion {
    /// Constructs a retained planar face from material and hole trim loops.
    ///
    /// Every trim loop must reference the same retained planar support surface.
    /// This validates support identity before any point-in-face predicate can
    /// consume UV topology.
    pub fn try_new(
        surface: BrepSurfaceId,
        material_loops: Vec<BrepPlanarTrimLoop>,
        hole_loops: Vec<BrepPlanarTrimLoop>,
    ) -> BrepPlanarResult<Self> {
        if material_loops.is_empty() {
            return Err(BrepPlanarError::MissingMaterialLoop);
        }
        if let Some(trim) = material_loops
            .iter()
            .chain(hole_loops.iter())
            .find(|trim| trim.surface != surface)
        {
            return Err(BrepPlanarError::SurfaceMismatch {
                expected: surface,
                actual: trim.surface,
            });
        }
        validate_planar_face_simple_trim_loops(&material_loops)?;
        validate_planar_face_simple_trim_loops(&hole_loops)?;
        validate_planar_face_distinct_trim_loops(&material_loops, &hole_loops)?;
        validate_planar_face_same_role_trim_separation(&material_loops)?;
        validate_planar_face_same_role_trim_separation(&hole_loops)?;
        validate_planar_face_hole_ownership(&material_loops, &hole_loops)?;
        Ok(Self {
            surface,
            material_loops,
            hole_loops,
        })
    }

    /// Returns the retained planar support surface.
    pub const fn surface(&self) -> BrepSurfaceId {
        self.surface
    }

    /// Returns material trim loops.
    pub fn material_loops(&self) -> &[BrepPlanarTrimLoop] {
        &self.material_loops
    }

    /// Returns hole trim loops.
    pub fn hole_loops(&self) -> &[BrepPlanarTrimLoop] {
        &self.hole_loops
    }

    /// Prepares this face for repeated support-surface and UV point queries.
    ///
    /// Preparation borrows the retained trim loops and caches the UV
    /// [`PreparedRegionView2`] used by repeated point-in-face calls. It does
    /// not certify any query by itself; every call still first checks the
    /// retained support-surface identity and then delegates to the exact
    /// boundary-first region classifier.
    pub fn prepare_point_queries(&self, policy: &CurvePolicy) -> PreparedBrepPlanarFaceRegion<'_> {
        let material = self
            .material_loops
            .iter()
            .map(|trim| trim.contour())
            .collect::<Vec<_>>();
        let holes = self
            .hole_loops
            .iter()
            .map(|trim| trim.contour())
            .collect::<Vec<_>>();
        let region = RegionView2::from_contours(material, holes);
        PreparedBrepPlanarFaceRegion {
            face: self,
            region: PreparedRegionView2::from_region_view(&region, policy),
        }
    }

    /// Prepares this face for repeated retained topology queries.
    ///
    /// This currently exposes the same point-query package as
    /// [`BrepPlanarFaceRegion::prepare_point_queries`]. Segment/edge-use and
    /// analytic-surface frame packages can extend the prepared face handle
    /// without changing the support-surface-first report contract.
    pub fn prepare_topology_queries(
        &self,
        policy: &CurvePolicy,
    ) -> PreparedBrepPlanarFaceRegion<'_> {
        self.prepare_point_queries(policy)
    }

    /// Classifies a UV point against this retained planar face.
    ///
    /// The query first checks retained support-surface identity. Only matching
    /// surfaces are passed to the exact UV region classifier, which checks trim
    /// boundaries before winding/inside status. This preserves the BREP
    /// distinction between support-surface agreement and trim containment
    /// rather than collapsing both into a sampled point-in-polygon test.
    pub fn classify_uv_point(
        &self,
        query_surface: BrepSurfaceId,
        uv: &Point2,
        policy: &CurvePolicy,
    ) -> BrepPlanarResult<Classification<BrepPlanarFacePointReport>> {
        if query_surface != self.surface {
            return Ok(Classification::Decided(BrepPlanarFacePointReport::new(
                BrepPlanarFacePointLocation::SurfaceMismatch,
                None,
                self.material_loops.len(),
                self.hole_loops.len(),
            )?));
        }

        let material = self
            .material_loops
            .iter()
            .map(|trim| trim.contour())
            .collect::<Vec<_>>();
        let holes = self
            .hole_loops
            .iter()
            .map(|trim| trim.contour())
            .collect::<Vec<_>>();
        let region = RegionView2::from_contours(material, holes);
        face_point_report_from_region_classification(
            region.classify_point(uv, policy),
            self.surface,
            self.material_loops.len(),
            self.hole_loops.len(),
        )
    }

    /// Reports whether an open retained planar pcurve is a face trim edge-use.
    ///
    /// This predicate is structural over retained UV segments: the pcurve must
    /// be an exact contiguous subchain of a material or hole trim loop, either
    /// directed or reversed. It deliberately does not project, sample, or
    /// overlap-split arbitrary curves. Combinatorial topology is accepted only
    /// after exact construction evidence replays.
    pub fn edge_use_report(
        &self,
        pcurve: &BrepPcurve,
    ) -> BrepPlanarResult<BrepPlanarFaceEdgeUseReport> {
        if pcurve.surface != self.surface {
            return BrepPlanarFaceEdgeUseReport::new(
                BrepPlanarFaceEdgeUseRelation::SurfaceMismatch,
                None,
                None,
                None,
                None,
                0,
            );
        }

        let Some(segments) = pcurve.native_segments() else {
            return BrepPlanarFaceEdgeUseReport::new(
                BrepPlanarFaceEdgeUseRelation::UnsupportedCurveFamily,
                Some(self.surface),
                None,
                None,
                None,
                0,
            );
        };
        face_edge_use_report_from_loops(self, segments)
    }
}

impl<'a> PreparedBrepPlanarFaceRegion<'a> {
    /// Returns the retained planar face that supplied this prepared view.
    pub const fn face(&self) -> &'a BrepPlanarFaceRegion {
        self.face
    }

    /// Returns the retained planar support surface.
    pub const fn surface(&self) -> BrepSurfaceId {
        self.face.surface
    }

    /// Returns the prepared borrowed UV region.
    pub const fn prepared_region(&self) -> &PreparedRegionView2<'a> {
        &self.region
    }

    /// Returns the number of retained material trim loops.
    pub fn material_loop_count(&self) -> usize {
        self.face.material_loops.len()
    }

    /// Returns the number of retained hole trim loops.
    pub fn hole_loop_count(&self) -> usize {
        self.face.hole_loops.len()
    }

    /// Classifies a UV point against this prepared retained planar face.
    ///
    /// The support-surface identity check stays outside the prepared UV region.
    /// Preparation retains reusable object structure; it cannot turn a query
    /// against the wrong surface into a valid geometric predicate.
    pub fn classify_uv_point(
        &self,
        query_surface: BrepSurfaceId,
        uv: &Point2,
        policy: &CurvePolicy,
    ) -> BrepPlanarResult<Classification<BrepPlanarFacePointReport>> {
        if query_surface != self.face.surface {
            return Ok(Classification::Decided(BrepPlanarFacePointReport::new(
                BrepPlanarFacePointLocation::SurfaceMismatch,
                None,
                self.material_loop_count(),
                self.hole_loop_count(),
            )?));
        }

        face_point_report_from_region_classification(
            self.region.classify_point(uv, policy),
            self.face.surface,
            self.material_loop_count(),
            self.hole_loop_count(),
        )
    }

    /// Reports whether an open retained planar pcurve is a prepared face trim edge-use.
    ///
    /// Preparation does not change the proof obligation: support-surface
    /// identity is still checked first, and the accepted edge-use must replay
    /// as an exact contiguous UV subchain of a retained trim. The prepared face
    /// owns the borrowed trim structure needed by future broad-phase segment
    /// filters while keeping this exact image predicate authoritative.
    pub fn edge_use_report(
        &self,
        pcurve: &BrepPcurve,
    ) -> BrepPlanarResult<BrepPlanarFaceEdgeUseReport> {
        if pcurve.surface != self.face.surface {
            return BrepPlanarFaceEdgeUseReport::new(
                BrepPlanarFaceEdgeUseRelation::SurfaceMismatch,
                None,
                None,
                None,
                None,
                0,
            );
        }

        let Some(segments) = pcurve.native_segments() else {
            return BrepPlanarFaceEdgeUseReport::new(
                BrepPlanarFaceEdgeUseRelation::UnsupportedCurveFamily,
                Some(self.face.surface),
                None,
                None,
                None,
                0,
            );
        };
        face_edge_use_report_from_loops(self.face, segments)
    }
}

fn validate_planar_face_distinct_trim_loops(
    material_loops: &[BrepPlanarTrimLoop],
    hole_loops: &[BrepPlanarTrimLoop],
) -> BrepPlanarResult<()> {
    for (index, trim) in material_loops.iter().enumerate() {
        if material_loops[index + 1..].contains(trim) || hole_loops.contains(trim) {
            return Err(BrepPlanarError::DuplicateTrimLoop);
        }
    }
    for (index, trim) in hole_loops.iter().enumerate() {
        if hole_loops[index + 1..].contains(trim) {
            return Err(BrepPlanarError::DuplicateTrimLoop);
        }
    }
    Ok(())
}

fn validate_planar_face_simple_trim_loops(loops: &[BrepPlanarTrimLoop]) -> BrepPlanarResult<()> {
    let policy = CurvePolicy::certified();
    for trim in loops {
        match trim.contour.has_self_contacts(&policy)? {
            Classification::Decided(false) => {}
            Classification::Decided(true) => return Err(BrepPlanarError::SelfContactingTrimLoop),
            Classification::Uncertain(reason) => {
                return Err(BrepPlanarError::UnresolvedTrimTopology(reason));
            }
        }
    }
    Ok(())
}

fn validate_planar_face_same_role_trim_separation(
    loops: &[BrepPlanarTrimLoop],
) -> BrepPlanarResult<()> {
    let policy = CurvePolicy::certified();
    for (index, trim) in loops.iter().enumerate() {
        for other in &loops[index + 1..] {
            if !trim
                .contour
                .intersect_contour(&other.contour, &policy)?
                .is_empty()
            {
                return Err(BrepPlanarError::IntersectingTrimLoops);
            }
        }
    }
    Ok(())
}

fn validate_planar_face_hole_ownership(
    material_loops: &[BrepPlanarTrimLoop],
    hole_loops: &[BrepPlanarTrimLoop],
) -> BrepPlanarResult<()> {
    let policy = CurvePolicy::certified();
    for hole in hole_loops {
        let Some(point) = hole
            .contour
            .segments()
            .first()
            .map(|segment| segment.start())
        else {
            return Err(BrepPlanarError::MissingTrimGeometry);
        };
        let mut owned_by_material = false;
        for material in material_loops {
            if !material
                .contour
                .intersect_contour(&hole.contour, &policy)?
                .is_empty()
            {
                return Err(BrepPlanarError::IntersectingTrimLoops);
            }
            match material.contour.classify_point(point, &policy) {
                Classification::Decided(ContourPointLocation::Inside) => {
                    owned_by_material = true;
                }
                Classification::Decided(
                    ContourPointLocation::Boundary | ContourPointLocation::Outside,
                ) => {}
                Classification::Uncertain(reason) => {
                    return Err(BrepPlanarError::UnresolvedTrimTopology(reason));
                }
            }
        }
        if !owned_by_material {
            return Err(BrepPlanarError::UnownedHole);
        }
    }
    Ok(())
}

impl BrepPlanarFacePointLocation {
    /// Returns true when the query reached an exact inside/outside/boundary result.
    pub const fn is_trim_classification(self) -> bool {
        !matches!(self, Self::SurfaceMismatch)
    }
}

impl BrepPlanarTrimLoopRole {
    /// Returns true for material loops.
    pub const fn is_material(self) -> bool {
        matches!(self, Self::Material)
    }

    /// Returns true for hole loops.
    pub const fn is_hole(self) -> bool {
        matches!(self, Self::Hole)
    }
}

impl BrepPlanarFaceEdgeUseRelation {
    /// Returns true when the pcurve is certified as a retained trim boundary.
    pub const fn is_boundary(self) -> bool {
        matches!(
            self,
            Self::BoundarySameDirected | Self::BoundarySameReversed
        )
    }

    /// Returns true when the matched boundary image has opposite traversal.
    pub const fn is_reversed(self) -> bool {
        matches!(self, Self::BoundarySameReversed)
    }
}

impl BrepPlanarFaceEdgeUseReport {
    /// Constructs a retained planar face edge-use report.
    ///
    /// Boundary reports are produced by retained-face query methods because
    /// they require face extent evidence to certify trim-loop and segment
    /// indices. This constructor accepts only self-contained blocker reports.
    pub fn new(
        relation: BrepPlanarFaceEdgeUseRelation,
        surface: Option<BrepSurfaceId>,
        trim_role: Option<BrepPlanarTrimLoopRole>,
        trim_loop_index: Option<usize>,
        trim_segment_index: Option<usize>,
        segment_count: usize,
    ) -> BrepPlanarResult<Self> {
        let report = Self {
            relation,
            surface,
            trim_role,
            trim_loop_index,
            trim_segment_index,
            segment_count,
            trim_role_loop_count: None,
            trim_loop_segment_count: None,
        };
        validate_planar_face_edge_use_report(&report)?;
        Ok(report)
    }

    #[allow(clippy::too_many_arguments)]
    fn new_with_face_extent_evidence(
        relation: BrepPlanarFaceEdgeUseRelation,
        surface: BrepSurfaceId,
        trim_role: BrepPlanarTrimLoopRole,
        trim_loop_index: usize,
        trim_segment_index: usize,
        segment_count: usize,
        trim_role_loop_count: usize,
        trim_loop_segment_count: usize,
    ) -> BrepPlanarResult<Self> {
        let report = Self {
            relation,
            surface: Some(surface),
            trim_role: Some(trim_role),
            trim_loop_index: Some(trim_loop_index),
            trim_segment_index: Some(trim_segment_index),
            segment_count,
            trim_role_loop_count: Some(trim_role_loop_count),
            trim_loop_segment_count: Some(trim_loop_segment_count),
        };
        validate_planar_face_edge_use_report(&report)?;
        Ok(report)
    }

    /// Returns the certified edge-use relation or blocker.
    pub const fn relation(&self) -> BrepPlanarFaceEdgeUseRelation {
        self.relation
    }

    /// Returns the matching retained surface when edge-use matching ran.
    pub const fn surface(&self) -> Option<BrepSurfaceId> {
        self.surface
    }

    /// Returns the role of the matched trim loop, when boundary evidence exists.
    pub const fn trim_role(&self) -> Option<BrepPlanarTrimLoopRole> {
        self.trim_role
    }

    /// Returns the matched trim loop index inside its material or hole bin.
    pub const fn trim_loop_index(&self) -> Option<usize> {
        self.trim_loop_index
    }

    /// Returns the matched trim segment index where the pcurve traversal starts.
    ///
    /// For reversed matches, this is the original trim segment whose reversed
    /// image supplies the first pcurve segment.
    pub const fn trim_segment_index(&self) -> Option<usize> {
        self.trim_segment_index
    }

    /// Returns the number of pcurve segments accepted as trim-boundary evidence.
    pub const fn segment_count(&self) -> usize {
        self.segment_count
    }
}

impl BrepPlanarFacePointReport {
    /// Constructs a retained planar face point-query report.
    pub fn new(
        location: BrepPlanarFacePointLocation,
        surface: Option<BrepSurfaceId>,
        material_loop_count: usize,
        hole_loop_count: usize,
    ) -> BrepPlanarResult<Self> {
        validate_planar_face_point_report(location, surface, material_loop_count)?;
        Ok(Self {
            location,
            surface,
            material_loop_count,
            hole_loop_count,
        })
    }

    /// Returns the exact query location or blocker.
    pub const fn location(&self) -> BrepPlanarFacePointLocation {
        self.location
    }

    /// Returns the matching retained surface when the query reached trim classification.
    pub const fn surface(&self) -> Option<BrepSurfaceId> {
        self.surface
    }

    /// Returns the number of material trim loops in the face.
    pub const fn material_loop_count(&self) -> usize {
        self.material_loop_count
    }

    /// Returns the number of hole trim loops in the face.
    pub const fn hole_loop_count(&self) -> usize {
        self.hole_loop_count
    }
}

fn validate_planar_pcurve_image_report(
    relation: BrepPcurveImageRelation,
    surface: Option<BrepSurfaceId>,
    curve_count: usize,
) -> BrepPlanarResult<()> {
    match relation {
        BrepPcurveImageRelation::SurfaceMismatch => {
            if surface.is_some() || curve_count != 0 {
                return Err(BrepPlanarError::InvalidReport(
                    "surface-mismatch pcurve image report must not carry image evidence".into(),
                ));
            }
        }
        BrepPcurveImageRelation::Different => {
            if surface.is_none() || curve_count != 0 {
                return Err(BrepPlanarError::InvalidReport(
                    "different pcurve image report must carry only matching-surface evidence"
                        .into(),
                ));
            }
        }
        BrepPcurveImageRelation::SameDirected | BrepPcurveImageRelation::SameReversed => {
            if surface.is_none() || curve_count == 0 {
                return Err(BrepPlanarError::InvalidReport(
                    "same-image pcurve report must carry surface and positive curve evidence"
                        .into(),
                ));
            }
        }
    }
    Ok(())
}

fn validate_planar_face_point_report(
    location: BrepPlanarFacePointLocation,
    surface: Option<BrepSurfaceId>,
    material_loop_count: usize,
) -> BrepPlanarResult<()> {
    if material_loop_count == 0 {
        return Err(BrepPlanarError::InvalidReport(
            "retained planar face point report must reference a face with material loops".into(),
        ));
    }
    match location {
        BrepPlanarFacePointLocation::SurfaceMismatch => {
            if surface.is_some() {
                return Err(BrepPlanarError::InvalidReport(
                    "surface-mismatch point report must not carry trim-classification surface evidence"
                        .into(),
                ));
            }
        }
        BrepPlanarFacePointLocation::Outside
        | BrepPlanarFacePointLocation::Boundary
        | BrepPlanarFacePointLocation::Inside => {
            if surface.is_none() {
                return Err(BrepPlanarError::InvalidReport(
                    "trim-classified point report must carry matching surface evidence".into(),
                ));
            }
        }
    }
    Ok(())
}

fn validate_planar_face_edge_use_report(
    report: &BrepPlanarFaceEdgeUseReport,
) -> BrepPlanarResult<()> {
    match report.relation {
        BrepPlanarFaceEdgeUseRelation::SurfaceMismatch => {
            if report.surface.is_some()
                || report.trim_role.is_some()
                || report.trim_loop_index.is_some()
                || report.trim_segment_index.is_some()
                || report.segment_count != 0
                || report.trim_role_loop_count.is_some()
                || report.trim_loop_segment_count.is_some()
            {
                return Err(BrepPlanarError::InvalidReport(
                    "surface-mismatch edge-use report must not carry trim evidence".into(),
                ));
            }
        }
        BrepPlanarFaceEdgeUseRelation::NotTrimBoundary
        | BrepPlanarFaceEdgeUseRelation::UnsupportedCurveFamily => {
            if report.surface.is_none()
                || report.trim_role.is_some()
                || report.trim_loop_index.is_some()
                || report.trim_segment_index.is_some()
                || report.segment_count != 0
                || report.trim_role_loop_count.is_some()
                || report.trim_loop_segment_count.is_some()
            {
                return Err(BrepPlanarError::InvalidReport(
                    "non-boundary edge-use report must carry only matching-surface evidence".into(),
                ));
            }
        }
        BrepPlanarFaceEdgeUseRelation::BoundarySameDirected
        | BrepPlanarFaceEdgeUseRelation::BoundarySameReversed => {
            if report.surface.is_none()
                || report.trim_role.is_none()
                || report.trim_loop_index.is_none()
                || report.trim_segment_index.is_none()
                || report.segment_count == 0
                || report.trim_role_loop_count.is_none()
                || report.trim_loop_segment_count.is_none()
            {
                return Err(BrepPlanarError::InvalidReport(
                    "boundary edge-use report must carry complete positive trim evidence".into(),
                ));
            }
            let (
                Some(trim_loop_index),
                Some(trim_segment_index),
                Some(trim_role_loop_count),
                Some(trim_loop_segment_count),
            ) = (
                report.trim_loop_index,
                report.trim_segment_index,
                report.trim_role_loop_count,
                report.trim_loop_segment_count,
            )
            else {
                return Err(BrepPlanarError::InvalidReport(
                    "boundary edge-use report must carry complete positive trim evidence".into(),
                ));
            };
            if trim_role_loop_count == 0
                || trim_loop_segment_count == 0
                || trim_loop_index >= trim_role_loop_count
                || trim_segment_index >= trim_loop_segment_count
                || report.segment_count > trim_loop_segment_count
            {
                return Err(BrepPlanarError::InvalidReport(
                    "boundary edge-use report trim indices must be certified by face extent evidence"
                        .into(),
                ));
            }
        }
    }
    Ok(())
}

fn same_directed_curve_paths(first: &CurvePath2, second: &CurvePath2) -> bool {
    first.curves().len() == second.curves().len()
        && first
            .curves()
            .iter()
            .zip(second.curves())
            .all(|(left, right)| left.geometry() == right.geometry())
}

fn same_directed_segment_cycle(first: &[Segment2], second: &[Segment2]) -> bool {
    let len = first.len();
    if len != second.len() {
        return false;
    }
    (0..len).any(|offset| {
        first
            .iter()
            .enumerate()
            .all(|(index, segment)| segment == &second[(offset + index) % len])
    })
}

fn same_reversed_segment_cycle(first: &[Segment2], second: &[Segment2]) -> bool {
    let len = first.len();
    if len != second.len() {
        return false;
    }
    (0..len).any(|offset| {
        first.iter().enumerate().all(|(index, segment)| {
            let reversed_index = (offset + len - 1 - index) % len;
            segment == &second[reversed_index].reversed()
        })
    })
}

fn face_edge_use_report_from_loops(
    face: &BrepPlanarFaceRegion,
    query_segments: &[Segment2],
) -> BrepPlanarResult<BrepPlanarFaceEdgeUseReport> {
    for (loop_index, trim) in face.material_loops.iter().enumerate() {
        if let Some((relation, segment_index)) =
            segment_subchain_relation(query_segments, trim.contour.segments())
        {
            return BrepPlanarFaceEdgeUseReport::new_with_face_extent_evidence(
                relation,
                face.surface,
                BrepPlanarTrimLoopRole::Material,
                loop_index,
                segment_index,
                query_segments.len(),
                face.material_loops.len(),
                trim.contour.len(),
            );
        }
    }
    for (loop_index, trim) in face.hole_loops.iter().enumerate() {
        if let Some((relation, segment_index)) =
            segment_subchain_relation(query_segments, trim.contour.segments())
        {
            return BrepPlanarFaceEdgeUseReport::new_with_face_extent_evidence(
                relation,
                face.surface,
                BrepPlanarTrimLoopRole::Hole,
                loop_index,
                segment_index,
                query_segments.len(),
                face.hole_loops.len(),
                trim.contour.len(),
            );
        }
    }

    BrepPlanarFaceEdgeUseReport::new(
        BrepPlanarFaceEdgeUseRelation::NotTrimBoundary,
        Some(face.surface),
        None,
        None,
        None,
        0,
    )
}

fn segment_subchain_relation(
    query_segments: &[Segment2],
    loop_segments: &[Segment2],
) -> Option<(BrepPlanarFaceEdgeUseRelation, usize)> {
    if query_segments.is_empty() || query_segments.len() > loop_segments.len() {
        return None;
    }
    if let Some(segment_index) = directed_segment_subchain_start(query_segments, loop_segments) {
        return Some((
            BrepPlanarFaceEdgeUseRelation::BoundarySameDirected,
            segment_index,
        ));
    }
    reversed_segment_subchain_start(query_segments, loop_segments).map(|segment_index| {
        (
            BrepPlanarFaceEdgeUseRelation::BoundarySameReversed,
            segment_index,
        )
    })
}

fn directed_segment_subchain_start(
    query_segments: &[Segment2],
    loop_segments: &[Segment2],
) -> Option<usize> {
    let len = loop_segments.len();
    (0..len).find(|&offset| {
        query_segments
            .iter()
            .enumerate()
            .all(|(index, segment)| segment == &loop_segments[(offset + index) % len])
    })
}

fn reversed_segment_subchain_start(
    query_segments: &[Segment2],
    loop_segments: &[Segment2],
) -> Option<usize> {
    let len = loop_segments.len();
    (0..len).find(|&offset| {
        query_segments.iter().enumerate().all(|(index, segment)| {
            let loop_index = (offset + len - index) % len;
            segment == &loop_segments[loop_index].reversed()
        })
    })
}

fn face_point_report_from_region_classification(
    classification: Classification<RegionPointLocation>,
    surface: BrepSurfaceId,
    material_loop_count: usize,
    hole_loop_count: usize,
) -> BrepPlanarResult<Classification<BrepPlanarFacePointReport>> {
    let location = match classification {
        Classification::Decided(RegionPointLocation::Outside) => {
            BrepPlanarFacePointLocation::Outside
        }
        Classification::Decided(RegionPointLocation::Boundary) => {
            BrepPlanarFacePointLocation::Boundary
        }
        Classification::Decided(RegionPointLocation::Inside) => BrepPlanarFacePointLocation::Inside,
        Classification::Uncertain(UncertaintyReason::Boundary) => {
            BrepPlanarFacePointLocation::Boundary
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(Classification::Decided(BrepPlanarFacePointReport::new(
        location,
        Some(surface),
        material_loop_count,
        hole_loop_count,
    )?))
}
