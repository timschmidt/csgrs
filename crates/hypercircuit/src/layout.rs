//! Declarative PCB connectivity and layout intent.
//!
//! These objects retain electrical identity, package intent, placement, and
//! manufacturable feature parameters before geometry is materialized. Routes
//! and vias lower to `hyperpath` carriers for certification; the optional
//! `geometry` feature lowers the same retained objects to `csgrs::Profile`.

use std::{
    cell::RefCell,
    cmp::Ordering,
    collections::{BTreeMap, BTreeSet},
    fmt::{Display, Formatter},
    rc::Rc,
};

use hypercurve::{
    Aabb2 as CurveAabb2, CircularArc2, Classification, CubicBezier2, Curve2, CurveGeometry2,
    CurvePath2, CurvePolicy, CurveRegion2, CurveRegionLoopRole, FillRule, LineSeg2,
    Point2 as CurvePoint2, RegionPointLocation, UncertaintyReason,
};
use hyperlattice::Point2;
use hyperpath::{
    CubicBezier, ExplicitCircularArc, LinePathSegment, NetId as RoutingNetId, PcbTrace,
    PcbViaStack, SpecctraRoute, SpecctraRouteArc, SpecctraRouteBezier, SweptLineSegment,
    TraceLayer, ViaDrillIntent,
};
use hyperreal::{Real, RealSign};

use crate::{
    BoardId, Circuit, CircuitInstanceId, DifferentialPairId, EscapePolicyId, KeepoutId,
    LandPatternGraphicId, LandPatternId, LengthTuningPatternId, NetClassId, NetId, PadId,
    PhaseTuningGroupId, PinRef, PlacementConstraintId, RouteConstraintRegionId, RouteId,
    RouteRuleRegionId, ViaId, ViaStyleId, ZoneId,
};

/// Manufacturing/display role of a non-routing or routing layer.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LayerRole {
    /// Copper routing or plane layer.
    Copper(
        #[cfg_attr(
            feature = "interchange",
            serde(with = "crate::interchange::trace_layer")
        )]
        TraceLayer,
    ),
    /// Front solder mask.
    FrontSolderMask,
    /// Back solder mask.
    BackSolderMask,
    /// Front solder paste stencil.
    FrontPaste,
    /// Back solder paste stencil.
    BackPaste,
    /// Front silkscreen.
    FrontSilkscreen,
    /// Back silkscreen.
    BackSilkscreen,
    /// Board outline and routed cut geometry.
    EdgeCuts,
    /// Fabrication documentation.
    Fabrication,
    /// Component courtyard geometry.
    Courtyard,
    /// Named source-specific layer.
    Custom(String),
}

/// Retained footprint artwork primitive in land-pattern-local coordinates.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub enum LandPatternGraphicPrimitive {
    /// Straight documentation or process line.
    Line {
        /// Local start point.
        #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::point"))]
        start: Point2,
        /// Local end point.
        #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::point"))]
        end: Point2,
    },
    /// Circle defined by center and exact radius.
    Circle {
        /// Local center.
        #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::point"))]
        center: Point2,
        /// Exact radius.
        radius: Real,
    },
    /// Closed polygon, optionally filled by the target layer process.
    Polygon {
        /// Local vertices.
        #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::points"))]
        vertices: Vec<Point2>,
        /// Whether the polygon interior is authored as filled.
        filled: bool,
    },
    /// Retained documentation text.
    Text {
        /// Authored string.
        text: String,
        /// Local anchor.
        #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::point"))]
        position: Point2,
        /// Exact text height.
        height: Real,
        /// Counter-clockwise local rotation in degrees.
        rotation_degrees: Real,
    },
}

/// One source-addressable footprint artwork item.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct LandPatternGraphic {
    /// Stable identity within the land pattern.
    pub id: LandPatternGraphicId,
    /// Manufacturing/documentation layer role.
    pub layer: LayerRole,
    /// Optional exact stroke width; filled polygons need no stroke.
    pub stroke_width: Option<Real>,
    /// Retained local primitive.
    pub primitive: LandPatternGraphicPrimitive,
}

/// Physical kind of one ordered stackup layer.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum StackupLayerKind {
    /// Conductive layer with a stable routing index.
    Conductor(
        #[cfg_attr(
            feature = "interchange",
            serde(with = "crate::interchange::trace_layer")
        )]
        TraceLayer,
    ),
    /// Nonconductive laminate, prepreg, or core.
    Dielectric,
    /// Solder-mask or coverlay dielectric.
    SolderMask,
    /// Other retained material layer.
    Custom(String),
}

/// One exact-thickness layer in an ordered board stackup.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct StackupLayer {
    /// Human-readable stable layer name.
    pub name: String,
    /// Physical layer kind.
    pub kind: StackupLayerKind,
    /// Exact thickness in the design's authored length unit.
    pub thickness: Real,
    /// Material-library handle, such as `hyperphysics:material/fr4`.
    pub material: Option<String>,
}

/// Ordered physical PCB stackup.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PcbStackup {
    /// Layers ordered from the front surface to the back surface.
    pub layers: Vec<StackupLayer>,
}

impl PcbStackup {
    /// Creates a one-copper-layer stackup for simple prototypes.
    pub fn single_layer(thickness: Real, material: Option<String>) -> Self {
        Self {
            layers: vec![StackupLayer {
                name: "F.Cu".into(),
                kind: StackupLayerKind::Conductor(TraceLayer(0)),
                thickness,
                material,
            }],
        }
    }

    /// Creates a conventional two-copper-layer stackup with one dielectric core.
    pub fn two_layer(
        copper_thickness: Real,
        dielectric_thickness: Real,
        copper_material: Option<String>,
        dielectric_material: Option<String>,
    ) -> Self {
        Self {
            layers: vec![
                StackupLayer {
                    name: "F.Cu".into(),
                    kind: StackupLayerKind::Conductor(TraceLayer(0)),
                    thickness: copper_thickness.clone(),
                    material: copper_material.clone(),
                },
                StackupLayer {
                    name: "dielectric 1".into(),
                    kind: StackupLayerKind::Dielectric,
                    thickness: dielectric_thickness,
                    material: dielectric_material,
                },
                StackupLayer {
                    name: "B.Cu".into(),
                    kind: StackupLayerKind::Conductor(TraceLayer(1)),
                    thickness: copper_thickness,
                    material: copper_material,
                },
            ],
        }
    }
}

/// One exact segment of a closed board contour.
#[derive(Clone, Debug, PartialEq)]
pub enum BoardContourSegment {
    /// Straight edge owned by hyperpath.
    Line(LinePathSegment),
    /// Exact directed circular edge owned and certified by hyperpath.
    CircularArc(ExplicitCircularArc),
    /// Exact polynomial cubic Bezier edge owned by hyperpath.
    CubicBezier(CubicBezier),
}

impl From<LinePathSegment> for BoardContourSegment {
    fn from(segment: LinePathSegment) -> Self {
        Self::Line(segment)
    }
}

impl From<ExplicitCircularArc> for BoardContourSegment {
    fn from(arc: ExplicitCircularArc) -> Self {
        Self::CircularArc(arc)
    }
}

impl From<CubicBezier> for BoardContourSegment {
    fn from(bezier: CubicBezier) -> Self {
        Self::CubicBezier(bezier)
    }
}

impl BoardContourSegment {
    /// Exact start point.
    pub fn start(&self) -> &Point2 {
        match self {
            Self::Line(segment) => segment.start(),
            Self::CircularArc(arc) => arc.start(),
            Self::CubicBezier(bezier) => bezier.start(),
        }
    }

    /// Exact end point.
    pub fn end(&self) -> &Point2 {
        match self {
            Self::Line(segment) => segment.end(),
            Self::CircularArc(arc) => arc.end(),
            Self::CubicBezier(bezier) => bezier.end(),
        }
    }
}

/// Closed exact line/arc/Bezier substrate contour.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BoardContour {
    segments: Vec<BoardContourSegment>,
}

impl BoardContour {
    /// Retains an authored ordered contour without silently repairing topology.
    pub const fn from_segments(segments: Vec<BoardContourSegment>) -> Self {
        Self { segments }
    }

    /// Promotes polygon vertices into exact closing line segments.
    pub fn polygon(vertices: Vec<Point2>) -> Self {
        if vertices.is_empty() {
            return Self::default();
        }
        let mut segments = Vec::with_capacity(vertices.len());
        for index in 0..vertices.len() {
            segments.push(BoardContourSegment::Line(LinePathSegment::new(
                vertices[index].clone(),
                vertices[(index + 1) % vertices.len()].clone(),
            )));
        }
        Self { segments }
    }

    /// Ordered exact contour segments.
    pub fn segments(&self) -> &[BoardContourSegment] {
        &self.segments
    }

    /// True when at least one edge is curved.
    pub fn is_curved(&self) -> bool {
        self.segments
            .iter()
            .any(|segment| !matches!(segment, BoardContourSegment::Line(_)))
    }

    /// Recovers polygon vertices only when every retained edge is straight.
    pub fn linear_vertices(&self) -> Option<Vec<Point2>> {
        self.segments
            .iter()
            .map(|segment| match segment {
                BoardContourSegment::Line(line) => Some(line.start().clone()),
                BoardContourSegment::CircularArc(_) | BoardContourSegment::CubicBezier(_) => None,
            })
            .collect()
    }

    /// Returns whether segment topology is nondegenerate, connected, and closed.
    pub fn is_valid(&self) -> bool {
        if self.segments.len() < 2
            || (!self.is_curved() && self.segments.len() < 3)
            || self
                .segments
                .iter()
                .any(|segment| segment.start() == segment.end())
        {
            return false;
        }
        self.segments
            .iter()
            .zip(self.segments.iter().cycle().skip(1))
            .all(|(left, right)| left.end() == right.start())
    }
}

impl From<Vec<Point2>> for BoardContour {
    fn from(vertices: Vec<Point2>) -> Self {
        Self::polygon(vertices)
    }
}

/// Failure to construct or query the exact mixed-curve board boundary.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BoardBoundaryGeometryError {
    context: String,
}

impl BoardBoundaryGeometryError {
    fn new(context: impl Into<String>) -> Self {
        Self {
            context: context.into(),
        }
    }
}

impl Display for BoardBoundaryGeometryError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(&self.context)
    }
}

impl std::error::Error for BoardBoundaryGeometryError {}

/// Canonical exact query carrier for a board exterior and its cutouts.
///
/// Hypercircuit owns the PCB meaning of the loops while Hypercurve owns their
/// exact line/arc/Bezier topology and predicates. Geometry materialization,
/// placement, routing, stitching, and DRC handoffs should all derive from this
/// carrier instead of reconstructing polygon-only interpretations.
#[derive(Clone)]
pub struct BoardBoundaryGeometry {
    region: CurveRegion2,
    contour_paths: Vec<CurvePath2>,
    exterior_bounds: CurveAabb2,
    insets: BoardBoundaryInsetCache,
}

type BoardBoundaryInset = (Real, Classification<BoardBoundaryGeometry>);
type BoardBoundaryInsetCache = Rc<RefCell<Vec<BoardBoundaryInset>>>;

impl std::fmt::Debug for BoardBoundaryGeometry {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("BoardBoundaryGeometry")
            .field("contour_count", &self.contour_paths.len())
            .field("exterior_bounds", &self.exterior_bounds)
            .finish_non_exhaustive()
    }
}

impl BoardBoundaryGeometry {
    /// Exact filled exterior-minus-cutouts region.
    pub const fn region(&self) -> &CurveRegion2 {
        &self.region
    }

    /// Authored exterior path followed by authored cutout paths.
    pub fn contour_paths(&self) -> &[CurvePath2] {
        &self.contour_paths
    }

    /// Conservative exact bounds of the authored exterior, including curve extrema.
    pub fn exterior_bounds(&self) -> (Point2, Point2) {
        (
            lattice_point(self.exterior_bounds.min()),
            lattice_point(self.exterior_bounds.max()),
        )
    }

    /// Classifies a board-space point against the exact filled substrate.
    pub fn classify_point(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> Result<Classification<RegionPointLocation>, BoardBoundaryGeometryError> {
        self.region
            .classify_point(&curve_point(point), policy)
            .map_err(|error| {
                BoardBoundaryGeometryError::new(format!(
                    "board point classification failed: {error:?}"
                ))
            })
    }

    /// Contracts the filled substrate by an exact non-negative clearance.
    ///
    /// Polynomial/rational offsets that Hypercurve cannot represent remain an
    /// explicit `Uncertain(Unsupported)` classification.
    pub fn inset(
        &self,
        clearance: Real,
        policy: &CurvePolicy,
    ) -> Result<Classification<Self>, BoardBoundaryGeometryError> {
        if let Some((_, cached)) = self
            .insets
            .borrow()
            .iter()
            .find(|(candidate, _)| candidate == &clearance)
        {
            return Ok(cached.clone());
        }
        let result = match self
            .region
            .offset(-clearance.clone(), policy)
            .map_err(|error| {
                BoardBoundaryGeometryError::new(format!("board inset failed: {error:?}"))
            })? {
            Classification::Decided(region) => {
                let contour_paths = match region.materialized_boundary_paths().map_err(|error| {
                    BoardBoundaryGeometryError::new(format!(
                        "board inset boundary extraction failed: {error:?}"
                    ))
                })? {
                    Classification::Decided(paths) => paths,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let exterior_bounds = contour_paths
                    .first()
                    .ok_or_else(|| {
                        BoardBoundaryGeometryError::new(
                            "board inset collapsed to an empty substrate",
                        )
                    })?
                    .bounds()
                    .map_err(|error| {
                        BoardBoundaryGeometryError::new(format!(
                            "board inset bounds failed: {error:?}"
                        ))
                    })?
                    .clone();
                Ok(Classification::Decided(Self {
                    region,
                    contour_paths,
                    exterior_bounds,
                    insets: Rc::new(RefCell::new(Vec::new())),
                }))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }?;
        self.insets.borrow_mut().push((clearance, result.clone()));
        Ok(result)
    }

    /// Convenience classification for a closed-clearance disc centered at `point`.
    pub fn contains_disc(
        &self,
        point: &Point2,
        clearance: Real,
        policy: &CurvePolicy,
    ) -> Result<Classification<bool>, BoardBoundaryGeometryError> {
        if let Some(decision) =
            self.contains_disc_against_native_primitives(point, &clearance, policy)?
        {
            return Ok(decision);
        }
        let inset = match self.inset(clearance, policy)? {
            Classification::Decided(inset) => inset,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(inset
            .classify_point(point, policy)?
            .map(|location| location != RegionPointLocation::Outside))
    }

    fn contains_disc_against_native_primitives(
        &self,
        point: &Point2,
        clearance: &Real,
        policy: &CurvePolicy,
    ) -> Result<Option<Classification<bool>>, BoardBoundaryGeometryError> {
        match self.classify_point(point, policy)? {
            Classification::Decided(RegionPointLocation::Inside) => {}
            Classification::Decided(
                RegionPointLocation::Outside | RegionPointLocation::Boundary,
            ) => return Ok(Some(Classification::Decided(false))),
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        }
        let point = curve_point(point);
        let required_squared = clearance.clone() * clearance.clone();
        for contour in &self.contour_paths {
            for curve in contour.curves() {
                let distance_squared = match curve.geometry() {
                    CurveGeometry2::Line(line) => {
                        point_line_segment_distance_squared(&point, line, policy)?
                    }
                    CurveGeometry2::CircularArc(arc) => {
                        point_arc_distance_squared(&point, arc, policy)?
                    }
                    _ => return Ok(None),
                };
                let distance_squared = match distance_squared {
                    Classification::Decided(distance) => distance,
                    Classification::Uncertain(reason) => {
                        return Ok(Some(Classification::Uncertain(reason)));
                    }
                };
                match distance_squared.partial_cmp(&required_squared) {
                    Some(Ordering::Less) => {
                        return Ok(Some(Classification::Decided(false)));
                    }
                    Some(Ordering::Equal | Ordering::Greater) => {}
                    None => {
                        return Ok(Some(Classification::Uncertain(UncertaintyReason::Ordering)));
                    }
                }
            }
        }
        Ok(Some(Classification::Decided(true)))
    }

    /// Certifies that a straight centerline and its closed clearance radius
    /// remain inside the substrate.
    pub fn contains_segment(
        &self,
        start: &Point2,
        end: &Point2,
        clearance: Real,
        policy: &CurvePolicy,
    ) -> Result<Classification<bool>, BoardBoundaryGeometryError> {
        if let Some(decision) =
            self.contains_segment_against_native_primitives(start, end, &clearance, policy)?
        {
            return Ok(decision);
        }
        let inset = match self.inset(clearance, policy)? {
            Classification::Decided(inset) => inset,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        for point in [start, end] {
            match inset.classify_point(point, policy)? {
                Classification::Decided(RegionPointLocation::Outside) => {
                    return Ok(Classification::Decided(false));
                }
                Classification::Decided(
                    RegionPointLocation::Boundary | RegionPointLocation::Inside,
                ) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let centerline = CurvePath2::try_new(vec![
            LineSeg2::try_new(curve_point(start), curve_point(end))
                .map(Curve2::from)
                .map_err(|error| {
                    BoardBoundaryGeometryError::new(format!(
                        "board segment construction failed: {error:?}"
                    ))
                })?,
        ])
        .map_err(|error| {
            BoardBoundaryGeometryError::new(format!(
                "board segment path construction failed: {error:?}"
            ))
        })?;
        for contour in &inset.contour_paths {
            let result = centerline
                .retain_intersection(contour, policy)
                .and_then(|retained| retained.result())
                .map_err(|error| {
                    BoardBoundaryGeometryError::new(format!(
                        "board segment intersection failed: {error:?}"
                    ))
                })?;
            if !result.is_complete() {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
            if !result.is_disjoint() {
                return Ok(Classification::Decided(false));
            }
        }
        Ok(Classification::Decided(true))
    }

    fn contains_segment_against_native_primitives(
        &self,
        start: &Point2,
        end: &Point2,
        clearance: &Real,
        policy: &CurvePolicy,
    ) -> Result<Option<Classification<bool>>, BoardBoundaryGeometryError> {
        if self.contour_paths.iter().any(|contour| {
            contour.curves().iter().any(|curve| {
                !matches!(
                    curve.geometry(),
                    CurveGeometry2::Line(_) | CurveGeometry2::CircularArc(_)
                )
            })
        }) {
            return Ok(None);
        }
        for point in [start, end] {
            match self.contains_disc_against_native_primitives(point, clearance, policy)? {
                Some(Classification::Decided(true)) => {}
                Some(Classification::Decided(false)) => {
                    return Ok(Some(Classification::Decided(false)));
                }
                Some(Classification::Uncertain(reason)) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
                None => unreachable!("native primitive inventory was checked"),
            }
        }
        let line = LineSeg2::try_new(curve_point(start), curve_point(end)).map_err(|error| {
            BoardBoundaryGeometryError::new(format!("board segment construction failed: {error:?}"))
        })?;
        let centerline =
            CurvePath2::try_new(vec![Curve2::from(line.clone())]).map_err(|error| {
                BoardBoundaryGeometryError::new(format!(
                    "board segment path construction failed: {error:?}"
                ))
            })?;
        let required_squared = clearance.clone() * clearance.clone();
        for contour in &self.contour_paths {
            let result = centerline
                .retain_intersection(contour, policy)
                .and_then(|retained| retained.result())
                .map_err(|error| {
                    BoardBoundaryGeometryError::new(format!(
                        "board segment intersection failed: {error:?}"
                    ))
                })?;
            if !result.is_complete() {
                return Ok(Some(Classification::Uncertain(
                    UncertaintyReason::Predicate,
                )));
            }
            if !result.is_disjoint() {
                return Ok(Some(Classification::Decided(false)));
            }
            for curve in contour.curves() {
                let distance_squared = match curve.geometry() {
                    CurveGeometry2::Line(boundary) => {
                        line_line_segment_distance_squared(&line, boundary, policy)?
                    }
                    CurveGeometry2::CircularArc(boundary) => {
                        line_arc_distance_squared(&line, boundary, policy)?
                    }
                    _ => unreachable!("native primitive inventory was checked"),
                };
                let distance_squared = match distance_squared {
                    Classification::Decided(distance) => distance,
                    Classification::Uncertain(reason) => {
                        return Ok(Some(Classification::Uncertain(reason)));
                    }
                };
                match distance_squared.partial_cmp(&required_squared) {
                    Some(Ordering::Less) => {
                        return Ok(Some(Classification::Decided(false)));
                    }
                    Some(Ordering::Equal | Ordering::Greater) => {}
                    None => {
                        return Ok(Some(Classification::Uncertain(UncertaintyReason::Ordering)));
                    }
                }
            }
        }
        Ok(Some(Classification::Decided(true)))
    }

    /// Certifies that an axis-aligned closed rectangle lies strictly inside the
    /// substrate and neither crosses nor encloses a cutout.
    pub fn contains_axis_aligned_box(
        &self,
        min: &Point2,
        max: &Point2,
        policy: &CurvePolicy,
    ) -> Result<Classification<bool>, BoardBoundaryGeometryError> {
        let corners = [
            Point2::new(min.x.clone(), min.y.clone()),
            Point2::new(max.x.clone(), min.y.clone()),
            Point2::new(max.x.clone(), max.y.clone()),
            Point2::new(min.x.clone(), max.y.clone()),
        ];
        for corner in &corners {
            match self.classify_point(corner, policy)? {
                Classification::Decided(RegionPointLocation::Inside) => {}
                Classification::Decided(
                    RegionPointLocation::Outside | RegionPointLocation::Boundary,
                ) => return Ok(Classification::Decided(false)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let rectangle = CurvePath2::try_new(
            (0..corners.len())
                .map(|index| {
                    LineSeg2::try_new(
                        curve_point(&corners[index]),
                        curve_point(&corners[(index + 1) % corners.len()]),
                    )
                    .map(Curve2::from)
                })
                .collect::<Result<Vec<_>, _>>()
                .map_err(|error| {
                    BoardBoundaryGeometryError::new(format!(
                        "placement envelope construction failed: {error:?}"
                    ))
                })?,
        )
        .map_err(|error| {
            BoardBoundaryGeometryError::new(format!(
                "placement envelope path construction failed: {error:?}"
            ))
        })?;
        for contour in &self.contour_paths {
            let result = rectangle
                .retain_intersection(contour, policy)
                .and_then(|retained| retained.result())
                .map_err(|error| {
                    BoardBoundaryGeometryError::new(format!(
                        "placement envelope intersection failed: {error:?}"
                    ))
                })?;
            if !result.is_complete() {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
            if !result.is_disjoint() {
                return Ok(Classification::Decided(false));
            }
        }
        for cutout in self.contour_paths.iter().skip(1) {
            match curve_point_in_closed_box(cutout.start(), min, max) {
                Some(true) => return Ok(Classification::Decided(false)),
                Some(false) => {}
                None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
            }
        }
        Ok(Classification::Decided(true))
    }
}

fn curve_point_in_closed_box(point: &CurvePoint2, min: &Point2, max: &Point2) -> Option<bool> {
    Some(
        point.x().partial_cmp(&min.x)? != Ordering::Less
            && point.x().partial_cmp(&max.x)? != Ordering::Greater
            && point.y().partial_cmp(&min.y)? != Ordering::Less
            && point.y().partial_cmp(&max.y)? != Ordering::Greater,
    )
}

fn point_line_segment_distance_squared(
    point: &CurvePoint2,
    line: &LineSeg2,
    _policy: &CurvePolicy,
) -> Result<Classification<Real>, BoardBoundaryGeometryError> {
    let dx = line.end().x() - line.start().x();
    let dy = line.end().y() - line.start().y();
    let length_squared = dx.clone() * dx.clone() + dy.clone() * dy.clone();
    let px = point.x() - line.start().x();
    let py = point.y() - line.start().y();
    let projection = px.clone() * dx.clone() + py.clone() * dy.clone();
    let projection_order = projection.partial_cmp(&Real::zero());
    let length_order = projection.partial_cmp(&length_squared);
    match (projection_order, length_order) {
        (Some(Ordering::Less | Ordering::Equal), _) => {
            Ok(Classification::Decided(px.clone() * px + py.clone() * py))
        }
        (_, Some(Ordering::Greater | Ordering::Equal)) => {
            Ok(Classification::Decided(point.distance_squared(line.end())))
        }
        (Some(Ordering::Greater), Some(Ordering::Less)) => {
            let cross = px * dy - py * dx;
            let distance = (cross.clone() * cross / length_squared).map_err(|error| {
                BoardBoundaryGeometryError::new(format!("point-to-line distance failed: {error:?}"))
            })?;
            Ok(Classification::Decided(distance))
        }
        _ => Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
    }
}

fn point_arc_distance_squared(
    point: &CurvePoint2,
    arc: &CircularArc2,
    policy: &CurvePolicy,
) -> Result<Classification<Real>, BoardBoundaryGeometryError> {
    let radial = point.distance_squared(arc.center());
    if radial.partial_cmp(&Real::zero()) == Some(Ordering::Equal) {
        return Ok(Classification::Decided(arc.radius_squared_ref().clone()));
    }
    let radial_length = radial.sqrt().map_err(|error| {
        BoardBoundaryGeometryError::new(format!("point-to-arc radial root failed: {error:?}"))
    })?;
    let radius = arc.radius_squared_ref().clone().sqrt().map_err(|error| {
        BoardBoundaryGeometryError::new(format!("arc radius root failed: {error:?}"))
    })?;
    let scale = (radius.clone() / &radial_length).map_err(|error| {
        BoardBoundaryGeometryError::new(format!("point-to-arc radial scale failed: {error:?}"))
    })?;
    let candidate = CurvePoint2::new(
        arc.center().x() + (point.x() - arc.center().x()) * &scale,
        arc.center().y() + (point.y() - arc.center().y()) * scale,
    );
    match arc.contains_sweep_point(&candidate, policy) {
        Classification::Decided(true) => {
            let delta = radial_length - radius;
            Ok(Classification::Decided(delta.clone() * delta))
        }
        Classification::Decided(false) => classified_minimum(
            point.distance_squared(arc.start()),
            point.distance_squared(arc.end()),
        ),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn line_line_segment_distance_squared(
    first: &LineSeg2,
    second: &LineSeg2,
    policy: &CurvePolicy,
) -> Result<Classification<Real>, BoardBoundaryGeometryError> {
    let distances = [
        point_line_segment_distance_squared(first.start(), second, policy)?,
        point_line_segment_distance_squared(first.end(), second, policy)?,
        point_line_segment_distance_squared(second.start(), first, policy)?,
        point_line_segment_distance_squared(second.end(), first, policy)?,
    ];
    classified_minimum_many(distances)
}

fn line_arc_distance_squared(
    line: &LineSeg2,
    arc: &CircularArc2,
    policy: &CurvePolicy,
) -> Result<Classification<Real>, BoardBoundaryGeometryError> {
    let center_projection = closest_point_on_line_segment(arc.center(), line)?;
    match center_projection {
        Classification::Decided(point) => classified_minimum_many([
            point_arc_distance_squared(&point, arc, policy)?,
            point_line_segment_distance_squared(arc.start(), line, policy)?,
            point_line_segment_distance_squared(arc.end(), line, policy)?,
        ]),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn closest_point_on_line_segment(
    point: &CurvePoint2,
    line: &LineSeg2,
) -> Result<Classification<CurvePoint2>, BoardBoundaryGeometryError> {
    let dx = line.end().x() - line.start().x();
    let dy = line.end().y() - line.start().y();
    let length_squared = dx.clone() * dx.clone() + dy.clone() * dy.clone();
    let projection = (point.x() - line.start().x()) * dx + (point.y() - line.start().y()) * dy;
    match (
        projection.partial_cmp(&Real::zero()),
        projection.partial_cmp(&length_squared),
    ) {
        (Some(Ordering::Less | Ordering::Equal), _) => {
            Ok(Classification::Decided(line.start().clone()))
        }
        (_, Some(Ordering::Greater | Ordering::Equal)) => {
            Ok(Classification::Decided(line.end().clone()))
        }
        (Some(Ordering::Greater), Some(Ordering::Less)) => {
            let parameter = (projection / length_squared).map_err(|error| {
                BoardBoundaryGeometryError::new(format!(
                    "line projection parameter failed: {error:?}"
                ))
            })?;
            Ok(Classification::Decided(line.point_at(parameter)))
        }
        _ => Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
    }
}

fn classified_minimum_many<const N: usize>(
    values: [Classification<Real>; N],
) -> Result<Classification<Real>, BoardBoundaryGeometryError> {
    let mut minimum: Option<Real> = None;
    for value in values {
        let Classification::Decided(value) = value else {
            return Ok(value);
        };
        minimum = Some(match minimum {
            Some(current) => match current.partial_cmp(&value) {
                Some(Ordering::Less | Ordering::Equal) => current,
                Some(Ordering::Greater) => value,
                None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
            },
            None => value,
        });
    }
    Ok(Classification::Decided(
        minimum.expect("distance candidate array is nonempty"),
    ))
}

fn classified_minimum(
    first: Real,
    second: Real,
) -> Result<Classification<Real>, BoardBoundaryGeometryError> {
    Ok(match first.partial_cmp(&second) {
        Some(Ordering::Less | Ordering::Equal) => Classification::Decided(first),
        Some(Ordering::Greater) => Classification::Decided(second),
        None => Classification::Uncertain(UncertaintyReason::Ordering),
    })
}

fn board_contour_curve_path(
    contour: &BoardContour,
    source: &str,
) -> Result<CurvePath2, BoardBoundaryGeometryError> {
    let curves = contour
        .segments()
        .iter()
        .map(|segment| match segment {
            BoardContourSegment::Line(line) => {
                LineSeg2::try_new(curve_point(line.start()), curve_point(line.end()))
                    .map(Curve2::from)
            }
            BoardContourSegment::CircularArc(arc) => CircularArc2::try_from_center(
                curve_point(arc.start()),
                curve_point(arc.end()),
                curve_point(arc.center()),
                arc.direction() == hyperpath::ArcDirection::Cw,
            )
            .map(Curve2::from),
            BoardContourSegment::CubicBezier(bezier) => Ok(Curve2::from(CubicBezier2::new(
                curve_point(bezier.start()),
                curve_point(bezier.control0()),
                curve_point(bezier.control1()),
                curve_point(bezier.end()),
            ))),
        })
        .collect::<Result<Vec<_>, _>>()
        .map_err(|error| BoardBoundaryGeometryError::new(format!("{source}: {error:?}")))?;
    CurvePath2::try_new(curves)
        .map_err(|error| BoardBoundaryGeometryError::new(format!("{source}: {error:?}")))
}

fn curve_point(point: &Point2) -> CurvePoint2 {
    CurvePoint2::new(point.x.clone(), point.y.clone())
}

fn lattice_point(point: &CurvePoint2) -> Point2 {
    Point2::new(point.x().clone(), point.y().clone())
}

/// Closed mixed-curve substrate outline plus closed internal cutouts.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct BoardOutline {
    /// Exterior contour in authored order.
    #[cfg_attr(
        feature = "interchange",
        serde(with = "crate::interchange::board_contour")
    )]
    pub exterior: BoardContour,
    /// Interior contours representing substrate cutouts.
    #[cfg_attr(
        feature = "interchange",
        serde(with = "crate::interchange::board_contours")
    )]
    pub cutouts: Vec<BoardContour>,
}

impl BoardOutline {
    /// Creates an axis-aligned rectangular board from the origin to `(width, height)`.
    ///
    /// Structural validation remains authoritative for positive, nondegenerate
    /// dimensions so this constructor does not introduce a second geometry policy.
    pub fn rectangle(width: Real, height: Real) -> Self {
        Self {
            exterior: vec![
                Point2::new(Real::zero(), Real::zero()),
                Point2::new(width.clone(), Real::zero()),
                Point2::new(width.clone(), height.clone()),
                Point2::new(Real::zero(), height),
            ]
            .into(),
            cutouts: Vec::new(),
        }
    }

    /// Builds the canonical exact exterior-minus-cutouts query carrier.
    pub fn boundary_geometry(&self) -> Result<BoardBoundaryGeometry, BoardBoundaryGeometryError> {
        let mut contour_paths = Vec::with_capacity(1 + self.cutouts.len());
        contour_paths.push(board_contour_curve_path(&self.exterior, "board exterior")?);
        for (index, cutout) in self.cutouts.iter().enumerate() {
            contour_paths.push(board_contour_curve_path(
                cutout,
                &format!("board cutout {index}"),
            )?);
        }
        let roles = std::iter::once(CurveRegionLoopRole::Material)
            .chain(std::iter::repeat_n(
                CurveRegionLoopRole::Hole,
                self.cutouts.len(),
            ))
            .collect::<Vec<_>>();
        let fill_rules = vec![FillRule::NonZero; contour_paths.len()];
        let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
            &contour_paths,
            &roles,
            &fill_rules,
        )
        .map_err(|error| BoardBoundaryGeometryError::new(format!("board outline: {error:?}")))?;
        let exterior_bounds = contour_paths[0]
            .bounds()
            .map_err(|error| {
                BoardBoundaryGeometryError::new(format!("board exterior bounds: {error:?}"))
            })?
            .clone();
        Ok(BoardBoundaryGeometry {
            region,
            contour_paths,
            exterior_bounds,
            insets: Rc::new(RefCell::new(Vec::new())),
        })
    }
}

/// Board side used for component placement and generated documentation.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BoardSide {
    /// Front/top surface.
    Front,
    /// Back/bottom surface.
    Back,
}

/// Retained pad land shape in footprint-local coordinates.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub enum PadShape {
    /// Circular land with exact diameter.
    Circle { diameter: Real },
    /// Axis-aligned rectangular land before placement rotation.
    Rectangle { width: Real, height: Real },
    /// Rounded rectangle with an exact corner radius.
    RoundedRectangle {
        width: Real,
        height: Real,
        corner_radius: Real,
    },
    /// Capsule/obround land.
    Obround { width: Real, height: Real },
    /// Arbitrary retained polygon relative to the pad center.
    Polygon {
        #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::points"))]
        vertices: Vec<Point2>,
    },
}

/// Fabrication intent for a drilled or routed hole.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Plating {
    /// Hole wall is plated and electrically conductive.
    Plated,
    /// Hole is explicitly non-plated.
    NonPlated,
    /// Source did not retain plating intent.
    Unspecified,
}

/// Solder-mask treatment for one via on one board surface.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub enum ViaMaskDisposition {
    /// The source did not retain whether this surface is opened or tented.
    Unspecified,
    /// Solder mask covers the via land on this surface.
    Tented,
    /// Emit a circular opening around the copper land with this exact radial margin.
    Open { margin: Real },
}

/// Independently retained front/back solder-mask intent for a via.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct ViaMaskIntent {
    /// Front surface treatment when the via reaches the front copper layer.
    pub front: ViaMaskDisposition,
    /// Back surface treatment when the via reaches the back copper layer.
    pub back: ViaMaskDisposition,
}

impl Default for ViaMaskIntent {
    fn default() -> Self {
        Self {
            front: ViaMaskDisposition::Unspecified,
            back: ViaMaskDisposition::Unspecified,
        }
    }
}

impl ViaMaskIntent {
    /// Explicitly tents the via on both board surfaces.
    pub fn tented() -> Self {
        Self {
            front: ViaMaskDisposition::Tented,
            back: ViaMaskDisposition::Tented,
        }
    }

    /// Opens the via on both surfaces with one exact radial margin.
    pub fn open(margin: Real) -> Self {
        Self {
            front: ViaMaskDisposition::Open {
                margin: margin.clone(),
            },
            back: ViaMaskDisposition::Open { margin },
        }
    }
}

/// Drill or routed-slot shape attached to a pad stack.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub enum DrillShape {
    /// Round drill hit.
    Round { diameter: Real },
    /// Routed slot described by a local centerline and cutter width.
    Slot {
        #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::point"))]
        start: Point2,
        #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::point"))]
        end: Point2,
        width: Real,
    },
}

/// One pad in a reusable land pattern.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct LandPatternPad {
    /// Stable pad name/number.
    pub id: PadId,
    /// Footprint-local pad center.
    #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::point"))]
    pub center: Point2,
    /// Counter-clockwise rotation around the pad center in exact degrees.
    pub rotation_degrees: Real,
    /// Copper layers carrying a land for this pad.
    #[cfg_attr(
        feature = "interchange",
        serde(with = "crate::interchange::trace_layers")
    )]
    pub copper_layers: Vec<TraceLayer>,
    /// Retained copper shape.
    pub shape: PadShape,
    /// Optional drill or routed slot.
    pub drill: Option<DrillShape>,
    /// Retained plating intent for the drill.
    pub plating: Plating,
    /// Exact solder-mask expansion; `None` delegates to downstream policy.
    pub solder_mask_margin: Option<Real>,
    /// Exact paste expansion/reduction; `None` delegates to downstream policy.
    pub paste_margin: Option<Real>,
}

/// Mapping from an electrical pin to one or more physical pads.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PadPinMap {
    /// Logical package pin.
    pub pin: PinRef,
    /// Physical pad implementing that pin.
    pub pad: PadId,
}

/// Retained external package-model container.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Pcb3dModelFormat {
    /// Legacy handle whose container was not declared.
    Unspecified,
    /// Wavefront OBJ triangle/polygon mesh.
    WavefrontObj,
    /// ISO 10303 STEP B-rep or assembly.
    Step,
    /// VRML/WRL scene.
    Vrml,
    /// glTF 2.0 scene.
    Gltf,
}

pub(crate) fn pcb_3d_model_format_from_uri(uri: &str) -> Pcb3dModelFormat {
    let lower = uri.to_ascii_lowercase();
    if lower.ends_with(".obj") {
        Pcb3dModelFormat::WavefrontObj
    } else if lower.ends_with(".step") || lower.ends_with(".stp") {
        Pcb3dModelFormat::Step
    } else if lower.ends_with(".wrl") || lower.ends_with(".vrml") {
        Pcb3dModelFormat::Vrml
    } else if lower.ends_with(".gltf") || lower.ends_with(".glb") {
        Pcb3dModelFormat::Gltf
    } else {
        Pcb3dModelFormat::Unspecified
    }
}

/// Exact footprint-local transform applied after loading an external model.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct Pcb3dModelTransform {
    /// Translation along footprint-local X.
    pub offset_x: Real,
    /// Translation along footprint-local Y.
    pub offset_y: Real,
    /// Translation above the local mounting plane.
    pub offset_z: Real,
    /// Rotation around local X in degrees.
    pub rotate_x_degrees: Real,
    /// Rotation around local Y in degrees.
    pub rotate_y_degrees: Real,
    /// Rotation around local Z in degrees.
    pub rotate_z_degrees: Real,
    /// Nonzero local X scale.
    pub scale_x: Real,
    /// Nonzero local Y scale.
    pub scale_y: Real,
    /// Nonzero local Z scale.
    pub scale_z: Real,
}

impl Default for Pcb3dModelTransform {
    fn default() -> Self {
        Self {
            offset_x: Real::zero(),
            offset_y: Real::zero(),
            offset_z: Real::zero(),
            rotate_x_degrees: Real::zero(),
            rotate_y_degrees: Real::zero(),
            rotate_z_degrees: Real::zero(),
            scale_x: Real::one(),
            scale_y: Real::one(),
            scale_z: Real::one(),
        }
    }
}

/// Resolver-addressable external package model with explicit format/transform.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct Pcb3dModelReference {
    /// Stable URI or package-store handle; HyperCircuit does not assume a filesystem.
    pub uri: String,
    /// Declared source container, never guessed from a filename.
    pub format: Pcb3dModelFormat,
    /// Exact transform from model coordinates into footprint-local coordinates.
    pub transform: Pcb3dModelTransform,
}

/// Review envelope for a package body.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct LandPatternBody {
    /// Footprint-local closed body envelope used for native 3D review.
    #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::points"))]
    pub outline: Vec<Point2>,
    /// Exact body height above its mounting standoff.
    pub height: Real,
    /// Exact standoff from the board surface to the body envelope.
    pub standoff: Real,
}

/// Reusable PCB land pattern independent of component placement and nets.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct LandPattern {
    /// Stable land-pattern id or library handle.
    pub id: LandPatternId,
    /// Physical pads.
    pub pads: Vec<LandPatternPad>,
    /// Logical pin-to-pad mapping. Multiple pads may intentionally map to one pin.
    pub pin_map: Vec<PadPinMap>,
    /// Silkscreen, fabrication, courtyard, mask, paste, or custom artwork.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub graphics: Vec<LandPatternGraphic>,
    /// Optional native fallback envelope for review and DRC.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub body: Option<LandPatternBody>,
    /// Zero or more resolver-addressable package models, independent of the fallback envelope.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub models: Vec<Pcb3dModelReference>,
}

/// Placement of one logical circuit instance using a retained land pattern.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct PcbPlacement {
    /// Logical circuit instance being placed.
    pub instance: CircuitInstanceId,
    /// Reusable land pattern used by the instance.
    pub land_pattern: LandPatternId,
    /// Board-space origin of the footprint.
    #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::point"))]
    pub position: Point2,
    /// Counter-clockwise rotation in exact degrees.
    pub rotation_degrees: Real,
    /// Physical board side.
    pub side: BoardSide,
}

impl PcbPlacement {
    /// Transforms a footprint-local point into exact board coordinates.
    ///
    /// Back-side placement mirrors local X before applying the authored
    /// counter-clockwise rotation, matching pad materialization.
    pub fn transform_point(&self, point: &Point2) -> Point2 {
        let local_x = match self.side {
            BoardSide::Front => point.x.clone(),
            BoardSide::Back => -point.x.clone(),
        };
        let radians = self.rotation_degrees.clone().to_radians();
        let sin = radians.clone().sin();
        let cos = radians.cos();
        Point2::new(
            local_x.clone() * cos.clone() - point.y.clone() * sin.clone() + self.position.x.clone(),
            local_x * sin + point.y.clone() * cos + self.position.y.clone(),
        )
    }

    /// Maps a board-space point back into this footprint's exact local coordinates.
    ///
    /// This is the exact inverse of [`Self::transform_point`], including the
    /// back-side mirror. It lets placement-aware clearance and editing logic
    /// consume the same retained transform semantics as materialization.
    pub fn inverse_transform_point(&self, point: &Point2) -> Point2 {
        let dx = point.x.clone() - self.position.x.clone();
        let dy = point.y.clone() - self.position.y.clone();
        let radians = self.rotation_degrees.clone().to_radians();
        let sin = radians.clone().sin();
        let cos = radians.cos();
        let mut x = dx.clone() * cos.clone() + dy.clone() * sin.clone();
        let y = -dx * sin + dy * cos;
        if self.side == BoardSide::Back {
            x = -x;
        }
        Point2::new(x, y)
    }
}

/// Exact relational or regional component-placement intent.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub enum PlacementConstraintKind {
    /// Pin one instance origin to an exact board coordinate.
    Fixed {
        instance: CircuitInstanceId,
        #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::point"))]
        position: Point2,
    },
    /// Place one instance at an exact offset from another resolved instance.
    Relative {
        instance: CircuitInstanceId,
        anchor: CircuitInstanceId,
        #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::point"))]
        offset: Point2,
    },
    /// Require instance origins to share one X coordinate.
    AlignX { instances: Vec<CircuitInstanceId> },
    /// Require instance origins to share one Y coordinate.
    AlignY { instances: Vec<CircuitInstanceId> },
    /// Require an instance origin inside an inclusive axis-aligned region.
    Within {
        instance: CircuitInstanceId,
        #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::point"))]
        min: Point2,
        #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::point"))]
        max: Point2,
    },
    /// Restrict one instance to an authored set of exact rotations.
    AllowedRotations {
        instance: CircuitInstanceId,
        rotations_degrees: Vec<Real>,
    },
    /// Restrict one instance to one or both physical board sides.
    AllowedSides {
        instance: CircuitInstanceId,
        sides: Vec<BoardSide>,
    },
}

/// Stable source-addressable placement constraint.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct PlacementConstraint {
    /// Stable authored constraint identity.
    pub id: PlacementConstraintId,
    /// Retained constraint equation/predicate.
    pub kind: PlacementConstraintKind,
}

/// Unsatisfied result from exact placement resolution.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PlacementResolutionIssue {
    /// Source layout failed structural validation.
    InvalidLayout,
    /// Relative placement dependencies contain a cycle.
    RelativeCycle(CircuitInstanceId),
    /// Resolved origins do not share the required X coordinate.
    MisalignedX(PlacementConstraintId),
    /// Resolved origins do not share the required Y coordinate.
    MisalignedY(PlacementConstraintId),
    /// Resolved origin is outside the required region.
    OutsideRegion(PlacementConstraintId),
    /// Resolved rotation is absent from the allowed set.
    DisallowedRotation(PlacementConstraintId),
    /// Resolved physical side is absent from the allowed set.
    DisallowedSide(PlacementConstraintId),
}

/// Deterministically resolved placements plus exact constraint evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacementResolutionReport {
    /// Placements in authored order with fixed/relative origins applied.
    pub placements: Vec<PcbPlacement>,
    /// Unsatisfied or cyclic constraints.
    pub issues: Vec<PlacementResolutionIssue>,
}

impl PlacementResolutionReport {
    /// True when every retained placement constraint was exactly satisfied.
    pub fn is_satisfied(&self) -> bool {
        self.issues.is_empty()
    }
}

/// Retained routed copper centerline with one exact width and net identity.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct PcbRoute {
    /// Stable route id.
    pub id: RouteId,
    /// Logical net carried by the route.
    pub net: NetId,
    /// Copper routing layer.
    #[cfg_attr(
        feature = "interchange",
        serde(with = "crate::interchange::trace_layer")
    )]
    pub layer: TraceLayer,
    /// Exact finished copper width.
    pub width: Real,
    /// Ordered connected centerline segments.
    #[cfg_attr(
        feature = "interchange",
        serde(with = "crate::interchange::route_segments")
    )]
    pub segments: Vec<PcbRouteSegment>,
}

/// One ordered exact centerline segment in a semantic PCB route.
#[derive(Clone, Debug, PartialEq)]
pub enum PcbRouteSegment {
    /// Straight segment owned by hyperpath.
    Line(LinePathSegment),
    /// Exact directed circular arc owned and certified by hyperpath.
    CircularArc(ExplicitCircularArc),
    /// Exact polynomial cubic Bezier owned by hyperpath.
    CubicBezier(CubicBezier),
}

impl From<LinePathSegment> for PcbRouteSegment {
    fn from(segment: LinePathSegment) -> Self {
        Self::Line(segment)
    }
}

impl From<ExplicitCircularArc> for PcbRouteSegment {
    fn from(arc: ExplicitCircularArc) -> Self {
        Self::CircularArc(arc)
    }
}

impl From<CubicBezier> for PcbRouteSegment {
    fn from(bezier: CubicBezier) -> Self {
        Self::CubicBezier(bezier)
    }
}

impl PcbRouteSegment {
    /// Exact start point.
    pub fn start(&self) -> &Point2 {
        match self {
            Self::Line(segment) => segment.start(),
            Self::CircularArc(arc) => arc.start(),
            Self::CubicBezier(bezier) => bezier.start(),
        }
    }

    /// Exact end point.
    pub fn end(&self) -> &Point2 {
        match self {
            Self::Line(segment) => segment.end(),
            Self::CircularArc(arc) => arc.end(),
            Self::CubicBezier(bezier) => bezier.end(),
        }
    }
}

impl PcbRoute {
    /// Lowers ordered semantic segments to hyperpath's exact route carriers.
    pub fn to_hyperpath(&self, net: RoutingNetId) -> Result<SpecctraRoute, &'static str> {
        let mut traces = Vec::new();
        let mut arcs = Vec::new();
        let mut beziers = Vec::new();
        for segment in &self.segments {
            match segment {
                PcbRouteSegment::Line(segment) => {
                    let swept = SweptLineSegment::new(segment.clone(), self.width.clone())?;
                    traces.push(PcbTrace::new(net, self.layer, swept));
                }
                PcbRouteSegment::CircularArc(arc) => arcs.push(SpecctraRouteArc {
                    net,
                    layer: self.layer,
                    arc: arc.clone(),
                    width: self.width.clone(),
                }),
                PcbRouteSegment::CubicBezier(bezier) => beziers.push(SpecctraRouteBezier {
                    net,
                    layer: self.layer,
                    bezier: bezier.clone(),
                    width: self.width.clone(),
                }),
            }
        }
        Ok(SpecctraRoute::with_curves(
            traces,
            Vec::new(),
            arcs,
            beziers,
        ))
    }
}

/// Retained via connecting a contiguous copper-layer span.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct PcbVia {
    /// Stable via id.
    pub id: ViaId,
    /// Logical net carried by the via.
    pub net: NetId,
    /// Inclusive first connected copper layer.
    #[cfg_attr(
        feature = "interchange",
        serde(with = "crate::interchange::trace_layer")
    )]
    pub start_layer: TraceLayer,
    /// Inclusive last connected copper layer.
    #[cfg_attr(
        feature = "interchange",
        serde(with = "crate::interchange::trace_layer")
    )]
    pub end_layer: TraceLayer,
    /// Board-space via center.
    #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::point"))]
    pub center: Point2,
    /// Exact copper land diameter.
    pub land_diameter: Real,
    /// Exact drill diameter.
    pub drill_diameter: Real,
    /// Retained plating intent.
    pub plating: Plating,
    /// Independent front/back solder-mask opening or tenting intent.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub mask: ViaMaskIntent,
}

impl PcbVia {
    /// Lowers this via to the exact `hyperpath` via carrier.
    pub fn to_hyperpath(&self, net: RoutingNetId) -> Result<PcbViaStack, &'static str> {
        let intent = match self.plating {
            Plating::Plated => ViaDrillIntent::Plated,
            Plating::NonPlated => ViaDrillIntent::NonPlated,
            Plating::Unspecified => ViaDrillIntent::Unspecified,
        };
        PcbViaStack::with_drill_intent(
            net,
            self.start_layer,
            self.end_layer,
            self.center.clone(),
            self.land_diameter.clone(),
            self.drill_diameter.clone(),
            intent,
        )
    }
}

/// Retained fill pattern for a copper zone.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub enum CopperZoneFill {
    /// Continuous copper after clearance and connection realization.
    Solid,
    /// Parallel exact-width copper stripes clipped to the zone boundary.
    Hatched {
        /// Exact copper stripe width.
        line_width: Real,
        /// Exact clear gap between adjacent stripes.
        gap: Real,
        /// Counter-clockwise stripe angle in exact degrees.
        angle_degrees: Real,
    },
}

/// Connection policy between a zone and same-net pads or vias.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub enum CopperZoneConnection {
    /// Same-net pads and vias merge directly into the pour.
    Solid,
    /// Same-net pads and vias receive the ordinary zone clearance.
    Isolated,
    /// Same-net pads and vias connect through equally spaced radial spokes.
    ThermalRelief {
        /// Exact air gap surrounding the land outside spokes.
        air_gap: Real,
        /// Exact spoke width.
        spoke_width: Real,
        /// Number of equally spaced spokes.
        spoke_count: u8,
    },
}

/// Deterministic removal policy for disconnected or undersized pour islands.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct CopperZoneIslandPolicy {
    /// Remove islands that do not intersect retained same-net copper on this layer.
    pub remove_unconnected: bool,
    /// When present, remove only unconnected islands smaller than this exact area.
    pub minimum_area: Option<Real>,
}

impl CopperZoneIslandPolicy {
    /// Retains every island produced by clearance and connection realization.
    pub const fn retain_all() -> Self {
        Self {
            remove_unconnected: false,
            minimum_area: None,
        }
    }

    /// Removes islands without a same-net pad, via, route, or prior zone connection.
    pub const fn remove_unconnected() -> Self {
        Self {
            remove_unconnected: true,
            minimum_area: None,
        }
    }

    /// Limits removal to unconnected islands below an exact filled-area threshold.
    pub fn with_minimum_area(mut self, minimum_area: Real) -> Self {
        self.minimum_area = Some(minimum_area);
        self
    }
}

impl Default for CopperZoneIslandPolicy {
    fn default() -> Self {
        Self::retain_all()
    }
}

/// Deterministic square-grid stitching-via realization policy for one zone.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct CopperZoneStitchingPolicy {
    /// Exact center-to-center pitch in both board axes.
    pub pitch: Real,
    /// Exact copper/keepout inset beyond the generated via land radius.
    pub edge_clearance: Real,
    /// Inclusive first connected copper layer.
    #[cfg_attr(
        feature = "interchange",
        serde(with = "crate::interchange::trace_layer")
    )]
    pub start_layer: TraceLayer,
    /// Inclusive last connected copper layer.
    #[cfg_attr(
        feature = "interchange",
        serde(with = "crate::interchange::trace_layer")
    )]
    pub end_layer: TraceLayer,
    /// Exact generated copper land diameter.
    pub land_diameter: Real,
    /// Exact generated plated drill diameter.
    pub drill_diameter: Real,
    /// Independent surface mask intent copied to every generated via.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub mask: ViaMaskIntent,
    /// Hard deterministic output bound.
    pub maximum_vias: usize,
}

/// Authored copper-pour boundary and retained fill/connection policy.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct CopperZone {
    /// Stable zone id.
    pub id: ZoneId,
    /// Logical net assigned to the zone.
    pub net: NetId,
    /// Copper layer receiving the zone.
    #[cfg_attr(
        feature = "interchange",
        serde(with = "crate::interchange::trace_layer")
    )]
    pub layer: TraceLayer,
    /// Closed authored boundary.
    #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::points"))]
    pub boundary: Vec<Point2>,
    /// Exact clearance from foreign-net copper and lower-priority zones.
    pub clearance: Real,
    /// Fill realization policy.
    pub fill: CopperZoneFill,
    /// Same-net pad/via connection policy.
    pub connection: CopperZoneConnection,
    /// Post-realization island cleanup policy.
    pub islands: CopperZoneIslandPolicy,
    /// Optional deterministic plated stitching-via grid.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub stitching: Option<CopperZoneStitchingPolicy>,
    /// Higher values claim overlapping copper before lower values.
    pub priority: i32,
}

impl CopperZone {
    /// Constructs a solid, solid-connected zone with zero clearance and priority.
    pub fn solid(id: ZoneId, net: NetId, layer: TraceLayer, boundary: Vec<Point2>) -> Self {
        Self {
            id,
            net,
            layer,
            boundary,
            clearance: Real::zero(),
            fill: CopperZoneFill::Solid,
            connection: CopperZoneConnection::Solid,
            islands: CopperZoneIslandPolicy::retain_all(),
            stitching: None,
            priority: 0,
        }
    }
}

/// Which generated features a keepout constrains.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum KeepoutScope {
    /// All physical feature types and layers.
    All,
    /// Copper on selected routing layers.
    Copper(
        #[cfg_attr(
            feature = "interchange",
            serde(with = "crate::interchange::trace_layers")
        )]
        Vec<TraceLayer>,
    ),
    /// Via placement and drilling.
    Vias,
    /// Component placement.
    Components,
}

/// Retained physical-layout keepout boundary and scope.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct PcbKeepout {
    /// Stable keepout id.
    pub id: KeepoutId,
    /// Closed authored boundary.
    #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::points"))]
    pub boundary: Vec<Point2>,
    /// Feature scope.
    pub scope: KeepoutScope,
}

/// Authored routing and verification policy for a named set of nets.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct NetClass {
    /// Stable net-class identity.
    pub id: NetClassId,
    /// Optional base class supplying unspecified policy fields.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub parent: Option<NetClassId>,
    /// Nets assigned to the class.
    pub nets: Vec<NetId>,
    /// Minimum trace width.
    pub min_trace_width: Option<Real>,
    /// Preferred trace width used by routing tools when no stronger choice is supplied.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub preferred_trace_width: Option<Real>,
    /// Minimum same-layer copper clearance to other nets.
    pub min_clearance: Option<Real>,
    /// Preferred via land diameter.
    pub preferred_via_land_diameter: Option<Real>,
    /// Preferred finished via drill diameter.
    pub preferred_via_drill_diameter: Option<Real>,
    /// Preferred named layer-transition construction.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub preferred_via_style: Option<ViaStyleId>,
    /// Maximum routed length.
    pub max_length: Option<Real>,
    /// Maximum number of vias in one routed net.
    pub max_via_count: Option<usize>,
    /// Target single-ended impedance in ohms.
    pub target_impedance_ohms: Option<Real>,
    /// Allowed impedance deviation in ohms.
    pub impedance_tolerance_ohms: Option<Real>,
    /// Whether routing must have a qualified reference plane.
    pub requires_reference_plane: bool,
}

/// Effective net-class policy plus deterministic inheritance provenance.
#[derive(Clone, Debug, PartialEq)]
pub struct ResolvedNetClass {
    /// Most-derived class identity.
    pub id: NetClassId,
    /// Base-to-derived inheritance chain, including `id`.
    pub lineage: Vec<NetClassId>,
    /// Nets assigned directly to the most-derived class.
    pub nets: Vec<NetId>,
    /// Effective minimum trace width.
    pub min_trace_width: Option<Real>,
    /// Effective preferred trace width.
    pub preferred_trace_width: Option<Real>,
    /// Effective minimum same-layer clearance.
    pub min_clearance: Option<Real>,
    /// Effective preferred via land diameter.
    pub preferred_via_land_diameter: Option<Real>,
    /// Effective preferred finished drill diameter.
    pub preferred_via_drill_diameter: Option<Real>,
    /// Effective preferred named via construction.
    pub preferred_via_style: Option<ViaStyleId>,
    /// Effective maximum routed length.
    pub max_length: Option<Real>,
    /// Effective maximum via count.
    pub max_via_count: Option<usize>,
    /// Effective target single-ended impedance.
    pub target_impedance_ohms: Option<Real>,
    /// Effective allowed impedance deviation.
    pub impedance_tolerance_ohms: Option<Real>,
    /// Effective reference-plane requirement.
    pub requires_reference_plane: bool,
}

/// Why a net-class inheritance graph could not resolve.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum NetClassResolutionError {
    /// Two source classes share one stable identity.
    DuplicateClass(NetClassId),
    /// A class references an absent base class.
    UnknownParent {
        /// Derived class.
        class: NetClassId,
        /// Missing base class.
        parent: NetClassId,
    },
    /// A base-class cycle includes this class.
    InheritanceCycle(NetClassId),
}

/// One layer span on which a named via construction may be used.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ViaStyleSpan {
    /// Inclusive first connected copper layer.
    #[cfg_attr(
        feature = "interchange",
        serde(with = "crate::interchange::trace_layer")
    )]
    pub start_layer: TraceLayer,
    /// Inclusive last connected copper layer.
    #[cfg_attr(
        feature = "interchange",
        serde(with = "crate::interchange::trace_layer")
    )]
    pub end_layer: TraceLayer,
}

/// Named manufacturable via construction selected by net classes.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct ViaStyle {
    /// Stable rule identity.
    pub id: ViaStyleId,
    /// Exact copper land diameter.
    pub land_diameter: Real,
    /// Exact finished drill diameter.
    pub drill_diameter: Real,
    /// Retained plating intent.
    pub plating: Plating,
    /// Independent front/back surface-mask policy.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub mask: ViaMaskIntent,
    /// Allowed layer spans. Empty permits any adjacent-layer transition.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub allowed_spans: Vec<ViaStyleSpan>,
}

/// Reduced-width and reduced-spacing terminal fanout policy for one differential pair.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct DifferentialPairNeckdown {
    /// Exact trace width used by both members through the terminal fanout.
    pub trace_width: Real,
    /// Reduced edge-to-edge spacing accepted only at paired terminals.
    pub spacing: Real,
    /// Maximum exact centerline transition length for each member.
    pub maximum_transition_length: Real,
}

/// Coupled routing constraint for two complementary nets.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct DifferentialPair {
    /// Stable pair identity.
    pub id: DifferentialPairId,
    /// Positive member net.
    pub positive: NetId,
    /// Negative member net.
    pub negative: NetId,
    /// Preferred edge-to-edge pair spacing.
    pub spacing: Real,
    /// Maximum allowed length skew.
    pub max_skew: Option<Real>,
    /// Target differential (odd-mode) impedance in ohms.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub target_impedance_ohms: Option<Real>,
    /// Allowed absolute differential-impedance deviation in ohms.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub impedance_tolerance_ohms: Option<Real>,
    /// Optional bounded symmetric fanout from reduced to nominal pair spacing.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub neckdown: Option<DifferentialPairNeckdown>,
}

/// Planar direction permitted by retained routing intent.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum RouteDirection {
    /// Trace motion parallel to the board X axis.
    Horizontal,
    /// Trace motion parallel to the board Y axis.
    Vertical,
    /// Exact 45-degree motion whose X and Y coordinates change with the same sign.
    DiagonalRising,
    /// Exact 45-degree motion whose X and Y coordinates change with opposite signs.
    DiagonalFalling,
    /// Straight planar motion at an angle outside the axis/45-degree families.
    Arbitrary,
}

/// Polygonal region applying stricter width and clearance to selected nets.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct RouteRuleRegion {
    /// Stable source identity.
    pub id: RouteRuleRegionId,
    /// Closed board-space boundary. An edge touching the boundary is governed.
    #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::points"))]
    pub boundary: Vec<Point2>,
    /// Logical nets governed by the region; empty applies to every routed net.
    pub nets: Vec<NetId>,
    /// Regional minimum trace width; combined with other rules by exact maximum.
    pub min_trace_width: Option<Real>,
    /// Regional minimum copper clearance; combined with other rules by exact maximum.
    pub min_clearance: Option<Real>,
}

/// Polygonal region that restricts selected nets to authored layers and directions.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct RouteConstraintRegion {
    /// Stable source identity.
    pub id: RouteConstraintRegionId,
    /// Closed board-space boundary. An edge touching the boundary is constrained.
    #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::points"))]
    pub boundary: Vec<Point2>,
    /// Logical nets governed by the region; empty applies to every routed net.
    pub nets: Vec<NetId>,
    /// Copper layers permitted while a governed route touches the region.
    #[cfg_attr(
        feature = "interchange",
        serde(with = "crate::interchange::trace_layers")
    )]
    pub allowed_layers: Vec<TraceLayer>,
    /// Permitted planar motion while a governed route touches the region.
    pub allowed_directions: Vec<RouteDirection>,
    /// Whether an adjacent-layer transition may be centered in the region.
    pub allow_vias: bool,
}

/// Terminal-local fanout policy enforced inside an exact Manhattan radius.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct EscapePolicy {
    /// Stable source identity.
    pub id: EscapePolicyId,
    /// Placed instances whose matching terminals receive this policy.
    pub instances: Vec<CircuitInstanceId>,
    /// Logical nets governed on those instances; empty selects every bound net.
    pub nets: Vec<NetId>,
    /// Exact Manhattan distance from each pad center over which policy applies.
    pub max_distance: Real,
    /// Copper layers permitted in the escape envelope.
    #[cfg_attr(
        feature = "interchange",
        serde(with = "crate::interchange::trace_layers")
    )]
    pub allowed_layers: Vec<TraceLayer>,
    /// Permitted planar escape directions.
    pub allowed_directions: Vec<RouteDirection>,
    /// Whether the fanout may transition layers inside the escape envelope.
    pub allow_vias: bool,
}

/// Side of a directed orthogonal route span used for exact serpentine insertion.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LengthTuningSide {
    /// Offset to the left of the selected span's authored direction.
    Left,
    /// Offset to the right of the selected span's authored direction.
    Right,
}

/// Bounded exact serpentine intent for one routed logical net.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct LengthTuningPattern {
    /// Stable source identity.
    pub id: LengthTuningPatternId,
    /// Logical net whose total orthogonal centerline length is targeted.
    pub net: NetId,
    /// Optional stable route selector; `None` chooses deterministically by capacity.
    pub route: Option<RouteId>,
    /// Board-space region that must contain every generated tuning segment.
    #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::points"))]
    pub region: Vec<Point2>,
    /// Desired exact total centerline length over every route carrying `net`.
    pub target_length: Real,
    /// Accepted absolute deviation from the target.
    pub tolerance: Real,
    /// Exact perpendicular excursion for each serpentine cycle.
    pub amplitude: Real,
    /// Exact forward run before each perpendicular excursion.
    pub pitch: Real,
    /// Hard bound on generated cycles.
    pub maximum_cycles: usize,
    /// Directed side on which the serpentine is generated.
    pub side: LengthTuningSide,
}

/// Atomic length/phase-matching intent over two or more tuning patterns.
///
/// Member patterns retain their own route selector, region, target, and
/// serpentine geometry. The group adds an all-or-nothing application boundary,
/// a final member-skew contract, and retained-copper/keepout clearance
/// certification.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct PhaseTuningGroup {
    /// Stable group identity.
    pub id: PhaseTuningGroupId,
    /// Member tuning requests applied in retained source order.
    pub patterns: Vec<LengthTuningPatternId>,
    /// Optional differential pair whose two member nets must be covered exactly.
    pub differential_pair: Option<DifferentialPairId>,
    /// Maximum exact centerline-length skew across all member nets.
    pub maximum_skew: Real,
    /// Minimum exact edge clearance to foreign retained copper and keepouts.
    pub minimum_clearance: Real,
}

/// Circuit-owned design intent consumed by routing and `hyperdrc` adapters.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PcbDesignRules {
    /// Named per-net routing classes.
    pub net_classes: Vec<NetClass>,
    /// Named manufacturable via constructions selected by net classes.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub via_styles: Vec<ViaStyle>,
    /// Differential-pair declarations.
    pub differential_pairs: Vec<DifferentialPair>,
    /// Polygonal layer/direction constraints consumed by retained routers.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub route_constraint_regions: Vec<RouteConstraintRegion>,
    /// Polygonal width/clearance overrides consumed by retained routers.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub route_rule_regions: Vec<RouteRuleRegion>,
    /// Terminal-local package escape/fanout policies.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub escape_policies: Vec<EscapePolicy>,
    /// Exact post-route serpentine requests.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub length_tuning_patterns: Vec<LengthTuningPattern>,
    /// Atomic multi-pattern phase/length matching requests.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub phase_tuning_groups: Vec<PhaseTuningGroup>,
}

impl PcbDesignRules {
    /// Resolves every net class in source order with base-to-derived provenance.
    pub fn resolve_net_classes(&self) -> Result<Vec<ResolvedNetClass>, NetClassResolutionError> {
        let mut indices = BTreeMap::new();
        for (index, class) in self.net_classes.iter().enumerate() {
            if indices.insert(class.id.clone(), index).is_some() {
                return Err(NetClassResolutionError::DuplicateClass(class.id.clone()));
            }
        }
        let mut states = vec![0_u8; self.net_classes.len()];
        let mut resolved = vec![None; self.net_classes.len()];
        for index in 0..self.net_classes.len() {
            resolve_net_class(
                index,
                &self.net_classes,
                &indices,
                &mut states,
                &mut resolved,
            )?;
        }
        Ok(resolved
            .into_iter()
            .map(|class| class.expect("every source class was resolved"))
            .collect())
    }
}

fn resolve_net_class(
    index: usize,
    classes: &[NetClass],
    indices: &BTreeMap<NetClassId, usize>,
    states: &mut [u8],
    cache: &mut [Option<ResolvedNetClass>],
) -> Result<ResolvedNetClass, NetClassResolutionError> {
    if let Some(resolved) = &cache[index] {
        return Ok(resolved.clone());
    }
    if states[index] == 1 {
        return Err(NetClassResolutionError::InheritanceCycle(
            classes[index].id.clone(),
        ));
    }
    states[index] = 1;
    let class = &classes[index];
    let parent = class
        .parent
        .as_ref()
        .map(|parent| {
            let parent_index = indices.get(parent).copied().ok_or_else(|| {
                NetClassResolutionError::UnknownParent {
                    class: class.id.clone(),
                    parent: parent.clone(),
                }
            })?;
            resolve_net_class(parent_index, classes, indices, states, cache)
        })
        .transpose()?;
    let inherit = |child: &Option<Real>, parent: Option<&Option<Real>>| {
        child.clone().or_else(|| parent.and_then(Clone::clone))
    };
    let mut lineage = parent
        .as_ref()
        .map_or_else(Vec::new, |parent| parent.lineage.clone());
    lineage.push(class.id.clone());
    let resolved = ResolvedNetClass {
        id: class.id.clone(),
        lineage,
        nets: class.nets.clone(),
        min_trace_width: inherit(
            &class.min_trace_width,
            parent.as_ref().map(|parent| &parent.min_trace_width),
        ),
        preferred_trace_width: inherit(
            &class.preferred_trace_width,
            parent.as_ref().map(|parent| &parent.preferred_trace_width),
        ),
        min_clearance: inherit(
            &class.min_clearance,
            parent.as_ref().map(|parent| &parent.min_clearance),
        ),
        preferred_via_land_diameter: inherit(
            &class.preferred_via_land_diameter,
            parent
                .as_ref()
                .map(|parent| &parent.preferred_via_land_diameter),
        ),
        preferred_via_drill_diameter: inherit(
            &class.preferred_via_drill_diameter,
            parent
                .as_ref()
                .map(|parent| &parent.preferred_via_drill_diameter),
        ),
        preferred_via_style: class.preferred_via_style.clone().or_else(|| {
            parent
                .as_ref()
                .and_then(|parent| parent.preferred_via_style.clone())
        }),
        max_length: inherit(
            &class.max_length,
            parent.as_ref().map(|parent| &parent.max_length),
        ),
        max_via_count: class
            .max_via_count
            .or_else(|| parent.as_ref().and_then(|parent| parent.max_via_count)),
        target_impedance_ohms: inherit(
            &class.target_impedance_ohms,
            parent.as_ref().map(|parent| &parent.target_impedance_ohms),
        ),
        impedance_tolerance_ohms: inherit(
            &class.impedance_tolerance_ohms,
            parent
                .as_ref()
                .map(|parent| &parent.impedance_tolerance_ohms),
        ),
        requires_reference_plane: class.requires_reference_plane
            || parent
                .as_ref()
                .is_some_and(|parent| parent.requires_reference_plane),
    };
    states[index] = 2;
    cache[index] = Some(resolved.clone());
    Ok(resolved)
}

/// Stable mapping between circuit net identities and `hyperpath` numeric ids.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct RoutingNetAliases {
    aliases: BTreeMap<NetId, RoutingNetId>,
}

impl RoutingNetAliases {
    /// Builds deterministic aliases in circuit net declaration order.
    pub fn from_circuit(circuit: &Circuit) -> Result<Self, &'static str> {
        let mut aliases = BTreeMap::new();
        for (index, net) in circuit.nets.iter().enumerate() {
            let numeric = u32::try_from(index).map_err(|_| "too many nets for hyperpath ids")?;
            if aliases
                .insert(net.id.clone(), RoutingNetId(numeric))
                .is_some()
            {
                return Err("duplicate circuit net cannot receive a routing alias");
            }
        }
        Ok(Self { aliases })
    }

    /// Returns the routing alias for one logical net.
    pub fn get(&self, net: &NetId) -> Option<RoutingNetId> {
        self.aliases.get(net).copied()
    }

    /// Returns the logical identity for one numeric routing alias.
    pub fn logical(&self, routing_net: RoutingNetId) -> Option<&NetId> {
        self.aliases
            .iter()
            .find_map(|(logical, numeric)| (*numeric == routing_net).then_some(logical))
    }

    /// Iterates logical/numeric aliases in stable logical-id order.
    pub fn iter(&self) -> impl Iterator<Item = (&NetId, RoutingNetId)> {
        self.aliases
            .iter()
            .map(|(logical, numeric)| (logical, *numeric))
    }
}

/// Complete declarative PCB layout associated with a circuit.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct PcbLayout {
    /// Stable board id.
    pub id: BoardId,
    /// Substrate boundary and cutouts.
    pub outline: BoardOutline,
    /// Ordered physical stackup.
    pub stackup: PcbStackup,
    /// Reusable physical package definitions.
    pub land_patterns: Vec<LandPattern>,
    /// Placements of logical circuit instances.
    pub placements: Vec<PcbPlacement>,
    /// Authored fixed, relative, alignment and regional placement intent.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub placement_constraints: Vec<PlacementConstraint>,
    /// Routed copper.
    pub routes: Vec<PcbRoute>,
    /// Layer transitions and plated/non-plated drilled lands.
    pub vias: Vec<PcbVia>,
    /// Copper-pour source boundaries.
    pub zones: Vec<CopperZone>,
    /// Physical-layout keepouts.
    pub keepouts: Vec<PcbKeepout>,
    /// Authored routing and verification constraints.
    pub rules: PcbDesignRules,
}

/// Structural issue discovered before geometry or DRC lowering.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LayoutValidationIssue {
    /// Exterior or cutout polygon has fewer than three vertices.
    InvalidBoardContour,
    /// Stackup is empty or has no conductor.
    MissingCopperStackup,
    /// Stackup layer thickness is not strictly positive.
    NonPositiveLayerThickness(String),
    /// Two conductor layers use one routing index.
    DuplicateRoutingLayer(TraceLayer),
    /// Two land patterns share one id.
    DuplicateLandPattern(LandPatternId),
    /// Two pads in one land pattern share one id.
    DuplicatePad {
        land_pattern: LandPatternId,
        pad: PadId,
    },
    /// Pin map references a pad absent from its land pattern.
    UnknownMappedPad {
        land_pattern: LandPatternId,
        pad: PadId,
    },
    /// Two artwork primitives share one identity in a land pattern.
    DuplicateLandPatternGraphic {
        land_pattern: LandPatternId,
        graphic: LandPatternGraphicId,
    },
    /// Artwork dimensions or required vertex counts are invalid.
    InvalidLandPatternGraphic {
        land_pattern: LandPatternId,
        graphic: LandPatternGraphicId,
    },
    /// Package body outline or dimensions are invalid.
    InvalidLandPatternBody(LandPatternId),
    /// One independently retained package-model reference is invalid.
    InvalidPcb3dModelReference {
        /// Land pattern owning the reference.
        land_pattern: LandPatternId,
        /// Zero-based authored model record.
        model_index: usize,
    },
    /// One logical pin/pad pair is repeated in a land pattern.
    DuplicatePinPadMapping {
        land_pattern: LandPatternId,
        pin: PinRef,
        pad: PadId,
    },
    /// Placement references a logical instance absent from the circuit.
    UnknownPlacedInstance(CircuitInstanceId),
    /// Placement references an absent land pattern.
    UnknownPlacementLandPattern(LandPatternId),
    /// One logical instance is placed more than once.
    DuplicatePlacement(CircuitInstanceId),
    /// Two placement constraints share one stable id.
    DuplicatePlacementConstraint(PlacementConstraintId),
    /// A placement constraint references an unplaced logical instance.
    UnknownPlacementConstraintInstance {
        constraint: PlacementConstraintId,
        instance: CircuitInstanceId,
    },
    /// Constraint has invalid arity, self-reference, or region ordering.
    InvalidPlacementConstraint(PlacementConstraintId),
    /// More than one fixed/relative equation drives one instance origin.
    MultiplePlacementDrivers(CircuitInstanceId),
    /// A connected pin on a placed instance has no physical pad mapping.
    MissingPlacedPinPad {
        instance: CircuitInstanceId,
        pin: PinRef,
    },
    /// A placed land pattern maps a pin absent from the instance device model.
    UnknownPlacedPatternPin {
        instance: CircuitInstanceId,
        pin: PinRef,
    },
    /// Route references an absent logical net.
    UnknownRouteNet { route: RouteId, net: NetId },
    /// Route references a conductor layer absent from the stackup.
    UnknownRouteLayer { route: RouteId, layer: TraceLayer },
    /// Route has no centerline segments.
    EmptyRoute(RouteId),
    /// Route width is not strictly positive.
    NonPositiveRouteWidth(RouteId),
    /// Consecutive route segments are not connected exactly.
    DisconnectedRoute(RouteId),
    /// A circular route segment is as wide as or wider than its retained diameter.
    InvalidRouteArcWidth(RouteId),
    /// Two routes share one id.
    DuplicateRoute(RouteId),
    /// Via references an absent logical net.
    UnknownViaNet { via: ViaId, net: NetId },
    /// Via layer span, land, drill, or plating geometry is invalid.
    InvalidVia(ViaId),
    /// Two vias share one id.
    DuplicateVia(ViaId),
    /// Zone references an absent logical net.
    UnknownZoneNet { zone: ZoneId, net: NetId },
    /// Zone references a conductor layer absent from the stackup.
    UnknownZoneLayer { zone: ZoneId, layer: TraceLayer },
    /// Zone boundary has fewer than three vertices.
    InvalidZoneBoundary(ZoneId),
    /// Zone clearance, hatch, or connection policy is invalid.
    InvalidZonePolicy(ZoneId),
    /// Two zones share one id.
    DuplicateZone(ZoneId),
    /// Keepout boundary has fewer than three vertices.
    InvalidKeepoutBoundary(KeepoutId),
    /// Two keepouts share one id.
    DuplicateKeepout(KeepoutId),
    /// Two net classes share one identity.
    DuplicateNetClass(NetClassId),
    /// A net class references an absent base class.
    UnknownNetClassParent {
        class: NetClassId,
        parent: NetClassId,
    },
    /// Net-class inheritance contains a cycle.
    NetClassInheritanceCycle(NetClassId),
    /// One net is assigned directly to more than one class.
    NetInMultipleClasses(NetId),
    /// Two named via constructions share one identity.
    DuplicateViaStyle(ViaStyleId),
    /// A named via construction has invalid geometry, mask, or layer spans.
    InvalidViaStyle(ViaStyleId),
    /// A net class references an absent net.
    UnknownNetClassNet { class: NetClassId, net: NetId },
    /// A net class selects an absent named via construction.
    UnknownNetClassViaStyle {
        class: NetClassId,
        style: ViaStyleId,
    },
    /// A rule scalar that must be positive is zero, negative, or indeterminate.
    InvalidNetClassConstraint(NetClassId),
    /// Two differential pairs share one identity.
    DuplicateDifferentialPair(DifferentialPairId),
    /// A differential pair references an absent net, repeats a net, or has invalid dimensions.
    InvalidDifferentialPair(DifferentialPairId),
    /// Two route-constraint regions share one identity.
    DuplicateRouteConstraintRegion(RouteConstraintRegionId),
    /// A route-constraint region references an absent logical net.
    UnknownRouteConstraintRegionNet {
        region: RouteConstraintRegionId,
        net: NetId,
    },
    /// Region boundary, layer set, or direction set is invalid.
    InvalidRouteConstraintRegion(RouteConstraintRegionId),
    /// Two regional width/clearance rules share one identity.
    DuplicateRouteRuleRegion(RouteRuleRegionId),
    /// A regional rule references an absent logical net.
    UnknownRouteRuleRegionNet {
        region: RouteRuleRegionId,
        net: NetId,
    },
    /// Regional boundary or width/clearance override is invalid.
    InvalidRouteRuleRegion(RouteRuleRegionId),
    /// Two escape policies share one identity.
    DuplicateEscapePolicy(EscapePolicyId),
    /// An escape policy references an unplaced instance.
    UnknownEscapePolicyInstance {
        policy: EscapePolicyId,
        instance: CircuitInstanceId,
    },
    /// An escape policy references an absent logical net.
    UnknownEscapePolicyNet { policy: EscapePolicyId, net: NetId },
    /// Escape distance, layer set, direction set, or instance set is invalid.
    InvalidEscapePolicy(EscapePolicyId),
    /// Two tuning requests share one identity.
    DuplicateLengthTuningPattern(LengthTuningPatternId),
    /// A tuning request references an absent net or a route on another net.
    InvalidLengthTuningTarget(LengthTuningPatternId),
    /// Tuning geometry, tolerance, or generation bound is invalid.
    InvalidLengthTuningPattern(LengthTuningPatternId),
    /// Two phase-tuning groups share one identity.
    DuplicatePhaseTuningGroup(PhaseTuningGroupId),
    /// A phase group references absent/repeated patterns or the wrong pair members.
    InvalidPhaseTuningTarget(PhaseTuningGroupId),
    /// A phase group has too few members or invalid skew/clearance dimensions.
    InvalidPhaseTuningGroup(PhaseTuningGroupId),
}

/// Deterministic structural validation result for a PCB layout.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LayoutValidationReport {
    /// Every discovered issue in source order.
    pub issues: Vec<LayoutValidationIssue>,
}

impl LayoutValidationReport {
    /// True when layout identity and authored topology are structurally valid.
    pub fn is_valid(&self) -> bool {
        self.issues.is_empty()
    }
}

impl PcbLayout {
    /// Creates an empty retained board around an authored outline and stackup.
    pub fn new(id: BoardId, outline: BoardOutline, stackup: PcbStackup) -> Self {
        Self {
            id,
            outline,
            stackup,
            land_patterns: Vec::new(),
            placements: Vec::new(),
            placement_constraints: Vec::new(),
            routes: Vec::new(),
            vias: Vec::new(),
            zones: Vec::new(),
            keepouts: Vec::new(),
            rules: PcbDesignRules::default(),
        }
    }

    /// Validates retained layout identity and references against its circuit.
    ///
    /// This is structural validation, not DRC. Clearance, manufacturability,
    /// stackup policy, and release readiness remain `hyperdrc` responsibilities.
    pub fn validate(&self, circuit: &Circuit) -> LayoutValidationReport {
        let mut issues = Vec::new();
        if !self.outline.exterior.is_valid()
            || self.outline.cutouts.iter().any(|cutout| !cutout.is_valid())
        {
            issues.push(LayoutValidationIssue::InvalidBoardContour);
        }

        let mut routing_layers = BTreeSet::new();
        let mut conductor_count = 0_usize;
        for layer in &self.stackup.layers {
            if !is_strictly_positive(&layer.thickness) {
                issues.push(LayoutValidationIssue::NonPositiveLayerThickness(
                    layer.name.clone(),
                ));
            }
            if let StackupLayerKind::Conductor(index) = layer.kind {
                conductor_count += 1;
                if !routing_layers.insert(index) {
                    issues.push(LayoutValidationIssue::DuplicateRoutingLayer(index));
                }
            }
        }
        if conductor_count == 0 {
            issues.push(LayoutValidationIssue::MissingCopperStackup);
        }

        let mut land_pattern_ids = BTreeSet::new();
        for pattern in &self.land_patterns {
            if !land_pattern_ids.insert(pattern.id.clone()) {
                issues.push(LayoutValidationIssue::DuplicateLandPattern(
                    pattern.id.clone(),
                ));
            }
            let mut pads = BTreeSet::new();
            for pad in &pattern.pads {
                if !pads.insert(pad.id.clone()) {
                    issues.push(LayoutValidationIssue::DuplicatePad {
                        land_pattern: pattern.id.clone(),
                        pad: pad.id.clone(),
                    });
                }
            }
            for mapping in &pattern.pin_map {
                if !pads.contains(&mapping.pad) {
                    issues.push(LayoutValidationIssue::UnknownMappedPad {
                        land_pattern: pattern.id.clone(),
                        pad: mapping.pad.clone(),
                    });
                }
            }
            let mut graphic_ids = BTreeSet::new();
            for graphic in &pattern.graphics {
                if !graphic_ids.insert(graphic.id.clone()) {
                    issues.push(LayoutValidationIssue::DuplicateLandPatternGraphic {
                        land_pattern: pattern.id.clone(),
                        graphic: graphic.id.clone(),
                    });
                }
                let valid_stroke = graphic
                    .stroke_width
                    .as_ref()
                    .is_none_or(is_strictly_positive);
                let valid_primitive = match &graphic.primitive {
                    LandPatternGraphicPrimitive::Line { start, end } => start != end,
                    LandPatternGraphicPrimitive::Circle { radius, .. } => {
                        is_strictly_positive(radius)
                    }
                    LandPatternGraphicPrimitive::Polygon { vertices, .. } => vertices.len() >= 3,
                    LandPatternGraphicPrimitive::Text { text, height, .. } => {
                        !text.is_empty() && is_strictly_positive(height)
                    }
                };
                if !valid_stroke || !valid_primitive {
                    issues.push(LayoutValidationIssue::InvalidLandPatternGraphic {
                        land_pattern: pattern.id.clone(),
                        graphic: graphic.id.clone(),
                    });
                }
            }
            if let Some(body) = &pattern.body {
                let valid = body.outline.len() >= 3
                    && is_strictly_positive(&body.height)
                    && is_non_negative(&body.standoff);
                if !valid {
                    issues.push(LayoutValidationIssue::InvalidLandPatternBody(
                        pattern.id.clone(),
                    ));
                }
            }
            for (model_index, model) in pattern.models.iter().enumerate() {
                if model.uri.trim().is_empty()
                    || model.transform.scale_x == Real::zero()
                    || model.transform.scale_y == Real::zero()
                    || model.transform.scale_z == Real::zero()
                {
                    issues.push(LayoutValidationIssue::InvalidPcb3dModelReference {
                        land_pattern: pattern.id.clone(),
                        model_index,
                    });
                }
            }
            let mut mappings = BTreeSet::new();
            for mapping in &pattern.pin_map {
                if !mappings.insert((mapping.pin.clone(), mapping.pad.clone())) {
                    issues.push(LayoutValidationIssue::DuplicatePinPadMapping {
                        land_pattern: pattern.id.clone(),
                        pin: mapping.pin.clone(),
                        pad: mapping.pad.clone(),
                    });
                }
            }
        }

        let instance_ids = circuit
            .instances
            .iter()
            .map(|instance| instance.id.clone())
            .collect::<BTreeSet<_>>();
        let mut placed_instances = BTreeSet::new();
        for placement in &self.placements {
            if !placed_instances.insert(placement.instance.clone()) {
                issues.push(LayoutValidationIssue::DuplicatePlacement(
                    placement.instance.clone(),
                ));
            }
            if !instance_ids.contains(&placement.instance) {
                issues.push(LayoutValidationIssue::UnknownPlacedInstance(
                    placement.instance.clone(),
                ));
            }
            if !land_pattern_ids.contains(&placement.land_pattern) {
                issues.push(LayoutValidationIssue::UnknownPlacementLandPattern(
                    placement.land_pattern.clone(),
                ));
            }
            let instance = circuit
                .instances
                .iter()
                .find(|instance| instance.id == placement.instance);
            let pattern = self
                .land_patterns
                .iter()
                .find(|pattern| pattern.id == placement.land_pattern);
            if let (Some(instance), Some(pattern)) = (instance, pattern) {
                let mapped_pins = pattern
                    .pin_map
                    .iter()
                    .map(|mapping| mapping.pin.clone())
                    .collect::<BTreeSet<_>>();
                for binding in &instance.pins {
                    if !mapped_pins.contains(&binding.pin) {
                        issues.push(LayoutValidationIssue::MissingPlacedPinPad {
                            instance: instance.id.clone(),
                            pin: binding.pin.clone(),
                        });
                    }
                }
                if let Some(model) = circuit
                    .device_models
                    .iter()
                    .find(|model| model.id == instance.model)
                {
                    for pin in mapped_pins {
                        if !model.pins.iter().any(|candidate| candidate.pin == pin) {
                            issues.push(LayoutValidationIssue::UnknownPlacedPatternPin {
                                instance: instance.id.clone(),
                                pin,
                            });
                        }
                    }
                }
            }
        }

        let mut constraint_ids = BTreeSet::new();
        let mut driven_instances = BTreeSet::new();
        for constraint in &self.placement_constraints {
            if !constraint_ids.insert(constraint.id.clone()) {
                issues.push(LayoutValidationIssue::DuplicatePlacementConstraint(
                    constraint.id.clone(),
                ));
            }
            let referenced = match &constraint.kind {
                PlacementConstraintKind::Fixed { instance, .. } => vec![instance],
                PlacementConstraintKind::Relative {
                    instance, anchor, ..
                } => vec![instance, anchor],
                PlacementConstraintKind::AlignX { instances }
                | PlacementConstraintKind::AlignY { instances } => instances.iter().collect(),
                PlacementConstraintKind::Within { instance, .. } => vec![instance],
                PlacementConstraintKind::AllowedRotations { instance, .. }
                | PlacementConstraintKind::AllowedSides { instance, .. } => vec![instance],
            };
            for instance in referenced {
                if !placed_instances.contains(instance) {
                    issues.push(LayoutValidationIssue::UnknownPlacementConstraintInstance {
                        constraint: constraint.id.clone(),
                        instance: instance.clone(),
                    });
                }
            }
            let invalid = match &constraint.kind {
                PlacementConstraintKind::Fixed { instance, .. } => {
                    if !driven_instances.insert(instance.clone()) {
                        issues.push(LayoutValidationIssue::MultiplePlacementDrivers(
                            instance.clone(),
                        ));
                    }
                    false
                }
                PlacementConstraintKind::Relative {
                    instance, anchor, ..
                } => {
                    if !driven_instances.insert(instance.clone()) {
                        issues.push(LayoutValidationIssue::MultiplePlacementDrivers(
                            instance.clone(),
                        ));
                    }
                    instance == anchor
                }
                PlacementConstraintKind::AlignX { instances }
                | PlacementConstraintKind::AlignY { instances } => instances.len() < 2,
                PlacementConstraintKind::Within { min, max, .. } => {
                    !(min.x <= max.x && min.y <= max.y)
                }
                PlacementConstraintKind::AllowedRotations {
                    rotations_degrees, ..
                } => rotations_degrees.is_empty(),
                PlacementConstraintKind::AllowedSides { sides, .. } => sides.is_empty(),
            };
            if invalid {
                issues.push(LayoutValidationIssue::InvalidPlacementConstraint(
                    constraint.id.clone(),
                ));
            }
        }

        let net_ids = circuit
            .nets
            .iter()
            .map(|net| net.id.clone())
            .collect::<BTreeSet<_>>();
        let mut route_ids = BTreeSet::new();
        for route in &self.routes {
            if !route_ids.insert(route.id.clone()) {
                issues.push(LayoutValidationIssue::DuplicateRoute(route.id.clone()));
            }
            if !net_ids.contains(&route.net) {
                issues.push(LayoutValidationIssue::UnknownRouteNet {
                    route: route.id.clone(),
                    net: route.net.clone(),
                });
            }
            if !routing_layers.contains(&route.layer) {
                issues.push(LayoutValidationIssue::UnknownRouteLayer {
                    route: route.id.clone(),
                    layer: route.layer,
                });
            }
            if route.segments.is_empty() {
                issues.push(LayoutValidationIssue::EmptyRoute(route.id.clone()));
            }
            if !is_strictly_positive(&route.width) {
                issues.push(LayoutValidationIssue::NonPositiveRouteWidth(
                    route.id.clone(),
                ));
            }
            if route
                .segments
                .windows(2)
                .any(|pair| pair[0].end() != pair[1].start())
            {
                issues.push(LayoutValidationIssue::DisconnectedRoute(route.id.clone()));
            }
            if route.segments.iter().any(|segment| {
                matches!(
                    segment,
                    PcbRouteSegment::CircularArc(arc)
                        if route.width >= arc.radius().clone() + arc.radius().clone()
                )
            }) {
                issues.push(LayoutValidationIssue::InvalidRouteArcWidth(
                    route.id.clone(),
                ));
            }
        }

        let mut via_ids = BTreeSet::new();
        for via in &self.vias {
            if !via_ids.insert(via.id.clone()) {
                issues.push(LayoutValidationIssue::DuplicateVia(via.id.clone()));
            }
            if !net_ids.contains(&via.net) {
                issues.push(LayoutValidationIssue::UnknownViaNet {
                    via: via.id.clone(),
                    net: via.net.clone(),
                });
            }
            if via.start_layer > via.end_layer
                || (via.start_layer.0..=via.end_layer.0)
                    .any(|layer| !routing_layers.contains(&TraceLayer(layer)))
                || !is_strictly_positive(&via.land_diameter)
                || !is_strictly_positive(&via.drill_diameter)
                || via.drill_diameter > via.land_diameter
                || [&via.mask.front, &via.mask.back]
                    .into_iter()
                    .any(|mask| match mask {
                        ViaMaskDisposition::Open { margin } => {
                            !is_strictly_positive(&(via.land_diameter.clone() + margin + margin))
                        }
                        ViaMaskDisposition::Unspecified | ViaMaskDisposition::Tented => false,
                    })
            {
                issues.push(LayoutValidationIssue::InvalidVia(via.id.clone()));
            }
        }

        let mut zone_ids = BTreeSet::new();
        for zone in &self.zones {
            if !zone_ids.insert(zone.id.clone()) {
                issues.push(LayoutValidationIssue::DuplicateZone(zone.id.clone()));
            }
            if !net_ids.contains(&zone.net) {
                issues.push(LayoutValidationIssue::UnknownZoneNet {
                    zone: zone.id.clone(),
                    net: zone.net.clone(),
                });
            }
            if !routing_layers.contains(&zone.layer) {
                issues.push(LayoutValidationIssue::UnknownZoneLayer {
                    zone: zone.id.clone(),
                    layer: zone.layer,
                });
            }
            if zone.boundary.len() < 3 {
                issues.push(LayoutValidationIssue::InvalidZoneBoundary(zone.id.clone()));
            }
            let fill_valid = match &zone.fill {
                CopperZoneFill::Solid => true,
                CopperZoneFill::Hatched {
                    line_width, gap, ..
                } => is_strictly_positive(line_width) && is_non_negative(gap),
            };
            let connection_valid = match &zone.connection {
                CopperZoneConnection::Solid | CopperZoneConnection::Isolated => true,
                CopperZoneConnection::ThermalRelief {
                    air_gap,
                    spoke_width,
                    spoke_count,
                } => {
                    is_non_negative(air_gap)
                        && is_strictly_positive(spoke_width)
                        && *spoke_count >= 2
                }
            };
            let islands_valid = zone
                .islands
                .minimum_area
                .as_ref()
                .is_none_or(is_non_negative)
                && (zone.islands.remove_unconnected || zone.islands.minimum_area.is_none());
            let stitching_valid = zone.stitching.as_ref().is_none_or(|stitching| {
                let required_pitch = stitching.land_diameter.clone()
                    + stitching.edge_clearance.clone()
                    + stitching.edge_clearance.clone();
                is_strictly_positive(&stitching.pitch)
                    && is_non_negative(&stitching.edge_clearance)
                    && is_strictly_positive(&stitching.land_diameter)
                    && is_strictly_positive(&stitching.drill_diameter)
                    && stitching.drill_diameter <= stitching.land_diameter
                    && stitching.pitch >= required_pitch
                    && stitching.start_layer <= zone.layer
                    && zone.layer <= stitching.end_layer
                    && stitching.start_layer < stitching.end_layer
                    && routing_layers.contains(&stitching.start_layer)
                    && routing_layers.contains(&stitching.end_layer)
                    && stitching.maximum_vias != 0
                    && [&stitching.mask.front, &stitching.mask.back]
                        .into_iter()
                        .all(|mask| match mask {
                            ViaMaskDisposition::Open { margin } => is_strictly_positive(
                                &(stitching.land_diameter.clone() + margin + margin),
                            ),
                            ViaMaskDisposition::Unspecified | ViaMaskDisposition::Tented => true,
                        })
            });
            if !is_non_negative(&zone.clearance)
                || !fill_valid
                || !connection_valid
                || !islands_valid
                || !stitching_valid
            {
                issues.push(LayoutValidationIssue::InvalidZonePolicy(zone.id.clone()));
            }
        }

        let mut keepout_ids = BTreeSet::new();
        for keepout in &self.keepouts {
            if !keepout_ids.insert(keepout.id.clone()) {
                issues.push(LayoutValidationIssue::DuplicateKeepout(keepout.id.clone()));
            }
            if keepout.boundary.len() < 3 {
                issues.push(LayoutValidationIssue::InvalidKeepoutBoundary(
                    keepout.id.clone(),
                ));
            }
        }

        let mut via_style_ids = BTreeSet::new();
        for style in &self.rules.via_styles {
            if !via_style_ids.insert(style.id.clone()) {
                issues.push(LayoutValidationIssue::DuplicateViaStyle(style.id.clone()));
            }
            let mut spans = BTreeSet::new();
            let spans_valid = style.allowed_spans.iter().all(|span| {
                span.start_layer < span.end_layer
                    && routing_layers.contains(&span.start_layer)
                    && routing_layers.contains(&span.end_layer)
                    && (span.start_layer.0..=span.end_layer.0)
                        .all(|layer| routing_layers.contains(&TraceLayer(layer)))
                    && spans.insert((span.start_layer, span.end_layer))
            });
            let mask_valid =
                [&style.mask.front, &style.mask.back]
                    .into_iter()
                    .all(|mask| match mask {
                        ViaMaskDisposition::Open { margin } => {
                            is_strictly_positive(&(style.land_diameter.clone() + margin + margin))
                        }
                        ViaMaskDisposition::Unspecified | ViaMaskDisposition::Tented => true,
                    });
            if !is_strictly_positive(&style.land_diameter)
                || !is_strictly_positive(&style.drill_diameter)
                || style.drill_diameter > style.land_diameter
                || style.plating != Plating::Plated
                || !spans_valid
                || !mask_valid
            {
                issues.push(LayoutValidationIssue::InvalidViaStyle(style.id.clone()));
            }
        }

        let mut class_ids = BTreeSet::new();
        let mut classified_nets = BTreeSet::new();
        for class in &self.rules.net_classes {
            if !class_ids.insert(class.id.clone()) {
                issues.push(LayoutValidationIssue::DuplicateNetClass(class.id.clone()));
            }
            for net in &class.nets {
                if !classified_nets.insert(net.clone()) {
                    issues.push(LayoutValidationIssue::NetInMultipleClasses(net.clone()));
                }
                if !net_ids.contains(net) {
                    issues.push(LayoutValidationIssue::UnknownNetClassNet {
                        class: class.id.clone(),
                        net: net.clone(),
                    });
                }
            }
            if let Some(style) = &class.preferred_via_style
                && !via_style_ids.contains(style)
            {
                issues.push(LayoutValidationIssue::UnknownNetClassViaStyle {
                    class: class.id.clone(),
                    style: style.clone(),
                });
            }
            let dimensions = [
                &class.min_trace_width,
                &class.preferred_trace_width,
                &class.min_clearance,
                &class.preferred_via_land_diameter,
                &class.preferred_via_drill_diameter,
                &class.max_length,
                &class.target_impedance_ohms,
                &class.impedance_tolerance_ohms,
            ];
            if dimensions
                .into_iter()
                .flatten()
                .any(|value| !is_strictly_positive(value))
            {
                issues.push(LayoutValidationIssue::InvalidNetClassConstraint(
                    class.id.clone(),
                ));
            }
        }
        if class_ids.len() == self.rules.net_classes.len()
            && let Err(error) = self.rules.resolve_net_classes()
        {
            match error {
                NetClassResolutionError::DuplicateClass(class) => {
                    issues.push(LayoutValidationIssue::DuplicateNetClass(class));
                }
                NetClassResolutionError::UnknownParent { class, parent } => {
                    issues.push(LayoutValidationIssue::UnknownNetClassParent { class, parent });
                }
                NetClassResolutionError::InheritanceCycle(class) => {
                    issues.push(LayoutValidationIssue::NetClassInheritanceCycle(class));
                }
            }
        }

        let mut pair_ids = BTreeSet::new();
        for pair in &self.rules.differential_pairs {
            if !pair_ids.insert(pair.id.clone()) {
                issues.push(LayoutValidationIssue::DuplicateDifferentialPair(
                    pair.id.clone(),
                ));
            }
            if pair.positive == pair.negative
                || !net_ids.contains(&pair.positive)
                || !net_ids.contains(&pair.negative)
                || !is_strictly_positive(&pair.spacing)
                || pair
                    .max_skew
                    .as_ref()
                    .is_some_and(|value| !is_strictly_positive(value))
                || pair
                    .target_impedance_ohms
                    .as_ref()
                    .is_some_and(|value| !is_strictly_positive(value))
                || pair
                    .impedance_tolerance_ohms
                    .as_ref()
                    .is_some_and(|value| !is_strictly_positive(value))
                || pair.target_impedance_ohms.is_some() != pair.impedance_tolerance_ohms.is_some()
                || pair.neckdown.as_ref().is_some_and(|neckdown| {
                    !is_strictly_positive(&neckdown.trace_width)
                        || !is_strictly_positive(&neckdown.spacing)
                        || !is_strictly_positive(&neckdown.maximum_transition_length)
                        || neckdown.spacing.partial_cmp(&pair.spacing)
                            != Some(std::cmp::Ordering::Less)
                })
            {
                issues.push(LayoutValidationIssue::InvalidDifferentialPair(
                    pair.id.clone(),
                ));
            }
        }

        let mut region_ids = BTreeSet::new();
        for region in &self.rules.route_constraint_regions {
            if !region_ids.insert(region.id.clone()) {
                issues.push(LayoutValidationIssue::DuplicateRouteConstraintRegion(
                    region.id.clone(),
                ));
            }
            for net in &region.nets {
                if !net_ids.contains(net) {
                    issues.push(LayoutValidationIssue::UnknownRouteConstraintRegionNet {
                        region: region.id.clone(),
                        net: net.clone(),
                    });
                }
            }
            let unique_nets = region.nets.iter().collect::<BTreeSet<_>>();
            let unique_layers = region.allowed_layers.iter().collect::<BTreeSet<_>>();
            let unique_directions = region.allowed_directions.iter().collect::<BTreeSet<_>>();
            if region.boundary.len() < 3
                || region.allowed_layers.is_empty()
                || region.allowed_directions.is_empty()
                || unique_nets.len() != region.nets.len()
                || unique_layers.len() != region.allowed_layers.len()
                || unique_directions.len() != region.allowed_directions.len()
                || region
                    .allowed_layers
                    .iter()
                    .any(|layer| !routing_layers.contains(layer))
            {
                issues.push(LayoutValidationIssue::InvalidRouteConstraintRegion(
                    region.id.clone(),
                ));
            }
        }

        let mut rule_region_ids = BTreeSet::new();
        for region in &self.rules.route_rule_regions {
            if !rule_region_ids.insert(region.id.clone()) {
                issues.push(LayoutValidationIssue::DuplicateRouteRuleRegion(
                    region.id.clone(),
                ));
            }
            for net in &region.nets {
                if !net_ids.contains(net) {
                    issues.push(LayoutValidationIssue::UnknownRouteRuleRegionNet {
                        region: region.id.clone(),
                        net: net.clone(),
                    });
                }
            }
            let unique_nets = region.nets.iter().collect::<BTreeSet<_>>();
            if region.boundary.len() < 3
                || unique_nets.len() != region.nets.len()
                || region.min_trace_width.is_none() && region.min_clearance.is_none()
                || region
                    .min_trace_width
                    .as_ref()
                    .is_some_and(|width| !is_strictly_positive(width))
                || region
                    .min_clearance
                    .as_ref()
                    .is_some_and(|clearance| !is_non_negative(clearance))
            {
                issues.push(LayoutValidationIssue::InvalidRouteRuleRegion(
                    region.id.clone(),
                ));
            }
        }

        let mut escape_ids = BTreeSet::new();
        for policy in &self.rules.escape_policies {
            if !escape_ids.insert(policy.id.clone()) {
                issues.push(LayoutValidationIssue::DuplicateEscapePolicy(
                    policy.id.clone(),
                ));
            }
            for instance in &policy.instances {
                if !placed_instances.contains(instance) {
                    issues.push(LayoutValidationIssue::UnknownEscapePolicyInstance {
                        policy: policy.id.clone(),
                        instance: instance.clone(),
                    });
                }
            }
            for net in &policy.nets {
                if !net_ids.contains(net) {
                    issues.push(LayoutValidationIssue::UnknownEscapePolicyNet {
                        policy: policy.id.clone(),
                        net: net.clone(),
                    });
                }
            }
            let unique_instances = policy.instances.iter().collect::<BTreeSet<_>>();
            let unique_nets = policy.nets.iter().collect::<BTreeSet<_>>();
            let unique_layers = policy.allowed_layers.iter().collect::<BTreeSet<_>>();
            let unique_directions = policy.allowed_directions.iter().collect::<BTreeSet<_>>();
            if policy.instances.is_empty()
                || !is_strictly_positive(&policy.max_distance)
                || policy.allowed_layers.is_empty()
                || policy.allowed_directions.is_empty()
                || unique_instances.len() != policy.instances.len()
                || unique_nets.len() != policy.nets.len()
                || unique_layers.len() != policy.allowed_layers.len()
                || unique_directions.len() != policy.allowed_directions.len()
                || policy
                    .allowed_layers
                    .iter()
                    .any(|layer| !routing_layers.contains(layer))
            {
                issues.push(LayoutValidationIssue::InvalidEscapePolicy(
                    policy.id.clone(),
                ));
            }
        }

        let mut tuning_ids = BTreeSet::new();
        for pattern in &self.rules.length_tuning_patterns {
            if !tuning_ids.insert(pattern.id.clone()) {
                issues.push(LayoutValidationIssue::DuplicateLengthTuningPattern(
                    pattern.id.clone(),
                ));
            }
            let selected_route = pattern
                .route
                .as_ref()
                .and_then(|route| self.routes.iter().find(|candidate| &candidate.id == route));
            if !net_ids.contains(&pattern.net)
                || pattern.route.is_some()
                    && selected_route.is_none_or(|route| route.net != pattern.net)
            {
                issues.push(LayoutValidationIssue::InvalidLengthTuningTarget(
                    pattern.id.clone(),
                ));
            }
            if pattern.region.len() < 3
                || !is_strictly_positive(&pattern.target_length)
                || !is_non_negative(&pattern.tolerance)
                || !is_strictly_positive(&pattern.amplitude)
                || !is_strictly_positive(&pattern.pitch)
                || pattern.maximum_cycles == 0
            {
                issues.push(LayoutValidationIssue::InvalidLengthTuningPattern(
                    pattern.id.clone(),
                ));
            }
        }

        let mut phase_group_ids = BTreeSet::new();
        for group in &self.rules.phase_tuning_groups {
            if !phase_group_ids.insert(group.id.clone()) {
                issues.push(LayoutValidationIssue::DuplicatePhaseTuningGroup(
                    group.id.clone(),
                ));
            }
            let unique_patterns = group.patterns.iter().collect::<BTreeSet<_>>();
            let member_patterns = group
                .patterns
                .iter()
                .filter_map(|id| {
                    self.rules
                        .length_tuning_patterns
                        .iter()
                        .find(|pattern| &pattern.id == id)
                })
                .collect::<Vec<_>>();
            let unique_nets = member_patterns
                .iter()
                .map(|pattern| &pattern.net)
                .collect::<BTreeSet<_>>();
            let differential_pair_is_valid = group.differential_pair.as_ref().is_none_or(|id| {
                self.rules
                    .differential_pairs
                    .iter()
                    .find(|pair| &pair.id == id)
                    .is_some_and(|pair| {
                        member_patterns.len() == 2
                            && unique_nets.len() == 2
                            && unique_nets.contains(&pair.positive)
                            && unique_nets.contains(&pair.negative)
                    })
            });
            if unique_patterns.len() != group.patterns.len()
                || member_patterns.len() != group.patterns.len()
                || unique_nets.len() != group.patterns.len()
                || !differential_pair_is_valid
            {
                issues.push(LayoutValidationIssue::InvalidPhaseTuningTarget(
                    group.id.clone(),
                ));
            }
            if group.patterns.len() < 2
                || !is_non_negative(&group.maximum_skew)
                || !is_non_negative(&group.minimum_clearance)
            {
                issues.push(LayoutValidationIssue::InvalidPhaseTuningGroup(
                    group.id.clone(),
                ));
            }
        }

        LayoutValidationReport { issues }
    }

    /// Resolves fixed/relative origin equations, then certifies alignment and regions.
    pub fn resolve_placement_constraints(&self, circuit: &Circuit) -> PlacementResolutionReport {
        if !self.validate(circuit).is_valid() {
            return PlacementResolutionReport {
                placements: self.placements.clone(),
                issues: vec![PlacementResolutionIssue::InvalidLayout],
            };
        }
        let authored = self
            .placements
            .iter()
            .map(|placement| (placement.instance.clone(), placement.position.clone()))
            .collect::<BTreeMap<_, _>>();
        let drivers = self
            .placement_constraints
            .iter()
            .filter_map(|constraint| match &constraint.kind {
                PlacementConstraintKind::Fixed { instance, .. }
                | PlacementConstraintKind::Relative { instance, .. } => {
                    Some((instance.clone(), &constraint.kind))
                }
                _ => None,
            })
            .collect::<BTreeMap<_, _>>();
        let mut resolved = BTreeMap::new();
        let mut visiting = BTreeSet::new();
        let mut issues = Vec::new();
        for instance in authored.keys() {
            resolve_position(
                instance,
                &authored,
                &drivers,
                &mut resolved,
                &mut visiting,
                &mut issues,
            );
        }
        let mut placements = self.placements.clone();
        for placement in &mut placements {
            if let Some(position) = resolved.get(&placement.instance) {
                placement.position.clone_from(position);
            }
        }
        for constraint in &self.placement_constraints {
            match &constraint.kind {
                PlacementConstraintKind::AlignX { instances } => {
                    if !aligned(instances, &resolved, |point| &point.x) {
                        issues.push(PlacementResolutionIssue::MisalignedX(constraint.id.clone()));
                    }
                }
                PlacementConstraintKind::AlignY { instances } => {
                    if !aligned(instances, &resolved, |point| &point.y) {
                        issues.push(PlacementResolutionIssue::MisalignedY(constraint.id.clone()));
                    }
                }
                PlacementConstraintKind::Within { instance, min, max } => {
                    let inside = resolved.get(instance).is_some_and(|point| {
                        min.x <= point.x && point.x <= max.x && min.y <= point.y && point.y <= max.y
                    });
                    if !inside {
                        issues.push(PlacementResolutionIssue::OutsideRegion(
                            constraint.id.clone(),
                        ));
                    }
                }
                PlacementConstraintKind::AllowedRotations {
                    instance,
                    rotations_degrees,
                } => {
                    let allowed = placements.iter().any(|placement| {
                        placement.instance == *instance
                            && rotations_degrees.contains(&placement.rotation_degrees)
                    });
                    if !allowed {
                        issues.push(PlacementResolutionIssue::DisallowedRotation(
                            constraint.id.clone(),
                        ));
                    }
                }
                PlacementConstraintKind::AllowedSides { instance, sides } => {
                    let allowed = placements.iter().any(|placement| {
                        placement.instance == *instance && sides.contains(&placement.side)
                    });
                    if !allowed {
                        issues.push(PlacementResolutionIssue::DisallowedSide(
                            constraint.id.clone(),
                        ));
                    }
                }
                PlacementConstraintKind::Fixed { .. }
                | PlacementConstraintKind::Relative { .. } => {}
            }
        }
        PlacementResolutionReport { placements, issues }
    }
}

fn resolve_position(
    instance: &CircuitInstanceId,
    authored: &BTreeMap<CircuitInstanceId, Point2>,
    drivers: &BTreeMap<CircuitInstanceId, &PlacementConstraintKind>,
    resolved: &mut BTreeMap<CircuitInstanceId, Point2>,
    visiting: &mut BTreeSet<CircuitInstanceId>,
    issues: &mut Vec<PlacementResolutionIssue>,
) -> Point2 {
    if let Some(position) = resolved.get(instance) {
        return position.clone();
    }
    if !visiting.insert(instance.clone()) {
        issues.push(PlacementResolutionIssue::RelativeCycle(instance.clone()));
        return authored[instance].clone();
    }
    let position = match drivers.get(instance) {
        Some(PlacementConstraintKind::Fixed { position, .. }) => position.clone(),
        Some(PlacementConstraintKind::Relative { anchor, offset, .. }) => {
            let anchor = resolve_position(anchor, authored, drivers, resolved, visiting, issues);
            Point2::new(anchor.x + offset.x.clone(), anchor.y + offset.y.clone())
        }
        _ => authored[instance].clone(),
    };
    visiting.remove(instance);
    resolved.insert(instance.clone(), position.clone());
    position
}

fn aligned<'a>(
    instances: &[CircuitInstanceId],
    positions: &'a BTreeMap<CircuitInstanceId, Point2>,
    coordinate: impl Fn(&'a Point2) -> &'a Real,
) -> bool {
    let Some(first) = instances
        .first()
        .and_then(|instance| positions.get(instance))
    else {
        return false;
    };
    let first = coordinate(first);
    instances.iter().skip(1).all(|instance| {
        positions
            .get(instance)
            .is_some_and(|point| coordinate(point) == first)
    })
}

fn is_strictly_positive(value: &Real) -> bool {
    value.structural_facts().sign == Some(RealSign::Positive)
}

fn is_non_negative(value: &Real) -> bool {
    matches!(
        value.structural_facts().sign,
        Some(RealSign::Zero | RealSign::Positive)
    )
}
