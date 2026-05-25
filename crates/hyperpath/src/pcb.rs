//! PCB trace path predicates.
//!
//! Classical PCB autorouters such as Lee's maze router and Hightower's line
//! router are excellent candidate generators, but their candidates must be
//! checked against exact swept geometry before they become trusted topology.
//! This module starts that boundary for straight trace segments. See Lee,
//! "An Algorithm for Path Connections and Its Applications," *IRE
//! Transactions on Electronic Computers* 1961, and Hightower, "A solution to
//! line-routing problems on the continuous plane," DAC 1969. The exact
//! predicate discipline follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997).

use std::cmp::Ordering;

use hyperlimit::{
    Point2, PredicatePolicy, SegmentIntersection, classify_segment_intersection_with_facts,
    compare_reals_with_policy,
};
use hyperreal::{Real, RealExactSetFacts};

use crate::provenance::PathProvenance;
use crate::segment::{Axis, LinePathSegment, real_sign};
use crate::swept::SweptLineSegment;

/// Stable PCB net identifier.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct NetId(pub u32);

/// PCB copper layer identifier.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct TraceLayer(pub u16);

/// Exact discrete via transition class for a known board layer count.
///
/// Autorouters and DSN/SES-style interchange treat vias as layer-transition
/// objects before any geometric clearance is checked. This enum keeps that
/// discrete topology explicit and exact: layer indices are retained integers,
/// while copper/drill geometry remains in the `Real` predicates. The split
/// follows Yap's exact object/predicate model and the Lee/Hightower routing
/// tradition where graph transitions are proposed separately from final
/// geometric certification.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ViaLayerTransitionClass {
    /// The land is confined to one layer; no routing layer transition occurs.
    SingleLayerLand,
    /// The via starts at the top or bottom board surface and stops internally.
    BlindVia,
    /// The via connects internal layers only.
    BuriedVia,
    /// The via spans from the first to the last board layer.
    ThroughVia,
}

/// Exact via layer-transition report.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ViaLayerTransitionReport {
    /// Certified transition class.
    pub class: ViaLayerTransitionClass,
    /// Total board layer count used for classification.
    pub board_layer_count: u16,
    /// Inclusive start layer.
    pub start_layer: TraceLayer,
    /// Inclusive end layer.
    pub end_layer: TraceLayer,
    /// Inclusive number of layers spanned by the via object.
    pub spanned_layers: u16,
}

/// Exact relation between two inclusive via layer spans.
///
/// PCB routers and DSN/SES importers need to distinguish actual shared copper
/// layers from merely adjacent transition intervals. This discrete predicate
/// keeps via topology in the exact object layer before geometry clearance is
/// evaluated, matching Yap's separation of candidate topology from certified
/// geometric predicates and the Lee/Hightower routing split between graph
/// transitions and physical checks.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ViaLayerSpanRelation {
    /// The first via span is entirely below the second with at least one layer gap.
    DisjointBelow,
    /// The first via span ends immediately before the second begins.
    AdjacentBelow,
    /// The spans share exactly one layer.
    TouchingLayer,
    /// The spans share two or more layers.
    OverlappingLayers,
    /// The first via span begins immediately after the second ends.
    AdjacentAbove,
    /// The first via span is entirely above the second with at least one layer gap.
    DisjointAbove,
}

/// Exact via span relation report.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ViaLayerSpanReport {
    /// Certified relation between inclusive layer intervals.
    pub relation: ViaLayerSpanRelation,
    /// First via inclusive start layer.
    pub first_start: TraceLayer,
    /// First via inclusive end layer.
    pub first_end: TraceLayer,
    /// Second via inclusive start layer.
    pub second_start: TraceLayer,
    /// Second via inclusive end layer.
    pub second_end: TraceLayer,
    /// Inclusive overlap start when layers are shared.
    pub overlap_start: Option<TraceLayer>,
    /// Inclusive overlap end when layers are shared.
    pub overlap_end: Option<TraceLayer>,
    /// Number of shared layers.
    pub shared_layers: u16,
}

/// Exact retained intent for a via drill.
///
/// Drill fabrication is not the same predicate as copper clearance. A drilled
/// hole can be plated as a routing via, non-plated as a mechanical hole, or
/// retained from an interchange file without enough process intent. Keeping
/// this discrete fact explicit follows Yap's exact object/predicate split and
/// IPC-style separation between annular-ring and hole-to-copper rules.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ViaDrillIntent {
    /// No drill process intent was retained.
    Unspecified,
    /// The drill is intended to be plated and must satisfy annular-ring policy.
    Plated,
    /// The drill is non-plated and should be treated as a hole keepout, not a via land.
    NonPlated,
}

/// Exact fabrication-policy class for a retained via drill.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ViaDrillPolicyClass {
    /// No drill diameter was available.
    MissingDrill,
    /// A plated drill is present; annular-ring certification applies.
    PlatedCopperVia,
    /// A non-plated drill is present; annular-ring certification is not applicable.
    NonPlatedMechanicalHole,
    /// A drill exists, but the plating intent was not retained.
    UnspecifiedDrilledHole,
}

/// Exact via drill fabrication-policy report.
#[derive(Clone, Debug, PartialEq)]
pub struct ViaDrillPolicyReport {
    /// Classified drill policy.
    pub class: ViaDrillPolicyClass,
    /// Retained drill intent.
    pub intent: ViaDrillIntent,
    /// Exact drill diameter when present.
    pub drill_diameter: Option<Real>,
    /// Annular-ring certification for plated drills.
    pub annular_ring: Option<ViaAnnularRingReport>,
}

/// Coarse trace width class for routing schedule decisions.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TraceWidthClass {
    /// Width is exactly zero.
    Zero,
    /// Width is structurally positive.
    Positive,
    /// Width sign is not structurally known.
    Unknown,
}

/// Cached facts for one trace segment.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbTraceFacts {
    /// Exact-set facts across geometry and width.
    pub exact: RealExactSetFacts,
    /// Trace width class.
    pub width_class: TraceWidthClass,
    /// Whether this trace is axis-aligned.
    pub axis_aligned: Option<Axis>,
    /// Source provenance inherited from the swept geometry.
    pub provenance: PathProvenance,
}

/// Straight swept PCB trace segment.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbTrace {
    net: NetId,
    layer: TraceLayer,
    swept: SweptLineSegment,
    facts: PcbTraceFacts,
}

/// Exact axis-aligned PCB board outline.
///
/// This is the first board-edge carrier for routing certification. It models
/// the common rectangular board-envelope rule exactly and keeps arbitrary board
/// contours for the later arrangement layer. The split follows Yap's exact
/// object/predicate boundary: a router may propose traces from grid search, but
/// edge clearance is accepted only after exact comparison against retained
/// outline coordinates.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbBoardOutline {
    min: Point2,
    max: Point2,
    provenance: PathProvenance,
    exact: RealExactSetFacts,
}

/// Orientation of a retained convex PCB board contour.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BoardContourOrientation {
    /// Vertices wind counter-clockwise.
    CounterClockwise,
    /// Vertices wind clockwise.
    Clockwise,
}

/// Errors while constructing exact non-rectangular board contours.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BoardContourError {
    /// Fewer than three vertices were supplied.
    TooFewVertices,
    /// The polygon has zero signed area.
    DegenerateArea,
    /// A convexity or orientation comparison could not be decided exactly.
    UnknownOrientation,
    /// Consecutive vertices were collinear under the exact predicate.
    CollinearEdge,
    /// The polygon is not strictly convex.
    NonConvex,
    /// At least one retained edge is not exactly horizontal or vertical.
    NonOrthogonal,
    /// Non-adjacent edges intersect, so the contour is not a simple polygon.
    SelfIntersecting,
}

/// Exact strictly convex polygonal PCB board outline.
///
/// This is the first non-rectangular board-edge carrier. It deliberately
/// accepts only strictly convex straight-edge contours; arbitrary nonconvex,
/// curved, and boolean-composed board outlines remain arrangement-kernel work.
/// Clearance is checked with squared line-distance predicates so no tolerance
/// or square-root approximation is introduced. This follows Yap's exact
/// object/predicate boundary and the same staged routing discipline used for
/// Lee/Hightower-style candidate routes.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbConvexBoardOutline {
    vertices: Vec<Point2>,
    orientation: BoardContourOrientation,
    provenance: PathProvenance,
    exact: RealExactSetFacts,
}

/// Exact simple orthogonal PCB board outline.
///
/// Rectilinear boards are common in panel and enclosure-constrained PCB work,
/// including nonconvex notches and tabs that a strictly convex carrier cannot
/// represent. This type keeps that useful subset exact: every edge must be
/// horizontal or vertical, the polygon must be simple, and point containment is
/// decided by an exact ray-crossing predicate in the tradition of Shimrat's
/// point-in-polygon algorithm and Haines' survey of crossing tests. The trust
/// boundary remains Yap-style: an autorouter may propose centerlines, but this
/// object certifies containment and edge clearance before route acceptance.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbOrthogonalBoardOutline {
    vertices: Vec<Point2>,
    orientation: BoardContourOrientation,
    provenance: PathProvenance,
    exact: RealExactSetFacts,
}

/// Cached facts for a circular pad or via land.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbPadFacts {
    /// Exact-set facts across center coordinates and diameter.
    pub exact: RealExactSetFacts,
    /// Diameter class.
    pub diameter_class: TraceWidthClass,
    /// Source provenance.
    pub provenance: PathProvenance,
}

/// Exact circular pad/via approximation for first routing predicates.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbCircularPad {
    net: NetId,
    layer: TraceLayer,
    center: Point2,
    diameter: Real,
    facts: PcbPadFacts,
}

/// Exact axis-aligned rectangular pad.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbRectPad {
    net: NetId,
    layer: TraceLayer,
    center: Point2,
    width: Real,
    height: Real,
    provenance: PathProvenance,
}

/// Cardinal rotation for exact rectangular-pad predicates.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CardinalRotation {
    /// Zero degrees.
    Deg0,
    /// Ninety degrees counter-clockwise.
    Deg90,
    /// One hundred eighty degrees.
    Deg180,
    /// Two hundred seventy degrees counter-clockwise.
    Deg270,
}

/// Exact rectangular pad with a cardinal rotation.
///
/// Many footprints use rectangular SMD pads rotated by 90 degree increments.
/// Those rotations are exact: the local rectangle remains axis-aligned after
/// swapping width and height. Arbitrary-angle rectangles should be promoted to
/// exact polygon/arrangement geometry later, but cardinal rotations can be
/// certified by the same squared-distance predicate as axis-aligned pads.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbCardinalRectPad {
    net: NetId,
    layer: TraceLayer,
    center: Point2,
    width: Real,
    height: Real,
    rotation: CardinalRotation,
    provenance: PathProvenance,
}

impl PcbBoardOutline {
    /// Construct an axis-aligned board outline with native provenance.
    pub fn new(min: Point2, max: Point2) -> Result<Self, &'static str> {
        Self::with_provenance(min, max, PathProvenance::native())
    }

    /// Construct an axis-aligned board outline with source provenance.
    ///
    /// Both axes must be ordered when the comparison can be decided exactly.
    /// Unknown symbolic order is rejected for now because board-edge clearance
    /// needs a certified inside/outside orientation before routing predicates
    /// can make manufacturing decisions.
    pub fn with_provenance(
        min: Point2,
        max: Point2,
        provenance: PathProvenance,
    ) -> Result<Self, &'static str> {
        if !matches!(
            compare_reals_with_policy(&min.x, &max.x, PredicatePolicy::default()).value(),
            Some(Ordering::Less | Ordering::Equal)
        ) {
            return Err("board outline x bounds must be ordered");
        }
        if !matches!(
            compare_reals_with_policy(&min.y, &max.y, PredicatePolicy::default()).value(),
            Some(Ordering::Less | Ordering::Equal)
        ) {
            return Err("board outline y bounds must be ordered");
        }
        let exact = Real::exact_set_facts([&min.x, &min.y, &max.x, &max.y]);
        Ok(Self {
            min,
            max,
            provenance,
            exact,
        })
    }

    /// Return the exact minimum board corner.
    pub const fn min(&self) -> &Point2 {
        &self.min
    }

    /// Return the exact maximum board corner.
    pub const fn max(&self) -> &Point2 {
        &self.max
    }

    /// Return source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.provenance
    }

    /// Return exact-set facts for the outline coordinates.
    pub const fn exact_facts(&self) -> &RealExactSetFacts {
        &self.exact
    }
}

impl PcbConvexBoardOutline {
    /// Construct a strictly convex board outline with native provenance.
    pub fn new(vertices: Vec<Point2>) -> Result<Self, BoardContourError> {
        Self::with_provenance(vertices, PathProvenance::native())
    }

    /// Construct a strictly convex board outline with source provenance.
    pub fn with_provenance(
        vertices: Vec<Point2>,
        provenance: PathProvenance,
    ) -> Result<Self, BoardContourError> {
        if vertices.len() < 3 {
            return Err(BoardContourError::TooFewVertices);
        }
        let signed_area_twice = polygon_signed_area_twice(&vertices);
        let orientation = match compare_reals_with_policy(
            &signed_area_twice,
            &Real::zero(),
            PredicatePolicy::default(),
        )
        .value()
        {
            Some(Ordering::Greater) => BoardContourOrientation::CounterClockwise,
            Some(Ordering::Less) => BoardContourOrientation::Clockwise,
            Some(Ordering::Equal) => return Err(BoardContourError::DegenerateArea),
            None => return Err(BoardContourError::UnknownOrientation),
        };
        validate_strict_convexity(&vertices, orientation)?;
        let refs = vertices
            .iter()
            .flat_map(|point| [&point.x, &point.y])
            .collect::<Vec<_>>();
        let exact = Real::exact_set_facts(refs);
        Ok(Self {
            vertices,
            orientation,
            provenance,
            exact,
        })
    }

    /// Return retained vertices in winding order.
    pub fn vertices(&self) -> &[Point2] {
        &self.vertices
    }

    /// Return certified contour orientation.
    pub const fn orientation(&self) -> BoardContourOrientation {
        self.orientation
    }

    /// Return source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.provenance
    }

    /// Return exact-set facts for vertex coordinates.
    pub const fn exact_facts(&self) -> &RealExactSetFacts {
        &self.exact
    }
}

impl PcbOrthogonalBoardOutline {
    /// Construct a simple orthogonal board outline with native provenance.
    pub fn new(vertices: Vec<Point2>) -> Result<Self, BoardContourError> {
        Self::with_provenance(vertices, PathProvenance::native())
    }

    /// Construct a simple orthogonal board outline with source provenance.
    ///
    /// The constructor validates the exact object shape up front: there must be
    /// at least three vertices, nonzero signed area, horizontal or vertical
    /// nondegenerate edges, and no non-adjacent edge intersections. General
    /// arbitrary-angle and curved board outlines remain arrangement-kernel
    /// work; this carrier intentionally covers the high-value rectilinear
    /// nonconvex subset without tolerance geometry.
    pub fn with_provenance(
        vertices: Vec<Point2>,
        provenance: PathProvenance,
    ) -> Result<Self, BoardContourError> {
        if vertices.len() < 3 {
            return Err(BoardContourError::TooFewVertices);
        }
        let signed_area_twice = polygon_signed_area_twice(&vertices);
        let orientation = match compare_reals_with_policy(
            &signed_area_twice,
            &Real::zero(),
            PredicatePolicy::default(),
        )
        .value()
        {
            Some(Ordering::Greater) => BoardContourOrientation::CounterClockwise,
            Some(Ordering::Less) => BoardContourOrientation::Clockwise,
            Some(Ordering::Equal) => return Err(BoardContourError::DegenerateArea),
            None => return Err(BoardContourError::UnknownOrientation),
        };
        validate_orthogonal_edges(&vertices)?;
        validate_simple_polygon_edges(&vertices)?;
        let refs = vertices
            .iter()
            .flat_map(|point| [&point.x, &point.y])
            .collect::<Vec<_>>();
        let exact = Real::exact_set_facts(refs);
        Ok(Self {
            vertices,
            orientation,
            provenance,
            exact,
        })
    }

    /// Return retained vertices in winding order.
    pub fn vertices(&self) -> &[Point2] {
        &self.vertices
    }

    /// Return certified contour orientation.
    pub const fn orientation(&self) -> BoardContourOrientation {
        self.orientation
    }

    /// Return source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.provenance
    }

    /// Return exact-set facts for vertex coordinates.
    pub const fn exact_facts(&self) -> &RealExactSetFacts {
        &self.exact
    }
}

impl PcbRectPad {
    /// Construct an axis-aligned rectangular pad.
    pub fn new(
        net: NetId,
        layer: TraceLayer,
        center: Point2,
        width: Real,
        height: Real,
    ) -> Result<Self, &'static str> {
        Self::with_provenance(net, layer, center, width, height, PathProvenance::native())
    }

    /// Construct an axis-aligned rectangular pad with source provenance.
    ///
    /// This is the first non-circular pad carrier. Rotated pads and rounded
    /// rectangles should be represented later as exact polygon/arc path
    /// geometry; this type keeps common SMD rectangular-pad clearance inside a
    /// small exact predicate surface.
    pub fn with_provenance(
        net: NetId,
        layer: TraceLayer,
        center: Point2,
        width: Real,
        height: Real,
        provenance: PathProvenance,
    ) -> Result<Self, &'static str> {
        if real_sign(&width) == Some(hyperreal::RealSign::Negative) {
            return Err("rect pad width must be nonnegative");
        }
        if real_sign(&height) == Some(hyperreal::RealSign::Negative) {
            return Err("rect pad height must be nonnegative");
        }
        Ok(Self {
            net,
            layer,
            center,
            width,
            height,
            provenance,
        })
    }

    /// Return pad net.
    pub const fn net(&self) -> NetId {
        self.net
    }

    /// Return pad layer.
    pub const fn layer(&self) -> TraceLayer {
        self.layer
    }

    /// Return exact center.
    pub const fn center(&self) -> &Point2 {
        &self.center
    }

    /// Return exact width.
    pub const fn width(&self) -> &Real {
        &self.width
    }

    /// Return exact height.
    pub const fn height(&self) -> &Real {
        &self.height
    }

    /// Return source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.provenance
    }
}

impl PcbCardinalRectPad {
    /// Construct a cardinally rotated rectangular pad.
    pub fn new(
        net: NetId,
        layer: TraceLayer,
        center: Point2,
        width: Real,
        height: Real,
        rotation: CardinalRotation,
    ) -> Result<Self, &'static str> {
        Self::with_provenance(
            net,
            layer,
            center,
            width,
            height,
            rotation,
            PathProvenance::native(),
        )
    }

    /// Construct a cardinally rotated rectangular pad with source provenance.
    ///
    /// The exact transformation is intentionally limited to cardinal
    /// rotations. This keeps footprint import and routing predicates on Yap's
    /// object layer: no trigonometric approximation is required to know the
    /// effective rectangle used by this specialized predicate.
    pub fn with_provenance(
        net: NetId,
        layer: TraceLayer,
        center: Point2,
        width: Real,
        height: Real,
        rotation: CardinalRotation,
        provenance: PathProvenance,
    ) -> Result<Self, &'static str> {
        if real_sign(&width) == Some(hyperreal::RealSign::Negative) {
            return Err("cardinal rect pad width must be nonnegative");
        }
        if real_sign(&height) == Some(hyperreal::RealSign::Negative) {
            return Err("cardinal rect pad height must be nonnegative");
        }
        Ok(Self {
            net,
            layer,
            center,
            width,
            height,
            rotation,
            provenance,
        })
    }

    /// Return pad net.
    pub const fn net(&self) -> NetId {
        self.net
    }

    /// Return pad layer.
    pub const fn layer(&self) -> TraceLayer {
        self.layer
    }

    /// Return exact center.
    pub const fn center(&self) -> &Point2 {
        &self.center
    }

    /// Return unrotated local width.
    pub const fn width(&self) -> &Real {
        &self.width
    }

    /// Return unrotated local height.
    pub const fn height(&self) -> &Real {
        &self.height
    }

    /// Return cardinal rotation.
    pub const fn rotation(&self) -> CardinalRotation {
        self.rotation
    }

    /// Return source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.provenance
    }

    /// Return the exact axis-aligned rectangle equivalent for this rotation.
    pub fn effective_rect(&self) -> Result<PcbRectPad, &'static str> {
        match self.rotation {
            CardinalRotation::Deg0 | CardinalRotation::Deg180 => PcbRectPad::with_provenance(
                self.net,
                self.layer,
                self.center.clone(),
                self.width.clone(),
                self.height.clone(),
                self.provenance,
            ),
            CardinalRotation::Deg90 | CardinalRotation::Deg270 => PcbRectPad::with_provenance(
                self.net,
                self.layer,
                self.center.clone(),
                self.height.clone(),
                self.width.clone(),
                self.provenance,
            ),
        }
    }
}

/// Exact via stack spanning a contiguous layer range.
#[derive(Clone, Debug, PartialEq)]
pub struct PcbViaStack {
    net: NetId,
    start_layer: TraceLayer,
    end_layer: TraceLayer,
    pad: PcbCircularPad,
    drill_diameter: Option<Real>,
    drill_intent: ViaDrillIntent,
}

impl PcbViaStack {
    /// Construct a via stack with the same circular land on every spanned layer.
    pub fn new(
        net: NetId,
        start_layer: TraceLayer,
        end_layer: TraceLayer,
        center: Point2,
        land_diameter: Real,
    ) -> Result<Self, &'static str> {
        if start_layer > end_layer {
            return Err("via start layer must not be above end layer");
        }
        let pad = PcbCircularPad::new(net, start_layer, center, land_diameter)?;
        Ok(Self {
            net,
            start_layer,
            end_layer,
            pad,
            drill_diameter: None,
            drill_intent: ViaDrillIntent::Unspecified,
        })
    }

    /// Construct a via stack with an exact drill diameter.
    pub fn with_drill(
        net: NetId,
        start_layer: TraceLayer,
        end_layer: TraceLayer,
        center: Point2,
        land_diameter: Real,
        drill_diameter: Real,
    ) -> Result<Self, &'static str> {
        Self::with_drill_intent(
            net,
            start_layer,
            end_layer,
            center,
            land_diameter,
            drill_diameter,
            ViaDrillIntent::Plated,
        )
    }

    /// Construct a via stack with an exact drill diameter and retained process intent.
    pub fn with_drill_intent(
        net: NetId,
        start_layer: TraceLayer,
        end_layer: TraceLayer,
        center: Point2,
        land_diameter: Real,
        drill_diameter: Real,
        drill_intent: ViaDrillIntent,
    ) -> Result<Self, &'static str> {
        let mut via = Self::new(net, start_layer, end_layer, center, land_diameter)?;
        if real_sign(&drill_diameter) == Some(hyperreal::RealSign::Negative) {
            return Err("via drill diameter must be nonnegative");
        }
        via.drill_diameter = Some(drill_diameter);
        via.drill_intent = drill_intent;
        Ok(via)
    }

    /// Return whether this via is present on `layer`.
    pub const fn spans_layer(&self, layer: TraceLayer) -> bool {
        self.start_layer.0 <= layer.0 && layer.0 <= self.end_layer.0
    }

    /// Return inclusive start layer.
    pub const fn start_layer(&self) -> TraceLayer {
        self.start_layer
    }

    /// Return inclusive end layer.
    pub const fn end_layer(&self) -> TraceLayer {
        self.end_layer
    }

    /// Return the inclusive number of spanned layers.
    pub const fn spanned_layer_count(&self) -> u16 {
        self.end_layer.0 - self.start_layer.0 + 1
    }

    /// Classify this via as a discrete board-layer transition.
    ///
    /// `board_layer_count` is the number of copper layers in the board, so
    /// valid retained layer indices are `0..board_layer_count`. The method
    /// rejects zero-layer boards and via spans outside that range instead of
    /// silently clamping them. It is intentionally a topological PCB predicate:
    /// annular-ring, drill, and clearance acceptance remain separate exact
    /// geometric/manufacturing checks.
    pub fn classify_layer_transition(
        &self,
        board_layer_count: u16,
    ) -> Result<ViaLayerTransitionReport, &'static str> {
        if board_layer_count == 0 {
            return Err("board layer count must be positive");
        }
        let last_layer = board_layer_count - 1;
        if self.end_layer.0 > last_layer {
            return Err("via layer span exceeds board layer count");
        }
        let class = if self.start_layer == self.end_layer {
            ViaLayerTransitionClass::SingleLayerLand
        } else if self.start_layer.0 == 0 && self.end_layer.0 == last_layer {
            ViaLayerTransitionClass::ThroughVia
        } else if self.start_layer.0 == 0 || self.end_layer.0 == last_layer {
            ViaLayerTransitionClass::BlindVia
        } else {
            ViaLayerTransitionClass::BuriedVia
        };
        Ok(ViaLayerTransitionReport {
            class,
            board_layer_count,
            start_layer: self.start_layer,
            end_layer: self.end_layer,
            spanned_layers: self.spanned_layer_count(),
        })
    }

    /// Classify this via's inclusive layer interval against another via.
    ///
    /// This is a pure discrete-topology predicate. It intentionally does not
    /// inspect XY position, copper land diameter, or drill data; those remain
    /// exact geometric/manufacturing predicates after a router has established
    /// which layer intervals can interact.
    pub fn classify_layer_span_with(&self, other: &Self) -> ViaLayerSpanReport {
        let relation = if self.end_layer.0 < other.start_layer.0 {
            if self.end_layer.0.checked_add(1) == Some(other.start_layer.0) {
                ViaLayerSpanRelation::AdjacentBelow
            } else {
                ViaLayerSpanRelation::DisjointBelow
            }
        } else if other.end_layer.0 < self.start_layer.0 {
            if other.end_layer.0.checked_add(1) == Some(self.start_layer.0) {
                ViaLayerSpanRelation::AdjacentAbove
            } else {
                ViaLayerSpanRelation::DisjointAbove
            }
        } else if self.end_layer.0.checked_add(1) == Some(other.start_layer.0) {
            ViaLayerSpanRelation::AdjacentBelow
        } else if other.end_layer.0.checked_add(1) == Some(self.start_layer.0) {
            ViaLayerSpanRelation::AdjacentAbove
        } else {
            let overlap_start = self.start_layer.0.max(other.start_layer.0);
            let overlap_end = self.end_layer.0.min(other.end_layer.0);
            if overlap_start == overlap_end {
                ViaLayerSpanRelation::TouchingLayer
            } else {
                ViaLayerSpanRelation::OverlappingLayers
            }
        };
        let (overlap_start, overlap_end, shared_layers) = match relation {
            ViaLayerSpanRelation::TouchingLayer | ViaLayerSpanRelation::OverlappingLayers => {
                let start = self.start_layer.0.max(other.start_layer.0);
                let end = self.end_layer.0.min(other.end_layer.0);
                (
                    Some(TraceLayer(start)),
                    Some(TraceLayer(end)),
                    end - start + 1,
                )
            }
            _ => (None, None, 0),
        };
        ViaLayerSpanReport {
            relation,
            first_start: self.start_layer,
            first_end: self.end_layer,
            second_start: other.start_layer,
            second_end: other.end_layer,
            overlap_start,
            overlap_end,
            shared_layers,
        }
    }

    /// Return via net.
    pub const fn net(&self) -> NetId {
        self.net
    }

    /// Return via center.
    pub const fn center(&self) -> &Point2 {
        self.pad.center()
    }

    /// Return via land diameter.
    pub const fn land_diameter(&self) -> &Real {
        self.pad.diameter()
    }

    /// Return optional exact drill diameter.
    pub const fn drill_diameter(&self) -> Option<&Real> {
        self.drill_diameter.as_ref()
    }

    /// Return retained drill process intent.
    pub const fn drill_intent(&self) -> ViaDrillIntent {
        self.drill_intent
    }

    /// Certify the annular ring against an exact minimum requirement.
    ///
    /// This checks `land >= drill + 2*minimum`. The comparison is exact and
    /// belongs at the fabrication-rule boundary; trace/via topology still uses
    /// clearance predicates. IPC-style annular-ring policy can layer richer
    /// manufacturing classes over this exact primitive later.
    pub fn certify_annular_ring(
        &self,
        minimum: &Real,
        policy: PredicatePolicy,
    ) -> ViaAnnularRingReport {
        let Some(drill) = self.drill_diameter.as_ref() else {
            return ViaAnnularRingReport::UnknownNoDrill;
        };
        if real_sign(minimum) == Some(hyperreal::RealSign::Negative) {
            return ViaAnnularRingReport::InvalidMinimum;
        }
        let required = drill.clone() + minimum.clone() * Real::from(2);
        match compare_reals_with_policy(self.land_diameter(), &required, policy).value() {
            Some(Ordering::Less) => ViaAnnularRingReport::Violation,
            Some(Ordering::Equal | Ordering::Greater) => ViaAnnularRingReport::Certified,
            None => ViaAnnularRingReport::Unknown,
        }
    }

    /// Classify retained drill fabrication policy exactly.
    ///
    /// Plated drills replay the annular-ring predicate; non-plated drills keep
    /// their hole geometry but skip annular-ring acceptance because no plated
    /// copper barrel is intended. Missing and unspecified cases are explicit so
    /// importers cannot accidentally treat absent fabrication intent as a
    /// certified via.
    pub fn classify_drill_policy(
        &self,
        minimum_annular_ring: &Real,
        policy: PredicatePolicy,
    ) -> ViaDrillPolicyReport {
        let Some(drill_diameter) = self.drill_diameter.as_ref() else {
            return ViaDrillPolicyReport {
                class: ViaDrillPolicyClass::MissingDrill,
                intent: self.drill_intent,
                drill_diameter: None,
                annular_ring: None,
            };
        };
        match self.drill_intent {
            ViaDrillIntent::Plated => ViaDrillPolicyReport {
                class: ViaDrillPolicyClass::PlatedCopperVia,
                intent: self.drill_intent,
                drill_diameter: Some(drill_diameter.clone()),
                annular_ring: Some(self.certify_annular_ring(minimum_annular_ring, policy)),
            },
            ViaDrillIntent::NonPlated => ViaDrillPolicyReport {
                class: ViaDrillPolicyClass::NonPlatedMechanicalHole,
                intent: self.drill_intent,
                drill_diameter: Some(drill_diameter.clone()),
                annular_ring: None,
            },
            ViaDrillIntent::Unspecified => ViaDrillPolicyReport {
                class: ViaDrillPolicyClass::UnspecifiedDrilledHole,
                intent: self.drill_intent,
                drill_diameter: Some(drill_diameter.clone()),
                annular_ring: None,
            },
        }
    }
}

/// Exact annular-ring certification result.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ViaAnnularRingReport {
    /// The via land is certified large enough for drill plus minimum ring.
    Certified,
    /// The via land is certified too small.
    Violation,
    /// No drill diameter was available.
    UnknownNoDrill,
    /// The minimum annular ring was invalid.
    InvalidMinimum,
    /// Exact comparison could not decide.
    Unknown,
}

impl PcbCircularPad {
    /// Construct a circular pad or via land with native provenance.
    pub fn new(
        net: NetId,
        layer: TraceLayer,
        center: Point2,
        diameter: Real,
    ) -> Result<Self, &'static str> {
        Self::with_provenance(net, layer, center, diameter, PathProvenance::native())
    }

    /// Construct a circular pad or via land with source provenance.
    ///
    /// This stores the exact center and diameter. More specific pad shapes
    /// should add exact polygon/arc carriers later; this circular carrier is
    /// enough for first via and round-pad clearance predicates.
    pub fn with_provenance(
        net: NetId,
        layer: TraceLayer,
        center: Point2,
        diameter: Real,
        provenance: PathProvenance,
    ) -> Result<Self, &'static str> {
        let diameter_class = match real_sign(&diameter) {
            Some(hyperreal::RealSign::Negative) => {
                return Err("pad diameter must be nonnegative");
            }
            Some(hyperreal::RealSign::Zero) => TraceWidthClass::Zero,
            Some(hyperreal::RealSign::Positive) => TraceWidthClass::Positive,
            None => TraceWidthClass::Unknown,
        };
        let facts = PcbPadFacts {
            exact: Real::exact_set_facts([&center.x, &center.y, &diameter]),
            diameter_class,
            provenance,
        };
        Ok(Self {
            net,
            layer,
            center,
            diameter,
            facts,
        })
    }

    /// Return pad net.
    pub const fn net(&self) -> NetId {
        self.net
    }

    /// Return pad layer.
    pub const fn layer(&self) -> TraceLayer {
        self.layer
    }

    /// Return exact center point.
    pub const fn center(&self) -> &Point2 {
        &self.center
    }

    /// Return exact diameter.
    pub const fn diameter(&self) -> &Real {
        &self.diameter
    }

    /// Return cached facts.
    pub const fn facts(&self) -> &PcbPadFacts {
        &self.facts
    }
}

impl PcbTrace {
    /// Construct a PCB trace from swept geometry and net metadata.
    pub fn new(net: NetId, layer: TraceLayer, swept: SweptLineSegment) -> Self {
        let width_class = match real_sign(swept.width()) {
            Some(hyperreal::RealSign::Zero) => TraceWidthClass::Zero,
            Some(hyperreal::RealSign::Positive) => TraceWidthClass::Positive,
            _ => TraceWidthClass::Unknown,
        };
        let facts = PcbTraceFacts {
            exact: swept.facts().exact,
            width_class,
            axis_aligned: swept.centerline().facts().axis_aligned,
            provenance: swept.provenance(),
        };
        Self {
            net,
            layer,
            swept,
            facts,
        }
    }

    /// Return net identifier.
    pub const fn net(&self) -> NetId {
        self.net
    }

    /// Return trace layer.
    pub const fn layer(&self) -> TraceLayer {
        self.layer
    }

    /// Return swept trace geometry.
    pub const fn swept(&self) -> &SweptLineSegment {
        &self.swept
    }

    /// Return cached facts.
    pub const fn facts(&self) -> &PcbTraceFacts {
        &self.facts
    }

    /// Return source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.facts.provenance
    }
}

/// Exact clearance classification between two straight trace segments.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ClearanceStatus {
    /// Same-net or different-layer traces are not clearance violations here.
    NotApplicable,
    /// Centerlines intersect on the same layer and different nets.
    NoShortViolation,
    /// Axis-aligned swept geometry is certified separated.
    CertifiedClear,
    /// Axis-aligned swept geometry is certified too close.
    ClearanceViolation,
    /// Current exact specialized predicates could not decide this shape.
    Unknown,
}

/// Report for one via drill to board-outline clearance check.
#[derive(Clone, Debug, PartialEq)]
pub struct DrillBoardClearanceReport {
    /// Classification status.
    pub status: ClearanceStatus,
    /// Exact minimum center margin to the board edge when it was computed.
    pub axis_gap: Option<Real>,
    /// Whether the via had no drill diameter.
    pub missing_drill: bool,
}

/// Report for one pad to board-outline clearance check.
#[derive(Clone, Debug, PartialEq)]
pub struct PadBoardClearanceReport {
    /// Classification status.
    pub status: ClearanceStatus,
    /// Exact minimum copper margin to the board edge when it was computed.
    pub copper_gap: Option<Real>,
}

/// Report for one trace-pair clearance check.
#[derive(Clone, Debug, PartialEq)]
pub struct TraceClearanceReport {
    /// Classification status.
    pub status: ClearanceStatus,
    /// Centerline intersection, if it was decided.
    pub centerline_intersection: Option<SegmentIntersection>,
    /// Exact distance gap used by specialized axis-aligned checks.
    pub axis_gap: Option<Real>,
}

impl TraceClearanceReport {
    /// Returns whether this report proves the pair is route-safe.
    pub const fn is_certified_clear(&self) -> bool {
        matches!(
            self.status,
            ClearanceStatus::NotApplicable | ClearanceStatus::CertifiedClear
        )
    }
}

/// Check same-layer different-net trace clearance.
pub fn check_trace_clearance(
    first: &PcbTrace,
    second: &PcbTrace,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> TraceClearanceReport {
    if first.layer != second.layer || first.net == second.net {
        return TraceClearanceReport {
            status: ClearanceStatus::NotApplicable,
            centerline_intersection: None,
            axis_gap: None,
        };
    }

    let centerline = classify_segment_intersection_with_facts(
        first.swept.centerline().start(),
        first.swept.centerline().end(),
        second.swept.centerline().start(),
        second.swept.centerline().end(),
        first.swept.centerline().facts().segment,
        second.swept.centerline().facts().segment,
    )
    .value();

    if matches!(
        centerline,
        Some(
            SegmentIntersection::Proper
                | SegmentIntersection::EndpointTouch
                | SegmentIntersection::CollinearOverlap
                | SegmentIntersection::Identical
        )
    ) {
        return TraceClearanceReport {
            status: ClearanceStatus::NoShortViolation,
            centerline_intersection: centerline,
            axis_gap: Some(Real::zero()),
        };
    }

    match axis_aligned_gap(first.swept.centerline(), second.swept.centerline(), policy) {
        Some(gap) => {
            let doubled_gap = gap.clone() * Real::from(2);
            let required = first.swept.width().clone()
                + second.swept.width().clone()
                + required_clearance.clone() * Real::from(2);
            let status = match compare_reals_with_policy(&doubled_gap, &required, policy).value() {
                Some(Ordering::Less) => ClearanceStatus::ClearanceViolation,
                Some(Ordering::Equal | Ordering::Greater) => ClearanceStatus::CertifiedClear,
                None => ClearanceStatus::Unknown,
            };
            TraceClearanceReport {
                status,
                centerline_intersection: centerline,
                axis_gap: Some(gap),
            }
        }
        None => TraceClearanceReport {
            status: ClearanceStatus::Unknown,
            centerline_intersection: centerline,
            axis_gap: None,
        },
    }
}

/// Check same-layer different-net clearance between a trace and a circular pad.
///
/// The predicate compares squared exact distances and squared required
/// separation, avoiding square roots and primitive tolerances. This is the
/// same exact-predicate strategy as the trace/trace checker: candidate routing
/// may be heuristic, but copper overlap and clearance acceptance are certified
/// by exact scalar comparisons.
pub fn check_trace_pad_clearance(
    trace: &PcbTrace,
    pad: &PcbCircularPad,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> TraceClearanceReport {
    if trace.layer != pad.layer || trace.net == pad.net {
        return TraceClearanceReport {
            status: ClearanceStatus::NotApplicable,
            centerline_intersection: None,
            axis_gap: None,
        };
    }
    let Some(distance_squared) =
        axis_aligned_point_segment_distance_squared(trace.swept.centerline(), pad.center(), policy)
    else {
        return TraceClearanceReport {
            status: ClearanceStatus::Unknown,
            centerline_intersection: None,
            axis_gap: None,
        };
    };
    let four_distance_squared = distance_squared * Real::from(4);
    let overlap_limit = trace.swept.width().clone() + pad.diameter().clone();
    let overlap_limit_squared = overlap_limit.clone() * overlap_limit;
    let clearance_limit = trace.swept.width().clone()
        + pad.diameter().clone()
        + required_clearance.clone() * Real::from(2);
    let clearance_limit_squared = clearance_limit.clone() * clearance_limit;

    let status =
        match compare_reals_with_policy(&four_distance_squared, &overlap_limit_squared, policy)
            .value()
        {
            Some(Ordering::Less | Ordering::Equal) => ClearanceStatus::NoShortViolation,
            Some(Ordering::Greater) => {
                match compare_reals_with_policy(
                    &four_distance_squared,
                    &clearance_limit_squared,
                    policy,
                )
                .value()
                {
                    Some(Ordering::Less) => ClearanceStatus::ClearanceViolation,
                    Some(Ordering::Equal | Ordering::Greater) => ClearanceStatus::CertifiedClear,
                    None => ClearanceStatus::Unknown,
                }
            }
            None => ClearanceStatus::Unknown,
        };

    TraceClearanceReport {
        status,
        centerline_intersection: None,
        axis_gap: None,
    }
}

/// Check a trace against a via stack on the trace's layer.
pub fn check_trace_via_clearance(
    trace: &PcbTrace,
    via: &PcbViaStack,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> TraceClearanceReport {
    if !via.spans_layer(trace.layer()) {
        return TraceClearanceReport {
            status: ClearanceStatus::NotApplicable,
            centerline_intersection: None,
            axis_gap: None,
        };
    }
    let pad = PcbCircularPad::new(
        via.net(),
        trace.layer(),
        via.center().clone(),
        via.land_diameter().clone(),
    )
    .expect("validated via stack land diameter remains valid");
    check_trace_pad_clearance(trace, &pad, required_clearance, policy)
}

/// Check a trace against the exact drill keepout of a via stack.
///
/// This is a fabrication-rule predicate distinct from copper-land clearance:
/// the via land participates in routing clearance, while the drill diameter is
/// the manufactured hole that must not cut into unrelated copper. The drill is
/// represented as an exact circular keepout and checked with the same
/// squared-distance predicate as round pads. This follows Yap's exact
/// object/predicate split and mirrors IPC-2221-style separation between
/// annular ring and hole-to-copper clearance rules.
pub fn check_trace_via_drill_clearance(
    trace: &PcbTrace,
    via: &PcbViaStack,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> TraceClearanceReport {
    if !via.spans_layer(trace.layer()) {
        return TraceClearanceReport {
            status: ClearanceStatus::NotApplicable,
            centerline_intersection: None,
            axis_gap: None,
        };
    }
    let Some(drill_diameter) = via.drill_diameter() else {
        return unknown_clearance_report();
    };
    let drill = PcbCircularPad::new(
        via.net(),
        trace.layer(),
        via.center().clone(),
        drill_diameter.clone(),
    )
    .expect("validated via drill diameter remains valid");
    check_trace_pad_clearance(trace, &drill, required_clearance, policy)
}

/// Check an axis-aligned trace against an axis-aligned rectangular pad.
///
/// The checker compares exact squared distances to squared copper-overlap and
/// clearance limits, avoiding square roots and primitive tolerances. It is a
/// specialized exact predicate for common rectangular SMD pads; general pad
/// outlines should later route through exact polygon/arc arrangements.
pub fn check_trace_rect_pad_clearance(
    trace: &PcbTrace,
    pad: &PcbRectPad,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> TraceClearanceReport {
    if trace.layer != pad.layer || trace.net == pad.net {
        return TraceClearanceReport {
            status: ClearanceStatus::NotApplicable,
            centerline_intersection: None,
            axis_gap: None,
        };
    }
    let Some(distance_squared) =
        axis_aligned_segment_rect_distance_squared(trace.swept.centerline(), pad, policy)
    else {
        return TraceClearanceReport {
            status: ClearanceStatus::Unknown,
            centerline_intersection: None,
            axis_gap: None,
        };
    };
    let four_distance_squared = distance_squared * Real::from(4);
    let overlap_limit = trace.swept.width().clone();
    let overlap_limit_squared = overlap_limit.clone() * overlap_limit;
    let clearance_limit = trace.swept.width().clone() + required_clearance.clone() * Real::from(2);
    let clearance_limit_squared = clearance_limit.clone() * clearance_limit;
    let status =
        match compare_reals_with_policy(&four_distance_squared, &overlap_limit_squared, policy)
            .value()
        {
            Some(Ordering::Less | Ordering::Equal) => ClearanceStatus::NoShortViolation,
            Some(Ordering::Greater) => {
                match compare_reals_with_policy(
                    &four_distance_squared,
                    &clearance_limit_squared,
                    policy,
                )
                .value()
                {
                    Some(Ordering::Less) => ClearanceStatus::ClearanceViolation,
                    Some(Ordering::Equal | Ordering::Greater) => ClearanceStatus::CertifiedClear,
                    None => ClearanceStatus::Unknown,
                }
            }
            None => ClearanceStatus::Unknown,
        };
    TraceClearanceReport {
        status,
        centerline_intersection: None,
        axis_gap: None,
    }
}

/// Check an axis-aligned trace against a cardinally rotated rectangular pad.
///
/// Cardinal rotations are exact extent swaps, so this function lowers to the
/// axis-aligned rectangular-pad predicate without introducing sine/cosine
/// approximations. That gives practical footprint coverage while arbitrary pad
/// rotations remain future polygon/arrangement work.
pub fn check_trace_cardinal_rect_pad_clearance(
    trace: &PcbTrace,
    pad: &PcbCardinalRectPad,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> TraceClearanceReport {
    match pad.effective_rect() {
        Ok(rect) => check_trace_rect_pad_clearance(trace, &rect, required_clearance, policy),
        Err(_) => TraceClearanceReport {
            status: ClearanceStatus::Unknown,
            centerline_intersection: None,
            axis_gap: None,
        },
    }
}

/// Check exact clearance from a trace to an axis-aligned board outline.
///
/// The trace centerline must stay inside the rectangle shrunk by
/// `trace_width / 2 + required_clearance`. The implementation compares doubled
/// margins against `trace_width + 2 * clearance`, avoiding square roots and
/// avoiding a division in the predicate decision. This is the rectangular-board
/// analogue of exact swept trace clearance; arbitrary board contours should be
/// routed through exact polygon/arc arrangements.
pub fn check_trace_board_clearance(
    trace: &PcbTrace,
    board: &PcbBoardOutline,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> TraceClearanceReport {
    let Some(min_x) = real_min(
        &trace.swept.centerline().start().x,
        &trace.swept.centerline().end().x,
        policy,
    ) else {
        return unknown_clearance_report();
    };
    let Some(max_x) = real_max(
        &trace.swept.centerline().start().x,
        &trace.swept.centerline().end().x,
        policy,
    ) else {
        return unknown_clearance_report();
    };
    let Some(min_y) = real_min(
        &trace.swept.centerline().start().y,
        &trace.swept.centerline().end().y,
        policy,
    ) else {
        return unknown_clearance_report();
    };
    let Some(max_y) = real_max(
        &trace.swept.centerline().start().y,
        &trace.swept.centerline().end().y,
        policy,
    ) else {
        return unknown_clearance_report();
    };

    let margins = [
        min_x.to_owned() - board.min().x.clone(),
        board.max().x.clone() - max_x.to_owned(),
        min_y.to_owned() - board.min().y.clone(),
        board.max().y.clone() - max_y.to_owned(),
    ];
    let required_doubled = trace.swept.width().clone() + required_clearance.clone() * Real::from(2);
    match classify_doubled_margins(&margins, &required_doubled, policy) {
        Some((status, margin)) => TraceClearanceReport {
            status,
            centerline_intersection: None,
            axis_gap: Some(margin),
        },
        None => unknown_clearance_report(),
    }
}

/// Check exact clearance from a trace to a strictly convex board outline.
///
/// For each oriented edge, both trace endpoints must lie inside the half-plane
/// offset by `trace_width / 2 + required_clearance`. The predicate compares
/// squared signed parallelogram area against
/// `(trace_width + 2*clearance)^2 * edge_length_squared`, avoiding square roots
/// while retaining exact geometry. Convexity makes endpoint checks sufficient
/// for the whole straight segment.
pub fn check_trace_convex_board_clearance(
    trace: &PcbTrace,
    board: &PcbConvexBoardOutline,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> TraceClearanceReport {
    let required_doubled = trace.swept.width().clone() + required_clearance.clone() * Real::from(2);
    let required_squared = required_doubled.clone() * required_doubled;
    for edge_index in 0..board.vertices.len() {
        let edge_start = &board.vertices[edge_index];
        let edge_end = &board.vertices[(edge_index + 1) % board.vertices.len()];
        let edge_length_squared = squared_distance_points(edge_start, edge_end);
        for point in [
            trace.swept.centerline().start(),
            trace.swept.centerline().end(),
        ] {
            let signed = oriented_edge_side(edge_start, edge_end, point, board.orientation);
            match compare_reals_with_policy(&signed, &Real::zero(), policy).value() {
                Some(Ordering::Less) => {
                    return TraceClearanceReport {
                        status: ClearanceStatus::ClearanceViolation,
                        centerline_intersection: None,
                        axis_gap: None,
                    };
                }
                Some(Ordering::Equal | Ordering::Greater) => {}
                None => return unknown_clearance_report(),
            }
            let lhs = signed.clone() * signed * Real::from(4);
            let rhs = required_squared.clone() * edge_length_squared.clone();
            match compare_reals_with_policy(&lhs, &rhs, policy).value() {
                Some(Ordering::Less) => {
                    return TraceClearanceReport {
                        status: ClearanceStatus::ClearanceViolation,
                        centerline_intersection: None,
                        axis_gap: None,
                    };
                }
                Some(Ordering::Equal | Ordering::Greater) => {}
                None => return unknown_clearance_report(),
            }
        }
    }
    TraceClearanceReport {
        status: ClearanceStatus::CertifiedClear,
        centerline_intersection: None,
        axis_gap: None,
    }
}

/// Check exact clearance from an axis-aligned trace to an orthogonal board.
///
/// The centerline endpoints must be inside or on the retained simple polygon,
/// the centerline must not properly cross or overlap a board edge, and the
/// squared distance to every exact board edge must be at least
/// `(trace_width / 2 + clearance)^2`. This is a rectilinear-board specialization
/// of the arrangement stage described by Yap: point-in-polygon and segment-edge
/// predicates decide route validity exactly, while more general nonorthogonal
/// arrangements remain outside this carrier.
pub fn check_trace_orthogonal_board_clearance(
    trace: &PcbTrace,
    board: &PcbOrthogonalBoardOutline,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> TraceClearanceReport {
    if trace.facts().axis_aligned.is_none() {
        return unknown_clearance_report();
    }
    for point in [
        trace.swept.centerline().start(),
        trace.swept.centerline().end(),
    ] {
        match classify_point_in_orthogonal_polygon(point, board.vertices(), policy) {
            Some(OrthogonalPointLocation::Inside | OrthogonalPointLocation::Boundary) => {}
            Some(OrthogonalPointLocation::Outside) => {
                return TraceClearanceReport {
                    status: ClearanceStatus::ClearanceViolation,
                    centerline_intersection: None,
                    axis_gap: None,
                };
            }
            None => return unknown_clearance_report(),
        }
    }

    let trace_segment = trace.swept.centerline();
    let required_doubled = trace.swept.width().clone() + required_clearance.clone() * Real::from(2);
    let required_squared = required_doubled.clone() * required_doubled;
    let mut minimum_distance_squared: Option<Real> = None;
    for edge_index in 0..board.vertices.len() {
        let edge = LinePathSegment::new(
            board.vertices[edge_index].clone(),
            board.vertices[(edge_index + 1) % board.vertices.len()].clone(),
        );
        match classify_segment_intersection_with_facts(
            trace_segment.start(),
            trace_segment.end(),
            edge.start(),
            edge.end(),
            trace_segment.facts().segment,
            edge.facts().segment,
        )
        .value()
        {
            Some(SegmentIntersection::Proper | SegmentIntersection::CollinearOverlap) => {
                return TraceClearanceReport {
                    status: ClearanceStatus::ClearanceViolation,
                    centerline_intersection: None,
                    axis_gap: None,
                };
            }
            Some(
                SegmentIntersection::Disjoint
                | SegmentIntersection::EndpointTouch
                | SegmentIntersection::Identical,
            )
            | None => {}
        }
        let Some(distance_squared) =
            axis_aligned_segment_segment_distance_squared(trace_segment, &edge, policy)
        else {
            return unknown_clearance_report();
        };
        let replace_minimum = match minimum_distance_squared.as_ref() {
            Some(minimum) => matches!(
                compare_reals_with_policy(&distance_squared, minimum, policy).value(),
                Some(Ordering::Less)
            ),
            None => true,
        };
        if replace_minimum {
            minimum_distance_squared = Some(distance_squared);
        }
    }
    let Some(minimum_distance_squared) = minimum_distance_squared else {
        return unknown_clearance_report();
    };
    let lhs = minimum_distance_squared * Real::from(4);
    let status = match compare_reals_with_policy(&lhs, &required_squared, policy).value() {
        Some(Ordering::Less) => ClearanceStatus::ClearanceViolation,
        Some(Ordering::Equal | Ordering::Greater) => ClearanceStatus::CertifiedClear,
        None => ClearanceStatus::Unknown,
    };
    TraceClearanceReport {
        status,
        centerline_intersection: None,
        axis_gap: None,
    }
}

/// Check exact clearance from a circular pad to an axis-aligned board outline.
///
/// Round pad board-edge clearance is the same exact decision as via-drill
/// board clearance, but it is a copper rule rather than a fabrication-hole
/// rule. Following Yap's exact-predicate model, the candidate footprint shape
/// is retained as a circle and the decision compares doubled center margins
/// against `diameter + 2 * clearance`, avoiding square roots and tolerances.
pub fn check_circular_pad_board_clearance(
    pad: &PcbCircularPad,
    board: &PcbBoardOutline,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> PadBoardClearanceReport {
    let margins = [
        pad.center().x.clone() - board.min().x.clone(),
        board.max().x.clone() - pad.center().x.clone(),
        pad.center().y.clone() - board.min().y.clone(),
        board.max().y.clone() - pad.center().y.clone(),
    ];
    let required_doubled = pad.diameter().clone() + required_clearance.clone() * Real::from(2);
    let Some((status, minimum_center_margin)) =
        classify_doubled_margins(&margins, &required_doubled, policy)
    else {
        return unknown_pad_board_report();
    };
    let copper_gap = (pad.diameter().clone() / Real::from(2))
        .ok()
        .map(|radius| minimum_center_margin - radius);
    PadBoardClearanceReport { status, copper_gap }
}

/// Check exact clearance from an axis-aligned rectangular pad to a board.
///
/// This certifies common SMD pads against a rectangular board envelope by
/// comparing exact copper-edge margins to the requested clearance. Arbitrary
/// pad polygons and non-rectangular boards remain arrangement-kernel work; this
/// specialized predicate keeps the high-volume cardinal case exact and cheap.
pub fn check_rect_pad_board_clearance(
    pad: &PcbRectPad,
    board: &PcbBoardOutline,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> PadBoardClearanceReport {
    let Some((min_x, max_x, min_y, max_y)) = rect_pad_bounds(pad) else {
        return unknown_pad_board_report();
    };
    let margins = [
        min_x - board.min().x.clone(),
        board.max().x.clone() - max_x,
        min_y - board.min().y.clone(),
        board.max().y.clone() - max_y,
    ];
    match classify_margins(&margins, required_clearance, policy) {
        Some((status, copper_gap)) => PadBoardClearanceReport {
            status,
            copper_gap: Some(copper_gap),
        },
        None => unknown_pad_board_report(),
    }
}

/// Check exact clearance from a cardinally rotated rectangular pad to a board.
///
/// The cardinal transform is an exact extent swap, so this lowers to the
/// axis-aligned rectangular-pad predicate without trigonometric approximation.
pub fn check_cardinal_rect_pad_board_clearance(
    pad: &PcbCardinalRectPad,
    board: &PcbBoardOutline,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> PadBoardClearanceReport {
    match pad.effective_rect() {
        Ok(rect) => check_rect_pad_board_clearance(&rect, board, required_clearance, policy),
        Err(_) => unknown_pad_board_report(),
    }
}

/// Check exact drill clearance from a via to an axis-aligned board outline.
///
/// The drill center must stay inside the board rectangle by at least
/// `drill_diameter / 2 + required_clearance`. As with trace board-edge
/// clearance, this compares doubled exact margins against
/// `drill_diameter + 2 * clearance` and therefore does not introduce a
/// square-root or floating tolerance. The rule is intentionally rectangular;
/// arbitrary board contours belong in the later exact arrangement layer.
pub fn check_via_drill_board_clearance(
    via: &PcbViaStack,
    board: &PcbBoardOutline,
    required_clearance: &Real,
    policy: PredicatePolicy,
) -> DrillBoardClearanceReport {
    let Some(drill_diameter) = via.drill_diameter() else {
        return DrillBoardClearanceReport {
            status: ClearanceStatus::Unknown,
            axis_gap: None,
            missing_drill: true,
        };
    };
    let margins = [
        via.center().x.clone() - board.min().x.clone(),
        board.max().x.clone() - via.center().x.clone(),
        via.center().y.clone() - board.min().y.clone(),
        board.max().y.clone() - via.center().y.clone(),
    ];
    let required_doubled = drill_diameter.clone() + required_clearance.clone() * Real::from(2);
    let Some((status, minimum_margin)) =
        classify_doubled_margins(&margins, &required_doubled, policy)
    else {
        return DrillBoardClearanceReport {
            status: ClearanceStatus::Unknown,
            axis_gap: None,
            missing_drill: false,
        };
    };
    DrillBoardClearanceReport {
        status,
        axis_gap: Some(minimum_margin),
        missing_drill: false,
    }
}

fn unknown_pad_board_report() -> PadBoardClearanceReport {
    PadBoardClearanceReport {
        status: ClearanceStatus::Unknown,
        copper_gap: None,
    }
}

fn axis_aligned_gap(
    first: &LinePathSegment,
    second: &LinePathSegment,
    policy: PredicatePolicy,
) -> Option<Real> {
    match (first.facts().axis_aligned, second.facts().axis_aligned) {
        (Some(Axis::X), Some(Axis::X))
            if intervals_overlap(
                &first.start().x,
                &first.end().x,
                &second.start().x,
                &second.end().x,
                policy,
            )? =>
        {
            coordinate_gap(&first.start().y, &second.start().y, policy)
        }
        (Some(Axis::Y), Some(Axis::Y))
            if intervals_overlap(
                &first.start().y,
                &first.end().y,
                &second.start().y,
                &second.end().y,
                policy,
            )? =>
        {
            coordinate_gap(&first.start().x, &second.start().x, policy)
        }
        (Some(Axis::X), Some(Axis::Y)) | (Some(Axis::Y), Some(Axis::X)) => {
            // If centerlines did not intersect, perpendicular axis-aligned
            // segments are separated in at least one axis. Use the larger
            // certified axis gap as a conservative lower bound only when one
            // axis gap is zero and the other is known.
            let x_gap = interval_gap(
                &first.start().x,
                &first.end().x,
                &second.start().x,
                &second.end().x,
                policy,
            )?;
            let y_gap = interval_gap(
                &first.start().y,
                &first.end().y,
                &second.start().y,
                &second.end().y,
                policy,
            )?;
            if real_sign(&x_gap) == Some(hyperreal::RealSign::Zero) {
                Some(y_gap)
            } else if real_sign(&y_gap) == Some(hyperreal::RealSign::Zero) {
                Some(x_gap)
            } else {
                None
            }
        }
        _ => None,
    }
}

fn unknown_clearance_report() -> TraceClearanceReport {
    TraceClearanceReport {
        status: ClearanceStatus::Unknown,
        centerline_intersection: None,
        axis_gap: None,
    }
}

fn classify_doubled_margins(
    margins: &[Real; 4],
    required_doubled: &Real,
    policy: PredicatePolicy,
) -> Option<(ClearanceStatus, Real)> {
    let mut minimum_margin = margins[0].clone();
    for margin in margins {
        if matches!(
            compare_reals_with_policy(margin, &minimum_margin, policy).value(),
            Some(Ordering::Less)
        ) {
            minimum_margin = margin.clone();
        }
        let doubled_margin = margin.clone() * Real::from(2);
        match compare_reals_with_policy(&doubled_margin, required_doubled, policy).value()? {
            Ordering::Less => return Some((ClearanceStatus::ClearanceViolation, margin.clone())),
            Ordering::Equal | Ordering::Greater => {}
        }
    }
    Some((ClearanceStatus::CertifiedClear, minimum_margin))
}

fn polygon_signed_area_twice(vertices: &[Point2]) -> Real {
    let mut area = Real::zero();
    for index in 0..vertices.len() {
        let current = &vertices[index];
        let next = &vertices[(index + 1) % vertices.len()];
        area = area + current.x.clone() * next.y.clone() - next.x.clone() * current.y.clone();
    }
    area
}

fn validate_strict_convexity(
    vertices: &[Point2],
    orientation: BoardContourOrientation,
) -> Result<(), BoardContourError> {
    for index in 0..vertices.len() {
        let previous = &vertices[index];
        let current = &vertices[(index + 1) % vertices.len()];
        let next = &vertices[(index + 2) % vertices.len()];
        let cross = edge_cross(previous, current, next);
        let expected = match orientation {
            BoardContourOrientation::CounterClockwise => Ordering::Greater,
            BoardContourOrientation::Clockwise => Ordering::Less,
        };
        match compare_reals_with_policy(&cross, &Real::zero(), PredicatePolicy::default()).value() {
            Some(Ordering::Equal) => return Err(BoardContourError::CollinearEdge),
            Some(ordering) if ordering == expected => {}
            Some(_) => return Err(BoardContourError::NonConvex),
            None => return Err(BoardContourError::UnknownOrientation),
        }
    }
    Ok(())
}

fn validate_orthogonal_edges(vertices: &[Point2]) -> Result<(), BoardContourError> {
    for index in 0..vertices.len() {
        let start = &vertices[index];
        let end = &vertices[(index + 1) % vertices.len()];
        let same_x = compare_reals_with_policy(&start.x, &end.x, PredicatePolicy::default())
            .value()
            .ok_or(BoardContourError::UnknownOrientation)?
            == Ordering::Equal;
        let same_y = compare_reals_with_policy(&start.y, &end.y, PredicatePolicy::default())
            .value()
            .ok_or(BoardContourError::UnknownOrientation)?
            == Ordering::Equal;
        match (same_x, same_y) {
            (true, true) => return Err(BoardContourError::CollinearEdge),
            (true, false) | (false, true) => {}
            (false, false) => return Err(BoardContourError::NonOrthogonal),
        }
    }
    Ok(())
}

fn validate_simple_polygon_edges(vertices: &[Point2]) -> Result<(), BoardContourError> {
    let edges = polygon_edges(vertices);
    for first in 0..edges.len() {
        for second in (first + 1)..edges.len() {
            if polygon_edges_are_adjacent(first, second, edges.len()) {
                continue;
            }
            let intersection = classify_segment_intersection_with_facts(
                edges[first].start(),
                edges[first].end(),
                edges[second].start(),
                edges[second].end(),
                edges[first].facts().segment,
                edges[second].facts().segment,
            )
            .value()
            .ok_or(BoardContourError::UnknownOrientation)?;
            if !matches!(intersection, SegmentIntersection::Disjoint) {
                return Err(BoardContourError::SelfIntersecting);
            }
        }
    }
    Ok(())
}

fn polygon_edges(vertices: &[Point2]) -> Vec<LinePathSegment> {
    (0..vertices.len())
        .map(|index| {
            LinePathSegment::new(
                vertices[index].clone(),
                vertices[(index + 1) % vertices.len()].clone(),
            )
        })
        .collect()
}

fn polygon_edges_are_adjacent(first: usize, second: usize, edge_count: usize) -> bool {
    first + 1 == second || (first == 0 && second + 1 == edge_count)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum OrthogonalPointLocation {
    Inside,
    Boundary,
    Outside,
}

fn classify_point_in_orthogonal_polygon(
    point: &Point2,
    vertices: &[Point2],
    policy: PredicatePolicy,
) -> Option<OrthogonalPointLocation> {
    let mut inside = false;
    for index in 0..vertices.len() {
        let start = &vertices[index];
        let end = &vertices[(index + 1) % vertices.len()];
        if point_on_axis_aligned_segment(point, start, end, policy)? {
            return Some(OrthogonalPointLocation::Boundary);
        }
        if compare_reals_with_policy(&start.x, &end.x, policy).value()? != Ordering::Equal {
            continue;
        }
        let y_min = real_min(&start.y, &end.y, policy)?;
        let y_max = real_max(&start.y, &end.y, policy)?;
        let crosses_lower = !matches!(
            compare_reals_with_policy(&point.y, y_min, policy).value()?,
            Ordering::Less
        );
        let crosses_upper = matches!(
            compare_reals_with_policy(&point.y, y_max, policy).value()?,
            Ordering::Less
        );
        let right_of_point = matches!(
            compare_reals_with_policy(&start.x, &point.x, policy).value()?,
            Ordering::Greater
        );
        if crosses_lower && crosses_upper && right_of_point {
            inside = !inside;
        }
    }
    Some(if inside {
        OrthogonalPointLocation::Inside
    } else {
        OrthogonalPointLocation::Outside
    })
}

fn point_on_axis_aligned_segment(
    point: &Point2,
    start: &Point2,
    end: &Point2,
    policy: PredicatePolicy,
) -> Option<bool> {
    let same_x = compare_reals_with_policy(&start.x, &end.x, policy).value()? == Ordering::Equal;
    let same_y = compare_reals_with_policy(&start.y, &end.y, policy).value()? == Ordering::Equal;
    if same_x {
        let point_same_x =
            compare_reals_with_policy(&point.x, &start.x, policy).value()? == Ordering::Equal;
        return Some(point_same_x && interval_contains_point(&start.y, &end.y, &point.y, policy)?);
    }
    if same_y {
        let point_same_y =
            compare_reals_with_policy(&point.y, &start.y, policy).value()? == Ordering::Equal;
        return Some(point_same_y && interval_contains_point(&start.x, &end.x, &point.x, policy)?);
    }
    Some(false)
}

fn oriented_edge_side(
    edge_start: &Point2,
    edge_end: &Point2,
    point: &Point2,
    orientation: BoardContourOrientation,
) -> Real {
    let cross = edge_cross(edge_start, edge_end, point);
    match orientation {
        BoardContourOrientation::CounterClockwise => cross,
        BoardContourOrientation::Clockwise => -cross,
    }
}

fn edge_cross(a: &Point2, b: &Point2, c: &Point2) -> Real {
    let ab_x = b.x.clone() - a.x.clone();
    let ab_y = b.y.clone() - a.y.clone();
    let ac_x = c.x.clone() - a.x.clone();
    let ac_y = c.y.clone() - a.y.clone();
    ab_x * ac_y - ab_y * ac_x
}

fn squared_distance_points(a: &Point2, b: &Point2) -> Real {
    let dx = b.x.clone() - a.x.clone();
    let dy = b.y.clone() - a.y.clone();
    dx.clone() * dx + dy.clone() * dy
}

fn classify_margins(
    margins: &[Real; 4],
    required: &Real,
    policy: PredicatePolicy,
) -> Option<(ClearanceStatus, Real)> {
    let mut minimum_margin = margins[0].clone();
    for margin in margins {
        if matches!(
            compare_reals_with_policy(margin, &minimum_margin, policy).value(),
            Some(Ordering::Less)
        ) {
            minimum_margin = margin.clone();
        }
        match compare_reals_with_policy(margin, required, policy).value()? {
            Ordering::Less => return Some((ClearanceStatus::ClearanceViolation, margin.clone())),
            Ordering::Equal | Ordering::Greater => {}
        }
    }
    Some((ClearanceStatus::CertifiedClear, minimum_margin))
}

fn intervals_overlap(
    a0: &Real,
    a1: &Real,
    b0: &Real,
    b1: &Real,
    policy: PredicatePolicy,
) -> Option<bool> {
    let lower = real_max(real_min(a0, a1, policy)?, real_min(b0, b1, policy)?, policy)?;
    let upper = real_min(real_max(a0, a1, policy)?, real_max(b0, b1, policy)?, policy)?;
    Some(!matches!(
        compare_reals_with_policy(lower, upper, policy).value()?,
        Ordering::Greater
    ))
}

fn interval_contains_point(
    a0: &Real,
    a1: &Real,
    point: &Real,
    policy: PredicatePolicy,
) -> Option<bool> {
    let min = real_min(a0, a1, policy)?;
    let max = real_max(a0, a1, policy)?;
    Some(
        !matches!(
            compare_reals_with_policy(point, min, policy).value()?,
            Ordering::Less
        ) && !matches!(
            compare_reals_with_policy(point, max, policy).value()?,
            Ordering::Greater
        ),
    )
}

fn interval_gap(
    a0: &Real,
    a1: &Real,
    b0: &Real,
    b1: &Real,
    policy: PredicatePolicy,
) -> Option<Real> {
    if intervals_overlap(a0, a1, b0, b1, policy)? {
        return Some(Real::zero());
    }
    let a_max = real_max(a0, a1, policy)?;
    let b_min = real_min(b0, b1, policy)?;
    if compare_reals_with_policy(a_max, b_min, policy).value()? == Ordering::Less {
        return Some(b_min.clone() - a_max.clone());
    }
    let b_max = real_max(b0, b1, policy)?;
    let a_min = real_min(a0, a1, policy)?;
    Some(a_min.clone() - b_max.clone())
}

fn coordinate_gap(first: &Real, second: &Real, policy: PredicatePolicy) -> Option<Real> {
    match compare_reals_with_policy(first, second, policy).value()? {
        Ordering::Less | Ordering::Equal => Some(second.clone() - first.clone()),
        Ordering::Greater => Some(first.clone() - second.clone()),
    }
}

fn axis_aligned_point_segment_distance_squared(
    segment: &LinePathSegment,
    point: &Point2,
    policy: PredicatePolicy,
) -> Option<Real> {
    let (axis_gap, cross_gap) = match segment.facts().axis_aligned {
        Some(Axis::X) => (
            interval_point_gap(&segment.start().x, &segment.end().x, &point.x, policy)?,
            coordinate_gap(&segment.start().y, &point.y, policy)?,
        ),
        Some(Axis::Y) => (
            interval_point_gap(&segment.start().y, &segment.end().y, &point.y, policy)?,
            coordinate_gap(&segment.start().x, &point.x, policy)?,
        ),
        None => return None,
    };
    Some(Real::signed_product_sum(
        [true, true],
        [[&axis_gap, &axis_gap], [&cross_gap, &cross_gap]],
    ))
}

fn interval_point_gap(a0: &Real, a1: &Real, point: &Real, policy: PredicatePolicy) -> Option<Real> {
    let min = real_min(a0, a1, policy)?;
    let max = real_max(a0, a1, policy)?;
    if compare_reals_with_policy(point, min, policy).value()? == Ordering::Less {
        return Some(min.clone() - point.clone());
    }
    if compare_reals_with_policy(point, max, policy).value()? == Ordering::Greater {
        return Some(point.clone() - max.clone());
    }
    Some(Real::zero())
}

fn axis_aligned_segment_rect_distance_squared(
    segment: &LinePathSegment,
    pad: &PcbRectPad,
    policy: PredicatePolicy,
) -> Option<Real> {
    let half_width = (pad.width().clone() / Real::from(2)).ok()?;
    let half_height = (pad.height().clone() / Real::from(2)).ok()?;
    let min_x = pad.center().x.clone() - half_width.clone();
    let max_x = pad.center().x.clone() + half_width;
    let min_y = pad.center().y.clone() - half_height.clone();
    let max_y = pad.center().y.clone() + half_height;

    let x_gap = interval_gap(&segment.start().x, &segment.end().x, &min_x, &max_x, policy)?;
    let y_gap = interval_gap(&segment.start().y, &segment.end().y, &min_y, &max_y, policy)?;
    Some(Real::signed_product_sum(
        [true, true],
        [[&x_gap, &x_gap], [&y_gap, &y_gap]],
    ))
}

fn axis_aligned_segment_segment_distance_squared(
    first: &LinePathSegment,
    second: &LinePathSegment,
    policy: PredicatePolicy,
) -> Option<Real> {
    if first.facts().axis_aligned.is_none() || second.facts().axis_aligned.is_none() {
        return None;
    }
    let x_gap = interval_gap(
        &first.start().x,
        &first.end().x,
        &second.start().x,
        &second.end().x,
        policy,
    )?;
    let y_gap = interval_gap(
        &first.start().y,
        &first.end().y,
        &second.start().y,
        &second.end().y,
        policy,
    )?;
    Some(Real::signed_product_sum(
        [true, true],
        [[&x_gap, &x_gap], [&y_gap, &y_gap]],
    ))
}

fn rect_pad_bounds(pad: &PcbRectPad) -> Option<(Real, Real, Real, Real)> {
    let half_width = (pad.width().clone() / Real::from(2)).ok()?;
    let half_height = (pad.height().clone() / Real::from(2)).ok()?;
    Some((
        pad.center().x.clone() - half_width.clone(),
        pad.center().x.clone() + half_width,
        pad.center().y.clone() - half_height.clone(),
        pad.center().y.clone() + half_height,
    ))
}

fn real_min<'a>(left: &'a Real, right: &'a Real, policy: PredicatePolicy) -> Option<&'a Real> {
    match compare_reals_with_policy(left, right, policy).value()? {
        Ordering::Less | Ordering::Equal => Some(left),
        Ordering::Greater => Some(right),
    }
}

fn real_max<'a>(left: &'a Real, right: &'a Real, policy: PredicatePolicy) -> Option<&'a Real> {
    match compare_reals_with_policy(left, right, policy).value()? {
        Ordering::Less | Ordering::Equal => Some(right),
        Ordering::Greater => Some(left),
    }
}
