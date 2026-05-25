//! Retained planar pcurve image-equality evidence.
//!
//! BREP trims are usually carried as parameter-space curves on a supporting
//! surface. For planar faces, the first exact question is not a sampled 3D
//! proximity test: it is whether two pcurves lie on the same retained planar
//! surface and replay the same UV image. This module keeps that evidence
//! explicit, following Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 3-23 (1997), and the pcurve-on-surface
//! representation used in Piegl and Tiller, *The NURBS Book* (2nd ed., 1997).

use crate::{
    Classification, Contour2, CurveError, CurvePolicy, CurveResult, CurveString2, Point2,
    PreparedRegionView2, RegionPointLocation, RegionView2, Segment2, UncertaintyReason,
};

/// Opaque identity of a retained planar support surface.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RetainedPlanarSurfaceIdentity2 {
    source_index: u64,
}

/// Exact image relation between two retained planar pcurves.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PlanarPcurveImageRelation2 {
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
pub struct PlanarPcurveImageEqualityReport2 {
    relation: PlanarPcurveImageRelation2,
    surface: Option<RetainedPlanarSurfaceIdentity2>,
    segment_count: usize,
}

/// Open retained pcurve on a planar support surface.
#[derive(Clone, Debug, PartialEq)]
pub struct RetainedPlanarPcurve2 {
    surface: RetainedPlanarSurfaceIdentity2,
    curve: CurveString2,
}

/// Closed retained trim-loop pcurve on a planar support surface.
#[derive(Clone, Debug, PartialEq)]
pub struct RetainedPlanarTrimLoop2 {
    surface: RetainedPlanarSurfaceIdentity2,
    contour: Contour2,
}

/// Retained planar face assembled from material and hole pcurve trim loops.
#[derive(Clone, Debug, PartialEq)]
pub struct RetainedPlanarFace2 {
    surface: RetainedPlanarSurfaceIdentity2,
    material_loops: Vec<RetainedPlanarTrimLoop2>,
    hole_loops: Vec<RetainedPlanarTrimLoop2>,
}

/// Prepared retained planar face for repeated support-surface and UV queries.
///
/// The prepared object keeps the retained BREP support identity beside a
/// prepared borrowed UV region. Cached boxes and prepared segment predicates
/// are only broad-phase evidence: support-surface mismatch, boundary hits, and
/// inside/outside status still replay through the exact classifiers. That
/// separation follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7(1-2), 3-23 (1997), and the pcurve-on-surface
/// face model in Piegl and Tiller, *The NURBS Book* (2nd ed., 1997).
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedRetainedPlanarFace2<'a> {
    face: &'a RetainedPlanarFace2,
    region: PreparedRegionView2<'a>,
}

/// Point classification result for a retained planar face.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RetainedPlanarFacePointLocation2 {
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
pub struct RetainedPlanarFacePointReport2 {
    location: RetainedPlanarFacePointLocation2,
    surface: Option<RetainedPlanarSurfaceIdentity2>,
    material_loop_count: usize,
    hole_loop_count: usize,
}

impl RetainedPlanarSurfaceIdentity2 {
    /// Constructs an opaque retained planar surface identity.
    pub const fn new(source_index: u64) -> Self {
        Self { source_index }
    }

    /// Returns the opaque source index for this planar support surface.
    pub const fn source_index(self) -> u64 {
        self.source_index
    }
}

impl PlanarPcurveImageRelation2 {
    /// Returns true when the reports certify equal UV images.
    pub const fn is_same_image(self) -> bool {
        matches!(self, Self::SameDirected | Self::SameReversed)
    }

    /// Returns true when equal images have opposite traversal orientation.
    pub const fn is_reversed(self) -> bool {
        matches!(self, Self::SameReversed)
    }
}

impl PlanarPcurveImageEqualityReport2 {
    /// Constructs a planar pcurve image-equality report.
    pub const fn new(
        relation: PlanarPcurveImageRelation2,
        surface: Option<RetainedPlanarSurfaceIdentity2>,
        segment_count: usize,
    ) -> Self {
        Self {
            relation,
            surface,
            segment_count,
        }
    }

    /// Returns the certified relation.
    pub const fn relation(&self) -> PlanarPcurveImageRelation2 {
        self.relation
    }

    /// Returns the common retained surface when both pcurves share one.
    pub const fn surface(&self) -> Option<RetainedPlanarSurfaceIdentity2> {
        self.surface
    }

    /// Returns the segment count in the compared UV image when it matched.
    pub const fn segment_count(&self) -> usize {
        self.segment_count
    }
}

impl RetainedPlanarPcurve2 {
    /// Constructs an open retained planar pcurve.
    pub const fn new(surface: RetainedPlanarSurfaceIdentity2, curve: CurveString2) -> Self {
        Self { surface, curve }
    }

    /// Returns the retained planar surface identity.
    pub const fn surface(&self) -> RetainedPlanarSurfaceIdentity2 {
        self.surface
    }

    /// Returns the retained UV curve string.
    pub const fn curve(&self) -> &CurveString2 {
        &self.curve
    }

    /// Compares two open planar pcurves by exact UV image.
    ///
    /// This is a structural exact predicate over already split native segments:
    /// equal images must have identical segment boundaries in UV, either in
    /// the same order or in exact reverse order. It deliberately does not
    /// sample or merge unsplit overlaps; those remain later trim-splitting
    /// work under Yap's construction/predicate boundary.
    pub fn image_equality_report(&self, other: &Self) -> PlanarPcurveImageEqualityReport2 {
        if self.surface != other.surface {
            return PlanarPcurveImageEqualityReport2::new(
                PlanarPcurveImageRelation2::SurfaceMismatch,
                None,
                0,
            );
        }
        let relation = if same_directed_segments(self.curve.segments(), other.curve.segments()) {
            PlanarPcurveImageRelation2::SameDirected
        } else if same_reversed_segments(self.curve.segments(), other.curve.segments()) {
            PlanarPcurveImageRelation2::SameReversed
        } else {
            PlanarPcurveImageRelation2::Different
        };
        let segment_count = usize::from(relation.is_same_image()) * self.curve.len();
        PlanarPcurveImageEqualityReport2::new(relation, Some(self.surface), segment_count)
    }
}

impl RetainedPlanarTrimLoop2 {
    /// Constructs a closed retained planar trim-loop pcurve.
    pub const fn new(surface: RetainedPlanarSurfaceIdentity2, contour: Contour2) -> Self {
        Self { surface, contour }
    }

    /// Returns the retained planar surface identity.
    pub const fn surface(&self) -> RetainedPlanarSurfaceIdentity2 {
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
    pub fn image_equality_report(&self, other: &Self) -> PlanarPcurveImageEqualityReport2 {
        if self.surface != other.surface {
            return PlanarPcurveImageEqualityReport2::new(
                PlanarPcurveImageRelation2::SurfaceMismatch,
                None,
                0,
            );
        }
        let relation =
            if same_directed_segment_cycle(self.contour.segments(), other.contour.segments()) {
                PlanarPcurveImageRelation2::SameDirected
            } else if same_reversed_segment_cycle(self.contour.segments(), other.contour.segments())
            {
                PlanarPcurveImageRelation2::SameReversed
            } else {
                PlanarPcurveImageRelation2::Different
            };
        let segment_count = usize::from(relation.is_same_image()) * self.contour.len();
        PlanarPcurveImageEqualityReport2::new(relation, Some(self.surface), segment_count)
    }
}

impl RetainedPlanarFace2 {
    /// Constructs a retained planar face from material and hole trim loops.
    ///
    /// Every trim loop must reference the same retained planar support surface.
    /// This validates the support-surface part of the BREP face before any
    /// point-in-face predicate is allowed to consume UV topology. That is the
    /// construction/predicate boundary from Yap, "Towards Exact Geometric
    /// Computation" (1997), applied to planar pcurves as described by Piegl
    /// and Tiller, *The NURBS Book* (2nd ed., 1997).
    pub fn try_new(
        surface: RetainedPlanarSurfaceIdentity2,
        material_loops: Vec<RetainedPlanarTrimLoop2>,
        hole_loops: Vec<RetainedPlanarTrimLoop2>,
    ) -> CurveResult<Self> {
        if material_loops.is_empty() {
            return Err(CurveError::InvalidPlanarFace);
        }
        if material_loops
            .iter()
            .chain(hole_loops.iter())
            .any(|trim| trim.surface != surface)
        {
            return Err(CurveError::InvalidPlanarFace);
        }
        Ok(Self {
            surface,
            material_loops,
            hole_loops,
        })
    }

    /// Returns the retained planar support surface.
    pub const fn surface(&self) -> RetainedPlanarSurfaceIdentity2 {
        self.surface
    }

    /// Returns material trim loops.
    pub fn material_loops(&self) -> &[RetainedPlanarTrimLoop2] {
        &self.material_loops
    }

    /// Returns hole trim loops.
    pub fn hole_loops(&self) -> &[RetainedPlanarTrimLoop2] {
        &self.hole_loops
    }

    /// Prepares this face for repeated support-surface and UV point queries.
    ///
    /// Preparation borrows the retained trim loops and caches the UV
    /// [`PreparedRegionView2`] used by repeated point-in-face calls. It does
    /// not certify any query by itself; every call still first checks the
    /// retained support-surface identity and then delegates to the exact
    /// boundary-first region classifier.
    pub fn prepare_point_queries(&self, policy: &CurvePolicy) -> PreparedRetainedPlanarFace2<'_> {
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
        PreparedRetainedPlanarFace2 {
            face: self,
            region: PreparedRegionView2::from_region_view(&region, policy),
        }
    }

    /// Prepares this face for repeated retained topology queries.
    ///
    /// This currently exposes the same point-query package as
    /// [`RetainedPlanarFace2::prepare_point_queries`]. Segment/edge-use and
    /// analytic-surface frame packages can extend the prepared face handle
    /// without changing the support-surface-first report contract.
    pub fn prepare_topology_queries(
        &self,
        policy: &CurvePolicy,
    ) -> PreparedRetainedPlanarFace2<'_> {
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
        query_surface: RetainedPlanarSurfaceIdentity2,
        uv: &Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RetainedPlanarFacePointReport2>> {
        if query_surface != self.surface {
            return Ok(Classification::Decided(
                RetainedPlanarFacePointReport2::new(
                    RetainedPlanarFacePointLocation2::SurfaceMismatch,
                    None,
                    self.material_loops.len(),
                    self.hole_loops.len(),
                ),
            ));
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
}

impl<'a> PreparedRetainedPlanarFace2<'a> {
    /// Returns the retained planar face that supplied this prepared view.
    pub const fn face(&self) -> &'a RetainedPlanarFace2 {
        self.face
    }

    /// Returns the retained planar support surface.
    pub const fn surface(&self) -> RetainedPlanarSurfaceIdentity2 {
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
    /// The support-surface identity check intentionally stays outside the
    /// prepared UV region. In Yap's EGC terms, preparation only retains
    /// reusable object structure; it does not turn a query against the wrong
    /// supporting surface into a geometric predicate.
    pub fn classify_uv_point(
        &self,
        query_surface: RetainedPlanarSurfaceIdentity2,
        uv: &Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RetainedPlanarFacePointReport2>> {
        if query_surface != self.face.surface {
            return Ok(Classification::Decided(
                RetainedPlanarFacePointReport2::new(
                    RetainedPlanarFacePointLocation2::SurfaceMismatch,
                    None,
                    self.material_loop_count(),
                    self.hole_loop_count(),
                ),
            ));
        }

        face_point_report_from_region_classification(
            self.region.classify_point(uv, policy),
            self.face.surface,
            self.material_loop_count(),
            self.hole_loop_count(),
        )
    }
}

impl RetainedPlanarFacePointLocation2 {
    /// Returns true when the query reached an exact inside/outside/boundary result.
    pub const fn is_trim_classification(self) -> bool {
        !matches!(self, Self::SurfaceMismatch)
    }
}

impl RetainedPlanarFacePointReport2 {
    /// Constructs a retained planar face point-query report.
    pub const fn new(
        location: RetainedPlanarFacePointLocation2,
        surface: Option<RetainedPlanarSurfaceIdentity2>,
        material_loop_count: usize,
        hole_loop_count: usize,
    ) -> Self {
        Self {
            location,
            surface,
            material_loop_count,
            hole_loop_count,
        }
    }

    /// Returns the exact query location or blocker.
    pub const fn location(&self) -> RetainedPlanarFacePointLocation2 {
        self.location
    }

    /// Returns the matching retained surface when the query reached trim classification.
    pub const fn surface(&self) -> Option<RetainedPlanarSurfaceIdentity2> {
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

fn same_directed_segments(first: &[Segment2], second: &[Segment2]) -> bool {
    first == second
}

fn same_reversed_segments(first: &[Segment2], second: &[Segment2]) -> bool {
    first.len() == second.len()
        && first
            .iter()
            .zip(second.iter().rev())
            .all(|(left, right)| left == &right.reversed())
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

fn face_point_report_from_region_classification(
    classification: Classification<RegionPointLocation>,
    surface: RetainedPlanarSurfaceIdentity2,
    material_loop_count: usize,
    hole_loop_count: usize,
) -> CurveResult<Classification<RetainedPlanarFacePointReport2>> {
    let location = match classification {
        Classification::Decided(RegionPointLocation::Outside) => {
            RetainedPlanarFacePointLocation2::Outside
        }
        Classification::Decided(RegionPointLocation::Boundary) => {
            RetainedPlanarFacePointLocation2::Boundary
        }
        Classification::Decided(RegionPointLocation::Inside) => {
            RetainedPlanarFacePointLocation2::Inside
        }
        Classification::Uncertain(UncertaintyReason::Boundary) => {
            RetainedPlanarFacePointLocation2::Boundary
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(Classification::Decided(
        RetainedPlanarFacePointReport2::new(
            location,
            Some(surface),
            material_loop_count,
            hole_loop_count,
        ),
    ))
}
