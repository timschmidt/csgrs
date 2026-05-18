//! Area certificates for native contour and region objects.
//!
//! The reports in this module keep exact area facts attached to the object
//! that produced them. That follows Yap's exact-geometric-computation
//! discipline: topology and metric decisions should expose the certified facts
//! used for branching instead of hiding them behind an approximate scalar.
//! See Yap, "Towards Exact Geometric Computation," *Computational Geometry*
//! 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).

use crate::{CurvePolicy, Real};

/// Reason a contour segment cannot yet contribute an exact signed area.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContourAreaUnsupportedReason {
    /// The circular arc was constructed from center/sweep state without the
    /// CAD bulge value needed to form an exact `4 * atan(bulge)` sweep.
    CenterOnlyArcSweepAngle,
}

/// Unsupported contour segment recorded while building an area report.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ContourAreaUnsupportedSegment2 {
    /// Segment index in contour traversal order.
    pub segment_index: usize,
    /// Why this segment cannot currently provide an exact contribution.
    pub reason: ContourAreaUnsupportedReason,
}

/// Exact signed-area report for a native closed contour.
///
/// When [`ContourSignedAreaReport2::signed_area`] is `Some`, every segment has
/// contributed exactly to `1/2 * integral(x dy - y dx)`. When it is `None`, the
/// report still records the supported segment classes and every unsupported
/// segment. This keeps unsupported sweep-angle work visible instead of
/// silently approximating it.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourSignedAreaReport2 {
    /// Exact signed contour area when every segment is supported.
    pub signed_area: Option<Real>,
    /// Number of segments examined.
    pub segment_count: usize,
    /// Number of exact line-segment contributions.
    pub line_segment_count: usize,
    /// Number of exact bulge-arc contributions.
    pub bulge_arc_segment_count: usize,
    /// Segments that could not contribute exact area with the current model.
    pub unsupported_segments: Vec<ContourAreaUnsupportedSegment2>,
}

impl ContourSignedAreaReport2 {
    /// Creates an empty report accumulator.
    pub(crate) fn empty() -> Self {
        Self {
            signed_area: Some(Real::zero()),
            segment_count: 0,
            line_segment_count: 0,
            bulge_arc_segment_count: 0,
            unsupported_segments: Vec::new(),
        }
    }

    /// Returns true when every segment contributed to the exact area.
    pub fn is_complete(&self) -> bool {
        self.signed_area.is_some() && self.unsupported_segments.is_empty()
    }
}

/// Region contour role used by filled-area reports.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RegionAreaContourRole {
    /// Additive material contour.
    Material,
    /// Subtractive hole contour.
    Hole,
}

/// Unsupported contour recorded while building a region filled-area report.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionAreaUnsupportedContour2 {
    /// Region role of the contour.
    pub role: RegionAreaContourRole,
    /// Index inside the role-specific contour bin.
    pub contour_index: usize,
    /// Contour-level area report carrying the unsupported segment details.
    pub contour_report: ContourSignedAreaReport2,
}

/// Exact filled-area report for a material/hole region.
///
/// Material contours add their certified absolute area and hole contours
/// subtract it; contour orientation is deliberately not treated as topology.
/// If [`RegionFilledAreaReport2::filled_area`] is `None`, at least one contour
/// lacked an exact segment contribution. In that case
/// [`RegionFilledAreaReport2::material_area`] and
/// [`RegionFilledAreaReport2::hole_area`] are the certified totals for the
/// supported contours only, and [`RegionFilledAreaReport2::unsupported_contours`]
/// lists the missing pieces.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionFilledAreaReport2 {
    /// Exact role-normalized filled area when every contour is supported.
    pub filled_area: Option<Real>,
    /// Certified additive area from supported material contours.
    pub material_area: Real,
    /// Certified subtractive area magnitude from supported hole contours.
    pub hole_area: Real,
    /// Number of material contours examined.
    pub material_contour_count: usize,
    /// Number of hole contours examined.
    pub hole_contour_count: usize,
    /// Contours that could not contribute exact area.
    pub unsupported_contours: Vec<RegionAreaUnsupportedContour2>,
    /// Policy used to certify area sign decisions.
    pub construction_policy: CurvePolicy,
}

impl RegionFilledAreaReport2 {
    /// Returns true when all region contours contributed exact area.
    pub fn is_complete(&self) -> bool {
        self.filled_area.is_some() && self.unsupported_contours.is_empty()
    }
}
