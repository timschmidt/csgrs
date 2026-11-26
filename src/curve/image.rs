//! Direct raster-boundary construction for native filled curve topology.
//!
//! Every solid pixel is an exact unit cell on an integer lattice. The tracer
//! emits only cell edges adjacent to empty space, joins those directed edges
//! into closed loops, and hands the loops directly to Hypercurve. No SVG
//! string, parser, or floating-point coordinate conversion participates in the
//! topology decision.

use std::collections::BTreeMap;

use crate::hyper_math::hreal_sign;
use hypercurve::{
    Contour2, CurveError, CurveRegion2, ExactCurveError, LineSeg2, Point2, Segment2,
};
use hyperreal::{Real, RealSign};
use image::GrayImage;

/// Failure while converting a raster into exact integer-grid contours.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum RasterTraceError {
    /// A traced boundary did not close, indicating inconsistent edge topology.
    #[error("raster boundary is open at grid vertex ({x}, {y})")]
    OpenBoundary {
        /// Grid x coordinate where traversal stopped.
        x: u32,
        /// Grid y coordinate where traversal stopped.
        y: u32,
    },
    /// Hypercurve rejected an exact contour assembled from the raster boundary.
    #[error(transparent)]
    Curve(
        /// Native contour or segment cause, exposed through the error source.
        #[from]
        CurveError,
    ),
    /// Hypercurve rejected explicit material/hole topology.
    #[error(transparent)]
    ExactCurve(
        /// Native filled-region admission cause, exposed through the error source.
        #[from]
        ExactCurveError,
    ),
    /// A closed boundary had no decidable nonzero orientation.
    #[error("raster contour orientation is zero, unsupported, or uncertain")]
    UncertainContourOrientation,
}

/// Report for direct integer-grid raster tracing.
///
/// Counts describe the input cells and emitted boundary work before native
/// admission. They are not final segment counts or resource budgets. Exactness
/// is relative to the thresholded cell model, not the scene represented by the
/// raster. [`super::from_image`] consumes this internal report and returns only
/// its region.
#[derive(Clone, Debug)]
pub struct RasterTraceReport {
    /// Native filled region after loop orientation and material/hole admission.
    region: CurveRegion2,
    /// Source width in pixels, including empty columns.
    width: u32,
    /// Source height in pixels, including empty rows.
    height: u32,
    /// Number of source pixels whose grayscale value met the inclusive threshold.
    solid_pixel_count: usize,
    /// Exposed unit edges before collinear runs are simplified.
    boundary_edge_count: usize,
    /// Number of traced loops submitted to native contour/region admission.
    contour_count: usize,
}

impl RasterTraceReport {
    /// Return the constructed exact-grid region.
    pub const fn region(&self) -> &CurveRegion2 {
        &self.region
    }

    /// Consume the report and return its region, discarding dimensions and work counts.
    pub fn into_region(self) -> CurveRegion2 {
        self.region
    }

    /// Return the source raster width.
    pub const fn width(&self) -> u32 {
        self.width
    }

    /// Return the source raster height.
    pub const fn height(&self) -> u32 {
        self.height
    }

    /// Return the number of pixels at or above the threshold.
    pub const fn solid_pixel_count(&self) -> usize {
        self.solid_pixel_count
    }

    /// Return exposed unit-edge count before collinear simplification.
    /// A long straight run contributes one count per pixel edge, not one per final segment.
    pub const fn boundary_edge_count(&self) -> usize {
        self.boundary_edge_count
    }

    /// Return the number of closed loops handed to Hypercurve.
    pub const fn contour_count(&self) -> usize {
        self.contour_count
    }
}

/// Integer pixel corner used for exact endpoint identity and outgoing-edge lookup.
/// Coordinates keep the raster's row direction; no Y reflection is applied.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct GridPoint {
    /// Horizontal cell-corner coordinate, possibly at the image's right edge.
    x: u32,
    /// Vertical cell-corner coordinate, possibly at the image's final row boundary.
    y: u32,
}

/// Directed exposed cell edge with selected material on its left in numeric XY coordinates.
/// The raster emitter supplies distinct axis-aligned unit endpoints.
#[derive(Clone, Copy, Debug)]
struct GridEdge {
    /// Traversal origin used as the outgoing-map key.
    start: GridPoint,
    /// Traversal destination where the next unused edge is selected.
    end: GridPoint,
}

/// Trace solid pixels directly into exact Hypercurve boundary topology.
///
/// Pixels whose grayscale value is at least `threshold` are closed unit
/// cells. The report retains source dimensions and boundary-work counts.
/// Emits exposed edges in row-major pixel order, then top/right/bottom/left
/// order per cell. Tracing separates diagonal contacts into loops; signed area
/// selects material versus hole roles before native region admission. Empty
/// images or no selected pixels produce an empty region and zero work counts.
///
/// Zero, unavailable or undecided contour orientation shares the typed
/// `UncertainContourOrientation` outcome. Curve construction/admission causes
/// remain wrapped separately. Buffer growth uses ordinary collections here;
/// counts are not a checked allocation, cancellation or total-work contract.
pub(crate) fn try_from_image(
    img: &GrayImage,
    threshold: u8,
) -> Result<RasterTraceReport, RasterTraceError> {
    let mut edges = Vec::new();
    let mut solid_pixel_count = 0usize;

    for y in 0..img.height() {
        for x in 0..img.width() {
            if !pixel_is_solid(img, x, y, threshold) {
                continue;
            }
            solid_pixel_count += 1;

            let x1 = x + 1;
            let y1 = y + 1;
            if y == 0 || !pixel_is_solid(img, x, y - 1, threshold) {
                edges.push(GridEdge::new(x, y, x1, y));
            }
            if x1 == img.width() || !pixel_is_solid(img, x1, y, threshold) {
                edges.push(GridEdge::new(x1, y, x1, y1));
            }
            if y1 == img.height() || !pixel_is_solid(img, x, y1, threshold) {
                edges.push(GridEdge::new(x1, y1, x, y1));
            }
            if x == 0 || !pixel_is_solid(img, x - 1, y, threshold) {
                edges.push(GridEdge::new(x, y1, x, y));
            }
        }
    }

    let boundary_edge_count = edges.len();
    let loops = trace_boundary_loops(&edges)?;
    let contour_count = loops.len();
    let mut material = Vec::new();
    let mut holes = Vec::new();
    for points in loops {
        let contour = grid_loop_to_contour(points)?;
        let area = contour
            .signed_area()?
            .ok_or(RasterTraceError::UncertainContourOrientation)?;
        match hreal_sign(&area) {
            Some(RealSign::Positive) => material.push(contour),
            Some(RealSign::Negative) => holes.push(contour),
            Some(RealSign::Zero) | None => {
                return Err(RasterTraceError::UncertainContourOrientation);
            },
        }
    }

    let region = CurveRegion2::try_from_native_contours(material, holes)?;
    Ok(RasterTraceReport {
        region,
        width: img.width(),
        height: img.height(),
        solid_pixel_count,
        boundary_edge_count,
        contour_count,
    })
}

impl GridEdge {
    /// Stores endpoints without validating axis alignment, length or nondegeneracy.
    /// The exposed-cell emitter establishes those invariants before traversal.
    const fn new(start_x: u32, start_y: u32, end_x: u32, end_y: u32) -> Self {
        Self {
            start: GridPoint {
                x: start_x,
                y: start_y,
            },
            end: GridPoint { x: end_x, y: end_y },
        }
    }

    /// Encodes +X, +Y, -X, -Y as 0, 1, 2, 3 for turn selection.
    /// Accepts any nonzero axis-aligned length; diagonal or zero edges panic as
    /// violations of the emitter's invariant.
    fn direction(self) -> u8 {
        match (self.end.x.cmp(&self.start.x), self.end.y.cmp(&self.start.y)) {
            (std::cmp::Ordering::Greater, std::cmp::Ordering::Equal) => 0,
            (std::cmp::Ordering::Equal, std::cmp::Ordering::Greater) => 1,
            (std::cmp::Ordering::Less, std::cmp::Ordering::Equal) => 2,
            (std::cmp::Ordering::Equal, std::cmp::Ordering::Less) => 3,
            _ => unreachable!("raster boundary edges are axis-aligned unit segments"),
        }
    }
}

/// Inclusive grayscale threshold at a caller-validated in-bounds pixel.
/// All neighbors are bounds-checked by the emitter before this indexing call.
fn pixel_is_solid(img: &GrayImage, x: u32, y: u32, threshold: u8) -> bool {
    img.get_pixel(x, y)[0] >= threshold
}

/// Walks unused directed edges into separate closed pixel-boundary loops.
///
/// Seeds follow input order. At each destination, [`turn_priority`] chooses
/// among unused outgoing edges, with input candidate order breaking ties. The
/// positive-turn preference keeps diagonal cell contacts separate instead of
/// joining them into a crossing contour. Returning to the seed vertex closes
/// the walk without repeating that vertex; collinear runs are then simplified.
///
/// Requires the emitter's oriented axis-aligned edge set. A missing continuation
/// returns `OpenBoundary` at its destination, discarding all partial local loops.
/// This is not a validator for arbitrary directed graphs or their embedding.
fn trace_boundary_loops(edges: &[GridEdge]) -> Result<Vec<Vec<GridPoint>>, RasterTraceError> {
    let mut outgoing = BTreeMap::<GridPoint, Vec<usize>>::new();
    for (index, edge) in edges.iter().enumerate() {
        outgoing.entry(edge.start).or_default().push(index);
    }

    let mut used = vec![false; edges.len()];
    let mut loops = Vec::new();
    for seed in 0..edges.len() {
        if used[seed] {
            continue;
        }
        let start = edges[seed].start;
        let mut current = seed;
        let mut points = Vec::new();

        loop {
            used[current] = true;
            let edge = edges[current];
            points.push(edge.start);
            if edge.end == start {
                break;
            }
            current = outgoing
                .get(&edge.end)
                .and_then(|candidates| {
                    candidates
                        .iter()
                        .copied()
                        .filter(|candidate| !used[*candidate])
                        .min_by_key(|candidate| {
                            turn_priority(edge.direction(), edges[*candidate].direction())
                        })
                })
                .ok_or(RasterTraceError::OpenBoundary {
                    x: edge.end.x,
                    y: edge.end.y,
                })?;
        }
        loops.push(remove_collinear_grid_points(points));
    }
    Ok(loops)
}

/// Ranks a positive quarter turn, straight motion, negative quarter turn and reversal.
/// Inputs must be direction codes 0..=3 from [`GridEdge::direction`]; smaller
/// output wins. Positive turn refers to numeric XY orientation, not screen Y.
fn turn_priority(incoming: u8, outgoing: u8) -> u8 {
    match (outgoing + 4 - incoming) % 4 {
        1 => 0,
        0 => 1,
        3 => 2,
        2 => 3,
        _ => unreachable!(),
    }
}

/// Removes axis-collinear vertices of an implicitly closed raster loop until stable.
///
/// Retained vertices keep their traversal order, and wraparound neighbors count.
/// Stops at three or fewer vertices without validating them. The caller supplies
/// a nonbacktracking cell boundary: the coordinate-equality test does not check
/// betweenness or justify removing reversals from arbitrary input paths.
fn remove_collinear_grid_points(mut points: Vec<GridPoint>) -> Vec<GridPoint> {
    loop {
        let len = points.len();
        if len <= 3 {
            return points;
        }
        let mut retained = Vec::with_capacity(len);
        for index in 0..len {
            let previous = points[(index + len - 1) % len];
            let point = points[index];
            let next = points[(index + 1) % len];
            if !((previous.x == point.x && point.x == next.x)
                || (previous.y == point.y && point.y == next.y))
            {
                retained.push(point);
            }
        }
        if retained.len() == len {
            return retained;
        }
        points = retained;
    }
}

/// Promotes one simplified grid loop to exact lines, including last-to-first closure.
/// Native line and contour constructors own degeneracy and connectivity errors;
/// no orientation or filled-region role is assigned in this helper.
fn grid_loop_to_contour(points: Vec<GridPoint>) -> Result<Contour2, CurveError> {
    let mut segments = Vec::with_capacity(points.len());
    for index in 0..points.len() {
        let start = grid_point(points[index]);
        let end = grid_point(points[(index + 1) % points.len()]);
        segments.push(Segment2::Line(LineSeg2::try_new(start, end)?));
    }
    Contour2::try_new(segments)
}

/// Promotes unsigned grid coordinates exactly, preserving numeric Y and unit scale.
fn grid_point(point: GridPoint) -> Point2 {
    Point2::new(Real::from(point.x), Real::from(point.y))
}

/// Raster count/ownership witnesses for empty input, holes and diagonal contacts.
#[cfg(test)]
mod tests {
    use super::*;

    use image::Luma;

    /// Requires native material/hole loop counts, propagating classification failures.
    fn assert_role_counts(region: &CurveRegion2, expected: (usize, usize)) {
        let actual = region
            .loop_role_counts()
            .expect("raster topology should classify");
        assert_eq!(actual, expected);
    }

    /// A zero-by-zero raster produces empty geometry and no traced work.
    #[test]
    fn empty_image_returns_an_empty_report_without_panicking() {
        let report = try_from_image(&GrayImage::new(0, 0), 128).unwrap();
        assert!(report.region().is_empty());
        assert_eq!(report.solid_pixel_count(), 0);
        assert_eq!(report.boundary_edge_count(), 0);
        assert_eq!(report.contour_count(), 0);
    }

    /// Six solid cells expose ten unit edges and one material loop without holes.
    /// The historical test name mentions four segments, but no final segment-count
    /// assertion is made here; the measured edge count precedes simplification.
    #[test]
    fn solid_rectangle_collapses_pixel_edges_to_four_exact_segments() {
        let image = GrayImage::from_pixel(3, 2, Luma([255]));
        let report = try_from_image(&image, 128).unwrap();
        assert_eq!(report.solid_pixel_count(), 6);
        assert_eq!(report.boundary_edge_count(), 10);
        assert_eq!(report.contour_count(), 1);
        assert_role_counts(report.region(), (1, 0));
    }

    /// Removing the center cell of a 3x3 block yields one material loop and one hole.
    #[test]
    fn ring_preserves_one_material_and_one_hole() {
        let image = GrayImage::from_fn(3, 3, |x, y| {
            if x == 1 && y == 1 {
                Luma([0])
            } else {
                Luma([255])
            }
        });
        let report = try_from_image(&image, 128).unwrap();
        assert_eq!(report.contour_count(), 2);
        assert_role_counts(report.region(), (1, 1));
    }

    /// Two corner-touching cells remain separate material loops under the turn rule.
    #[test]
    fn diagonal_pixels_remain_two_four_connected_components() {
        let image =
            GrayImage::from_fn(2, 2, |x, y| if x == y { Luma([255]) } else { Luma([0]) });
        let report = try_from_image(&image, 128).unwrap();
        assert_eq!(report.contour_count(), 2);
        assert_role_counts(report.region(), (2, 0));
    }
}
