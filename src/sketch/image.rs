//! Direct raster-boundary construction for [`Profile`].
//!
//! Every solid pixel is an exact unit cell on an integer lattice. The tracer
//! emits only cell edges adjacent to empty space, joins those directed edges
//! into closed loops, and hands the loops directly to Hypercurve. No SVG
//! string, parser, or floating-point coordinate conversion participates in the
//! topology decision.

use std::collections::BTreeMap;

use crate::hyper_math::hreal_sign;
use crate::sketch::Profile;
use hypercurve::{
    Contour2, CurveError, CurvePolicy, CurveRegion2, ExactCurveError, LineSeg2, Point2,
    Segment2,
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
    Curve(#[from] CurveError),
    /// Hypercurve rejected explicit material/hole topology.
    #[error(transparent)]
    ExactCurve(#[from] ExactCurveError),
    /// A closed boundary had no decidable nonzero orientation.
    #[error("raster contour orientation is zero, unsupported, or uncertain")]
    UncertainContourOrientation,
}

/// Report for direct integer-grid raster tracing.
#[derive(Clone, Debug)]
pub struct RasterTraceReport {
    profile: Profile,
    width: u32,
    height: u32,
    solid_pixel_count: usize,
    boundary_edge_count: usize,
    contour_count: usize,
}

impl RasterTraceReport {
    /// Return the constructed exact-grid profile.
    pub const fn profile(&self) -> &Profile {
        &self.profile
    }

    /// Consume the report and return the constructed profile.
    pub fn into_profile(self) -> Profile {
        self.profile
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

    /// Return the number of unit cell edges retained on the boundary.
    pub const fn boundary_edge_count(&self) -> usize {
        self.boundary_edge_count
    }

    /// Return the number of closed loops handed to Hypercurve.
    pub const fn contour_count(&self) -> usize {
        self.contour_count
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct GridPoint {
    x: u32,
    y: u32,
}

#[derive(Clone, Copy, Debug)]
struct GridEdge {
    start: GridPoint,
    end: GridPoint,
}

impl Profile {
    /// Trace solid pixels directly into exact Hypercurve boundary topology.
    ///
    /// Pixels whose grayscale value is at least `threshold` are closed unit
    /// cells. The report retains source dimensions and boundary-work counts.
    pub fn try_from_image(
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

        let region = CurveRegion2::try_from_native_contours(
            material,
            holes,
            &CurvePolicy::certified(),
        )?;
        Ok(RasterTraceReport {
            profile: Profile::from_curve_region(region),
            width: img.width(),
            height: img.height(),
            solid_pixel_count,
            boundary_edge_count,
            contour_count,
        })
    }

    /// Build a profile from the solid pixels of a grayscale image.
    ///
    /// This compatibility wrapper preserves the historical third argument.
    /// Pixel-cell boundaries are intrinsically closed, so `closepaths` no
    /// longer changes construction. Use [`Self::try_from_image`] to receive
    /// tracing evidence and typed failures.
    ///
    /// # Panics
    ///
    /// Panics if exact boundary construction or role assignment fails.
    pub fn from_image(img: &GrayImage, threshold: u8, _closepaths: bool) -> Self {
        Self::try_from_image(img, threshold)
            .unwrap_or_else(|error| panic!("raster profile construction failed: {error}"))
            .into_profile()
    }
}

impl GridEdge {
    const fn new(start_x: u32, start_y: u32, end_x: u32, end_y: u32) -> Self {
        Self {
            start: GridPoint {
                x: start_x,
                y: start_y,
            },
            end: GridPoint { x: end_x, y: end_y },
        }
    }

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

fn pixel_is_solid(img: &GrayImage, x: u32, y: u32, threshold: u8) -> bool {
    img.get_pixel(x, y)[0] >= threshold
}

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

fn turn_priority(incoming: u8, outgoing: u8) -> u8 {
    match (outgoing + 4 - incoming) % 4 {
        1 => 0,
        0 => 1,
        3 => 2,
        2 => 3,
        _ => unreachable!(),
    }
}

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

fn grid_loop_to_contour(points: Vec<GridPoint>) -> Result<Contour2, CurveError> {
    let mut segments = Vec::with_capacity(points.len());
    for index in 0..points.len() {
        let start = grid_point(points[index]);
        let end = grid_point(points[(index + 1) % points.len()]);
        segments.push(Segment2::Line(LineSeg2::try_new(start, end)?));
    }
    Contour2::try_new(segments)
}

fn grid_point(point: GridPoint) -> Point2 {
    Point2::new(Real::from(point.x), Real::from(point.y))
}

#[cfg(test)]
mod tests {
    use super::*;
    use image::Luma;

    #[test]
    fn empty_image_returns_an_empty_report_without_panicking() {
        let report = Profile::try_from_image(&GrayImage::new(0, 0), 128).unwrap();
        assert!(report.profile().is_empty());
        assert_eq!(report.solid_pixel_count(), 0);
        assert_eq!(report.boundary_edge_count(), 0);
        assert_eq!(report.contour_count(), 0);
    }

    #[test]
    fn solid_rectangle_collapses_pixel_edges_to_four_exact_segments() {
        let image = GrayImage::from_pixel(3, 2, Luma([255]));
        let report = Profile::try_from_image(&image, 128).unwrap();
        assert_eq!(report.solid_pixel_count(), 6);
        assert_eq!(report.boundary_edge_count(), 10);
        assert_eq!(report.contour_count(), 1);
        assert_eq!(report.profile().material_contour_count(), 1);
        assert_eq!(report.profile().hole_contour_count(), 0);
    }

    #[test]
    fn ring_preserves_one_material_and_one_hole() {
        let image = GrayImage::from_fn(3, 3, |x, y| {
            if x == 1 && y == 1 {
                Luma([0])
            } else {
                Luma([255])
            }
        });
        let report = Profile::try_from_image(&image, 128).unwrap();
        assert_eq!(report.contour_count(), 2);
        assert_eq!(report.profile().material_contour_count(), 1);
        assert_eq!(report.profile().hole_contour_count(), 1);
    }

    #[test]
    fn diagonal_pixels_remain_two_four_connected_components() {
        let image =
            GrayImage::from_fn(2, 2, |x, y| if x == y { Luma([255]) } else { Luma([0]) });
        let report = Profile::try_from_image(&image, 128).unwrap();
        assert_eq!(report.contour_count(), 2);
        assert_eq!(report.profile().material_contour_count(), 2);
        assert_eq!(report.profile().hole_contour_count(), 0);
    }
}
