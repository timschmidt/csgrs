//! Exact-backed planar profile types used at Hyperdrc's report boundary.
//!
//! [`Polygon`] and [`MultiPolygon`] deliberately resemble the small subset of
//! the former finite polygon API used by parsers and reports, but every
//! topology-bearing polygon owns a
//! [`hypercurve::CurveRegion2`](https://github.com/timschmidt/hypercurve).
//! Area, bounds, containment, and Boolean consumers therefore remain in
//! Hypercurve; the coordinate rings are only finite output views.

use std::sync::{Arc, OnceLock};

use hypercurve::{Classification, Contour2, CurvePolicy, CurveRegion2};
use hyperlimit::{PredicatePolicy, Sign, classify_real_sign_with_policy};
use hyperreal::Real;

/// A finite report coordinate.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Coord<T = f64> {
    /// Horizontal coordinate.
    pub x: T,
    /// Vertical coordinate.
    pub y: T,
}

/// A finite report polyline.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LineString<T = f64>(pub Vec<Coord<T>>);

impl<T> From<Vec<Coord<T>>> for LineString<T> {
    fn from(points: Vec<Coord<T>>) -> Self {
        Self(points)
    }
}

impl LineString<f64> {
    /// Iterates over coordinates.
    pub fn coords(&self) -> impl Iterator<Item = Coord<f64>> + '_ {
        self.0.iter().copied()
    }

    /// Iterates over coordinate references.
    pub fn coords_iter(&self) -> impl Iterator<Item = &Coord<f64>> {
        self.0.iter()
    }

    fn signed_ring_area(&self) -> f64 {
        if self.0.len() < 3 {
            return 0.0;
        }
        let mut doubled = self
            .0
            .windows(2)
            .map(|edge| edge[0].x * edge[1].y - edge[1].x * edge[0].y)
            .sum::<f64>();
        if let (Some(first), Some(last)) = (self.0.first(), self.0.last())
            && first != last
        {
            doubled += last.x * first.y - first.x * last.y;
        }
        doubled / 2.0
    }
}

/// An exact-backed material profile with finite report rings.
#[derive(Clone, Debug)]
pub struct Polygon<T = f64> {
    exterior: LineString<T>,
    interiors: Vec<LineString<T>>,
    region: Arc<CurveRegion2>,
    exact_construction_error: Option<Arc<str>>,
    exact_area: Arc<OnceLock<Option<Real>>>,
    exact_bounds: Arc<OnceLock<Option<[Real; 4]>>>,
}

impl PartialEq for Polygon<f64> {
    fn eq(&self, other: &Self) -> bool {
        self.exterior == other.exterior && self.interiors == other.interiors
    }
}

impl Polygon<f64> {
    /// Builds exact line topology from finite parser coordinates.
    pub fn new(exterior: LineString<f64>, interiors: Vec<LineString<f64>>) -> Self {
        let (region, exact_construction_error) = match region_from_rings(&exterior, &interiors) {
            Ok(region) => (region, None),
            Err(error) => (CurveRegion2::empty(), Some(Arc::<str>::from(error))),
        };
        Self {
            exterior,
            interiors,
            region: Arc::new(region),
            exact_construction_error,
            exact_area: Arc::new(OnceLock::new()),
            exact_bounds: Arc::new(OnceLock::new()),
        }
    }

    /// Builds a finite report profile around an already-authoritative region.
    pub(crate) fn from_exact_region(
        region: CurveRegion2,
        exterior: LineString<f64>,
        interiors: Vec<LineString<f64>>,
    ) -> Self {
        Self {
            exterior,
            interiors,
            region: Arc::new(region),
            exact_construction_error: None,
            exact_area: Arc::new(OnceLock::new()),
            exact_bounds: Arc::new(OnceLock::new()),
        }
    }

    pub(crate) fn from_shared_exact_region(
        region: Arc<CurveRegion2>,
        exterior: LineString<f64>,
        interiors: Vec<LineString<f64>>,
    ) -> Self {
        Self {
            exterior,
            interiors,
            region,
            exact_construction_error: None,
            exact_area: Arc::new(OnceLock::new()),
            exact_bounds: Arc::new(OnceLock::new()),
        }
    }

    /// Returns the finite material boundary used by report code.
    pub const fn exterior(&self) -> &LineString<f64> {
        &self.exterior
    }

    /// Returns finite hole boundaries used by report code.
    pub fn interiors(&self) -> &[LineString<f64>] {
        &self.interiors
    }

    /// Returns the authoritative exact filled region.
    pub fn exact_region(&self) -> &CurveRegion2 {
        self.region.as_ref()
    }

    pub(crate) fn shared_exact_region(&self) -> Arc<CurveRegion2> {
        self.region.clone()
    }

    pub(crate) fn exact_construction_error(&self) -> Option<&str> {
        self.exact_construction_error.as_deref()
    }

    pub(crate) fn with_exact_construction_error(mut self, detail: impl Into<Arc<str>>) -> Self {
        self.exact_construction_error = Some(detail.into());
        self
    }

    pub(crate) fn exact_area(&self) -> Option<&Real> {
        self.exact_area
            .get_or_init(|| exact_region_area(self.region.as_ref()))
            .as_ref()
    }

    pub(crate) fn exact_bounds(&self) -> Option<&[Real; 4]> {
        self.exact_bounds
            .get_or_init(|| exact_region_bounds(self.region.as_ref()))
            .as_ref()
    }

    /// Returns the finite projected-ring area used in reports.
    ///
    /// Decision predicates use the authoritative region through
    /// `polygon_area_scalar` instead.
    pub fn unsigned_area(&self) -> f64 {
        self.exterior.signed_ring_area().abs()
            - self
                .interiors
                .iter()
                .map(|ring| ring.signed_ring_area().abs())
                .sum::<f64>()
    }

    /// Returns the exact exterior orientation projected to a finite scalar.
    pub fn signed_area(&self) -> f64 {
        exact_ring_signed_area(&self.exterior)
            .and_then(|area| area.to_f64_lossy())
            .unwrap_or(0.0)
    }

    /// Returns certified exact bounds projected only at the report boundary.
    pub fn bounding_rect(&self) -> Option<Rect<f64>> {
        self.exact_bounds()
            .and_then(exact_bounds_rect)
            .or_else(|| finite_diagnostic_rect(&self.exterior, &self.interiors))
    }
}

/// A collection of exact-backed material profiles.
#[derive(Clone, Debug, Default)]
pub struct MultiPolygon<T = f64>(pub Vec<Polygon<T>>);

impl PartialEq for MultiPolygon<f64> {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl MultiPolygon<f64> {
    /// Returns the sum of finite projected component areas for reports.
    pub fn unsigned_area(&self) -> f64 {
        self.0.iter().map(Polygon::unsigned_area).sum()
    }

    /// Returns the union of certified exact component bounds.
    pub fn bounding_rect(&self) -> Option<Rect<f64>> {
        let mut rectangles = self.0.iter().filter_map(Polygon::bounding_rect);
        let first = rectangles.next()?;
        Some(rectangles.fold(first, |bounds, next| {
            Rect::new(
                Coord {
                    x: bounds.min.x.min(next.min.x),
                    y: bounds.min.y.min(next.min.y),
                },
                Coord {
                    x: bounds.max.x.max(next.max.x),
                    y: bounds.max.y.max(next.max.y),
                },
            )
        }))
    }
}

/// A finite view of a certified exact axis-aligned bounding box.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rect<T = f64> {
    min: Coord<T>,
    max: Coord<T>,
}

impl Rect<f64> {
    /// Constructs a rectangle from ordered corners.
    pub const fn new(min: Coord<f64>, max: Coord<f64>) -> Self {
        Self { min, max }
    }

    /// Returns the minimum corner.
    pub const fn min(&self) -> Coord<f64> {
        self.min
    }

    /// Returns the maximum corner.
    pub const fn max(&self) -> Coord<f64> {
        self.max
    }

    /// Returns horizontal extent.
    pub fn width(&self) -> f64 {
        self.max.x - self.min.x
    }

    /// Returns vertical extent.
    pub fn height(&self) -> f64 {
        self.max.y - self.min.y
    }
}

pub(crate) fn exact_region_area(region: &CurveRegion2) -> Option<Real> {
    match region.filled_area(&CurvePolicy::certified()).ok()? {
        Classification::Decided(Some(area)) => Some(area),
        Classification::Decided(None) | Classification::Uncertain(_) => None,
    }
}

fn exact_region_bounds(region: &CurveRegion2) -> Option<[Real; 4]> {
    if region.is_empty() {
        return None;
    }
    let bounds = match region.bounds(&CurvePolicy::certified()).ok()? {
        Classification::Decided(bounds) => bounds,
        Classification::Uncertain(_) => return None,
    };
    Some([
        bounds.min_x().clone(),
        bounds.min_y().clone(),
        bounds.max_x().clone(),
        bounds.max_y().clone(),
    ])
}

fn exact_bounds_rect(bounds: &[Real; 4]) -> Option<Rect<f64>> {
    Some(Rect::new(
        Coord {
            x: bounds[0].to_f64_lossy()?,
            y: bounds[1].to_f64_lossy()?,
        },
        Coord {
            x: bounds[2].to_f64_lossy()?,
            y: bounds[3].to_f64_lossy()?,
        },
    ))
}

fn finite_diagnostic_rect(
    exterior: &LineString<f64>,
    interiors: &[LineString<f64>],
) -> Option<Rect<f64>> {
    let mut coordinates = exterior
        .0
        .iter()
        .chain(interiors.iter().flat_map(|ring| ring.0.iter()));
    let first = coordinates.next()?;
    if !first.x.is_finite() || !first.y.is_finite() {
        return None;
    }
    let mut min = *first;
    let mut max = *first;
    for coordinate in coordinates {
        if !coordinate.x.is_finite() || !coordinate.y.is_finite() {
            return None;
        }
        min.x = min.x.min(coordinate.x);
        min.y = min.y.min(coordinate.y);
        max.x = max.x.max(coordinate.x);
        max.y = max.y.max(coordinate.y);
    }
    Some(Rect::new(min, max))
}

fn region_from_rings(
    exterior: &LineString<f64>,
    interiors: &[LineString<f64>],
) -> Result<CurveRegion2, String> {
    let material = ring_to_contour(exterior, false)?;
    let holes = interiors
        .iter()
        .map(|ring| ring_to_contour(ring, true))
        .collect::<Result<Vec<_>, _>>()?;
    CurveRegion2::try_from_native_contours(vec![material], holes, &CurvePolicy::certified())
        .map_err(|error| format!("exact polygon region construction failed: {error}"))
}

fn ring_to_contour(ring: &LineString<f64>, hole: bool) -> Result<Contour2, String> {
    let mut points = ring
        .0
        .iter()
        .map(|point| [point.x, point.y])
        .collect::<Vec<_>>();
    if points.len() > 1 && points.first() == points.last() {
        points.pop();
    }
    if points.len() < 3 {
        return Err("exact polygon contour has fewer than three vertices".to_string());
    }
    if points.iter().flatten().any(|value| !value.is_finite()) {
        return Err("exact polygon contour contains a non-finite coordinate".to_string());
    }
    let signed_area = exact_ring_signed_area(ring)
        .ok_or_else(|| "exact polygon contour area could not be constructed".to_string())?;
    let orientation = classify_real_sign_with_policy(&signed_area, PredicatePolicy)
        .value()
        .ok_or_else(|| "exact polygon contour orientation is unresolved".to_string())?;
    if (!hole && orientation == Sign::Negative) || (hole && orientation == Sign::Positive) {
        points.reverse();
    }
    Contour2::from_finite_ring(&points)
        .map_err(|error| format!("exact polygon contour construction failed: {error}"))
}

fn exact_ring_signed_area(ring: &LineString<f64>) -> Option<Real> {
    let mut points = ring.0.iter();
    let first = points.next()?;
    if !first.x.is_finite() || !first.y.is_finite() {
        return None;
    }
    let mut previous = first;
    let mut doubled = Real::zero();
    for point in points {
        if !point.x.is_finite() || !point.y.is_finite() {
            return None;
        }
        doubled += Real::try_from(previous.x).ok()? * Real::try_from(point.y).ok()?
            - Real::try_from(point.x).ok()? * Real::try_from(previous.y).ok()?;
        previous = point;
    }
    if previous != first {
        doubled += Real::try_from(previous.x).ok()? * Real::try_from(first.y).ok()?
            - Real::try_from(first.x).ok()? * Real::try_from(previous.y).ok()?;
    }
    Some(crate::scalar::half(&doubled))
}

/// Applies an exact affine transformation to a profile's authoritative region.
pub(crate) fn transform_exact_region(
    polygon: &Polygon<f64>,
    cosine: Real,
    sine: Real,
    translate_x: Real,
    translate_y: Real,
) -> Result<CurveRegion2, String> {
    polygon
        .region
        .transform_affine(
            &cosine,
            &(-sine.clone()),
            &sine,
            &cosine,
            &translate_x,
            &translate_y,
            &CurvePolicy::certified(),
        )
        .map_err(|error| format!("exact affine region transformation failed: {error}"))
}
