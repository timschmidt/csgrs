//! Exact-aware planar similarity transforms for native curve geometry.
//!
//! Similarities preserve lines and circles. This module accepts finite `f64`
//! affine entries at the API boundary, certifies that the linear part is a
//! nonsingular similarity, promotes coefficients to [`Real`](hyperreal::Real),
//! and applies the transform to native line/circular-arc objects without
//! flattening. Keeping exact curve objects authoritative follows Yap, "Towards
//! Exact Geometric Computation," *Computational Geometry* 7(1-2), 1997
//! (<https://doi.org/10.1016/0925-7721(95)00040-2>). The line/circle
//! preservation property is the standard Euclidean similarity model described
//! in Schneider and Eberly, *Geometric Tools for Computer Graphics* (Morgan
//! Kaufmann, 2002).

use hyperreal::Real;

use crate::{
    CircularArc2, Contour2, CurveError, CurveResult, CurveString2, LineSeg2, Point2, Region2,
    Segment2,
};

/// A 2D affine transform whose linear part is a nonsingular similarity.
#[derive(Clone, Debug, PartialEq)]
pub struct Similarity2 {
    a: Real,
    b: Real,
    d: Real,
    e: Real,
    xoff: Real,
    yoff: Real,
    reverses_orientation: bool,
}

impl Similarity2 {
    /// Constructs a planar similarity from finite affine entries.
    ///
    /// The transform is:
    ///
    /// ```text
    /// x' = a*x + b*y + xoff
    /// y' = d*x + e*y + yoff
    /// ```
    ///
    /// The finite validation tolerance is only used to accept API-boundary
    /// matrix entries as a similarity. Once accepted, all transformed geometry
    /// is built with hyperreal coefficients.
    pub fn try_from_f64_affine(
        a: f64,
        b: f64,
        d: f64,
        e: f64,
        xoff: f64,
        yoff: f64,
        tolerance: f64,
    ) -> CurveResult<Self> {
        if ![a, b, d, e, xoff, yoff, tolerance]
            .into_iter()
            .all(f64::is_finite)
            || tolerance <= 0.0
        {
            return Err(CurveError::InvalidSimilarityTransform);
        }

        let first_len_squared = a * a + d * d;
        let second_len_squared = b * b + e * e;
        let dot = a * b + d * e;
        let determinant = a * e - b * d;

        if determinant.abs() <= tolerance
            || (first_len_squared - second_len_squared).abs() > tolerance
            || dot.abs() > tolerance
        {
            return Err(CurveError::InvalidSimilarityTransform);
        }

        Ok(Self {
            a: real_from_f64(a)?,
            b: real_from_f64(b)?,
            d: real_from_f64(d)?,
            e: real_from_f64(e)?,
            xoff: real_from_f64(xoff)?,
            yoff: real_from_f64(yoff)?,
            reverses_orientation: determinant < 0.0,
        })
    }

    /// Returns true when the transform reverses orientation.
    pub const fn reverses_orientation(&self) -> bool {
        self.reverses_orientation
    }

    /// Transforms a point with hyperreal arithmetic.
    pub fn transform_point(&self, point: &Point2) -> Point2 {
        Point2::new(
            (&self.a * point.x()) + (&self.b * point.y()) + self.xoff.clone(),
            (&self.d * point.x()) + (&self.e * point.y()) + self.yoff.clone(),
        )
    }
}

impl Point2 {
    /// Applies a certified planar similarity transform.
    pub fn transform_similarity(&self, transform: &Similarity2) -> Self {
        transform.transform_point(self)
    }
}

impl Segment2 {
    /// Applies a certified planar similarity transform while preserving segment type.
    pub fn transform_similarity(&self, transform: &Similarity2) -> CurveResult<Self> {
        match self {
            Self::Line(line) => line.transform_similarity(transform).map(Self::Line),
            Self::Arc(arc) => arc.transform_similarity(transform).map(Self::Arc),
        }
    }
}

impl LineSeg2 {
    /// Applies a certified planar similarity transform.
    pub fn transform_similarity(&self, transform: &Similarity2) -> CurveResult<Self> {
        Self::try_new(
            transform.transform_point(self.start()),
            transform.transform_point(self.end()),
        )
    }
}

impl CircularArc2 {
    /// Applies a certified planar similarity transform.
    ///
    /// Similarities preserve circular arcs. Reflections reverse orientation, so
    /// clockwise state is toggled exactly when the transform reverses
    /// orientation.
    pub fn transform_similarity(&self, transform: &Similarity2) -> CurveResult<Self> {
        Self::try_from_center(
            transform.transform_point(self.start()),
            transform.transform_point(self.end()),
            transform.transform_point(self.center()),
            self.is_clockwise() ^ transform.reverses_orientation(),
        )
    }
}

impl CurveString2 {
    /// Applies a certified planar similarity transform while preserving line/arc topology.
    pub fn transform_similarity(&self, transform: &Similarity2) -> CurveResult<Self> {
        let segments = self
            .segments()
            .iter()
            .map(|segment| segment.transform_similarity(transform))
            .collect::<CurveResult<Vec<_>>>()?;
        Self::try_new(segments)
    }
}

impl Contour2 {
    /// Applies a certified planar similarity transform while preserving the fill rule.
    pub fn transform_similarity(&self, transform: &Similarity2) -> CurveResult<Self> {
        let curve = self.curve_string().transform_similarity(transform)?;
        Self::try_new_with_fill_rule(curve.into_segments(), self.fill_rule())
    }
}

impl Region2 {
    /// Applies a certified planar similarity transform to every material and hole contour.
    pub fn transform_similarity(&self, transform: &Similarity2) -> CurveResult<Self> {
        let material = self
            .material_contours()
            .iter()
            .map(|contour| contour.transform_similarity(transform))
            .collect::<CurveResult<Vec<_>>>()?;
        let holes = self
            .hole_contours()
            .iter()
            .map(|contour| contour.transform_similarity(transform))
            .collect::<CurveResult<Vec<_>>>()?;
        Ok(Self::new(material, holes))
    }
}

fn real_from_f64(value: f64) -> CurveResult<Real> {
    if !value.is_finite() {
        return Err(CurveError::InvalidSimilarityTransform);
    }
    Ok(Real::try_from(value)?)
}
