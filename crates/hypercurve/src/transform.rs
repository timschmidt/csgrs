//! Exact-aware planar similarity transforms for native curve geometry.
//!
//! Similarities preserve lines and circles. This module accepts finite `f64`
//! affine entries at the API boundary, certifies that the linear part is a
//! nonsingular similarity, promotes coefficients to [`Real`](hyperreal::Real),
//! and applies the transform to native line/circular-arc objects without
//! flattening. Keeping exact curve objects authoritative follows exact-computation discipline. The line/circle
//! preservation property is the standard Euclidean similarity model described
//! in standard geometric constructions.

use hyperreal::{Real, RealSign};

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
    /// Constructs a planar similarity from exact affine entries.
    ///
    /// Equal axis scales, orthogonality, nonsingularity, and orientation are
    /// certified in `Real`; undecidable classifications are rejected.
    pub fn try_from_real_affine(
        a: Real,
        b: Real,
        d: Real,
        e: Real,
        xoff: Real,
        yoff: Real,
    ) -> CurveResult<Self> {
        let first_len_squared = a.clone() * a.clone() + d.clone() * d.clone();
        let second_len_squared = b.clone() * b.clone() + e.clone() * e.clone();
        let equal_scale = (first_len_squared - second_len_squared).refine_sign_until(-128);
        let orthogonal = (a.clone() * b.clone() + d.clone() * e.clone()).refine_sign_until(-128);
        let determinant = a.clone() * e.clone() - b.clone() * d.clone();
        let determinant_sign = determinant.refine_sign_until(-128);

        if equal_scale != Some(RealSign::Zero)
            || orthogonal != Some(RealSign::Zero)
            || !matches!(
                determinant_sign,
                Some(RealSign::Negative | RealSign::Positive)
            )
        {
            return Err(CurveError::InvalidSimilarityTransform);
        }

        Ok(Self {
            a,
            b,
            d,
            e,
            xoff,
            yoff,
            reverses_orientation: determinant_sign == Some(RealSign::Negative),
        })
    }

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
        self.map_points(|point| transform.transform_point(point))
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
        let source_start = self.start().ok_or(CurveError::EmptyCurveString)?;
        let transformed_start = transform.transform_point(source_start);
        let mut transformed_segment_start = transformed_start.clone();
        let mut segments = Vec::with_capacity(self.segments().len());
        for segment in self.segments() {
            let transformed_end = if segment.end() == source_start {
                transformed_start.clone()
            } else {
                transform.transform_point(segment.end())
            };
            let transformed = match segment {
                Segment2::Line(line) => line
                    .map_points_between(
                        transformed_segment_start,
                        transformed_end.clone(),
                        |point| transform.transform_point(point),
                    )
                    .map(Segment2::Line)?,
                Segment2::Arc(arc) => CircularArc2::try_from_center_with_bulge(
                    transformed_segment_start,
                    transformed_end.clone(),
                    transform.transform_point(arc.center()),
                    arc.is_clockwise() ^ transform.reverses_orientation(),
                    arc.bulge().cloned(),
                )
                .map(Segment2::Arc)?,
            };
            transformed_segment_start = transformed_end;
            segments.push(transformed);
        }
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

#[cfg(test)]
mod tests {
    use super::Similarity2;
    use crate::Point2;
    use hyperreal::Real;

    #[test]
    fn exact_similarity_preserves_translation_beyond_f64_resolution() {
        let base = Real::from(1_i64 << 60);
        let transform = Similarity2::try_from_real_affine(
            Real::one(),
            Real::zero(),
            Real::zero(),
            Real::one(),
            base.clone(),
            Real::zero(),
        )
        .unwrap();

        let transformed = transform.transform_point(&Point2::new(Real::one(), Real::zero()));

        assert_eq!(transformed.x(), &(base + Real::one()));
    }

    #[test]
    fn exact_similarity_rejects_anisotropic_scale() {
        assert!(
            Similarity2::try_from_real_affine(
                Real::from(2_u8),
                Real::zero(),
                Real::zero(),
                Real::one(),
                Real::zero(),
                Real::zero(),
            )
            .is_err()
        );
    }
}
