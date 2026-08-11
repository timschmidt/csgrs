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

use crate::region::LineArcRegion2;
use crate::{
    CircularArc2, Classification, Contour2, CurveContext, CurveError, CurveFamily2,
    CurveOperation2, CurveOutcome, CurveRegion2, CurveResult, CurveString2, ExactCurveError,
    ExactCurveResult, LineSeg2, Point2, Segment2,
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
    scale: Real,
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
        let policy = CurveContext::STRICT;
        let equal_scale =
            crate::classify::real_sign(&(&first_len_squared - &second_len_squared), &policy);
        let orthogonal =
            crate::classify::real_sign(&(a.clone() * b.clone() + d.clone() * e.clone()), &policy);
        let determinant = a.clone() * e.clone() - b.clone() * d.clone();
        let determinant_sign = crate::classify::real_sign(&determinant, &policy);

        if equal_scale != Some(RealSign::Zero)
            || orthogonal != Some(RealSign::Zero)
            || !matches!(
                determinant_sign,
                Some(RealSign::Negative | RealSign::Positive)
            )
        {
            return Err(CurveError::InvalidSimilarityTransform);
        }

        let scale = first_len_squared.sqrt()?;
        Ok(Self {
            a,
            b,
            d,
            e,
            xoff,
            yoff,
            scale,
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
    /// matrix entries as a similarity. Once accepted, the linear part is
    /// projected onto the nearest orientation-preserving or
    /// orientation-reversing similarity and all transformed geometry is built
    /// from that certified hyperreal matrix.
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

        // Store an exact similarity, not merely the approximately similar
        // matrix used at this finite API boundary.  The orthogonal projection
        // onto the selected orientation component is the closest similarity
        // linear part in Frobenius norm.  Reusing the exact constructor then
        // certifies the invariant consumed by every native-geometry method.
        let (a, b, d, e) = if determinant > 0.0 {
            let u = (a + e) * 0.5;
            let v = (d - b) * 0.5;
            (u, -v, v, u)
        } else {
            let u = (a - e) * 0.5;
            let v = (b + d) * 0.5;
            (u, v, v, -u)
        };
        Self::try_from_real_affine(
            real_from_f64(a)?,
            real_from_f64(b)?,
            real_from_f64(d)?,
            real_from_f64(e)?,
            real_from_f64(xoff)?,
            real_from_f64(yoff)?,
        )
    }

    /// Returns true when the transform reverses orientation.
    pub const fn reverses_orientation(&self) -> bool {
        self.reverses_orientation
    }

    /// Returns the certified positive uniform scale of the linear part.
    ///
    /// The value is retained once when the transform is constructed so a
    /// batch transform of exact normal-offset carriers does not repeatedly
    /// build the same square root.
    pub const fn scale(&self) -> &Real {
        &self.scale
    }

    /// Applies the affine map to one homogeneous coordinate pair.
    pub(crate) fn transform_homogeneous_coordinates(
        &self,
        x: &Real,
        y: &Real,
        weight: &Real,
    ) -> (Real, Real) {
        (
            &self.a * x + &self.b * y + &self.xoff * weight,
            &self.d * x + &self.e * y + &self.yoff * weight,
        )
    }

    /// Applies only the similarity's linear part to one vector.
    pub(crate) fn transform_vector_coordinates(&self, x: &Real, y: &Real) -> (Real, Real) {
        (&self.a * x + &self.b * y, &self.d * x + &self.e * y)
    }

    /// Borrows the exact affine entries retained by this certified
    /// similarity. Exact carrier transforms use these entries without
    /// reconstructing or re-certifying the same matrix per endpoint.
    pub(crate) const fn affine_components(&self) -> (&Real, &Real, &Real, &Real, &Real, &Real) {
        (&self.a, &self.b, &self.d, &self.e, &self.xoff, &self.yoff)
    }

    /// Returns the exact similarity obtained by applying `self` and then
    /// `next`.
    ///
    /// Both operands already carry certified similarity invariants, so their
    /// affine composition needs no repeated orthogonality tests or square-root
    /// construction.  Collapsing retained transform chains before moving an
    /// algebraic coefficient frame also prevents expression growth from
    /// applying every affine layer independently.
    pub(crate) fn then(&self, next: &Self) -> Self {
        Self {
            a: &next.a * &self.a + &next.b * &self.d,
            b: &next.a * &self.b + &next.b * &self.e,
            d: &next.d * &self.a + &next.e * &self.d,
            e: &next.d * &self.b + &next.e * &self.e,
            xoff: &next.a * &self.xoff + &next.b * &self.yoff + &next.xoff,
            yoff: &next.d * &self.xoff + &next.e * &self.yoff + &next.yoff,
            scale: &self.scale * &next.scale,
            reverses_orientation: self.reverses_orientation ^ next.reverses_orientation,
        }
    }

    /// Transforms a point with hyperreal arithmetic.
    pub fn transform_point(&self, point: &Point2) -> Point2 {
        let point_is_exact =
            point.x().exact_rational_ref().is_some() && point.y().exact_rational_ref().is_some();
        let one = Real::one();
        let transform_coordinate = |first: &Real, second: &Real, offset: &Real| {
            if point_is_exact
                && first.exact_rational_ref().is_some()
                && second.exact_rational_ref().is_some()
                && offset.exact_rational_ref().is_some()
            {
                return Real::exact_rational_signed_product_sum_known_exact(
                    [true; 3],
                    [[first, point.x()], [second, point.y()], [offset, &one]],
                );
            }
            (first * point.x()) + (second * point.y()) + offset.clone()
        };
        Point2::new(
            transform_coordinate(&self.a, &self.b, &self.xoff),
            transform_coordinate(&self.d, &self.e, &self.yoff),
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

impl LineArcRegion2 {
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

impl CurveRegion2 {
    /// Applies a certified planar similarity to every retained exact carrier.
    ///
    /// This is the region-level counterpart to [`CurveString2::transform_similarity`].
    /// It delegates to the general exact affine implementation while preserving
    /// authoritative roles, fill rules, algebraic endpoint evidence, and any
    /// regenerated native line/arc fast path.
    pub fn transform_similarity(
        &self,
        transform: &Similarity2,
        policy: &crate::CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        crate::policy::resolve_certified_operation(policy, |attempt| {
            self.transform_similarity_raw(transform, attempt)
        })
    }

    fn transform_similarity_raw(
        &self,
        transform: &Similarity2,
        policy: &crate::CurveContext,
    ) -> ExactCurveResult<Self> {
        if let Classification::Decided(native) =
            self.native_line_arc_region(policy).map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Transformation, CurveFamily2::Line, cause)
            })?
        {
            let transformed = native.transform_similarity(transform).map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Transformation, CurveFamily2::Line, cause)
            })?;
            return Self::try_from_line_arc_region_raw(&transformed, policy)
                .map_err(|error| error.with_operation(CurveOperation2::Transformation));
        }
        self.transform_affine_raw(
            &transform.a,
            &transform.b,
            &transform.d,
            &transform.e,
            &transform.xoff,
            &transform.yoff,
            policy,
        )
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

    #[test]
    fn point_transform_fuses_exact_affine_sums_and_preserves_symbolic_expression() {
        let exact = Similarity2::try_from_real_affine(
            Real::from(3),
            Real::from(-4),
            Real::from(4),
            Real::from(3),
            Real::from(11),
            Real::from(-13),
        )
        .unwrap();
        let exact_point = Point2::from_values(5, -7);
        let transformed = exact.transform_point(&exact_point);
        assert_eq!(
            transformed.x(),
            &(&exact.a * exact_point.x() + &exact.b * exact_point.y() + exact.xoff.clone())
        );
        assert_eq!(
            transformed.y(),
            &(&exact.d * exact_point.x() + &exact.e * exact_point.y() + exact.yoff.clone())
        );

        let symbolic_point =
            Point2::new(Real::from(2).sqrt().unwrap(), Real::from(3).sqrt().unwrap());
        let transformed = exact.transform_point(&symbolic_point);
        assert_eq!(
            transformed.x(),
            &(&exact.a * symbolic_point.x() + &exact.b * symbolic_point.y() + exact.xoff.clone())
        );
        assert_eq!(
            transformed.y(),
            &(&exact.d * symbolic_point.x() + &exact.e * symbolic_point.y() + exact.yoff.clone())
        );
    }

    #[test]
    fn finite_similarity_constructor_canonicalizes_the_accepted_linear_part() {
        let transform =
            Similarity2::try_from_f64_affine(1.0, 1.0e-12, 0.0, 1.0 + 5.0e-13, 7.0, -11.0, 1.0e-9)
                .unwrap();

        assert_eq!(transform.d, -transform.b.clone());
        assert_eq!(transform.e, transform.a);
        assert_eq!(
            transform.scale() * transform.scale(),
            &transform.a * &transform.a + &transform.d * &transform.d
        );
    }

    #[test]
    fn exact_similarity_composition_matches_sequential_application() {
        let first = Similarity2::try_from_real_affine(
            Real::zero(),
            Real::from(-2_i8),
            Real::from(2_i8),
            Real::zero(),
            Real::from(5_i8),
            Real::from(-7_i8),
        )
        .unwrap();
        let second = Similarity2::try_from_real_affine(
            Real::from(-3_i8),
            Real::zero(),
            Real::zero(),
            Real::from(3_i8),
            Real::from(11_i8),
            Real::from(-13_i8),
        )
        .unwrap();
        let combined = first.then(&second);
        let point = Point2::from_values(17, -19);

        assert_eq!(
            combined.transform_point(&point),
            second.transform_point(&first.transform_point(&point))
        );
        assert_eq!(combined.scale(), &(&Real::from(2_i8) * Real::from(3_i8)));
        assert!(combined.reverses_orientation());
    }
}
