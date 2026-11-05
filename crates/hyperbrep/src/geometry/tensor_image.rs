//! Exact images of retained parameter-plane curves on rational tensor patches.
//!
//! A surface intersection generally creates a spatial curve outside every
//! fixed-degree rational family. When one operand is a positive-weight
//! rational Bézier tensor, the intersection is the image of a selected planar
//! branch in that tensor's own unit chart. This carrier keeps the original
//! tensor and the original Hypercurve source; it never fits or materializes
//! independent spatial coordinates.

use super::*;

/// Borrowed definition of a retained tensor-image curve.
///
/// The image is `S(x(t), y(t))`, where `S` is the tensor and
/// `t = source_offset + source_scale*s` for spatial parameter `s`.
#[derive(Clone, Copy, Debug)]
pub struct TensorCurveGeometry<'a> {
    /// Positive-weight rational Bézier tensor on the unit square.
    pub surface: &'a Surface,
    /// Original source curve in the tensor's UV chart.
    pub source: &'a Curve2,
    /// Nonzero scale from spatial to source parameters.
    pub source_scale: &'a Real,
    /// Offset from spatial to source parameters.
    pub source_offset: &'a Real,
}

#[derive(Clone, Debug)]
pub(crate) struct TensorCurve3 {
    pub(crate) surface: Surface,
    pub(crate) source: Curve2,
    pub(crate) source_scale: Real,
    pub(crate) source_offset: Real,
}

impl Curve3 {
    /// Lifts an exact UV curve through a positive-weight rational Bézier
    /// tensor without fitting or materializing it.
    ///
    /// The source must lie in the tensor's closed unit chart; its complete
    /// exact bounds are checked. Represented endpoint intervals use the unit
    /// spatial chart; selected intervals retain the source chart. Injectivity
    /// of the tensor and simplicity of the image are not claimed here.
    pub fn tensor_image(surface: Surface, source: Curve2) -> GeometryResult<Self> {
        crate::context::strict(|| {
            let (start, end) = source.parameter_endpoints();
            let (source_scale, source_offset) =
                if let (Some(start), Some(end)) = (start.scalar(), end.scalar()) {
                    (end - start, start.clone())
                } else {
                    (
                        if start.compare(end)? == Ordering::Less {
                            Real::one()
                        } else {
                            -Real::one()
                        },
                        Real::zero(),
                    )
                };
            Self::from_tensor_image(TensorCurve3 {
                surface,
                source,
                source_scale,
                source_offset,
            })
        })
    }

    pub(crate) fn from_tensor_image(image: TensorCurve3) -> GeometryResult<Self> {
        crate::context::strict(|| {
            if !matches!(
                image.surface.data.geometry,
                SurfaceGeometry::RationalBezier(_)
            ) {
                return Err(GeometryError::UnsupportedImageSurface);
            }
            if decided_order(compare_reals(
                &image.source_scale,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? == Ordering::Equal
            {
                return Err(GeometryError::InvalidParameterDomain);
            }
            // The whole source, not just its endpoints, must lie in the chart.
            let bounds = image.source.bounds()?;
            for (value, limit, order) in [
                (bounds.min_x(), Real::zero(), Ordering::Less),
                (bounds.min_y(), Real::zero(), Ordering::Less),
                (bounds.max_x(), Real::one(), Ordering::Greater),
                (bounds.max_y(), Real::one(), Ordering::Greater),
            ] {
                if decided_order(compare_reals(value, &limit, crate::STRICT_PREDICATES))? == order {
                    return Err(GeometryError::SurfaceParameterOutsideDomain);
                }
            }
            let (start, end) = image.source.parameter_endpoints();
            image.source.point_at(start)?;
            image.source.point_at(end)?;
            let domain = image.spatial_domain()?;
            Ok(Self::from_parts(
                CurveGeometry3::TensorImage(Box::new(image)),
                domain,
            ))
        })
    }

    /// Borrows the retained tensor, UV source and parameter map.
    pub fn tensor_geometry(&self) -> Option<TensorCurveGeometry<'_>> {
        let CurveGeometry3::TensorImage(image) = &self.data.geometry else {
            return None;
        };
        Some(TensorCurveGeometry {
            surface: &image.surface,
            source: &image.source,
            source_scale: &image.source_scale,
            source_offset: &image.source_offset,
        })
    }
}

impl Curve3 {
    /// Returns a native query view of a tensor iso-line's complete image.
    /// Composes the source's axis-aligned similarity with its retained spatial
    /// chart, then restricts/reverses the native iso-curve over that image.
    /// The returned curve uses its native parameter domain; its parameter is
    /// not asserted equal to this curve's spatial parameter. `None` when the
    /// selected chart does not supply this proof.
    pub(crate) fn tensor_image_as_iso_curve(&self) -> GeometryResult<Option<Curve3>> {
        let CurveGeometry3::TensorImage(image) = &self.data.geometry else {
            return Ok(None);
        };
        let order =
            |a: &Real, b: &Real| decided_order(compare_reals(a, b, crate::STRICT_PREDICATES));
        let original = image
            .source
            .source_restriction()
            .map_or(&image.source, |(original, _, _)| original);
        // The recognizer supplies the varying tensor coordinate's affine
        // law in the original polynomial branch parameter.
        let Some(chart) = super::plane_tensor::iso_line_with_range(
            original,
            original.polynomial_branch().is_none(),
        )?
        else {
            return Ok(None);
        };
        let iso = image.surface.iso_curve(chart.axis, &chart.constant)?;
        // Compose both exact chart laws before selecting the native interval.
        let Some((start, end)) = self.domain().try_real_endpoints() else {
            return Ok(None);
        };
        let first = &chart.offset + &chart.scale * image.parameter(start);
        let last = &chart.offset + &chart.scale * image.parameter(end);
        if order(start, &Real::zero())? == Ordering::Equal
            && order(end, &Real::one())? == Ordering::Equal
            && order(&first, &Real::zero())? == Ordering::Equal
            && order(&last, &Real::one())? == Ordering::Equal
        {
            return Ok(Some(iso));
        }
        Ok(match order(&first, &last)? {
            Ordering::Less => Some(iso.subcurve(&first, &last)?),
            Ordering::Greater => Some(iso.subcurve(&last, &first)?.reversed()?),
            Ordering::Equal => None,
        })
    }
}

impl TensorCurve3 {
    pub(super) fn parameter(&self, s: &Real) -> Real {
        &self.source_offset + &self.source_scale * s
    }

    pub(super) fn spatial_domain(&self) -> GeometryResult<ParameterRange> {
        let inverse = (Real::one() / &self.source_scale).map_err(GeometryError::Numeric)?;
        let offset = -&self.source_offset * &inverse;
        let source = self.source.parameter_domain();
        let mut start = Parameter::from_curve(source.start())?.affine_image(&inverse, &offset)?;
        let mut end = Parameter::from_curve(source.end())?.affine_image(&inverse, &offset)?;
        if start.compare(&end)? == Ordering::Greater {
            std::mem::swap(&mut start, &mut end);
        }
        ParameterRange::new(start, end)
    }

    pub(super) fn point_at(&self, s: &Real) -> GeometryResult<Point3> {
        let uv = match represented_curve_point(&self.source, &self.parameter(s)) {
            Ok(uv) => uv,
            Err(GeometryError::UnsupportedPcurveContour) => self
                .certified_rational_uv(&self.parameter(s))?
                .ok_or(GeometryError::UnsupportedPcurveContour)?,
            Err(error) => return Err(error),
        };
        self.surface
            .evaluate_real(&Point2::new(uv.x().clone(), uv.y().clone()))
    }

    /// An exact scalar UV point for a selected source point, when one
    /// coordinate has a scalar view (or is proved equal to a patch-boundary
    /// value 0 or 1, or to an exact root candidate of the projection-event
    /// resultant) and the other is a root of the source's
    /// defining polynomial at that value that is rational, or a root of the
    /// remaining linear or quadratic factor after exact rational deflation.
    /// Rational candidates come from a bounded exact rational-root search;
    /// radical candidates from the exact quadratic formula. Each candidate is
    /// accepted only after an exact comparison with the selected coordinate;
    /// other coordinates return `None` and stay unmaterialized.
    fn certified_rational_uv(&self, t: &Real) -> GeometryResult<Option<CurvePoint2>> {
        self.certified_scalar_uv(&self.source.point_at(&t.clone().into())?)
    }

    /// Scalar evaluation at a possibly selected spatial parameter: the joint
    /// point when its UV preimage is certified scalar, as for
    /// [`Self::certified_rational_uv`].
    pub(super) fn certified_point(&self, s: &Parameter) -> GeometryResult<Point3> {
        if let Some(value) = s.try_real() {
            return self.point_at(value);
        }
        let mapped = s.affine_image(&self.source_scale, &self.source_offset)?;
        let selected = self.source.point_at(&mapped.curve_parameter())?;
        let uv = self
            .certified_scalar_uv(&selected)?
            .ok_or(GeometryError::UnrepresentableParameter)?;
        self.surface
            .evaluate_real(&Point2::new(uv.x().clone(), uv.y().clone()))
    }

    fn certified_scalar_uv(
        &self,
        selected: &hypercurve::CurvePoint2,
    ) -> GeometryResult<Option<CurvePoint2>> {
        let source = self
            .source
            .source_restriction()
            .map_or(&self.source, |(original, _, _)| original);
        let Some(branch) = source.polynomial_branch() else {
            return Ok(None);
        };
        if branch.image_transform().is_some() || !branch.rational_maps().is_empty() {
            return Ok(None);
        }
        certified_scalar_point(selected, branch.topology().geometric_equation())
    }
}

/// An exact scalar point for a selected planar point lying on the zero set of
/// `defining`: one coordinate is a scalar view, a boundary value 0 or 1, or a
/// projection-event root, and the other a root of the specialized defining
/// polynomial, each proposed by exact candidates and proved by exact
/// comparison. `None` when no candidate is proved.
pub(crate) fn certified_scalar_point(
    selected: &hypercurve::CurvePoint2,
    defining: &hypersolve::BivariatePolynomial,
) -> GeometryResult<Option<CurvePoint2>> {
    use hypersolve::bivariate_arithmetic::{
        bivariate_specialize_first, bivariate_specialize_second,
    };
    {
        let coordinate = |axis: usize| {
            let (a, b) = if axis == 0 {
                (Real::one(), Real::zero())
            } else {
                (Real::zero(), Real::one())
            };
            selected.affine_form(a, b, Real::zero())
        };
        // Proves a selected coordinate equal to one exact candidate.
        let certify = |selected: &hypercurve::CurvePointAffineForm2,
                       candidates: Vec<Real>|
         -> GeometryResult<Option<Real>> {
            first_proved_equal(candidates, |candidate| {
                Ok(selected.compare_to_real(candidate)?)
            })
        };
        for known in [0, 1] {
            // A scalar view, a patch-boundary value, or an event abscissa:
            // branch ends lie on the patch boundary or where the defining
            // polynomial and its derivative across `known` vanish together,
            // so the exact resultant eliminating the other coordinate
            // proposes event candidates.
            let selected_known = coordinate(known);
            let value = match selected_known.parameter()?.scalar().cloned() {
                Some(value) => value,
                None => {
                    let mut candidates = vec![Real::zero(), Real::one()];
                    if let Some(events) = event_polynomial(defining, known) {
                        candidates.extend(exact_root_candidates(events)?);
                    }
                    let Some(value) = certify(&selected_known, candidates)? else {
                        continue;
                    };
                    value
                }
            };
            let polynomial = if known == 0 {
                bivariate_specialize_first(defining, &value)
            } else {
                bivariate_specialize_second(defining, &value)
            };
            let mut candidates = vec![Real::zero(), Real::one()];
            candidates.extend(exact_root_candidates(polynomial)?);
            if let Some(other) = certify(&coordinate(1 - known), candidates)? {
                return Ok(Some(if known == 0 {
                    CurvePoint2::new(value, other)
                } else {
                    CurvePoint2::new(other, value)
                }));
            }
        }
    }
    Ok(None)
}

impl TensorCurve3 {
    /// Joint evaluation at a possibly selected spatial parameter. The UV point
    /// keeps its selected source; the tensor retains its joint rational image.
    pub(super) fn geometric_point_at(&self, s: &Parameter) -> GeometryResult<GeometricPoint3> {
        let mapped = s.affine_image(&self.source_scale, &self.source_offset)?;
        if let Some(value) = mapped.try_real() {
            match represented_curve_point(&self.source, value) {
                Ok(uv) => {
                    return self
                        .surface
                        .evaluate_real(&Point2::new(uv.x().clone(), uv.y().clone()))
                        .map(GeometricPoint3::from_coordinates);
                }
                Err(GeometryError::UnsupportedPcurveContour) => {}
                Err(error) => return Err(error),
            }
        }
        let uv = self.source.point_at(&mapped.curve_parameter())?;
        self.surface.point_at(&uv)
    }

    /// First derivatives where the UV point and source tangent are
    /// represented; higher orders and selected fields remain typed blockers.
    pub(super) fn derivative_at(
        &self,
        s: &Parameter,
        order: usize,
    ) -> GeometryResult<GeometricVector3> {
        if order != 1 {
            return Err(GeometryError::UnsupportedSelectedEvaluation);
        }
        let mapped = s.affine_image(&self.source_scale, &self.source_offset)?;
        let value = mapped
            .try_real()
            .ok_or(GeometryError::UnsupportedSelectedEvaluation)?;
        let uv = represented_curve_point(&self.source, value)?;
        let tangent = self.source.derivatives_at(&mapped.curve_parameter(), 1)?;
        let (du, dv) = tangent[0]
            .represented_coordinates()
            .ok_or(GeometryError::UnsupportedSelectedEvaluation)?;
        let partials = self.surface.partials_at(&uv.into())?;
        let (Some(su), Some(sv)) = (partials.u.try_coordinates(), partials.v.try_coordinates())
        else {
            return Err(GeometryError::UnsupportedSelectedEvaluation);
        };
        Ok(GeometricVector3::from_coordinates(
            (su.clone() * du + sv.clone() * dv) * &self.source_scale,
        ))
    }

    /// The tensor control hull encloses the complete image.
    pub(super) fn bounds(&self) -> GeometryResult<Aabb> {
        match self.surface.bounds()? {
            SurfaceBounds::Bounded(bounds) => Ok(*bounds),
            SurfaceBounds::Unbounded => unreachable!("rational tensors are bounded"),
        }
    }

    pub(super) fn restricted(&self, start: &Real, end: &Real) -> GeometryResult<Curve3> {
        let source_start = self.parameter(start);
        let source_end = self.parameter(end);
        let (lower, upper) = if decided_order(compare_reals(
            &source_start,
            &source_end,
            crate::STRICT_PREDICATES,
        ))? == Ordering::Less
        {
            (&source_start, &source_end)
        } else {
            (&source_end, &source_start)
        };
        let range =
            hypercurve::CurveParameterRange2::try_new(lower.clone().into(), upper.clone().into())?;
        Curve3::from_tensor_image(Self {
            source: self.source.restrict_source(range)?,
            source_scale: source_end - &source_start,
            source_offset: source_start,
            ..self.clone()
        })
    }

    pub(super) fn restricted_to(&self, range: &ParameterRange) -> GeometryResult<Self> {
        let source_range = range.affine_image(&self.source_scale, &self.source_offset)?;
        let domain = self.source.parameter_domain();
        let source = if source_range
            .start()
            .curve_parameter()
            .compare(domain.start())?
            == Ordering::Equal
            && source_range.end().curve_parameter().compare(domain.end())? == Ordering::Equal
        {
            self.source.clone()
        } else {
            self.source
                .restrict_source(hypercurve::CurveParameterRange2::try_new(
                    source_range.start().curve_parameter(),
                    source_range.end().curve_parameter(),
                )?)?
        };
        Ok(Self {
            source,
            ..self.clone()
        })
    }

    /// Positive-weight tensor control points are affine-equivariant, so the
    /// transformed tensor has the same UV chart and the source is unchanged.
    pub(super) fn transformed(&self, transform: &Matrix4) -> GeometryResult<Curve3> {
        let SurfaceGeometry::RationalBezier(surface) = &self.surface.data.geometry else {
            unreachable!("tensor images retain rational Bézier tensors");
        };
        let controls = surface
            .control_points
            .iter()
            .map(|row| {
                row.iter()
                    .map(|point| transform_point(transform, point))
                    .collect::<GeometryResult<Vec<_>>>()
            })
            .collect::<GeometryResult<Vec<_>>>()?;
        Curve3::from_tensor_image(Self {
            surface: Surface::rational_bezier(controls, surface.weights.clone())?,
            ..self.clone()
        })
    }

    /// Complete coefficient/source identity, including a proved unit-U chart
    /// reflection and the source's own reversal parameter transport.
    pub(crate) fn parameterization_equals(&self, other: &Self) -> GeometryResult<bool> {
        let equal = |a: &Real, b: &Real| -> GeometryResult<bool> {
            Ok(decided_order(compare_reals(a, b, crate::STRICT_PREDICATES))? == Ordering::Equal)
        };
        if !equal(&self.source_scale, &other.source_scale)?
            || !equal(&self.source_offset, &other.source_offset)?
        {
            let (scale, offset) = other.source.reversal_parameter_affine_map()?;
            let source_scale = &scale * &other.source_scale;
            let source_offset = scale * &other.source_offset + offset;
            return if equal(&self.source_scale, &source_scale)?
                && equal(&self.source_offset, &source_offset)?
            {
                self.parameterization_equals(&Self {
                    source: other.source.reversed()?,
                    source_scale,
                    source_offset,
                    surface: other.surface.clone(),
                })
            } else {
                Ok(false)
            };
        }
        let a = self
            .source
            .source_restriction()
            .map_or(&self.source, |(source, _, _)| source);
        let b = other
            .source
            .source_restriction()
            .map_or(&other.source, |(source, _, _)| source);
        let same = |a: &Curve2, b: &Curve2| {
            a == b
                || matches!((a.polynomial_branch(),b.polynomial_branch()),
                (Some(a),Some(b)) if a.has_same_parameterization(b))
        };
        if self.surface.has_same_tensor_definition(&other.surface)? && same(a, b) {
            return Ok(true);
        }
        if !self
            .surface
            .tensor_definition_matches(&other.surface, true)?
        {
            return Ok(false);
        }
        // B_i^n(1-u)=B_(n-i)^n(u). Reversing every tensor control/weight
        // row is the exact chart law S_other(u,v)=S_self(1-u,v).
        let reflection = hypercurve::Similarity2::try_from_real_affine(
            -Real::one(),
            Real::zero(),
            Real::zero(),
            Real::one(),
            Real::one(),
            Real::zero(),
        )?;
        let aligned = b.transform_similarity(&reflection)?;
        Ok(same(a, &aligned))
    }
}

impl Surface {
    /// Exact equality of two rational Bézier tensor definitions.
    pub(crate) fn has_same_tensor_definition(&self, other: &Self) -> GeometryResult<bool> {
        if Arc::ptr_eq(&self.data, &other.data) {
            return Ok(true);
        }
        self.tensor_definition_matches(other, false)
    }

    fn tensor_definition_matches(&self, other: &Self, reflect_u: bool) -> GeometryResult<bool> {
        let (SurfaceGeometry::RationalBezier(a), SurfaceGeometry::RationalBezier(b)) =
            (&self.data.geometry, &other.data.geometry)
        else {
            return Ok(false);
        };
        if a.control_points.len() != b.control_points.len()
            || a.control_points[0].len() != b.control_points[0].len()
        {
            return Ok(false);
        }
        let equal = |x: &Real, y: &Real| -> GeometryResult<bool> {
            Ok(decided_order(compare_reals(x, y, crate::STRICT_PREDICATES))? == Ordering::Equal)
        };
        for (row_a, row_b) in a.control_points.iter().zip(&b.control_points) {
            for (i, p) in row_a.iter().enumerate() {
                let q = &row_b[if reflect_u { row_b.len() - 1 - i } else { i }];
                if !equal(&p.x, &q.x)? || !equal(&p.y, &q.y)? || !equal(&p.z, &q.z)? {
                    return Ok(false);
                }
            }
        }
        for (row_a, row_b) in a.weights.iter().zip(&b.weights) {
            for (i, p) in row_a.iter().enumerate() {
                let q = &row_b[if reflect_u { row_b.len() - 1 - i } else { i }];
                if !equal(p, q)? {
                    return Ok(false);
                }
            }
        }
        Ok(true)
    }
}

impl TensorCurve3 {
    /// Deterministic order of two definitions: tensor controls, weights and
    /// parameter map, then source identity. Distinct sources with equal
    /// tensors and maps have no admitted canonical order.
    pub(crate) fn canonical_order(&self, other: &Self) -> GeometryResult<Ordering> {
        let (SurfaceGeometry::RationalBezier(a), SurfaceGeometry::RationalBezier(b)) =
            (&self.surface.data.geometry, &other.surface.data.geometry)
        else {
            unreachable!("tensor images retain rational Bézier tensors");
        };
        let order =
            |x: &Real, y: &Real| decided_order(compare_reals(x, y, crate::STRICT_PREDICATES));
        let shape = (a.control_points.len(), a.control_points[0].len())
            .cmp(&(b.control_points.len(), b.control_points[0].len()));
        if shape != Ordering::Equal {
            return Ok(shape);
        }
        for (row_a, row_b) in a.control_points.iter().zip(&b.control_points) {
            for (p, q) in row_a.iter().zip(row_b) {
                for (x, y) in [(&p.x, &q.x), (&p.y, &q.y), (&p.z, &q.z)] {
                    let value = order(x, y)?;
                    if value != Ordering::Equal {
                        return Ok(value);
                    }
                }
            }
        }
        for (row_a, row_b) in a.weights.iter().zip(&b.weights) {
            for (x, y) in row_a.iter().zip(row_b) {
                let value = order(x, y)?;
                if value != Ordering::Equal {
                    return Ok(value);
                }
            }
        }
        for (x, y) in [
            (&self.source_scale, &other.source_scale),
            (&self.source_offset, &other.source_offset),
        ] {
            let value = order(x, y)?;
            if value != Ordering::Equal {
                return Ok(value);
            }
        }
        if self.source == other.source {
            Ok(Ordering::Equal)
        } else {
            Err(GeometryError::UnsupportedIntersection)
        }
    }

    /// Translates the tensor's control points; the UV chart is unchanged.
    pub(crate) fn translated(&self, offset: &Vector3) -> GeometryResult<Curve3> {
        self.transformed(&Matrix4::affine_translation(offset.0.clone()))
    }
}

/// Exact candidate roots of a univariate polynomial (ascending powers): its
/// rational roots from Hypersolve's bounded exact search with exact
/// deflation, then the radical roots of a remaining factor of degree at most
/// two. Candidates are proposals only; callers prove each by exact comparison.
pub(super) fn exact_root_candidates(mut polynomial: Vec<Real>) -> GeometryResult<Vec<Real>> {
    use hypersolve::curve_resultant::divide_univariate_polynomial_exact;
    use hypersolve::exact_factor::exact_rational_polynomial_root;
    let mut candidates = Vec::new();
    // The bounded rational search skips zero roots: deflate factors of x.
    while polynomial.len() > 1
        && decided_order(compare_reals(
            &polynomial[0],
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))
        .is_ok_and(|order| order == Ordering::Equal)
    {
        if candidates.is_empty() {
            candidates.push(Real::zero());
        }
        polynomial.remove(0);
    }
    while let Some(root) = exact_rational_polynomial_root(&polynomial) {
        candidates.push(root.clone());
        let Some(quotient) = divide_univariate_polynomial_exact(&polynomial, &[-root, Real::one()])
        else {
            return Ok(candidates);
        };
        polynomial = quotient;
    }
    candidates.extend(radical_root_candidates(polynomial));
    Ok(candidates)
}

/// Radical roots of a polynomial without rational roots: directly up to
/// degree four, and above that after splitting off rational quadratic
/// factors found by a bounded Kronecker search and confirmed by exact
/// division. A family whose exact sign decisions are undecided proposes
/// nothing.
fn radical_root_candidates(polynomial: Vec<Real>) -> Vec<Real> {
    let degree = polynomial
        .iter()
        .rposition(|value| !value.definitely_zero())
        .unwrap_or(0);
    if degree <= 4 {
        let mut candidates = low_degree_roots(polynomial.clone()).unwrap_or_default();
        candidates.extend(cubic_roots(polynomial.clone()).unwrap_or_default());
        candidates.extend(quartic_roots(polynomial).unwrap_or_default());
        return candidates;
    }
    let Some((quadratic, quotient)) = rational_quadratic_factor(&polynomial[..=degree]) else {
        return Vec::new();
    };
    let mut candidates = low_degree_roots(quadratic).unwrap_or_default();
    candidates.extend(radical_root_candidates(quotient));
    candidates
}

/// A rational quadratic factor `g` of `polynomial` and the exact quotient.
/// Any such factor of the integer-scaled polynomial `F` satisfies
/// `g(k) | F(k)` for `k` in {0, 1, -1}; candidates interpolated from signed
/// divisor triples are kept only when they divide exactly. Values above
/// 10^6 are not searched.
fn rational_quadratic_factor(polynomial: &[Real]) -> Option<(Vec<Real>, Vec<Real>)> {
    use hypersolve::curve_resultant::divide_univariate_polynomial_exact;
    let parse = |text: String| text.parse::<i128>().ok();
    let mut scale: i128 = 1;
    let mut rationals = Vec::with_capacity(polynomial.len());
    for value in polynomial {
        let rational = value.exact_rational_normal_form()?;
        let denominator = parse(rational.denominator().to_string())?;
        let (mut a, mut b) = (scale, denominator);
        while b != 0 {
            (a, b) = (b, a % b);
        }
        scale = scale.checked_mul(denominator / a)?;
        if scale > 1_000_000_000 {
            return None;
        }
        rationals.push(rational);
    }
    let mut integers = Vec::with_capacity(rationals.len());
    for rational in &rationals {
        let numerator = parse(rational.numerator().to_string())?;
        let denominator = parse(rational.denominator().to_string())?;
        let signed = if Real::new(rational.clone()) < Real::zero() {
            -numerator
        } else {
            numerator
        };
        integers.push(signed.checked_mul(scale / denominator)?);
    }
    let evaluate = |x: i128| {
        integers
            .iter()
            .rev()
            .try_fold(0_i128, |sum, value| sum.checked_mul(x)?.checked_add(*value))
    };
    let values = [evaluate(0)?, evaluate(1)?, evaluate(-1)?];
    if values
        .iter()
        .any(|value| *value == 0 || value.abs() > 1_000_000)
    {
        return None;
    }
    let divisors = |value: i128| {
        let value = value.abs();
        (1..=value)
            .filter(|divisor| value % divisor == 0)
            .flat_map(|divisor| [divisor, -divisor])
            .collect::<Vec<_>>()
    };
    let (zero, one, minus) = (
        divisors(values[0]),
        divisors(values[1]),
        divisors(values[2]),
    );
    let half = |value: i128| (Real::from(value) / Real::from(2)).ok();
    for &c in &zero {
        for &at_one in &one {
            for &at_minus in &minus {
                let a = half(at_one + at_minus)? - Real::from(c);
                if a.definitely_zero() {
                    continue;
                }
                let b = half(at_one - at_minus)?;
                let quadratic = vec![Real::from(c), b, a];
                if let Some(quotient) = divide_univariate_polynomial_exact(polynomial, &quadratic) {
                    return Some((quadratic, quotient));
                }
            }
        }
    }
    None
}

/// The first exactly proved candidate. Unsupported/undecided proposals may
/// be skipped; an actual resource stop is returned before another candidate.
pub(super) fn first_proved_equal(
    candidates: Vec<Real>,
    compare: impl Fn(&Real) -> GeometryResult<Ordering>,
) -> GeometryResult<Option<Real>> {
    let resource_stop = |error: &GeometryError| match error {
        GeometryError::Numeric(hyperreal::Problem::Exhausted)
        | GeometryError::PolynomialTopology(
            hypersolve::bivariate_topology::BivariateTopologyError::BudgetExhausted,
        ) => true,
        GeometryError::PlanarClassificationUnresolved(reason)
        | GeometryError::PlanarCurveConstruction(
            hypercurve::CurveError::UncertainFiniteCurveProjection { reason },
        )
        | GeometryError::PlanarCurve(hypercurve::ExactCurveError::Invalid {
            cause: hypercurve::CurveError::UncertainFiniteCurveProjection { reason },
            ..
        }) => reason.is_resource_failure(),
        GeometryError::PlanarCurve(hypercurve::ExactCurveError::Blocked(cause)) => {
            cause.reason().is_resource_failure()
        }
        _ => false,
    };
    for candidate in candidates {
        match compare(&candidate) {
            Ok(Ordering::Equal) => return Ok(Some(candidate)),
            Err(error) if resource_stop(&error) => return Err(error),
            _ => {}
        }
    }
    Ok(None)
}

/// Exact real cube root, with `cbrt(-x) = -cbrt(x)`.
fn real_cube_root(value: &Real) -> GeometryResult<Real> {
    match decided_order(compare_reals(
        value,
        &Real::zero(),
        crate::STRICT_PREDICATES,
    ))? {
        Ordering::Equal => Ok(Real::zero()),
        Ordering::Greater => value.clone().root_n(3).map_err(GeometryError::Numeric),
        Ordering::Less => Ok(-(-value.clone()).root_n(3).map_err(GeometryError::Numeric)?),
    }
}

/// Exact radical real roots of a cubic (ascending powers, after dropping
/// exactly zero leading coefficients). The shift `x = t - c2/3` gives the
/// monic depressed cubic `t^3 + p t + q` with `D = (q/2)^2 + (p/3)^3`. With
/// `p = 0` the real root is the pure cube root `cbrt(-q)`; otherwise for
/// `D > 0` it is Cardano's `cbrt(-q/2 + sqrt(D)) + cbrt(-q/2 - sqrt(D))`, and
/// for `D = 0` the roots are `3q/p` and `-3q/(2p)`. Three distinct real roots
/// (`D < 0`) have only trigonometric forms and give no candidates. Exact
/// comparisons decide some Cardano forms and not others; callers prove every
/// candidate by exact comparison and skip undecided ones.
fn cubic_roots(mut polynomial: Vec<Real>) -> GeometryResult<Vec<Real>> {
    let sign = |value: &Real| {
        decided_order(compare_reals(
            value,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))
    };
    let divide = |a: Real, b: &Real| (a / b).map_err(GeometryError::Numeric);
    while let Some(leading) = polynomial.last() {
        if sign(leading)? != Ordering::Equal {
            break;
        }
        polynomial.pop();
    }
    if polynomial.len() != 4 {
        return Ok(Vec::new());
    }
    let leading = polynomial[3].clone();
    let c = polynomial
        .iter()
        .map(|value| divide(value.clone(), &leading))
        .collect::<GeometryResult<Vec<_>>>()?;
    let h = divide(-c[2].clone(), &Real::from(3))?;
    // Taylor shift by h: b_k = sum_{i >= k} C(i, k) c_i h^(i - k).
    let binomial = [[1, 0, 0, 0], [1, 1, 0, 0], [1, 2, 1, 0], [1, 3, 3, 1]];
    let powers = [Real::one(), h.clone(), &h * &h, &h * &h * &h];
    let shifted = (0..4)
        .map(|k| {
            (k..4).fold(Real::zero(), |total, i| {
                total + Real::from(binomial[i][k]) * &c[i] * &powers[i - k]
            })
        })
        .collect::<Vec<_>>();
    let (p, q) = (&shifted[1], &shifted[0]);
    let half_q = divide(q.clone(), &Real::from(2))?;
    let third_p = divide(p.clone(), &Real::from(3))?;
    let discriminant = &half_q * &half_q + &third_p * &third_p * &third_p;
    let depressed = match sign(&discriminant)? {
        Ordering::Less => Vec::new(),
        Ordering::Equal => {
            if sign(p)? == Ordering::Equal {
                vec![Real::zero()]
            } else {
                vec![
                    divide(Real::from(3) * q, p)?,
                    divide(-(Real::from(3) * q), &(Real::from(2) * p))?,
                ]
            }
        }
        Ordering::Greater if sign(p)? == Ordering::Equal => vec![real_cube_root(&(-q.clone()))?],
        // Cardano's sum: exact comparisons decide some of these forms and not
        // others; undecided ones certify nothing.
        Ordering::Greater => {
            let root = discriminant.sqrt().map_err(GeometryError::Numeric)?;
            vec![real_cube_root(&(-&half_q + &root))? + real_cube_root(&(-&half_q - &root))?]
        }
    };
    Ok(depressed.into_iter().map(|t| t + &h).collect())
}

/// Exact radical roots of a quartic (ascending powers, after dropping exactly
/// zero leading coefficients), when they exist in a form `Real` represents.
/// The rational shift `x = y - c3/4` gives the monic depressed quartic
/// `y^4 + p y^2 + q y + r`. With `q = 0` its roots are `±sqrt(z)` for the
/// nonnegative roots `z` of `z^2 + p z + r`. Otherwise a rational root `m` of
/// Ferrari's resolvent `8m^3 - 4p m^2 - 8r m + 4pr - q^2` with
/// `s^2 = 2m - p > 0` splits it into `y^2 -+ s y + m +- q/(2s)`. Other quartics
/// give no candidates; candidates are proved by exact comparison by callers.
fn quartic_roots(mut polynomial: Vec<Real>) -> GeometryResult<Vec<Real>> {
    use hypersolve::exact_factor::exact_rational_polynomial_root;
    let sign = |value: &Real| {
        decided_order(compare_reals(
            value,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))
    };
    let divide = |a: Real, b: &Real| (a / b).map_err(GeometryError::Numeric);
    while let Some(leading) = polynomial.last() {
        if sign(leading)? != Ordering::Equal {
            break;
        }
        polynomial.pop();
    }
    if polynomial.len() != 5 {
        return Ok(Vec::new());
    }
    let leading = polynomial[4].clone();
    let c = polynomial
        .iter()
        .map(|value| divide(value.clone(), &leading))
        .collect::<GeometryResult<Vec<_>>>()?;
    // Taylor shift by h = -c3/4: b_k = sum_{i >= k} C(i, k) c_i h^(i - k).
    let h = divide(-c[3].clone(), &Real::from(4))?;
    let binomial = [
        [1, 0, 0, 0, 0],
        [1, 1, 0, 0, 0],
        [1, 2, 1, 0, 0],
        [1, 3, 3, 1, 0],
        [1, 4, 6, 4, 1],
    ];
    let mut powers = vec![Real::one()];
    for _ in 0..4 {
        let next = powers.last().expect("nonempty powers") * &h;
        powers.push(next);
    }
    let shifted = (0..5)
        .map(|k| {
            (k..5).fold(Real::zero(), |total, i| {
                total + Real::from(binomial[i][k]) * &c[i] * &powers[i - k]
            })
        })
        .collect::<Vec<_>>();
    let (p, q, r) = (&shifted[2], &shifted[1], &shifted[0]);
    let mut depressed = Vec::new();
    if sign(q)? == Ordering::Equal {
        for z in low_degree_roots(vec![r.clone(), p.clone(), Real::one()])? {
            match sign(&z)? {
                Ordering::Less => {}
                Ordering::Equal => depressed.push(Real::zero()),
                Ordering::Greater => {
                    let root = z.sqrt().map_err(GeometryError::Numeric)?;
                    depressed.push(-root.clone());
                    depressed.push(root);
                }
            }
        }
    } else {
        // (y^2 + m)^2 - [(2m - p) y^2 - q y + (m^2 - r)]: the bracket is the
        // square (s y - q/(2s))^2 exactly when q^2 = 4(2m - p)(m^2 - r).
        let resolvent = vec![
            Real::from(4) * p * r - q * q,
            -(Real::from(8) * r),
            -(Real::from(4) * p),
            Real::from(8),
        ];
        let mut resolvent = resolvent;
        while let Some(m) = exact_rational_polynomial_root(&resolvent) {
            let square = Real::from(2) * &m - p;
            if sign(&square)? == Ordering::Greater {
                let s = square.sqrt().map_err(GeometryError::Numeric)?;
                let offset = divide(q.clone(), &(Real::from(2) * &s))?;
                for (linear, constant) in [(-s.clone(), &m + &offset), (s.clone(), &m - &offset)] {
                    depressed.extend(low_degree_roots(vec![constant, linear, Real::one()])?);
                }
                break;
            }
            let Some(quotient) = hypersolve::curve_resultant::divide_univariate_polynomial_exact(
                &resolvent,
                &[-m, Real::one()],
            ) else {
                break;
            };
            resolvent = quotient;
        }
    }
    Ok(depressed.into_iter().map(|y| y + &h).collect())
}

/// The exact resultant of `F` and `dF/d(other)` eliminating the other
/// coordinate: a polynomial in coordinate `known` vanishing at every
/// projection event of `F`. `None` when it is not constructed.
fn event_polynomial(defining: &hypersolve::BivariatePolynomial, known: usize) -> Option<Vec<Real>> {
    use hypersolve::bivariate_arithmetic::bivariate_parameter_derivative;
    use hypersolve::curve_resultant::resultant_bivariate_polynomial_system_complete;
    use hypersolve::{
        CurveIntersectionResultantConfig, CurveIntersectionResultantStatus, CurveResultantParameter,
    };
    let (retained, eliminated) = if known == 0 {
        (
            CurveResultantParameter::First,
            CurveResultantParameter::Second,
        )
    } else {
        (
            CurveResultantParameter::Second,
            CurveResultantParameter::First,
        )
    };
    let derivative = bivariate_parameter_derivative(defining, eliminated);
    let report = resultant_bivariate_polynomial_system_complete(
        defining,
        &derivative,
        retained,
        CurveIntersectionResultantConfig::default(),
    );
    (report.status == CurveIntersectionResultantStatus::Constructed
        && report.resultant_coefficients.len() > 1)
        .then_some(report.resultant_coefficients)
}

/// The real roots of a polynomial (ascending powers) of degree one or two
/// after dropping exactly zero leading coefficients, as exact radicals. Other
/// degrees, and a zero polynomial, give no candidates.
fn low_degree_roots(mut polynomial: Vec<Real>) -> GeometryResult<Vec<Real>> {
    let sign = |value: &Real| {
        decided_order(compare_reals(
            value,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))
    };
    while let Some(leading) = polynomial.last() {
        if sign(leading)? != Ordering::Equal {
            break;
        }
        polynomial.pop();
    }
    match polynomial.as_slice() {
        [c, b] => Ok(vec![(-c / b).map_err(GeometryError::Numeric)?]),
        [c, b, a] => {
            let discriminant = b * b - Real::from(4) * a * c;
            let twice = Real::from(2) * a;
            match sign(&discriminant)? {
                Ordering::Less => Ok(Vec::new()),
                Ordering::Equal => Ok(vec![(-b / &twice).map_err(GeometryError::Numeric)?]),
                Ordering::Greater => {
                    let root = discriminant.sqrt().map_err(GeometryError::Numeric)?;
                    Ok(vec![
                        ((-b - &root) / &twice).map_err(GeometryError::Numeric)?,
                        ((-b + &root) / &twice).map_err(GeometryError::Numeric)?,
                    ])
                }
            }
        }
        _ => Ok(Vec::new()),
    }
}

impl TensorCurve3 {
    /// Exact parameter location of a represented point on the tensor image.
    ///
    /// The coordinate pullbacks `g_k = X_k - p_k W` vanish together exactly at
    /// the point's UV preimages. The source is intersected with the complete
    /// zero set of one pullback (Hypercurve native branch intersection), and
    /// a contact is kept only where the other two pullbacks vanish too, by
    /// exact rational forms at its selected UV point. A pullback containing
    /// the source, or with coefficients the topology cannot decompose (a
    /// radical coordinate value), is replaced by another coordinate.
    pub(super) fn parameters_of(&self, point: &Point3) -> GeometryResult<CurveParameterLocation> {
        use hypersolve::BivariatePolynomial;
        use hypersolve::bivariate_arithmetic::{bivariate_scale, bivariate_subtract};
        let SurfaceGeometry::RationalBezier(patch) = &self.surface.data.geometry else {
            unreachable!("tensor images retain rational Bézier tensors");
        };
        let [x, y, z, weight] = surface_point::tensor_coefficients(patch);
        let pullbacks =
            [(&x, &point.x), (&y, &point.y), (&z, &point.z)].map(|(coordinate, value)| {
                bivariate_subtract(coordinate, &bivariate_scale(weight.clone(), value))
            });
        let one = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let vanishes = |uv: &hypercurve::CurvePoint2, polynomial: &BivariatePolynomial| {
            Ok::<bool, GeometryError>(
                uv.rational_form(polynomial.clone(), one.clone())?
                    .compare_to_real(&Real::zero())?
                    == Ordering::Equal,
            )
        };
        let inverse = (Real::one() / &self.source_scale).map_err(GeometryError::Numeric)?;
        let offset = -&self.source_offset * &inverse;
        let spatial_domain = self.spatial_domain()?;
        'primary: for primary in 0..3 {
            let zeros = match super::implicit_tensor::unit_square_zeros(&pullbacks[primary]) {
                Ok(zeros) => zeros,
                // A zero pullback contains the whole patch, rather than being
                // an invalid inverse query. Radical coefficients can also need
                // another coordinate whose topology the owner can decompose.
                Err(GeometryError::PolynomialTopology(
                    hypersolve::bivariate_topology::BivariateTopologyError::UnsupportedCoefficient
                    | hypersolve::bivariate_topology::BivariateTopologyError::ZeroPolynomial,
                )) => continue 'primary,
                Err(error) => return Err(error),
            };
            let others = (0..3).filter(|k| *k != primary).collect::<Vec<_>>();
            let mut found: Vec<Parameter> = Vec::new();
            let mut push = |parameter: Parameter| -> GeometryResult<()> {
                if !spatial_domain.contains(&parameter)? {
                    return Ok(());
                }
                for existing in &found {
                    if existing.compare(&parameter)? == Ordering::Equal {
                        return Ok(());
                    }
                }
                found.push(parameter);
                Ok(())
            };
            for branch in &zeros.curves {
                let intersection = self.source.intersect_curve(branch)?;
                if !intersection.overlaps().is_empty()
                    || !intersection.parameter_components().is_empty()
                {
                    // The source lies along this pullback; use another one.
                    continue 'primary;
                }
                if let Some(blocker) = intersection.blockers().first() {
                    return Err(match blocker.kind() {
                        hypercurve::CurveIntersectionPairBlockerKind2::SharedComponent => {
                            continue 'primary;
                        }
                        hypercurve::CurveIntersectionPairBlockerKind2::Uncertain(reason) => {
                            GeometryError::PlanarClassificationUnresolved(*reason)
                        }
                        hypercurve::CurveIntersectionPairBlockerKind2::IncompleteReplay => {
                            GeometryError::UnsupportedParameterLocation
                        }
                    });
                }
                for contact in intersection.contacts() {
                    let uv = contact.point();
                    if vanishes(uv, &pullbacks[others[0]])? && vanishes(uv, &pullbacks[others[1]])?
                    {
                        push(
                            Parameter::from_curve(&contact.first().parameter()?)?
                                .affine_image(&inverse, &offset)?,
                        )?;
                    }
                }
            }
            for isolated in &zeros.points {
                if vanishes(isolated, &pullbacks[others[0]])?
                    && vanishes(isolated, &pullbacks[others[1]])?
                {
                    match self.source.point_locations(isolated)? {
                        hypercurve::CurvePointLocations2::Components(_) => {
                            return Err(GeometryError::UnsupportedParameterLocation);
                        }
                        hypercurve::CurvePointLocations2::EntireCurve => {
                            return Ok(CurveParameterLocation::EntireDomain);
                        }
                        hypercurve::CurvePointLocations2::Locations(locations) => {
                            for location in locations {
                                push(
                                    Parameter::from_curve(&location.parameter()?)?
                                        .affine_image(&inverse, &offset)?,
                                )?;
                            }
                        }
                    }
                }
            }
            // Order by spatial parameter.
            for index in 1..found.len() {
                let mut position = index;
                while position > 0
                    && found[position - 1].compare(&found[position])? == Ordering::Greater
                {
                    found.swap(position - 1, position);
                    position -= 1;
                }
            }
            return Ok(if found.is_empty() {
                CurveParameterLocation::None
            } else {
                CurveParameterLocation::Parameters(found)
            });
        }
        Err(GeometryError::UnsupportedParameterLocation)
    }
}

#[cfg(test)]
mod quartic_tests {
    use super::*;

    /// (x^2 - 2)(x^3 - x - 1) = x^5 - 3x^3 - x^2 + 2x + 2 has no rational
    /// root; its rational quadratic factor yields exactly +-sqrt(2), and the
    /// cubic quotient its real (Cardano) root, each a root of the quintic.
    #[test]
    fn quintic_roots_split_rational_quadratic_factors() {
        let polynomial = vec![
            2.into(),
            2.into(),
            (-1).into(),
            (-3).into(),
            0.into(),
            1.into(),
        ];
        let candidates = exact_root_candidates(polynomial.clone()).unwrap();
        let root_two = Real::from(2).sqrt().unwrap();
        for expected in [root_two.clone(), -root_two] {
            assert!(candidates.iter().any(|root| {
                compare_reals(root, &expected, crate::STRICT_PREDICATES).value()
                    == Some(Ordering::Equal)
            }));
        }
        let evaluate = |x: &Real| {
            polynomial
                .iter()
                .rev()
                .fold(Real::zero(), |sum, value| sum * x + value)
        };
        // The Cardano root's exact zero test is not structurally decided;
        // a 10^-30 residual bound is (candidates are proposals only).
        assert_eq!(candidates.len(), 3);
        let scale = (0..30).fold(Real::one(), |value, _| value * Real::from(10));
        let tolerance = (Real::one() / scale).unwrap();
        for root in &candidates {
            assert_eq!(
                compare_reals(&evaluate(root).abs(), &tolerance, crate::STRICT_PREDICATES).value(),
                Some(Ordering::Less)
            );
        }
    }

    /// (x - 1)^4 - 5(x - 1)^2 + 4 = ((x-1)^2 - 1)((x-1)^2 - 4): a shifted
    /// biquadratic with roots -1, 0, 2, 3, each verified exactly.
    #[test]
    fn shifted_biquadratic_roots_are_exact() {
        // Expanded: x^4 - 4x^3 + x^2 + 6x.
        let polynomial = vec![0.into(), 6.into(), 1.into(), (-4).into(), 1.into()];
        let roots = quartic_roots(polynomial).unwrap();
        assert_eq!(roots.len(), 4);
        for expected in [-1, 0, 2, 3] {
            assert!(roots.iter().any(|root| {
                decided_order(compare_reals(
                    root,
                    &Real::from(expected),
                    crate::STRICT_PREDICATES,
                ))
                .unwrap()
                    == Ordering::Equal
            }));
        }
    }

    /// (y^2 - y - 1)(y^2 + y - 3) = y^4 - 5y^2 + 2y + 3 has q = 2 != 0. With
    /// p = -5, r = 3 the resolvent 8m^3 + 20m^2 - 24m - 64 = 4(m + 2)(2m^2 +
    /// m - 8) has the rational root m = -2 with s^2 = 2m - p = 1, so the split
    /// recovers both factors; the roots are checked exactly as (1 ± √5)/2 and
    /// (-1 ± √13)/2.
    #[test]
    fn ferrari_split_roots_are_exact() {
        let polynomial = vec![3.into(), 2.into(), (-5).into(), 0.into(), 1.into()];
        let roots = quartic_roots(polynomial).unwrap();
        assert_eq!(roots.len(), 4);
        let five = Real::from(5).sqrt().unwrap();
        let thirteen = Real::from(13).sqrt().unwrap();
        let half = |value: Real| (value / Real::from(2)).unwrap();
        for expected in [
            half(Real::one() + &five),
            half(Real::one() - &five),
            half(-Real::one() + &thirteen),
            half(-Real::one() - &thirteen),
        ] {
            assert!(roots.iter().any(|root| {
                decided_order(compare_reals(root, &expected, crate::STRICT_PREDICATES)).unwrap()
                    == Ordering::Equal
            }));
        }
    }

    fn q(n: i64, d: i64) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }

    /// Hypercurve's complete selected roots on [0, 1] are the independent
    /// authority; each must equal one exact candidate.
    fn assert_candidates_cover(power: Vec<Real>, bernstein: Vec<Real>, expected: usize) {
        let candidates = exact_root_candidates(power).unwrap();
        let roots = super::super::curve_implicit::roots(bernstein).unwrap();
        assert_eq!(roots.len(), expected);
        for (root, _) in roots {
            assert!(candidates.iter().any(|candidate| {
                root.compare(&Parameter::from_real(candidate.clone()))
                    .unwrap()
                    == Ordering::Equal
            }));
        }
    }

    /// x^3 - 1/2 and (x - 1/4)^3 - 1/16: pure cube roots cbrt(1/2) and
    /// 1/4 + cbrt(1/16) equal Hypercurve's selected roots exactly.
    #[test]
    fn pure_cube_roots_are_exact() {
        assert_candidates_cover(
            vec![q(-1, 2), 0.into(), 0.into(), 1.into()],
            vec![q(-1, 2), q(-1, 2), q(-1, 2), q(1, 2)],
            1,
        );
        // (x - 1/4)^3 - 1/16 = x^3 - 3x^2/4 + 3x/16 - 5/64.
        let power = vec![q(-5, 64), q(3, 16), q(-3, 4), 1.into()];
        let bernstein = vec![
            q(-5, 64),
            q(-5, 64) + q(1, 16),
            q(-5, 64) + q(2, 16) + q(-1, 4),
            q(-5, 64) + q(3, 16) + q(-3, 4) + Real::one(),
        ];
        assert_candidates_cover(power, bernstein, 1);
    }

    /// x^3 + x - 1 proposes its Cardano root, whose comparison with
    /// Hypercurve's selected root is undecided: certification finds nothing
    /// and reports no error, leaving the caller's typed outcome.
    #[test]
    fn undecided_cardano_roots_certify_nothing() {
        let candidates =
            exact_root_candidates(vec![(-1).into(), 1.into(), 0.into(), 1.into()]).unwrap();
        assert_eq!(candidates.len(), 1);
        let roots =
            super::super::curve_implicit::roots(vec![(-1).into(), q(-2, 3), q(-1, 3), 1.into()])
                .unwrap();
        let selected = roots[0].0.clone();
        assert!(
            first_proved_equal(candidates, |candidate| {
                selected.compare(&Parameter::from_real(candidate.clone()))
            })
            .unwrap()
            .is_none()
        );
    }

    #[test]
    fn exhausted_candidate_comparison_must_stop_before_a_later_candidate() {
        for error in [
            GeometryError::Numeric(hyperreal::Problem::Exhausted),
            GeometryError::PolynomialTopology(
                hypersolve::bivariate_topology::BivariateTopologyError::BudgetExhausted,
            ),
            GeometryError::PlanarClassificationUnresolved(
                hypercurve::UncertaintyReason::BudgetExhausted,
            ),
            GeometryError::PlanarClassificationUnresolved(
                hypercurve::UncertaintyReason::StorageUnavailable,
            ),
            GeometryError::PlanarCurveConstruction(
                hypercurve::CurveError::UncertainFiniteCurveProjection {
                    reason: hypercurve::UncertaintyReason::BudgetExhausted,
                },
            ),
            GeometryError::PlanarCurveConstruction(
                hypercurve::CurveError::UncertainFiniteCurveProjection {
                    reason: hypercurve::UncertaintyReason::StorageUnavailable,
                },
            ),
        ] {
            let calls = std::cell::Cell::new(0);
            let proved = first_proved_equal(vec![Real::zero(), Real::one()], |_| {
                calls.set(calls.get() + 1);
                if calls.get() == 1 {
                    Err(error.clone())
                } else {
                    Ok(Ordering::Equal)
                }
            });
            assert_eq!(proved, Err(error));
            assert_eq!(calls.get(), 1);
            // Failed attempts retain no state that could poison a retry.
            assert_eq!(
                first_proved_equal(vec![Real::one()], |_| Ok(Ordering::Equal)),
                Ok(Some(Real::one()))
            );
        }
    }

    /// The helper skips a failed comparison and proves the next candidate.
    #[test]
    fn failed_comparisons_do_not_hide_provable_candidates() {
        let proved = first_proved_equal(vec![Real::zero(), Real::one()], |candidate| {
            if *candidate == Real::zero() {
                Err(GeometryError::UnsupportedIntersection)
            } else {
                Ok(Ordering::Equal)
            }
        });
        assert!(proved.unwrap().is_some_and(|value| value == Real::one()));
    }
}

#[cfg(test)]
mod iso_chart_tests {
    use super::*;
    use hypersolve::bivariate_topology::{BivariateCurveDomain, decompose_bivariate_curve};

    fn q(n: i32, d: i32) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }

    fn fixture() -> (Surface, Curve2) {
        // S(u,v)=(u,v,u²+2v), in its exact Bernstein basis.
        let surface = Surface::rational_bezier(
            vec![
                vec![
                    Point3::origin(),
                    Point3::new(q(1, 2), 0.into(), 0.into()),
                    Point3::new(1.into(), 0.into(), 1.into()),
                ],
                vec![
                    Point3::new(0.into(), 1.into(), 2.into()),
                    Point3::new(q(1, 2), 1.into(), 2.into()),
                    Point3::new(1.into(), 1.into(), 3.into()),
                ],
            ],
            vec![vec![Real::one(); 3]; 2],
        )
        .unwrap();
        let topology = decompose_bivariate_curve(
            &hypersolve::BivariatePolynomial::new(vec![vec![(-1).into(), 2.into()]]),
            &BivariateCurveDomain {
                first: [0.into(), 1.into()],
                second: [0.into(), 1.into()],
            },
            Default::default(),
        )
        .unwrap();
        let set =
            hypercurve::PolynomialCurveSet2::from_topology(topology, Default::default()).unwrap();
        assert_eq!(set.curves().len(), 1);
        (surface, set.curves()[0].clone())
    }

    fn assert_controls(curve: &Curve3, expected: Vec<Point3>) {
        let Curve3ExactData::RationalBezier {
            control_points,
            weights,
        } = curve.exact_data()
        else {
            panic!("native tensor iso view must retain its rational family");
        };
        assert_eq!(control_points, expected);
        assert!(weights.iter().all(|w| w == &Real::one()));
    }

    #[test]
    fn swapped_scaled_iso_charts_preserve_exact_world_coefficients_and_traversal() {
        let (surface, source) = fixture();
        for (scale, offset, first, last) in [
            (Real::one(), Real::zero(), Real::zero(), Real::one()),
            (q(1, 2), q(1, 4), q(1, 4), q(3, 4)),
            (q(-1, 2), q(3, 4), q(3, 4), q(1, 4)),
        ] {
            // (u,v)=(scale/2+offset, scale*t+offset).
            let map = hypercurve::Similarity2::try_from_real_affine(
                0.into(),
                scale.clone(),
                scale.clone(),
                0.into(),
                offset.clone(),
                offset.clone(),
            )
            .unwrap();
            let retained =
                Curve3::tensor_image(surface.clone(), source.transform_similarity(&map).unwrap())
                    .unwrap();
            let native = retained.tensor_image_as_iso_curve().unwrap().unwrap();
            let u = &scale * q(1, 2) + &offset;
            let expected =
                |v: &Real| Point3::new(u.clone(), v.clone(), &u * &u + Real::from(2) * v);
            assert_controls(&native, vec![expected(&first), expected(&last)]);
            let reversed = retained
                .reversed()
                .unwrap()
                .tensor_image_as_iso_curve()
                .unwrap()
                .unwrap();
            assert_controls(&reversed, vec![expected(&last), expected(&first)]);
            // Consume the native result in a further exact restriction.
            let trimmed = native.subcurve(&q(1, 4), &q(3, 4)).unwrap();
            assert_controls(
                &trimmed,
                vec![
                    expected(&(&first + (&last - &first) * q(1, 4))),
                    expected(&(&first + (&last - &first) * q(3, 4))),
                ],
            );
        }
    }

    #[test]
    fn ordinary_iso_coefficients_and_non_axis_aligned_refusal_remain_distinct() {
        let (surface, source) = fixture();
        let retained = Curve3::tensor_image(surface.clone(), source.clone()).unwrap();
        assert_controls(
            &retained.tensor_image_as_iso_curve().unwrap().unwrap(),
            vec![
                Point3::new(0.into(), q(1, 2), 1.into()),
                Point3::new(q(1, 2), q(1, 2), 1.into()),
                Point3::new(1.into(), q(1, 2), 2.into()),
            ],
        );
        // Rotated/scaled source lies inside the unit square but is no iso-line.
        let map = hypercurve::Similarity2::try_from_real_affine(
            q(1, 4),
            q(-1, 4),
            q(1, 4),
            q(1, 4),
            q(1, 2),
            q(1, 4),
        )
        .unwrap();
        let retained =
            Curve3::tensor_image(surface, source.transform_similarity(&map).unwrap()).unwrap();
        assert!(retained.tensor_image_as_iso_curve().unwrap().is_none());
    }
}
