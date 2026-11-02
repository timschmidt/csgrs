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
        let uv = represented_curve_point(&self.source, &self.parameter(s))?;
        self.surface
            .evaluate_real(&Point2::new(uv.x().clone(), uv.y().clone()))
    }

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

    /// Definition equality in the same spatial chart: equal tensors (control
    /// points and weights) and equal UV sources under equal parameter maps.
    pub(crate) fn parameterization_equals(&self, other: &Self) -> GeometryResult<bool> {
        let equal = |a: &Real, b: &Real| -> GeometryResult<bool> {
            Ok(decided_order(compare_reals(a, b, crate::STRICT_PREDICATES))? == Ordering::Equal)
        };
        if !equal(&self.source_scale, &other.source_scale)?
            || !equal(&self.source_offset, &other.source_offset)?
            || !self.surface.has_same_tensor_definition(&other.surface)?
        {
            return Ok(false);
        }
        let a = self
            .source
            .source_restriction()
            .map_or(&self.source, |(source, _, _)| source);
        let b = other
            .source
            .source_restriction()
            .map_or(&other.source, |(source, _, _)| source);
        Ok(a == b
            || matches!((a.polynomial_branch(), b.polynomial_branch()),
                (Some(a), Some(b)) if a.has_same_parameterization(b)))
    }
}

impl Surface {
    /// Exact equality of two rational Bézier tensor definitions.
    pub(crate) fn has_same_tensor_definition(&self, other: &Self) -> GeometryResult<bool> {
        if Arc::ptr_eq(&self.data, &other.data) {
            return Ok(true);
        }
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
            for (p, q) in row_a.iter().zip(row_b) {
                if !equal(&p.x, &q.x)? || !equal(&p.y, &q.y)? || !equal(&p.z, &q.z)? {
                    return Ok(false);
                }
            }
        }
        for (row_a, row_b) in a.weights.iter().zip(&b.weights) {
            for (p, q) in row_a.iter().zip(row_b) {
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
