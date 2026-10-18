//! Exact affine lifts of retained planar curves, without replacing their source.

use super::*;

/// Borrowed definition of a planar spatial curve in its retained finite chart.
///
/// The image is `origin + u*x(t) + v*y(t)`, where
/// `t = source_offset + source_scale*s`. The source keeps its
/// procedural and selected-parameter evidence. The frame need not be orthogonal
/// or uniformly scaled. This is a definition, not a topology certificate.
#[derive(Clone, Copy, Debug)]
pub struct PlanarCurveGeometry<'a> {
    /// Original planar source.
    pub source: &'a Curve2,
    /// Origin of the injective affine plane frame.
    pub origin: &'a Point3,
    /// Image of the source x direction.
    pub u: &'a Vector3,
    /// Image of the source y direction.
    pub v: &'a Vector3,
    /// Nonzero scale from spatial to source parameters.
    pub source_scale: &'a Real,
    /// Offset from spatial to source parameters.
    pub source_offset: &'a Real,
}

#[derive(Clone, Debug)]
pub(crate) struct PlanarCurve3 {
    pub(crate) source: Curve2,
    pub(crate) origin: Point3,
    pub(crate) u: Vector3,
    pub(crate) v: Vector3,
    pub(crate) source_scale: Real,
    pub(crate) source_offset: Real,
}

/// Materialize only an authored scalar query through its owning evaluator.
/// A lazy point view is not itself a claim that the coordinates are unavailable.
pub(crate) fn represented_curve_point(curve: &Curve2, t: &Real) -> GeometryResult<CurvePoint2> {
    let point = curve.point_at(&t.clone().into())?;
    if let Some(point) = point.coordinates() {
        return Ok(point.clone());
    }
    if let Some(view) = curve.analytic_parallel() {
        return Ok(view.parallel().point_at(t)?);
    }
    Err(GeometryError::UnsupportedPcurveContour)
}

impl Curve3 {
    /// Lifts an exact planar source through an injective affine plane frame.
    ///
    /// Represented endpoint intervals use the unit chart. Selected intervals
    /// retain the source chart, negated when traversal is reversed. No selected
    /// endpoint is divided or materialized. Native evaluation, inverse queries and source authority are
    /// reused without fitting. This does not certify simplicity or a solid.
    pub fn planar_image(
        source: Curve2,
        origin: Point3,
        u: Vector3,
        v: Vector3,
    ) -> GeometryResult<Self> {
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
            Self::from_planar_image(PlanarCurve3 {
                source,
                origin,
                u,
                v,
                source_scale,
                source_offset,
            })
        })
    }

    pub(crate) fn from_planar_image(image: PlanarCurve3) -> GeometryResult<Self> {
        crate::context::strict(|| {
            require_positive(
                &image.u.cross(&image.v).norm_squared(),
                GeometryError::DegeneratePlaneBasis,
            )?;
            if decided_order(compare_reals(
                &image.source_scale,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? == Ordering::Equal
            {
                return Err(GeometryError::InvalidParameterDomain);
            }
            // Membership and source validity are owned by the original curve.
            let (start, end) = image.source.parameter_endpoints();
            image.source.point_at(start)?;
            image.source.point_at(end)?;
            let domain = image.spatial_domain()?;
            Ok(Self::from_parts(
                CurveGeometry3::PlanarImage(Box::new(image)),
                domain,
            ))
        })
    }

    /// Borrows the complete retained planar source, frame and parameter map.
    pub fn planar_geometry(&self) -> Option<PlanarCurveGeometry<'_>> {
        let CurveGeometry3::PlanarImage(image) = &self.data.geometry else {
            return None;
        };
        Some(PlanarCurveGeometry {
            source: &image.source,
            origin: &image.origin,
            u: &image.u,
            v: &image.v,
            source_scale: &image.source_scale,
            source_offset: &image.source_offset,
        })
    }
}

impl PlanarCurve3 {
    /// Sufficient whole-image identity in the same spatial chart.
    /// It compares definitions after an exactly proved change of plane frame;
    /// no endpoint or sample agreement is used as a complete curve proof.
    pub(crate) fn parameterization_equals(&self, other: &Self) -> GeometryResult<bool> {
        let equal = |a: &Real, b: &Real| -> GeometryResult<bool> {
            Ok(decided_order(compare_reals(a, b, crate::STRICT_PREDICATES))? == Ordering::Equal)
        };
        if !equal(&self.source_scale, &other.source_scale)?
            || !equal(&self.source_offset, &other.source_offset)?
        {
            return Ok(false);
        }
        let definitions_equal = |a: &Curve2, b: &Curve2| {
            // Source restrictions and their traversal retain the authored
            // parameter chart. The caller checks the requested domain and
            // endpoint map; complete carrier equality is independent of a
            // restriction's traversal flag and selected interval.
            let a = a.source_restriction().map_or(a, |(source, _, _)| source);
            let b = b.source_restriction().map_or(b, |(source, _, _)| source);
            a == b
                || matches!((a.analytic_parallel(), b.analytic_parallel()),
                (Some(a), Some(b)) if a.parallel() == b.parallel())
        };
        let mut frame_equal = true;
        for (a, b) in [
            &self.origin.x,
            &self.origin.y,
            &self.origin.z,
            &self.u[0],
            &self.u[1],
            &self.u[2],
            &self.v[0],
            &self.v[1],
            &self.v[2],
        ]
        .into_iter()
        .zip([
            &other.origin.x,
            &other.origin.y,
            &other.origin.z,
            &other.u[0],
            &other.u[1],
            &other.u[2],
            &other.v[0],
            &other.v[1],
            &other.v[2],
        ]) {
            if !equal(a, b)? {
                frame_equal = false;
                break;
            }
        }
        if frame_equal {
            return Ok(definitions_equal(&self.source, &other.source));
        }
        let normal = self.u.cross(&self.v);
        for vector in [
            &other.origin - &self.origin,
            other.u.clone(),
            other.v.clone(),
        ] {
            if !equal(&normal.dot(&vector), &Real::zero())? {
                return Ok(false);
            }
        }
        let origin = project_point_to_plane_frame(&self.origin, &self.u, &self.v, &other.origin)?;
        let x = project_point_to_plane_frame(
            &self.origin,
            &self.u,
            &self.v,
            &(other.origin.clone() + &other.u),
        )?;
        let y = project_point_to_plane_frame(
            &self.origin,
            &self.u,
            &self.v,
            &(other.origin.clone() + &other.v),
        )?;
        let transform = hypercurve::Similarity2::try_from_real_affine(
            &x.x - &origin.x,
            &y.x - &origin.x,
            &x.y - &origin.y,
            &y.y - &origin.y,
            origin.x,
            origin.y,
        )?;
        let projected = other.source.transform_similarity(&transform)?;
        Ok(definitions_equal(&self.source, &projected))
    }

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

    pub(super) fn lift(&self, point: &CurvePoint2) -> Point3 {
        self.origin.clone() + self.u.clone() * point.x() + self.v.clone() * point.y()
    }

    pub(super) fn point_at(&self, s: &Real) -> GeometryResult<Point3> {
        Ok(self.lift(&represented_curve_point(&self.source, &self.parameter(s))?))
    }

    pub(super) fn derivative_at(&self, s: &Real, order: usize) -> GeometryResult<Vector3> {
        let derivatives = self
            .source
            .derivatives_at(&self.parameter(s).into(), order)?;
        let (x, y) = derivatives
            .last()
            .expect("positive derivative order")
            .represented_coordinates()
            .ok_or(GeometryError::UnsupportedSelectedEvaluation)?;
        let mut power = Real::one();
        for _ in 0..order {
            power *= &self.source_scale;
        }
        Ok((self.u.clone() * x + self.v.clone() * y) * power)
    }

    /// Preserve the owner's chart and return its exact map to the target
    /// spatial interval. A procedural planar image needs a certified similarity
    /// in parameter space; arbitrary affine source images remain unsupported.
    pub(super) fn materialize_plane_projection(
        &self,
        origin: &Point3,
        u: &Vector3,
        v: &Vector3,
    ) -> GeometryResult<MaterializedSurfacePcurve> {
        let project = |point: &Point3| project_point_to_plane_frame(origin, u, v, point);
        let projected_origin = project(&self.origin)?;
        let x = project(&(self.origin.clone() + &self.u))?;
        let y = project(&(self.origin.clone() + &self.v))?;
        let transform = hypercurve::Similarity2::try_from_real_affine(
            &x.x - &projected_origin.x,
            &y.x - &projected_origin.x,
            &x.y - &projected_origin.y,
            &y.y - &projected_origin.y,
            projected_origin.x,
            projected_origin.y,
        )?;
        let curve = self.source.transform_similarity(&transform)?;
        let scale = (Real::one() / &self.source_scale).map_err(GeometryError::Numeric)?;
        let offset = -&self.source_offset * &scale;
        Ok(MaterializedSurfacePcurve {
            curve,
            correspondence: SurfacePcurveCorrespondence::Affine { scale, offset },
        })
    }

    pub(super) fn bounds(&self) -> GeometryResult<Aabb> {
        let bounds = self.source.bounds()?;
        exact_point_bounds(&[
            self.lift(&CurvePoint2::new(
                bounds.min_x().clone(),
                bounds.min_y().clone(),
            )),
            self.lift(&CurvePoint2::new(
                bounds.min_x().clone(),
                bounds.max_y().clone(),
            )),
            self.lift(&CurvePoint2::new(
                bounds.max_x().clone(),
                bounds.min_y().clone(),
            )),
            self.lift(&CurvePoint2::new(
                bounds.max_x().clone(),
                bounds.max_y().clone(),
            )),
        ])
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
        Curve3::from_planar_image(Self {
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

    pub(super) fn parameters_of(&self, point: &Point3) -> GeometryResult<CurveParameterLocation> {
        let normal = self.u.cross(&self.v);
        if decided_order(compare_reals(
            &normal.dot(&(point - &self.origin)),
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != Ordering::Equal
        {
            return Ok(CurveParameterLocation::None);
        }
        let uv = project_point_to_plane_frame(&self.origin, &self.u, &self.v, point)?;
        let locations = self
            .source
            .point_locations(&CurvePoint2::new(uv.x, uv.y).into())?;
        let locations = match locations {
            hypercurve::CurvePointLocations2::EntireCurve => {
                return Ok(CurveParameterLocation::EntireDomain);
            }
            hypercurve::CurvePointLocations2::Locations(locations) => locations,
        };
        let inverse = (Real::one() / &self.source_scale).map_err(GeometryError::Numeric)?;
        let offset = -&self.source_offset * &inverse;
        let domain = self.spatial_domain()?;
        let mut result = Vec::with_capacity(locations.len());
        for location in locations {
            let parameter =
                Parameter::from_curve(&location.parameter()?)?.affine_image(&inverse, &offset)?;
            if domain.contains(&parameter)? {
                result.push(parameter);
            }
        }
        // A reversed chart reverses the source query's ordered roots.
        if decided_order(compare_reals(
            &inverse,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? == Ordering::Less
        {
            result.reverse();
        }
        Ok(if result.is_empty() {
            CurveParameterLocation::None
        } else {
            CurveParameterLocation::Parameters(result)
        })
    }

    pub(super) fn transformed(&self, transform: &Matrix4) -> GeometryResult<Curve3> {
        Curve3::from_planar_image(Self {
            origin: transform_point(transform, &self.origin)?,
            u: transform.transform_direction3(&self.u),
            v: transform.transform_direction3(&self.v),
            ..self.clone()
        })
    }
}
