//! Exact affine lifts of retained planar curves, without replacing their source.

use super::*;

/// Borrowed definition of a planar spatial curve on its unit interval.
///
/// The image is `origin + u*x(t) + v*y(t)`, where
/// `t = source_start + (source_end-source_start)*s`. The source keeps its
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
    /// Source parameter at spatial parameter zero.
    pub source_start: &'a Real,
    /// Source parameter at spatial parameter one.
    pub source_end: &'a Real,
}

#[derive(Clone, Debug)]
pub(crate) struct PlanarCurve3 {
    pub(crate) source: Curve2,
    pub(crate) origin: Point3,
    pub(crate) u: Vector3,
    pub(crate) v: Vector3,
    pub(crate) source_start: Real,
    pub(crate) source_end: Real,
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
    /// Spatial parameter zero/one follow the source's directed endpoints.
    /// Their scalar views must exist; an unavailable selected-domain view is a
    /// typed error. Native evaluation, inverse queries and source authority are
    /// reused without fitting. This does not certify simplicity or a solid.
    pub fn planar_image(
        source: Curve2,
        origin: Point3,
        u: Vector3,
        v: Vector3,
    ) -> GeometryResult<Self> {
        crate::context::strict(|| {
            let (start, end) = source.parameter_endpoints();
            let source_start = start
                .scalar()
                .cloned()
                .ok_or(GeometryError::UnrepresentableParameter)?;
            let source_end = end
                .scalar()
                .cloned()
                .ok_or(GeometryError::UnrepresentableParameter)?;
            Self::from_planar_image(PlanarCurve3 {
                source,
                origin,
                u,
                v,
                source_start,
                source_end,
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
                &image.source_start,
                &image.source_end,
                crate::STRICT_PREDICATES,
            ))? == Ordering::Equal
            {
                return Err(GeometryError::InvalidParameterDomain);
            }
            // Membership and source validity are owned by the original curve.
            image.source.point_at(&image.source_start.clone().into())?;
            image.source.point_at(&image.source_end.clone().into())?;
            Ok(Self::from_parts(
                CurveGeometry3::PlanarImage(Box::new(image)),
                ParameterDomain::unit(),
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
            source_start: &image.source_start,
            source_end: &image.source_end,
        })
    }
}

impl PlanarCurve3 {
    /// Sufficient whole-image identity in the same spatial unit chart.
    /// It compares definitions after an exactly proved change of plane frame;
    /// no endpoint or sample agreement is used as a complete curve proof.
    pub(crate) fn parameterization_equals(&self, other: &Self) -> GeometryResult<bool> {
        let equal = |a: &Real, b: &Real| -> GeometryResult<bool> {
            Ok(decided_order(compare_reals(a, b, crate::STRICT_PREDICATES))? == Ordering::Equal)
        };
        if !equal(&self.source_start, &other.source_start)?
            || !equal(&self.source_end, &other.source_end)?
        {
            return Ok(false);
        }
        let definitions_equal = |a: &Curve2, b: &Curve2| {
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
        &self.source_start + (&self.source_end - &self.source_start) * s
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
        let scale = &self.source_end - &self.source_start;
        let mut power = Real::one();
        for _ in 0..order {
            power *= &scale;
        }
        Ok((self.u.clone() * x + self.v.clone() * y) * power)
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
        Curve3::from_planar_image(Self {
            source_start: self.parameter(start),
            source_end: self.parameter(end),
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
        let inverse = (Real::one() / (&self.source_end - &self.source_start))
            .map_err(GeometryError::Numeric)?;
        let offset = -&self.source_start * &inverse;
        let mut result = Vec::with_capacity(locations.len());
        for location in locations {
            let parameter =
                Parameter::from_curve(&location.parameter()?)?.affine_image(&inverse, &offset)?;
            if parameter.compare(&Real::zero().into())? != Ordering::Less
                && parameter.compare(&Real::one().into())? != Ordering::Greater
            {
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
