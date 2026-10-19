//! Joint UV evaluation without reconstructing independent selected coordinates.

use super::*;

mod rational;
pub(super) use rational::RationalSurfacePoint;

impl Surface {
    /// Evaluates a joint UV point, retaining its exact source when necessary.
    ///
    /// Stored UV coordinates use the native surface evaluator. A selected point
    /// on a plane retains the plane and the entire UV point, including any shared
    /// root or fiber selector. Its spatial coordinates need not be available as
    /// independent scalars. Coordinate and linear-form queries reuse that source.
    ///
    /// Rational Bézier and NURBS tensors, and extrusions of native rational
    /// profiles, retain joint rational images with checked denominators. NURBS
    /// queries retain the original knot chart, including internal knot ownership.
    /// Closed and lower-bounded domains are checked before evaluation; periodic
    /// axes accept every real parameter. Selected angular images and other
    /// unavailable fields return a typed blocker.
    ///
    /// ```
    /// use hyperbrep::{Surface, Point3, Vector3};
    /// let plane = Surface::plane(Point3::origin(), Vector3::x(), Vector3::y())?;
    /// let uv = hypercurve::Point2::from_values(2, 3).into();
    /// let point = plane.point_at(&uv)?;
    /// assert_eq!(point.try_coordinates(), Some(&Point3::new(2.into(), 3.into(), 0.into())));
    /// # Ok::<(), hyperbrep::GeometryError>(())
    /// ```
    pub fn point_at(&self, parameter: &hypercurve::CurvePoint2) -> GeometryResult<GeometricPoint3> {
        crate::context::strict(|| {
            if let Some(point) = parameter.coordinates() {
                return self
                    .evaluate_real(&Point2::new(point.x().clone(), point.y().clone()))
                    .map(GeometricPoint3::from_coordinates);
            }
            for (domain, axis) in [(&self.data.domain.u, 0), (&self.data.domain.v, 1)] {
                if matches!(
                    domain,
                    SurfaceParameterDomain::Unbounded | SurfaceParameterDomain::Periodic { .. }
                ) {
                    continue;
                }
                let form =
                    parameter.affine_form(Real::from(1 - axis), Real::from(axis), Real::zero());
                let contained = match domain {
                    SurfaceParameterDomain::Closed(range) => {
                        range.contains(&Parameter::from_curve(&form.parameter()?)?)?
                    }
                    SurfaceParameterDomain::LowerBounded { start } => {
                        form.compare_to_real(start)? != Ordering::Less
                    }
                    SurfaceParameterDomain::Unbounded | SurfaceParameterDomain::Periodic { .. } => {
                        true
                    }
                };
                if !contained {
                    return Err(GeometryError::SurfaceParameterOutsideDomain);
                }
            }
            match &self.data.geometry {
                SurfaceGeometry::Plane(_) => Ok(GeometricPoint3::from_surface_image(
                    self.clone(),
                    parameter.clone(),
                )),
                SurfaceGeometry::RationalBezier(_)
                | SurfaceGeometry::Nurbs(_)
                | SurfaceGeometry::Extrusion(_) => {
                    let image = RationalSurfacePoint::new(self, parameter)?;
                    Ok(GeometricPoint3::from_rational_surface_image(
                        self.clone(),
                        parameter.clone(),
                        image,
                    ))
                }
                _ => Err(GeometryError::UnsupportedSelectedEvaluation),
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn represented_uvs_preserve_every_surface_family() {
        let half = (Real::one() / Real::from(2)).unwrap();
        let profile = Curve3::line(
            Point3::new(1.into(), 0.into(), 0.into()),
            Point3::new(2.into(), 0.into(), 0.into()),
        )
        .unwrap();
        let controls = vec![
            vec![Point3::origin(), Point3::new(0.into(), 1.into(), 0.into())],
            vec![
                Point3::new(1.into(), 0.into(), 0.into()),
                Point3::new(1.into(), 1.into(), 1.into()),
            ],
        ];
        let cases = [
            (
                Surface::plane(Point3::origin(), Vector3::x(), Vector3::y()).unwrap(),
                (2.into(), 3.into()),
                Point3::new(2.into(), 3.into(), 0.into()),
            ),
            (
                Surface::cylinder(
                    Point3::origin(),
                    Vector3::x(),
                    Vector3::y(),
                    Vector3::z(),
                    3.into(),
                )
                .unwrap(),
                (0.into(), 2.into()),
                Point3::new(3.into(), 0.into(), 2.into()),
            ),
            (
                Surface::sphere(
                    Point3::origin(),
                    Vector3::x(),
                    Vector3::y(),
                    Vector3::z(),
                    3.into(),
                )
                .unwrap(),
                (0.into(), 0.into()),
                Point3::new(3.into(), 0.into(), 0.into()),
            ),
            (
                Surface::cone(
                    Point3::origin(),
                    Vector3::x(),
                    Vector3::y(),
                    Vector3::z(),
                    (Real::pi() / Real::from(4)).unwrap(),
                )
                .unwrap(),
                (0.into(), 0.into()),
                Point3::origin(),
            ),
            (
                Surface::torus(
                    Point3::origin(),
                    Vector3::x(),
                    Vector3::y(),
                    Vector3::z(),
                    3.into(),
                    1.into(),
                )
                .unwrap(),
                (0.into(), 0.into()),
                Point3::new(4.into(), 0.into(), 0.into()),
            ),
            (
                Surface::extrusion(profile.clone(), Vector3::z()).unwrap(),
                (half.clone(), 3.into()),
                Point3::new(Real::one() + &half, 0.into(), 3.into()),
            ),
            (
                Surface::revolution(profile, Point3::origin(), Vector3::z()).unwrap(),
                ((Real::pi() / Real::from(2)).unwrap(), half.clone()),
                Point3::new(0.into(), Real::one() + &half, 0.into()),
            ),
            (
                Surface::rational_bezier(controls.clone(), vec![vec![1.into(); 2]; 2]).unwrap(),
                (half.clone(), half.clone()),
                Point3::new(half.clone(), half.clone(), &half * &half),
            ),
            (
                Surface::nurbs(
                    1,
                    1,
                    controls,
                    vec![vec![1.into(); 2]; 2],
                    vec![0.into(), 0.into(), 1.into(), 1.into()],
                    vec![0.into(), 0.into(), 1.into(), 1.into()],
                )
                .unwrap(),
                (half.clone(), half.clone()),
                Point3::new(half.clone(), half.clone(), &half * &half),
            ),
        ];
        for (surface, (u, v), expected) in cases {
            let point = surface
                .point_at(&hypercurve::Point2::new(u, v).into())
                .unwrap();
            assert!(point.try_coordinates().is_some());
            assert!(
                point.coincides_with(&expected.into()).unwrap(),
                "{:?}",
                surface.kind()
            );
        }
    }
}
