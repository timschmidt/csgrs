//! Straight sweeps of retained points through an authored displacement.

use super::*;

/// Definition of a straight unit-chart curve `origin + t * displacement`.
/// The origin retains its own source and selected parameter.
#[derive(Clone, Copy, Debug)]
pub struct TranslationCurveGeometry<'a> {
    /// Exact starting point; independent coordinates may be unavailable.
    pub origin: &'a GeometricPoint3,
    /// Nonzero authored displacement over the unit interval.
    pub displacement: &'a Vector3,
}

#[derive(Clone, Debug)]
pub(crate) struct TranslationCurve3 {
    pub(crate) origin: GeometricPoint3,
    pub(crate) displacement: Vector3,
}

impl Curve3 {
    /// Constructs the straight sweep of a retained point over the unit interval.
    ///
    /// An authored origin uses the native line representation. Otherwise the
    /// source point is retained. Evaluation and subdivision at authored scalar
    /// parameters preserve that source through exact translation; evaluating
    /// an independently selected sweep parameter is currently unsupported.
    pub fn translation(origin: GeometricPoint3, displacement: Vector3) -> GeometryResult<Self> {
        crate::context::strict(|| {
            require_positive(&displacement.norm_squared(), GeometryError::DegenerateLine)?;
            if let Some(point) = origin.try_coordinates() {
                return Self::line(point.clone(), point.clone() + &displacement);
            }
            Ok(Self::from_parts(
                CurveGeometry3::Translation(Box::new(TranslationCurve3 {
                    origin,
                    displacement,
                })),
                ParameterDomain::unit(),
            ))
        })
    }

    /// Borrows the retained straight-sweep definition, when present.
    pub fn translation_geometry(&self) -> Option<TranslationCurveGeometry<'_>> {
        let CurveGeometry3::Translation(line) = &self.data.geometry else {
            return None;
        };
        Some(TranslationCurveGeometry {
            origin: &line.origin,
            displacement: &line.displacement,
        })
    }
}

impl TranslationCurve3 {
    pub(super) fn point_at(&self, parameter: &Real) -> GeometryResult<GeometricPoint3> {
        if parameter == &Real::zero() {
            return Ok(self.origin.clone());
        }
        self.origin
            .transformed(&Matrix4::affine_translation(std::array::from_fn(|axis| {
                &self.displacement.0[axis] * parameter
            })))
    }

    pub(super) fn bounds(&self) -> GeometryResult<Aabb> {
        let a = self.origin.bounds()?;
        let b = self.point_at(&Real::one())?.bounds()?;
        exact_point_bounds(&[a.mins, a.maxs, b.mins, b.maxs])
    }

    pub(super) fn restricted(&self, start: &Real, end: &Real) -> GeometryResult<Curve3> {
        Curve3::translation(
            self.point_at(start)?,
            self.displacement.clone() * (end - start),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn selected_translation_coordinate_query_reports_unrepresentable_point() {
        let parameter = hypercurve::BezierParameter2::Algebraic(
            hypercurve::BezierAlgebraicParameter2::try_isolate(
                hypercurve::BezierParameterPolynomial::try_new_power_basis(vec![
                    (-1).into(),
                    0.into(),
                    0.into(),
                    2.into(),
                ])
                .unwrap(),
                hypercurve::BezierParameterInterval::try_new(
                    (Real::from(3) / Real::from(4)).unwrap(),
                    (Real::from(7) / Real::from(8)).unwrap(),
                )
                .unwrap(),
            )
            .unwrap(),
        );
        let source =
            Curve3::line(Point3::origin(), Point3::new(1.into(), 0.into(), 0.into())).unwrap();
        let origin = source.point_at(&Parameter::from_bezier(parameter)).unwrap();
        let line = Curve3::translation(origin, Vector3::z()).unwrap();
        assert!(matches!(
            line.evaluate_real(&(Real::one() / Real::from(2)).unwrap()),
            Err(GeometryError::UnrepresentablePoint)
        ));
    }
}
