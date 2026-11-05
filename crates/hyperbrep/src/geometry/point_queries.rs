//! Correlated spatial queries retain their original point fields.

use super::*;
use hypersolve::BivariatePolynomial;

impl GeometricPoint3 {
    /// The exact affine functional `direction · point + offset`, as a retained
    /// parameter. No independent coordinate roots are reconstructed.
    pub(crate) fn linear_form_parameter(
        &self,
        direction: &Vector3,
        offset: &Real,
    ) -> GeometryResult<Parameter> {
        crate::context::strict(|| {
            if let Some(point) = self.try_coordinates() {
                return Ok((direction.dot(&Vector3::from(point.clone())) + offset).into());
            }
            if let Some((image, uv)) = self.rational_surface_source() {
                let form = image.linear_form(uv, direction)?;
                return Parameter::from_curve(&form.parameter()?)?
                    .affine_image(&Real::one(), offset);
            }
            if let Some(frame) = self.planar_source() {
                return Parameter::from_curve(
                    &frame
                        .point
                        .affine_form(
                            direction.dot(frame.u),
                            direction.dot(frame.v),
                            direction.dot(&Vector3::from(frame.origin.clone())) + offset,
                        )
                        .parameter()?,
                );
            }
            let PointData3::CurveImage(image) = &self.data else {
                return Err(GeometryError::UnsupportedSelectedEvaluation);
            };
            let curve = image.curve.planar_control_image(|point| {
                Point2::new(
                    direction.dot(&Vector3::from(point.clone())) + offset,
                    Real::zero(),
                )
            })?;
            Parameter::from_curve(
                &curve
                    .point_at(&image.parameter.curve_parameter())?
                    .affine_form(Real::one(), Real::zero(), Real::zero())
                    .parameter()?,
            )
        })
    }

    /// Exact affine projection into a nonsingular plane frame, retaining the
    /// whole joint point and the source's original denominator guards.
    pub(crate) fn projected_to_plane_frame(
        &self,
        origin: &Point3,
        u: &Vector3,
        v: &Vector3,
    ) -> GeometryResult<CurvePoint2> {
        crate::context::strict(|| {
            if let Some(point) = self.try_coordinates() {
                let point = super::super::project_point_to_plane_frame(origin, u, v, point)?;
                return Ok(Point2::new(point.x, point.y).into());
            }
            let uu = u.dot(u);
            let uv = u.dot(v);
            let vv = v.dot(v);
            let inverse =
                (Real::one() / (&uu * &vv - &uv * &uv)).map_err(GeometryError::Numeric)?;
            let dual = [
                (u.clone() * &vv - v.clone() * &uv) * &inverse,
                (v.clone() * &uu - u.clone() * &uv) * inverse,
            ];
            let constants = dual
                .each_ref()
                .map(|d| -d.dot(&Vector3::from(origin.clone())));
            if let Some((image, point)) = self.rational_surface_source() {
                let forms = [
                    image.linear_form(point, &dual[0])?,
                    image.linear_form(point, &dual[1])?,
                ];
                use hypersolve::bivariate_arithmetic::{bivariate_add, bivariate_scale};
                let numerators = std::array::from_fn(|i| {
                    bivariate_add(
                        forms[i].numerator(),
                        &bivariate_scale(forms[i].denominator().clone(), &constants[i]),
                    )
                });
                let map = hypercurve::RationalMap2::new(numerators, forms[0].denominator().clone());
                return Ok(point.rational_image(&map)?);
            }
            if let Some(frame) = self.planar_source() {
                let polynomials = std::array::from_fn(|i| {
                    BivariatePolynomial::new(vec![
                        vec![
                            dual[i].dot(&Vector3::from(frame.origin.clone())) + &constants[i],
                            dual[i].dot(frame.v),
                        ],
                        vec![dual[i].dot(frame.u)],
                    ])
                });
                return Ok(frame.point.rational_image(&hypercurve::RationalMap2::new(
                    polynomials,
                    BivariatePolynomial::new(vec![vec![Real::one()]]),
                ))?);
            }
            let PointData3::CurveImage(image) = &self.data else {
                return Err(GeometryError::UnsupportedSelectedEvaluation);
            };
            image
                .curve
                .planar_control_image(|point| {
                    let point = Vector3::from(point.clone());
                    Point2::new(
                        dual[0].dot(&point) + &constants[0],
                        dual[1].dot(&point) + &constants[1],
                    )
                })?
                .point_at(&image.parameter.curve_parameter())
                .map_err(GeometryError::from)
        })
    }
}

impl From<&Point3> for GeometricPoint3 {
    fn from(point: &Point3) -> Self {
        Self::from_coordinates(point.clone())
    }
}
impl From<&GeometricPoint3> for GeometricPoint3 {
    fn from(point: &GeometricPoint3) -> Self {
        point.clone()
    }
}

impl Curve3 {
    pub(in crate::geometry) fn selected_line_parameters(
        &self,
        line: &super::super::Line3,
        point: &GeometricPoint3,
    ) -> GeometryResult<super::super::CurveParameterLocation> {
        use super::super::CurveParameterLocation;
        let direction = &line.end - &line.start;
        // All cross-product equations are required for spatial incidence.
        // A single planar projection is not a proof that the point is on it.
        let normals = [
            Vector3::new([Real::zero(), -&direction[2], direction[1].clone()]),
            Vector3::new([direction[2].clone(), Real::zero(), -&direction[0]]),
            Vector3::new([-&direction[1], direction[0].clone(), Real::zero()]),
        ];
        for normal in normals {
            if point
                .compare_linear_form(&normal, &normal.dot(&Vector3::from(line.start.clone())))?
                != Ordering::Equal
            {
                return Ok(CurveParameterLocation::None);
            }
        }
        let inverse = (Real::one() / direction.norm_squared()).map_err(GeometryError::Numeric)?;
        let parameter = point.linear_form_parameter(
            &(direction.clone() * &inverse),
            &(-direction.dot(&Vector3::from(line.start.clone())) * inverse),
        )?;
        Ok(if self.domain().contains(&parameter)? {
            CurveParameterLocation::Parameters(vec![parameter])
        } else {
            CurveParameterLocation::None
        })
    }
}

impl Curve3 {
    /// A complete affine parameter correspondence between two native line
    /// supports. Restrictions must be transported and checked separately.
    pub(crate) fn affine_line_parameter_map_to(
        &self,
        target: &Self,
    ) -> GeometryResult<Option<(Real, Real)>> {
        let (CurveGeometry3::Line(first), CurveGeometry3::Line(second)) =
            (&self.data.geometry, &target.data.geometry)
        else {
            return Ok(None);
        };
        let first_direction = &first.end - &first.start;
        let second_direction = &second.end - &second.start;
        let displacement = &first.start - &second.start;
        for residual in [
            first_direction.cross(&second_direction).norm_squared(),
            displacement.cross(&second_direction).norm_squared(),
        ] {
            match super::super::decided_order(compare_reals(
                &residual,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? {
                Ordering::Equal => {}
                _ => return Ok(None),
            }
        }
        // Both nonzero directions were admitted by their constructors.
        let inverse =
            (Real::one() / second_direction.norm_squared()).map_err(GeometryError::Numeric)?;
        Ok(Some((
            first_direction.dot(&second_direction) * &inverse,
            displacement.dot(&second_direction) * inverse,
        )))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn native_line_chart_transport_preserves_selected_restrictions() {
        let half = (Real::one() / Real::from(2)).unwrap();
        let source =
            Curve3::line(Point3::origin(), Point3::new(4.into(), 0.into(), 0.into())).unwrap();
        let target = Curve3::line(
            Point3::new((-2).into(), 0.into(), 0.into()),
            Point3::new(2.into(), 0.into(), 0.into()),
        )
        .unwrap();
        let roots = hypercurve::BezierParameterPolynomial::try_new_power_basis(
            [-1, 1, 0, 0, 0, 1].into_iter().map(Real::from).collect(),
        )
        .unwrap()
        .isolate_unit_interval_roots()
        .unwrap();
        let alpha = Parameter::from_curve(&roots[0].clone().into()).unwrap();
        let start = alpha.affine_image(&half, &Real::zero()).unwrap();
        let range = crate::ParameterRange::new(start.clone(), half.clone().into()).unwrap();
        for target in [target.clone(), target.reversed().unwrap()] {
            let (scale, offset) = source
                .affine_line_parameter_map_to(&target)
                .unwrap()
                .unwrap();
            let mapped = range.affine_image(&scale, &offset).unwrap();
            for parameter in [&start, &Parameter::from_real(half.clone())] {
                let transported = parameter.affine_image(&scale, &offset).unwrap();
                assert!(mapped.contains(&transported).unwrap());
                assert!(
                    source
                        .point_at(parameter)
                        .unwrap()
                        .coincides_with(&target.point_at(&transported).unwrap())
                        .unwrap()
                );
            }
        }
        let epsilon = (Real::one() / Real::from(1_u64 << 60)).unwrap();
        let distinct = Curve3::line(
            Point3::new(0.into(), epsilon.clone(), 0.into()),
            Point3::new(4.into(), epsilon, 0.into()),
        )
        .unwrap();
        assert!(
            source
                .affine_line_parameter_map_to(&distinct)
                .unwrap()
                .is_none()
        );
        let transverse =
            Curve3::line(Point3::origin(), Point3::new(0.into(), 4.into(), 0.into())).unwrap();
        assert!(
            source
                .affine_line_parameter_map_to(&transverse)
                .unwrap()
                .is_none()
        );
    }
}

impl Curve3 {
    /// Sufficient whole-image proof of `self(t) = target(a*t+b)`. This does
    /// not assert equality of active domains; callers must transport them.
    pub(crate) fn affine_parameter_map_to(
        &self,
        target: &Self,
    ) -> GeometryResult<Option<(Real, Real)>> {
        crate::context::strict(|| {
            if self.has_certified_same_parameterization(target) {
                return Ok(Some((Real::one(), Real::zero())));
            }
            if let Some(map) = self.affine_line_parameter_map_to(target)? {
                return Ok(Some(map));
            }
            let (source, self_scale, self_offset, target_scale, target_offset) =
                match (&self.data.geometry, &target.data.geometry) {
                    (CurveGeometry3::PlanarImage(a), CurveGeometry3::PlanarImage(b)) => (
                        &a.source,
                        &a.source_scale,
                        &a.source_offset,
                        &b.source_scale,
                        &b.source_offset,
                    ),
                    (CurveGeometry3::TensorImage(a), CurveGeometry3::TensorImage(b)) => (
                        &a.source,
                        &a.source_scale,
                        &a.source_offset,
                        &b.source_scale,
                        &b.source_offset,
                    ),
                    _ => return Ok(None),
                };
            let inverse = (Real::one() / target_scale).map_err(GeometryError::Numeric)?;
            let reversal = source.reversal_parameter_affine_map()?;
            for (chart_scale, chart_offset) in [(Real::one(), Real::zero()), reversal] {
                let scale = &chart_scale * self_scale * &inverse;
                let offset = (&chart_scale * self_offset + chart_offset - target_offset) * &inverse;
                let mapped_scale = target_scale * &scale;
                let mapped_offset = target_scale * &offset + target_offset;
                let equal = match (&self.data.geometry, &target.data.geometry) {
                    (CurveGeometry3::PlanarImage(a), CurveGeometry3::PlanarImage(b)) => a
                        .parameterization_equals(&super::super::PlanarCurve3 {
                            source_scale: mapped_scale,
                            source_offset: mapped_offset,
                            ..(**b).clone()
                        })?,
                    (CurveGeometry3::TensorImage(a), CurveGeometry3::TensorImage(b)) => a
                        .parameterization_equals(&super::super::TensorCurve3 {
                            source_scale: mapped_scale,
                            source_offset: mapped_offset,
                            ..(**b).clone()
                        })?,
                    _ => unreachable!("families checked above"),
                };
                if equal {
                    return Ok(Some((scale, offset)));
                }
            }
            Ok(None)
        })
    }
}

#[cfg(test)]
mod image_map_tests {
    use super::*;

    #[test]
    fn intersecting_plane_line_images_prove_the_full_parameter_law() {
        let half = (Real::one() / Real::from(2)).unwrap();
        let quarter = (Real::one() / Real::from(4)).unwrap();
        let first_source = Curve2::from(
            hypercurve::LineSeg2::try_new(Point2::from_values(0, -1), Point2::from_values(0, 1))
                .unwrap(),
        );
        let target_source = Curve2::from(
            hypercurve::LineSeg2::try_new(
                Point2::new(quarter.clone(), half.clone()),
                Point2::new(&quarter + &half, half.clone()),
            )
            .unwrap(),
        );
        let first = Curve3::planar_image(
            first_source,
            Point3::new(0.into(), 0.into(), half.clone()),
            Vector3::x(),
            Vector3::y(),
        )
        .unwrap();
        let target = Curve3::planar_image(
            target_source,
            Point3::new(0.into(), (-2).into(), (-2).into()),
            Vector3::y() * Real::from(4),
            Vector3::z() * Real::from(5),
        )
        .unwrap();
        let roots = hypercurve::BezierParameterPolynomial::try_new_power_basis(
            [-1, 1, 0, 0, 0, 1].into_iter().map(Real::from).collect(),
        )
        .unwrap()
        .isolate_unit_interval_roots()
        .unwrap();
        let alpha = Parameter::from_curve(&roots[0].clone().into()).unwrap();
        let range = crate::ParameterRange::new(alpha.clone(), Real::one().into()).unwrap();
        let a = first.restrict_source(range.clone()).unwrap();
        let b = target.restrict_source(range).unwrap();
        let map = a.affine_parameter_map_to(&b).unwrap().unwrap();
        assert_eq!(map, (Real::one(), Real::zero()));
        for parameter in [&alpha, &Real::one().into()] {
            assert!(
                a.point_at(parameter)
                    .unwrap()
                    .coincides_with(&b.point_at(parameter).unwrap())
                    .unwrap()
            );
        }
        // Equal endpoints do not prove the affine parameter law: this source
        // traverses the same straight support quadratically.
        let nonlinear = Curve2::from(hypercurve::QuadraticBezier2::new(
            Point2::new(quarter.clone(), half.clone()),
            Point2::new(quarter.clone(), half.clone()),
            Point2::new(quarter + &half, half.clone()),
        ));
        let nonlinear = Curve3::planar_image(
            nonlinear,
            Point3::new(0.into(), (-2).into(), (-2).into()),
            Vector3::y() * Real::from(4),
            Vector3::z() * Real::from(5),
        )
        .unwrap();
        assert!(first.affine_parameter_map_to(&nonlinear).unwrap().is_none());
        assert!(
            !first
                .point_at(&half.clone().into())
                .unwrap()
                .coincides_with(&nonlinear.point_at(&half.into()).unwrap())
                .unwrap()
        );
    }

    #[test]
    fn selected_image_maps_transport_reversal_restriction_and_frame_evidence() {
        let roots = hypercurve::BezierParameterPolynomial::try_new_power_basis(
            [-1, 1, 0, 0, 0, 1].into_iter().map(Real::from).collect(),
        )
        .unwrap()
        .isolate_unit_interval_roots()
        .unwrap();
        let alpha: hypercurve::CurveParameter2 = roots[0].clone().into();
        let point = |y: Real| {
            Curve2::from(
                hypercurve::LineSeg2::try_new(
                    Point2::new(Real::zero(), y.clone()),
                    Point2::new(Real::one(), y),
                )
                .unwrap(),
            )
            .point_at(&alpha)
            .unwrap()
        };
        let uv = Curve2::try_line(point(Real::zero()), point(Real::one())).unwrap();
        let half = (Real::one() / Real::from(2)).unwrap();
        let hypercurve::CurvePointLocations2::Locations(locations) =
            uv.point_locations(&point(half)).unwrap()
        else {
            panic!("one interior line parameter");
        };
        let midpoint = Parameter::from_curve(&locations[0].parameter().unwrap()).unwrap();
        let planar =
            Curve3::planar_image(uv.clone(), Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
        let tensor = Surface::rational_bezier(
            vec![
                vec![Point3::origin(), Point3::new(1.into(), 0.into(), 0.into())],
                vec![
                    Point3::new(0.into(), 1.into(), 0.into()),
                    Point3::new(1.into(), 1.into(), 1.into()),
                ],
            ],
            vec![vec![Real::one(); 2]; 2],
        )
        .unwrap();
        for curve in [
            planar.clone(),
            Curve3::tensor_image(tensor, uv.clone()).unwrap(),
        ] {
            let reversed = curve.reversed().unwrap();
            let (scale, offset) = curve.affine_parameter_map_to(&reversed).unwrap().unwrap();
            assert_eq!(scale, -Real::one());
            assert_eq!(offset, Real::zero());
            let range =
                crate::ParameterRange::new(curve.domain().start().clone(), midpoint.clone())
                    .unwrap();
            let left = curve.restrict_source(range.clone()).unwrap();
            let mapped = reversed
                .restrict_source(range.affine_image(&scale, &offset).unwrap())
                .unwrap();
            let (a, b) = left.affine_parameter_map_to(&mapped).unwrap().unwrap();
            for parameter in [left.domain().start(), left.domain().end()] {
                let target = parameter.affine_image(&a, &b).unwrap();
                assert!(mapped.domain().contains(&target).unwrap());
                assert!(
                    left.point_at(parameter)
                        .unwrap()
                        .coincides_with(&mapped.point_at(&target).unwrap())
                        .unwrap()
                );
            }
        }
        let reflection = hypercurve::Similarity2::try_from_real_affine(
            (-1).into(),
            0.into(),
            0.into(),
            1.into(),
            0.into(),
            0.into(),
        )
        .unwrap();
        let reflected = Curve3::planar_image(
            uv.transform_similarity(&reflection).unwrap(),
            Point3::origin(),
            -Vector3::x(),
            Vector3::y(),
        )
        .unwrap();
        // The world line is identical, but its point-order parameters live in
        // different UV frames. This is not an affine scalar-chart alias.
        assert!(
            planar
                .affine_parameter_map_to(&reflected)
                .unwrap()
                .is_none()
        );
        let separate = Curve3::planar_image(
            uv,
            Point3::new(0.into(), 0.into(), 1.into()),
            Vector3::x(),
            Vector3::y(),
        )
        .unwrap();
        assert!(planar.affine_parameter_map_to(&separate).unwrap().is_none());
    }
}
