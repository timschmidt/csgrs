//! Selected spatial queries reuse the planar owner's complete map preimages.
use super::*;

impl TensorCurve3 {
    pub(super) fn mapped_parameters_of(
        &self,
        curve: &Curve3,
        point: &GeometricPoint3,
    ) -> GeometryResult<CurveParameterLocation> {
        let SurfaceGeometry::RationalBezier(patch) = &self.surface.data.geometry else {
            unreachable!("tensor images retain rational Bézier tensors");
        };
        let [x, y, z, weight] = surface_point::tensor_coefficients(patch);
        let coordinates = [&x, &y, &z];
        let axes = [Vector3::x(), Vector3::y(), Vector3::z()];
        let inverse = (Real::one() / &self.source_scale).map_err(GeometryError::Numeric)?;
        let offset = -&self.source_offset * &inverse;
        let mut accumulated = hypercurve::CurvePointLocations2::EntireCurve;
        let mut blocker = None;
        for (first, second) in [(0, 1), (0, 2), (1, 2)] {
            let query =
                point.projected_to_plane_frame(&Point3::origin(), &axes[first], &axes[second])?;
            let map = hypercurve::RationalMap2::new(
                [coordinates[first].clone(), coordinates[second].clone()],
                weight.clone(),
            );
            let locations = match self.source.rational_image_point_locations(&map, &query) {
                Ok(locations) => locations,
                Err(hypercurve::ExactCurveError::Blocked(cause))
                    if cause.reason() == hypercurve::UncertaintyReason::Unsupported =>
                {
                    blocker = Some(GeometryError::from(hypercurve::ExactCurveError::Blocked(
                        cause,
                    )));
                    continue;
                }
                Err(error) => return Err(error.into()),
            };
            accumulated = self
                .source
                .intersect_point_locations(&accumulated, &locations)?;
            let hypercurve::CurvePointLocations2::Locations(locations) = &accumulated else {
                continue;
            };
            let mut found: Vec<Parameter> = Vec::new();
            for location in locations {
                let parameter = Parameter::from_curve(&location.parameter()?)?
                    .affine_image(&inverse, &offset)?;
                if !curve.domain().contains(&parameter)?
                    || !curve.point_at(&parameter)?.coincides_with(point)?
                {
                    continue;
                }
                let mut index = 0;
                while index < found.len() && found[index].compare(&parameter)?.is_lt() {
                    index += 1;
                }
                if index == found.len() || !found[index].compare(&parameter)?.is_eq() {
                    found.insert(index, parameter);
                }
            }
            return Ok(if found.is_empty() {
                CurveParameterLocation::None
            } else {
                CurveParameterLocation::Parameters(found)
            });
        }
        if blocker.is_none() && matches!(accumulated, hypercurve::CurvePointLocations2::EntireCurve)
        {
            return Ok(CurveParameterLocation::EntireDomain);
        }
        Err(blocker.unwrap_or(GeometryError::UnsupportedParameterLocation))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn half(n: i32) -> Real {
        (Real::from(n) / Real::from(2)).unwrap()
    }
    fn actual() -> TensorCurve3 {
        let points = (0..2)
            .map(|v| {
                (0..3)
                    .map(|u| Point3::new(half(u), v.into(), half(u) * Real::from(v)))
                    .collect()
            })
            .collect();
        TensorCurve3 {
            surface: Surface::rational_bezier(points, vec![vec![Real::one(); 3]; 2]).unwrap(),
            source: Curve2::from(
                hypercurve::LineSeg2::try_new(
                    hypercurve::Point2::new(0.into(), (Real::one() / Real::from(4)).unwrap()),
                    hypercurve::Point2::new(1.into(), (Real::from(3) / Real::from(4)).unwrap()),
                )
                .unwrap(),
            ),
            source_scale: Real::one(),
            source_offset: Real::zero(),
        }
    }
    fn reflected(actual: &TensorCurve3, changed_interior: bool) -> TensorCurve3 {
        let SurfaceGeometry::RationalBezier(patch) = &actual.surface.data.geometry else {
            unreachable!()
        };
        let mut controls = patch.control_points.clone();
        let mut weights = patch.weights.clone();
        for row in &mut controls {
            row.reverse();
        }
        for row in &mut weights {
            row.reverse();
        }
        if changed_interior {
            controls[0][1].z += Real::one();
        }
        let reflect = hypercurve::Similarity2::try_from_real_affine(
            (-1).into(),
            0.into(),
            0.into(),
            1.into(),
            1.into(),
            0.into(),
        )
        .unwrap();
        TensorCurve3 {
            surface: Surface::rational_bezier(controls, weights).unwrap(),
            source: actual
                .source
                .transform_similarity(&reflect)
                .unwrap()
                .reversed()
                .unwrap(),
            source_scale: (-1).into(),
            source_offset: 1.into(),
        }
    }
    #[test]
    fn reflected_tensor_charts_prove_the_whole_function_and_parameter_law() {
        let actual = actual();
        let reflected = reflected(&actual, false);
        assert!(actual.parameterization_equals(&reflected).unwrap());
        assert!(reflected.parameterization_equals(&actual).unwrap());
        let wrong = TensorCurve3 {
            source_scale: 1.into(),
            source_offset: 0.into(),
            ..reflected
        };
        assert!(!actual.parameterization_equals(&wrong).unwrap());
    }
    #[test]
    fn reflected_tensor_endpoint_matches_do_not_certify_changed_interiors() {
        let actual = actual();
        let wrong = reflected(&actual, true);
        assert!(!actual.parameterization_equals(&wrong).unwrap());
        let actual = Curve3::from_tensor_image(actual).unwrap();
        let wrong = Curve3::from_tensor_image(wrong).unwrap();
        for t in [Real::zero(), Real::one()] {
            assert!(
                actual
                    .point_at(&t.clone().into())
                    .unwrap()
                    .coincides_with(&wrong.point_at(&t.into()).unwrap())
                    .unwrap()
            );
        }
        assert!(
            !actual
                .point_at(&half(1).into())
                .unwrap()
                .coincides_with(&wrong.point_at(&half(1).into()).unwrap())
                .unwrap()
        );
    }
}
