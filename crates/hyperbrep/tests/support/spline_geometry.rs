use hyperbrep::*;

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}
fn equal(a: &Real, b: &Real) {
    assert!((a - b).exact_rational_normal_form().unwrap().is_zero());
}
fn spline(nurbs: bool) -> Curve3 {
    let controls = vec![p(-2, 4, 0), p(0, -4, 0), p(2, 4, 0)];
    let weights = vec![2.into(), 3.into(), 5.into()];
    if nurbs {
        Curve3::nurbs(
            2,
            controls,
            weights,
            vec![2.into(), 2.into(), 2.into(), 4.into(), 4.into(), 4.into()],
        )
        .unwrap()
    } else {
        Curve3::rational_bezier(controls, weights).unwrap()
    }
}

pub fn spline_views_retain_weights_knots_and_reversed_subcurve_charts() {
    reject_overfull_endpoint_knots();
    for nurbs in [false, true] {
        let curve = spline(nurbs);
        let reversed = curve.reversed().unwrap();
        if nurbs {
            let view = curve.nurbs_geometry().unwrap();
            assert_eq!(view.degree, 2);
            assert_eq!(
                view.knots,
                &[
                    Real::from(2),
                    2.into(),
                    2.into(),
                    4.into(),
                    4.into(),
                    4.into()
                ]
            );
            assert_eq!(
                reversed.nurbs_geometry().unwrap().weights,
                &[Real::from(5), 3.into(), 2.into()]
            );
            assert!(curve.rational_bezier_geometry().is_none());
        } else {
            let view = curve.rational_bezier_geometry().unwrap();
            assert_eq!(view.control_points, &[p(-2, 4, 0), p(0, -4, 0), p(2, 4, 0)]);
            assert_eq!(view.weights, &[Real::from(2), 3.into(), 5.into()]);
            assert_eq!(
                reversed.rational_bezier_geometry().unwrap().weights,
                &[Real::from(5), 3.into(), 2.into()]
            );
            assert!(curve.nurbs_geometry().is_none());
        }
        let middle = ((curve.domain().represented_start().unwrap()
            + curve.domain().represented_end().unwrap())
            / Real::from(2))
        .unwrap();
        let trimmed = curve
            .subcurve(&middle, curve.domain().represented_end().unwrap())
            .unwrap();
        assert_eq!(trimmed.kind(), curve.kind());
        assert!(
            curve
                .point_at(&Parameter::from_real(middle))
                .unwrap()
                .coincides_with(&trimmed.start().unwrap().into())
                .unwrap()
        );
    }
}

fn reject_overfull_endpoint_knots() {
    for degree in [1, 2] {
        let count = degree + 2;
        let controls: Vec<_> = (0..count)
            .map(|i| {
                let i = i32::try_from(i).unwrap();
                p(i, i * i, 0)
            })
            .collect();
        for at_end in [false, true] {
            let mut knots = vec![Real::zero(); degree + 1];
            knots.push(if at_end { Real::one() } else { Real::zero() });
            knots.extend(vec![Real::one(); degree + 1]);
            let curve = Curve3::nurbs(
                degree,
                controls.clone(),
                vec![Real::one(); count],
                knots.clone(),
            );
            assert!(
                matches!(curve, Err(GeometryError::InvalidKnotMultiplicity)),
                "degree={degree} at_end={at_end}: {curve:?}"
            );
            // Curve and both surface axes must share the native invariant.
            for u_axis in [false, true] {
                let (u_count, v_count) = if u_axis { (count, 2) } else { (2, count) };
                let points = (0..v_count)
                    .map(|v| {
                        (0..u_count)
                            .map(|u| p(i32::try_from(u).unwrap(), i32::try_from(v).unwrap(), 0))
                            .collect()
                    })
                    .collect();
                let unit = vec![Real::zero(), Real::zero(), Real::one(), Real::one()];
                let surface = Surface::nurbs(
                    if u_axis { degree } else { 1 },
                    if u_axis { 1 } else { degree },
                    points,
                    vec![vec![Real::one(); u_count]; v_count],
                    if u_axis { knots.clone() } else { unit.clone() },
                    if u_axis { unit } else { knots.clone() },
                );
                assert!(
                    matches!(surface, Err(GeometryError::InvalidKnotMultiplicity)),
                    "degree={degree} at_end={at_end} u_axis={u_axis}: {surface:?}"
                );
            }
        }
        // The same control net with a genuine interior knot remains admitted.
        let mut knots = vec![Real::zero(); degree + 1];
        knots.push((Real::one() / Real::from(2)).unwrap());
        knots.extend(vec![Real::one(); degree + 1]);
        let valid =
            Curve3::nurbs(degree, controls.clone(), vec![Real::one(); count], knots).unwrap();
        assert_eq!(valid.start().unwrap(), controls[0]);
        assert_eq!(valid.end().unwrap(), controls[count - 1]);
    }
}

pub fn planar_pcurves_prove_whole_image_and_retain_parameter_correspondence() {
    let plane = Surface::plane(
        p(1, 2, 0),
        Vector3::x() * Real::from(2),
        Vector3::x() - Vector3::y() * Real::from(3),
    )
    .unwrap();
    for nurbs in [false, true] {
        for curve in [spline(nurbs), spline(nurbs).reversed().unwrap()] {
            let projected = plane.planar_pcurve(&curve).unwrap().unwrap();
            let native = projected.curve().parameter_domain();
            for t in [
                native.start().scalar().unwrap(),
                native.end().scalar().unwrap(),
            ] {
                let parameter = projected.spatial_parameter_at(&t.clone().into()).unwrap();
                let uv = projected.curve().point_at(&t.clone().into()).unwrap();
                let uv = uv.coordinates().unwrap();
                let on_plane = plane
                    .point_at(&{
                        let uv = &Point2::new(uv.x().clone(), uv.y().clone());
                        hypercurve::Point2::new(uv.x.clone(), uv.y.clone()).into()
                    })
                    .map(|point| {
                        point
                            .try_coordinates()
                            .expect("represented surface fixture")
                            .clone()
                    })
                    .unwrap();
                let on_curve = curve.point_at(&parameter).unwrap();
                assert!(on_curve.coincides_with(&on_plane.into()).unwrap());
            }
        }
    }
    let bowed = Curve3::rational_bezier(
        vec![p(-2, 4, 0), p(0, -4, 1), p(2, 4, 0)],
        vec![1.into(); 3],
    )
    .unwrap();
    assert!(plane.planar_pcurve(&bowed).unwrap().is_none());
}

pub fn extrusion_location_preserves_oblique_nonunit_direction_and_source_parameters() {
    for nurbs in [false, true] {
        for direction in [
            Vector3::z() * Real::from(3),
            Vector3::new([2.into(), 1.into(), 3.into()]),
        ] {
            let curve = spline(nurbs);
            let middle = ((curve.domain().represented_start().unwrap()
                + curve.domain().represented_end().unwrap())
                / Real::from(2))
            .unwrap();
            let surface = Surface::extrusion(curve, direction).unwrap();
            for v in [-2, 0, 3] {
                let uv = Point2::new(middle.clone(), v.into());
                let point = surface
                    .point_at(&{
                        let uv = &uv;
                        hypercurve::Point2::new(uv.x.clone(), uv.y.clone()).into()
                    })
                    .map(|point| {
                        point
                            .try_coordinates()
                            .expect("represented surface fixture")
                            .clone()
                    })
                    .unwrap();
                let (location, height) = surface.extrusion_parameters_of(&point).unwrap();
                equal(&height, &uv.y);
                let CurveParameterLocation::Parameters(parameters) = location else {
                    panic!("unique point")
                };
                assert_eq!(parameters.len(), 1);
                equal(parameters[0].try_real().unwrap(), &middle);
            }
        }
    }
}

pub fn extrusion_location_keeps_ambiguity_and_rejects_nontransverse_or_nonplanar_charts() {
    let retraced =
        Curve3::rational_bezier(vec![p(0, 0, 0), p(2, 0, 0), p(0, 0, 0)], vec![1.into(); 3])
            .unwrap();
    let surface = Surface::extrusion(retraced, Vector3::z()).unwrap();
    let (location, height) = surface.extrusion_parameters_of(&p(0, 0, 2)).unwrap();
    equal(&height, &2.into());
    assert!(
        matches!(location, CurveParameterLocation::Parameters(parameters) if parameters.len() == 2)
    );
    assert!(matches!(
        surface.extrusion_parameters_of(&p(2, 1, 2)).unwrap().0,
        CurveParameterLocation::None
    ));
    let parallel = Surface::extrusion(spline(false), Vector3::x()).unwrap();
    assert!(matches!(
        parallel.extrusion_parameters_of(&p(0, 0, 0)),
        Err(GeometryError::UnsupportedIntersection)
    ));
    let spatial = Curve3::rational_bezier(
        vec![p(0, 0, 0), p(1, 0, 0), p(0, 1, 0), p(0, 0, 1)],
        vec![1.into(); 4],
    )
    .unwrap();
    let surface = Surface::extrusion(spatial, Vector3::z()).unwrap();
    assert!(matches!(
        surface.extrusion_parameters_of(&p(0, 0, 0)),
        Err(GeometryError::UnsupportedIntersection)
    ));
}
