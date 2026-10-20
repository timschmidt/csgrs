use hyperbrep::*;
use std::cmp::Ordering;

fn quarter() -> Real {
    (Real::pi() / Real::from(2)).unwrap()
}

fn shear() -> Matrix4 {
    Matrix4::from_row_major([
        (-1).into(),
        2.into(),
        0.into(),
        4.into(),
        0.into(),
        1.into(),
        0.into(),
        (-2).into(),
        0.into(),
        0.into(),
        1.into(),
        1.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ])
}

fn replay(model: &Model) -> Model {
    let json = model.to_json().unwrap();
    let restored = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(restored.to_json().unwrap(), json);
    restored
}

pub fn affine_images_cover_native_and_retained_families() {
    let controls = vec![
        Point3::origin(),
        Point3::new(1.into(), 1.into(), 0.into()),
        Point3::new(2.into(), 0.into(), 1.into()),
    ];
    let origin = Point3::new(2.into(), 0.into(), 0.into());
    let curves = [
        Curve3::line(controls[0].clone(), controls[2].clone()).unwrap(),
        Curve3::rational_bezier(controls.clone(), vec![1.into(), 2.into(), 1.into()]).unwrap(),
        Curve3::nurbs(
            2,
            controls,
            vec![1.into(); 3],
            vec![0.into(), 0.into(), 0.into(), 1.into(), 1.into(), 1.into()],
        )
        .unwrap(),
        Curve3::circle_arc(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            2.into(),
            0.into(),
            quarter(),
        )
        .unwrap(),
        Curve3::ellipse_arc(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            2.into(),
            1.into(),
            0.into(),
            quarter(),
        )
        .unwrap(),
        Curve3::rotation(
            origin.clone().into(),
            Point3::origin(),
            Vector3::z(),
            0.into(),
            quarter(),
        )
        .unwrap(),
        Curve3::translation(origin.into(), Vector3::new([1.into(), 2.into(), 3.into()])).unwrap(),
        Curve3::planar_image(
            hypercurve::QuadraticBezier2::new(
                hypercurve::Point2::from_values(0, 0),
                hypercurve::Point2::from_values(1, 2),
                hypercurve::Point2::from_values(2, 0),
            )
            .into(),
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
        )
        .unwrap(),
    ];
    for source in curves {
        let image = Curve3::affine_image(source.clone(), shear()).unwrap();
        assert_eq!(image.kind(), Curve3Kind::AffineImage);
        assert_eq!(
            image.affine_geometry().unwrap().source.kind(),
            source.kind()
        );
        assert!(image.circular_geometry().is_none());
        let bounds = image.bounds().unwrap();
        for order in [1, 2, 5] {
            let at = source.domain().start();
            let derivative = source.derivative_at(at, order).unwrap();
            let d = derivative.try_coordinates().unwrap();
            assert_eq!(
                image
                    .derivative_at(at, order)
                    .unwrap()
                    .try_coordinates()
                    .unwrap(),
                &Vector3::new([-&d[0] + Real::from(2) * &d[1], d[1].clone(), d[2].clone()])
            );
        }

        for parameter in [source.domain().start(), source.domain().end()] {
            let original = source.point_at(parameter).unwrap();
            let p = original.try_coordinates().unwrap();
            // Independent explicit formula for this reflection/shear.
            let expected: GeometricPoint3 = Point3::new(
                -&p.x + Real::from(2) * &p.y + Real::from(4),
                &p.y - Real::from(2),
                &p.z + Real::one(),
            )
            .into();
            let actual = image.point_at(parameter).unwrap();
            assert!(actual.coincides_with(&expected).unwrap());
            for axis in [Axis::X, Axis::Y, Axis::Z] {
                assert_ne!(
                    actual
                        .compare_coordinate(&bounds.mins.clone().into(), axis)
                        .unwrap(),
                    Ordering::Less
                );
                assert_ne!(
                    actual
                        .compare_coordinate(&bounds.maxs.clone().into(), axis)
                        .unwrap(),
                    Ordering::Greater
                );
            }
        }
        let reversed = image.reversed().unwrap();
        assert!(
            reversed
                .point_at(reversed.domain().start())
                .unwrap()
                .coincides_with(&image.point_at(image.domain().end()).unwrap())
                .unwrap()
        );
        let middle = ((source.domain().start().try_real().unwrap()
            + source.domain().end().try_real().unwrap())
            / Real::from(2))
        .unwrap();
        let midpoint = image
            .point_at(&Parameter::from_real(middle.clone()))
            .unwrap();
        let (left, right) = image.split_at(&middle).unwrap();
        assert!(
            left.point_at(left.domain().end())
                .unwrap()
                .coincides_with(&midpoint)
                .unwrap()
        );
        assert!(
            right
                .point_at(right.domain().start())
                .unwrap()
                .coincides_with(&midpoint)
                .unwrap()
        );
        let restricted = image
            .subcurve(source.domain().start().try_real().unwrap(), &middle)
            .unwrap();
        assert!(
            restricted
                .point_at(restricted.domain().end())
                .unwrap()
                .coincides_with(&midpoint)
                .unwrap()
        );
        let mut builder = ModelBuilder::new();
        for curve in [image.clone(), reversed, left, right, restricted] {
            let a = builder
                .vertex(curve.point_at(curve.domain().start()).unwrap())
                .unwrap();
            let b = builder
                .vertex(curve.point_at(curve.domain().end()).unwrap())
                .unwrap();
            let domain = curve.domain().clone();
            let c = builder.curve(curve).unwrap();
            builder.edge(&a, &b, &c, domain).unwrap();
        }
        let transformed = builder.finish().unwrap().transformed(&shear()).unwrap();
        let restored = replay(&replay(&transformed));
        for curve in restored.curves() {
            let image = curve.geometry().affine_geometry().unwrap();
            assert!(image.source.affine_geometry().is_none());
        }
        if !matches!(
            source.kind(),
            Curve3Kind::Rotation | Curve3Kind::Translation
        ) {
            let point = image.point_at(image.domain().start()).unwrap();
            let CurveParameterLocation::Parameters(parameters) = image
                .parameters_of(point.try_coordinates().unwrap())
                .unwrap()
            else {
                panic!("complete parameter location");
            };
            assert!(
                parameters
                    .iter()
                    .any(|p| p.compare(image.domain().start()).unwrap() == Ordering::Equal)
            );
        }
    }
}

pub fn affine_images_preserve_periods_derivatives_and_admission() {
    let source = Curve3::rotation(
        Point3::new(2.into(), 0.into(), 0.into()).into(),
        Point3::origin(),
        Vector3::z(),
        0.into(),
        Real::tau(),
    )
    .unwrap();
    let image = Curve3::affine_image(source.clone(), shear()).unwrap();
    assert_eq!(
        image
            .derivative_at(&Real::zero().into(), 1)
            .unwrap()
            .try_coordinates()
            .unwrap(),
        &Vector3::new([4.into(), 2.into(), 0.into()])
    );
    assert_eq!(
        image
            .derivative_at(&Real::zero().into(), 2)
            .unwrap()
            .try_coordinates()
            .unwrap(),
        &Vector3::new([2.into(), 0.into(), 0.into()])
    );
    let mut builder = ModelBuilder::new();
    let a = builder
        .vertex(image.point_at(image.domain().start()).unwrap())
        .unwrap();
    let domain = image.domain().clone();
    let c = builder.curve(image.clone()).unwrap();
    builder.edge(&a, &a, &c, domain).unwrap();
    let model = builder.finish().unwrap();
    replay(&model.transformed(&shear()).unwrap());
    let mut repeated = image;
    for _ in 0..32 {
        repeated = Curve3::affine_image(
            repeated,
            Matrix4::affine_translation([1.into(), 0.into(), 0.into()]),
        )
        .unwrap();
        assert!(
            repeated
                .affine_geometry()
                .unwrap()
                .source
                .affine_geometry()
                .is_none()
        );
    }
    assert!(
        repeated
            .point_at(repeated.domain().start())
            .unwrap()
            .coincides_with(&Point3::new(34.into(), (-2).into(), 1.into()).into())
            .unwrap()
    );
    let mut singular = shear();
    singular.0[0] = singular.0[1].clone();
    assert!(matches!(
        Curve3::affine_image(source.clone(), singular),
        Err(GeometryError::SingularTransform)
    ));
    let mut nonaffine = shear();
    nonaffine.0[3][0] = 1.into();
    assert!(matches!(
        Curve3::affine_image(source, nonaffine),
        Err(GeometryError::NonAffineTransform)
    ));
    let json = model.to_json().unwrap();
    let raw: serde_json::Value = serde_json::from_str(&json).unwrap();
    let mut legacy = raw.clone();
    legacy["version"] = 11.into();
    assert!(matches!(
        RawModel::from_json(&legacy.to_string(), &ArchiveLimits::default()),
        Err(PersistenceError::UnsupportedVersion(11))
    ));
    let mut bad = raw;
    bad["curves"][0]["AffineImage"]["transform"][3][0] = serde_json::json!(Real::one());
    assert!(matches!(
        RawModel::from_json(&bad.to_string(), &ArchiveLimits::default())
            .unwrap()
            .validate(),
        Err(PersistenceError::Build(BuildError::Geometry(
            GeometryError::NonAffineTransform
        )))
    ));
    let controls = vec![Point3::origin(); 5];
    let curve = Curve3::affine_image(
        Curve3::rational_bezier(controls, vec![1.into(); 5]).unwrap(),
        shear(),
    )
    .unwrap();
    let mut builder = ModelBuilder::new();
    builder.curve(curve).unwrap();
    let mut raw: serde_json::Value =
        serde_json::from_str(&builder.finish().unwrap().to_json().unwrap()).unwrap();
    raw["curves"][0]["AffineImage"]["source"]["RationalBezier"]["control_points"][0][0] =
        "invalid scalar after admission".into();
    assert!(matches!(
        RawModel::from_json(
            &raw.to_string(),
            &ArchiveLimits {
                degree: 3,
                ..ArchiveLimits::default()
            }
        ),
        Err(PersistenceError::ResourceLimit(ArchiveLimitExceeded {
            resource: ArchiveResource::Degree,
            observed: 4,
            limit: 3
        }))
    ));
    replay(&model);
}
