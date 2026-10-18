use hyperbrep::{Curve3, Point2, Point3, Real, Surface, Vector3};

#[test]
fn borrowed_circle_maps_reversal_and_subdivision_without_a_new_carrier() {
    let source = Curve3::circle_arc(
        Point3::new(2.into(), 3.into(), 5.into()),
        Vector3::x(),
        Vector3::y(),
        3.into(),
        Real::pi(),
        Real::tau(),
    )
    .unwrap();
    let middle = (&Real::pi() * Real::from(3) / Real::from(2)).unwrap();
    for curve in [
        source.clone(),
        source.reversed().unwrap(),
        source.subcurve(&middle, &Real::tau()).unwrap(),
    ] {
        let view = curve.circular_geometry().unwrap();
        for t in [
            curve.domain().represented_start().unwrap().clone(),
            ((curve.domain().represented_start().unwrap()
                + curve.domain().represented_end().unwrap())
                / Real::from(2))
            .unwrap(),
            curve.domain().represented_end().unwrap().clone(),
        ] {
            let delta = &t - curve.domain().represented_start().unwrap();
            let angle = if view.increasing_angle {
                view.angle_at_start + delta
            } else {
                view.angle_at_start - delta
            };
            let expected = view.center.clone()
                + (view.x.clone() * angle.clone().cos() + view.y.clone() * angle.sin())
                    * view.radius;
            assert!(
                curve
                    .point_at(&t.into())
                    .unwrap()
                    .coincides_with(&expected.into())
                    .unwrap()
            );
        }
    }
    assert!(
        Curve3::line(Point3::origin(), Point3::new(1.into(), 0.into(), 0.into()))
            .unwrap()
            .circular_geometry()
            .is_none()
    );
    assert!(
        Curve3::ellipse_arc(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            3.into(),
            2.into(),
            Real::zero(),
            Real::pi()
        )
        .unwrap()
        .circular_geometry()
        .is_none()
    );
}

#[test]
fn borrowed_cylinder_frame_exposes_the_exact_authored_chart() {
    let source = Surface::cylinder(
        Point3::new(2.into(), 3.into(), 5.into()),
        Vector3::y(),
        Vector3::z(),
        Vector3::x(),
        3.into(),
    )
    .unwrap();
    let view = source.cylindrical_geometry().unwrap();
    for uv in [
        Point2::origin(),
        Point2::new((Real::pi() / Real::from(2)).unwrap(), 7.into()),
    ] {
        let expected = view.origin.clone()
            + (view.x.clone() * uv.x.clone().cos() + view.y.clone() * uv.x.clone().sin())
                * view.radius
            + view.axis.clone() * &uv.y;
        let actual = source.point_at(&uv).unwrap();
        assert!(
            hyperbrep::GeometricPoint3::from(actual)
                .coincides_with(&expected.into())
                .unwrap()
        );
    }
    assert!(
        Surface::plane(Point3::origin(), Vector3::x(), Vector3::y())
            .unwrap()
            .cylindrical_geometry()
            .is_none()
    );
}
