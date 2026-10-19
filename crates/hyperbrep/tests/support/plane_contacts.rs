use hyperbrep::*;
use hypercurve::{BezierParameter2, BezierParameterRange2, Curve2, Point2 as Uv, QuadraticBezier2};
use std::cmp::Ordering;

#[allow(dead_code)]
#[path = "parallel_profile_extrusion.rs"]
pub(crate) mod prior;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn xy_plane() -> Surface {
    Surface::plane(Point3::origin(), Vector3::x(), Vector3::y()).unwrap()
}
fn x_plane(x: Real) -> Surface {
    Surface::plane(
        Point3::new(x, 0.into(), 0.into()),
        Vector3::y(),
        Vector3::z(),
    )
    .unwrap()
}
fn y_plane(y: Real) -> Surface {
    Surface::plane(
        Point3::new(0.into(), y, 0.into()),
        Vector3::z(),
        Vector3::x(),
    )
    .unwrap()
}

fn points(plane: &Surface, curve: &Curve3) -> Vec<CurveSurfacePoint> {
    let CurveSurfaceIntersection::Points(points) = plane.intersect_curve(curve).unwrap() else {
        panic!("isolated contacts required");
    };
    assert!(!points.is_empty());
    for point in &points {
        assert!(
            point
                .point
                .coincides_with(&curve.point_at(&point.parameter).unwrap())
                .unwrap()
        );
    }
    points
}

fn native(coefficients: &[Real], weights: &[Real]) -> Curve3 {
    Curve3::rational_bezier(
        coefficients
            .iter()
            .zip(weights)
            .enumerate()
            .map(|(i, (z, w))| Point3::new(Real::from(i as u32), 0.into(), (z / w).unwrap()))
            .collect(),
        weights.to_vec(),
    )
    .unwrap()
}

fn cubic() -> Curve3 {
    native(&[-1, -1, -1, 1].map(Real::from), &vec![Real::one(); 4])
}

fn reload(model: &Model, name: &str) -> Model {
    let json = model.to_json().unwrap();
    let cold = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), json);
    #[cfg(not(target_arch = "wasm32"))]
    if let Some(directory) = std::env::var_os("HYPERBREP_PLANE_CONTACT_ARTIFACT_DIR") {
        let directory = std::path::PathBuf::from(directory);
        std::fs::create_dir_all(&directory).unwrap();
        std::fs::write(directory.join(format!("{name}.json")), json).unwrap();
    }
    #[cfg(target_arch = "wasm32")]
    let _ = name;
    cold
}

pub fn cubic_contacts_retain_original_source_and_certify_independent_equations() {
    for (i, weights) in [vec![Real::one(); 4], (1..=4).map(Real::from).collect()]
        .into_iter()
        .enumerate()
    {
        let curve = native(&[-1, -1, -1, 1].map(Real::from), &weights);
        // The numerator 2t^3-1 is strictly increasing at its unique root.
        let contacts = points(&xy_plane(), &curve);
        assert_eq!(contacts.len(), 1);
        let contact = &contacts[0];
        assert_eq!(contact.kind, CurveSurfaceContactKind::Simple);
        assert!(contact.parameter.try_real().is_none());
        assert!(contact.point.try_coordinates().is_none());
        assert_eq!(
            contact.parameter.compare(&q(3, 4).into()).unwrap(),
            Ordering::Greater
        );
        assert_eq!(
            contact.parameter.compare(&q(4, 5).into()).unwrap(),
            Ordering::Less
        );
        assert_eq!(
            contact
                .point
                .compare_linear_form(&Vector3::z(), &Real::zero())
                .unwrap(),
            Ordering::Equal
        );
        let (source, retained) = contact.point.curve_source().unwrap();
        assert_eq!(source.kind(), Curve3Kind::RationalBezier);
        assert_eq!(
            retained.compare(&contact.parameter).unwrap(),
            Ordering::Equal
        );
        if i == 0 {
            assert_eq!(
                contact
                    .point
                    .compare_linear_form(&Vector3::x(), &q(9, 4))
                    .unwrap(),
                Ordering::Greater
            );
            assert_eq!(
                contact
                    .point
                    .compare_linear_form(&Vector3::x(), &q(12, 5))
                    .unwrap(),
                Ordering::Less
            );
        }
        let reverse = points(&xy_plane(), &curve.reversed().unwrap());
        assert_eq!(reverse.len(), 1);
        assert_eq!(
            reverse[0].parameter.compare(&q(1, 5).into()).unwrap(),
            Ordering::Greater
        );
        assert_eq!(
            reverse[0].parameter.compare(&q(1, 4).into()).unwrap(),
            Ordering::Less
        );
        assert!(reverse[0].point.coincides_with(&contact.point).unwrap());
        let mut builder = ModelBuilder::new();
        builder.vertex(contact.point.clone()).unwrap();
        builder.curve(curve).unwrap();
        let cold = reload(&builder.finish().unwrap(), &format!("cubic-{i}"));
        assert!(
            cold.vertices()
                .next()
                .unwrap()
                .point()
                .coincides_with(&contact.point)
                .unwrap()
        );
        assert_eq!(
            points(&xy_plane(), cold.curves().next().unwrap().geometry()).len(),
            1
        );
    }
}

pub fn complete_root_sets_keep_repeated_roots_endpoints_and_distinct_visits() {
    // 6(4t-1)(2t-1)(4t-3) has Bernstein coefficients [-18,26,-26,18].
    let curve = native(&[-18, 26, -26, 18].map(Real::from), &vec![Real::one(); 4]);
    let contacts = points(&xy_plane(), &curve);
    assert_eq!(contacts.len(), 3);
    for (contact, expected) in contacts.iter().zip([q(1, 4), q(1, 2), q(3, 4)]) {
        assert_eq!(
            contact.parameter.compare(&expected.into()).unwrap(),
            Ordering::Equal
        );
        assert_eq!(contact.kind, CurveSurfaceContactKind::Simple);
    }
    // (2t-1)^3 crosses the plane with a zero first derivative.
    let contacts = points(
        &xy_plane(),
        &native(&[-1, 1, -1, 1].map(Real::from), &vec![Real::one(); 4]),
    );
    assert_eq!(contacts.len(), 1);
    assert_eq!(
        contacts[0].parameter.compare(&q(1, 2).into()).unwrap(),
        Ordering::Equal
    );
    assert_eq!(contacts[0].kind, CurveSurfaceContactKind::Repeated);
    // -3t^2(1-t): one repeated endpoint, one simple endpoint.
    let contacts = points(
        &xy_plane(),
        &native(&[0, 0, -1, 0].map(Real::from), &vec![Real::one(); 4]),
    );
    assert_eq!(contacts.len(), 2);
    for (contact, (expected, kind)) in contacts.iter().zip([
        (0, CurveSurfaceContactKind::Repeated),
        (1, CurveSurfaceContactKind::Simple),
    ]) {
        assert_eq!(
            contact
                .parameter
                .compare(&Real::from(expected).into())
                .unwrap(),
            Ordering::Equal
        );
        assert_eq!(contact.kind, kind);
    }
    let visits = Curve3::rational_bezier(
        [3, 3, -2, -12, 21]
            .map(|n| Point3::new(0.into(), 0.into(), q(n, 48)))
            .to_vec(),
        vec![Real::one(); 5],
    )
    .unwrap();
    // (t^2-1/8)(t^2-1/2): two distinct positive roots, one spatial image.
    let contacts = points(&xy_plane(), &visits);
    assert_eq!(contacts.len(), 2);
    assert_eq!(
        contacts[0]
            .parameter
            .compare(&contacts[1].parameter)
            .unwrap(),
        Ordering::Less
    );
    assert!(
        contacts[0]
            .point
            .coincides_with(&contacts[1].point)
            .unwrap()
    );
    assert!(
        contacts[0]
            .point
            .coincides_with(&Point3::origin().into())
            .unwrap()
    );
}

pub fn high_degree_empty_and_contained_results_are_complete() {
    for coefficients in [vec![Real::one(); 6], vec![Real::zero(); 6]] {
        let curve = native(&coefficients, &vec![Real::one(); 6]);
        let result = xy_plane().intersect_curve(&curve).unwrap();
        if coefficients[0] == Real::zero() {
            assert!(matches!(result, CurveSurfaceIntersection::Contained));
        } else {
            assert!(matches!(result, CurveSurfaceIntersection::None));
        }
    }
    let coefficients = [-5, -7, -9, -11, -13, 145].map(|n| q(n, 5));
    let contacts = points(&xy_plane(), &native(&coefficients, &vec![Real::one(); 6]));
    // 32t^5-2t-1 has one positive root between 1/2 and 3/4.
    assert_eq!(contacts.len(), 1);
    assert!(contacts[0].parameter.try_real().is_none());
    assert_eq!(
        contacts[0].parameter.compare(&q(1, 2).into()).unwrap(),
        Ordering::Greater
    );
    assert_eq!(
        contacts[0].parameter.compare(&q(3, 4).into()).unwrap(),
        Ordering::Less
    );
    assert_eq!(contacts[0].kind, CurveSurfaceContactKind::Simple);
}

pub fn nonunit_nurbs_contacts_use_original_curve_parameters() {
    let controls = (0..4)
        .zip([-1, -1, -1, 1])
        .map(|(x, z)| Point3::new(x.into(), 0.into(), z.into()))
        .collect();
    let curve = Curve3::nurbs(
        3,
        controls,
        vec![Real::one(); 4],
        [2, 2, 2, 2, 5, 5, 5, 5].map(Real::from).to_vec(),
    )
    .unwrap();
    let contacts = points(&xy_plane(), &curve);
    assert_eq!(contacts.len(), 1);
    let contact = &contacts[0];
    assert_eq!(
        contact.parameter.compare(&q(17, 4).into()).unwrap(),
        Ordering::Greater
    );
    assert_eq!(
        contact.parameter.compare(&q(22, 5).into()).unwrap(),
        Ordering::Less
    );
    assert_eq!(
        contact.point.curve_source().unwrap().0.kind(),
        Curve3Kind::Nurbs
    );
    assert_eq!(
        contact
            .point
            .compare_linear_form(&Vector3::z(), &Real::zero())
            .unwrap(),
        Ordering::Equal
    );
    let mut builder = ModelBuilder::new();
    builder.vertex(contact.point.clone()).unwrap();
    builder.curve(curve).unwrap();
    let cold = reload(&builder.finish().unwrap(), "nurbs-contact");
    assert!(
        cold.vertices()
            .next()
            .unwrap()
            .point()
            .coincides_with(&contact.point)
            .unwrap()
    );
}

fn mixed_nurbs() -> Curve3 {
    Curve3::nurbs(
        1,
        (0..6)
            .zip([0, 0, 1, -1, 0, 0])
            .map(|(x, z)| Point3::new(x.into(), 0.into(), z.into()))
            .collect(),
        vec![Real::one(); 6],
        [2, 2, 3, 5, 7, 11, 13, 13].map(Real::from).to_vec(),
    )
    .unwrap()
}

pub fn nurbs_coincidence_keeps_all_ranges_and_isolated_contacts() {
    let curve = mixed_nurbs();
    let CurveSurfaceIntersection::Components(components) =
        xy_plane().intersect_curve(&curve).unwrap()
    else {
        panic!("mixed components");
    };
    assert_eq!(components.points().len(), 1);
    assert_eq!(components.overlaps().len(), 2);
    assert_eq!(
        components.points()[0]
            .parameter
            .compare(&Real::from(6).into())
            .unwrap(),
        Ordering::Equal
    );
    for (range, (a, b)) in components.overlaps().iter().zip([(2, 3), (11, 13)]) {
        assert_eq!(
            range.start().compare(&Real::from(a).into()).unwrap(),
            Ordering::Equal
        );
        assert_eq!(
            range.end().compare(&Real::from(b).into()).unwrap(),
            Ordering::Equal
        );
    }
    for zs in [[0, 0, 0], [0, 0, 1]] {
        let curve = Curve3::nurbs(
            1,
            (0..3)
                .zip(zs)
                .map(|(x, z)| Point3::new(x.into(), 0.into(), z.into()))
                .collect(),
            vec![Real::one(); 3],
            [2, 2, 3, 5, 5].map(Real::from).to_vec(),
        )
        .unwrap();
        match xy_plane().intersect_curve(&curve).unwrap() {
            CurveSurfaceIntersection::Contained => assert_eq!(zs, [0, 0, 0]),
            CurveSurfaceIntersection::Overlap(range) => {
                assert_eq!(zs, [0, 0, 1]);
                assert_eq!(
                    range.start().compare(&Real::from(2).into()).unwrap(),
                    Ordering::Equal
                );
                assert_eq!(
                    range.end().compare(&Real::from(3).into()).unwrap(),
                    Ordering::Equal
                );
            }
            other => panic!("incomplete coincidence {other:?}"),
        }
    }
}

pub fn nurbs_joint_classification_preserves_each_incident_span() {
    let curve = Curve3::nurbs(
        2,
        (0..5)
            .zip([
                Real::one(),
                Real::zero(),
                Real::zero(),
                q(1, 2),
                Real::one(),
            ])
            .map(|(x, z)| Point3::new(x.into(), 0.into(), z))
            .collect(),
        vec![Real::one(); 5],
        [2, 2, 2, 3, 3, 5, 5, 5].map(Real::from).to_vec(),
    )
    .unwrap();
    for curve in [curve.clone(), curve.reversed().unwrap()] {
        let contacts = points(&xy_plane(), &curve);
        assert_eq!(contacts.len(), 1);
        assert_eq!(contacts[0].kind, CurveSurfaceContactKind::Repeated);
        assert!(
            contacts[0]
                .point
                .coincides_with(&Point3::new(2.into(), 0.into(), 0.into()).into())
                .unwrap()
        );
    }
    let source = Curve2::try_nurbs(
        2,
        (0..5)
            .zip([
                Real::one(),
                Real::zero(),
                Real::zero(),
                q(1, 2),
                Real::one(),
            ])
            .map(|(x, y)| Uv::new(x.into(), y))
            .collect(),
        vec![Real::one(); 5],
        [2, 2, 2, 3, 3, 5, 5, 5].map(Real::from).to_vec(),
    )
    .unwrap();
    for source in [source.clone(), source.reversed().unwrap()] {
        let contacts = points(&y_plane(0.into()), &lift(source));
        assert_eq!(contacts.len(), 1);
        assert_eq!(contacts[0].kind, CurveSurfaceContactKind::Repeated);
    }
    // Full interior knot multiplicity retains two distinct one-sided points.
    // A closed contact range must not hide the unresolved endpoint image.
    for overlap in [false, true] {
        let controls = if overlap {
            [(0, 0), (0, 1), (1, 1), (2, 1)]
        } else {
            [(-1, 0), (0, 0), (1, 1), (2, 1)]
        };
        let source = Curve2::try_nurbs(
            1,
            controls.map(|(x, y)| Uv::from_values(x, y)).to_vec(),
            vec![Real::one(); 4],
            [0, 0, 1, 1, 2, 2].map(Real::from).to_vec(),
        )
        .unwrap();
        let original = lift(source);
        for (curve, reversed) in [
            (original.clone(), false),
            (original.reversed().unwrap(), true),
        ] {
            curve.bounds().unwrap();
            for x in [0, 1] {
                assert!(matches!(
                    x_plane(x.into()).intersect_curve(&curve),
                    Err(GeometryError::PlanarCurve(hypercurve::ExactCurveError::Blocked(blocker)))
                    if blocker.reason() == hypercurve::UncertaintyReason::Boundary
                ));
            }
            // A different interior contact is unambiguous and still succeeds.
            let contacts = points(&x_plane(q(3, 2)), &curve);
            assert_eq!(contacts.len(), 1);
            assert_eq!(
                contacts[0]
                    .parameter
                    .compare(&q(if reversed { 1 } else { 3 }, 4).into())
                    .unwrap(),
                Ordering::Equal
            );
            assert_eq!(
                contacts[0]
                    .point
                    .compare_linear_form(&Vector3::x(), &q(3, 2))
                    .unwrap(),
                Ordering::Equal
            );
            if overlap {
                let (start, end) = if reversed {
                    (q(3, 4), Real::one())
                } else {
                    (Real::zero(), q(1, 4))
                };
                let restricted = curve.subcurve(&start, &end).unwrap();
                assert!(matches!(
                    x_plane(Real::zero()).intersect_curve(&restricted).unwrap(),
                    CurveSurfaceIntersection::Contained
                ));
            }
        }
    }
}

fn parallel(a: i32, b: i32) -> Curve2 {
    Curve2::try_analytic_parallel(
        QuadraticBezier2::new(
            Uv::from_values(0, 0),
            Uv::from_values(2, 2),
            Uv::from_values(4, 0),
        )
        .parallel_left(Real::one())
        .unwrap(),
        BezierParameterRange2::try_new(
            BezierParameter2::Exact(a.into()),
            BezierParameter2::Exact(b.into()),
        )
        .unwrap(),
    )
    .unwrap()
}
fn lift(source: Curve2) -> Curve3 {
    Curve3::planar_image(source, Point3::origin(), Vector3::x(), Vector3::y()).unwrap()
}
fn map_curve(curve: &Curve3, transform: &Matrix4) -> Curve3 {
    let mut builder = ModelBuilder::new();
    builder.curve(curve.clone()).unwrap();
    let model = builder.finish().unwrap().transformed(transform).unwrap();
    model.curves().next().unwrap().geometry().clone()
}
fn map_surface(surface: &Surface, transform: &Matrix4) -> Surface {
    let mut builder = ModelBuilder::new();
    builder.surface(surface.clone()).unwrap();
    let model = builder.finish().unwrap().transformed(transform).unwrap();
    model.surfaces().next().unwrap().geometry().clone()
}

fn transform() -> Matrix4 {
    Matrix4::from_row_major([
        (-2).into(),
        1.into(),
        0.into(),
        5.into(),
        1.into(),
        3.into(),
        0.into(),
        (-2).into(),
        (-1).into(),
        2.into(),
        2.into(),
        7.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ])
}

pub fn parallel_plane_contacts_preserve_reversal_frame_and_tangency() {
    let mut original = None;
    for (i, (a, b)) in [(0, 1), (1, 0), (-1, 2), (2, -1)].into_iter().enumerate() {
        let curve = lift(parallel(a, b));
        let plane = x_plane(1.into());
        let contacts = points(&plane, &curve);
        assert_eq!(contacts.len(), 1);
        let contact = &contacts[0];
        assert_eq!(contact.kind, CurveSurfaceContactKind::Simple);
        assert!(contact.parameter.try_real().is_none());
        assert!(contact.point.try_coordinates().is_none());
        assert_eq!(
            contact
                .point
                .compare_linear_form(&Vector3::x(), &Real::one())
                .unwrap(),
            Ordering::Equal
        );
        assert_eq!(
            contact
                .point
                .compare_linear_form(&Vector3::z(), &Real::zero())
                .unwrap(),
            Ordering::Equal
        );
        if let Some(expected) = &original {
            assert!(contact.point.coincides_with(expected).unwrap());
        } else {
            original = Some(contact.point.clone());
        }
        let trimmed = curve.subcurve(&q(1, 4), &q(3, 4)).unwrap();
        let trimmed_contacts = points(&plane, &trimmed);
        assert_eq!(trimmed_contacts.len(), 1);
        assert!(
            trimmed_contacts[0]
                .point
                .coincides_with(&contact.point)
                .unwrap()
        );
        trimmed.bounds().unwrap();
        let projection_plane = Surface::plane(
            Point3::new(5.into(), (-3).into(), 0.into()),
            Vector3::y() * Real::from(2),
            -Vector3::x() * Real::from(2),
        )
        .unwrap();
        let source = parallel(a, b);
        for (spatial, start, end) in [
            (&curve, Real::from(a), Real::from(b)),
            (
                &trimmed,
                Real::from(a) + Real::from(b - a) * q(1, 4),
                Real::from(a) + Real::from(b - a) * q(3, 4),
            ),
        ] {
            let pcurve = projection_plane.planar_pcurve(spatial).unwrap().unwrap();
            for t in [
                start.clone(),
                ((&start + &end) / Real::from(2)).unwrap(),
                end.clone(),
            ] {
                let expected = ((&t - &start) / (&end - &start)).unwrap();
                assert_eq!(
                    pcurve
                        .spatial_parameter_at(&t.clone().into())
                        .unwrap()
                        .compare(&expected.into())
                        .unwrap(),
                    Ordering::Equal
                );
                // Independently derived inverse plane frame:
                // U=(y+3)/2 and V=(5-x)/2, with the original source chart.
                let point = source
                    .analytic_parallel()
                    .unwrap()
                    .parallel()
                    .point_at(&t)
                    .unwrap();
                let expected = hypercurve::Point2::new(
                    ((point.y() + Real::from(3)) / Real::from(2)).unwrap(),
                    ((Real::from(5) - point.x()) / Real::from(2)).unwrap(),
                );
                assert!(
                    pcurve
                        .curve()
                        .point_at(&t.into())
                        .unwrap()
                        .coincides_with(&expected.into())
                        .unwrap()
                );
            }
        }
        let mapped_curve = map_curve(&curve, &transform());
        let mapped_plane = map_surface(&plane, &transform());
        let mapped = points(&mapped_plane, &mapped_curve);
        assert_eq!(mapped.len(), 1);
        // With x=1,z=0 and this reflected affine frame, 2X-Z=0 and 3X-Y=10.
        assert_eq!(
            mapped[0]
                .point
                .compare_linear_form(
                    &Vector3::new([2.into(), 0.into(), (-1).into()]),
                    &Real::zero()
                )
                .unwrap(),
            Ordering::Equal
        );
        assert_eq!(
            mapped[0]
                .point
                .compare_linear_form(
                    &Vector3::new([3.into(), (-1).into(), 0.into()]),
                    &Real::from(10)
                )
                .unwrap(),
            Ordering::Equal
        );
        assert_eq!(
            mapped[0].parameter.compare(&contact.parameter).unwrap(),
            Ordering::Equal
        );
        assert!(
            mapped[0]
                .point
                .coincides_with(&contact.point.transformed(&transform()).unwrap())
                .unwrap()
        );
        let mut builder = ModelBuilder::new();
        builder.vertex(mapped[0].point.clone()).unwrap();
        builder.curve(mapped_curve).unwrap();
        let cold = reload(&builder.finish().unwrap(), &format!("parallel-frame-{i}"));
        assert_eq!(
            points(&mapped_plane, cold.curves().next().unwrap().geometry()).len(),
            1
        );
    }
    let contacts = points(&y_plane(2.into()), &lift(parallel(0, 1)));
    assert_eq!(contacts.len(), 1);
    assert_eq!(contacts[0].kind, CurveSurfaceContactKind::Repeated);
    assert_eq!(
        contacts[0].parameter.compare(&q(1, 2).into()).unwrap(),
        Ordering::Equal
    );
    assert!(
        contacts[0]
            .point
            .coincides_with(&Point3::new(2.into(), 2.into(), 0.into()).into())
            .unwrap()
    );
    let nonuniform_plane = Surface::plane(
        Point3::origin(),
        Vector3::x() * Real::from(2),
        Vector3::y() * Real::from(3),
    )
    .unwrap();
    assert_eq!(
        nonuniform_plane
            .planar_pcurve(&lift(parallel(0, 1)))
            .unwrap_err(),
        GeometryError::PlanarCurveConstruction(hypercurve::CurveError::InvalidSimilarityTransform)
    );
}

pub fn planar_nurbs_overlaps_use_authored_spans_before_spatial_transport() {
    let source = Curve2::try_nurbs(
        1,
        (0..6)
            .zip([0, 0, 1, -1, 0, 0])
            .map(|(x, y)| Uv::from_values(x, y))
            .collect(),
        vec![Real::one(); 6],
        [2, 2, 3, 5, 7, 11, 13, 13].map(Real::from).to_vec(),
    )
    .unwrap();
    let curve = lift(source);
    let CurveSurfaceIntersection::Components(components) =
        y_plane(0.into()).intersect_curve(&curve).unwrap()
    else {
        panic!("mixed planar components");
    };
    assert_eq!(components.points().len(), 1);
    assert_eq!(components.overlaps().len(), 2);
    assert_eq!(
        components.points()[0]
            .parameter
            .compare(&q(4, 11).into())
            .unwrap(),
        Ordering::Equal
    );
    for (range, (a, b)) in components
        .overlaps()
        .iter()
        .zip([(q(0, 1), q(1, 11)), (q(9, 11), q(1, 1))])
    {
        assert_eq!(range.start().compare(&a.into()).unwrap(), Ordering::Equal);
        assert_eq!(range.end().compare(&b.into()).unwrap(), Ordering::Equal);
    }
    let trimmed = curve.subcurve(&q(1, 20), &q(19, 20)).unwrap();
    let CurveSurfaceIntersection::Components(components) =
        y_plane(0.into()).intersect_curve(&trimmed).unwrap()
    else {
        panic!("trimmed mixed components");
    };
    assert_eq!(components.points().len(), 1);
    assert_eq!(components.overlaps().len(), 2);
    assert_eq!(
        components.points()[0]
            .parameter
            .compare(&q(23, 66).into())
            .unwrap(),
        Ordering::Equal
    );
    for (range, (a, b)) in components
        .overlaps()
        .iter()
        .zip([(q(0, 1), q(1, 22)), (q(169, 198), q(1, 1))])
    {
        assert_eq!(range.start().compare(&a.into()).unwrap(), Ordering::Equal);
        assert_eq!(range.end().compare(&b.into()).unwrap(), Ordering::Equal);
    }
}

pub fn plane_contacts_keep_tiny_exact_separations() {
    for (i, tiny) in ["1e-12", "1e-1000"].into_iter().enumerate() {
        let tiny: Real = tiny.parse().unwrap();
        let transform = Matrix4::from_row_major([
            1.into(),
            0.into(),
            0.into(),
            0.into(),
            0.into(),
            1.into(),
            0.into(),
            0.into(),
            0.into(),
            0.into(),
            1.into(),
            tiny.clone(),
            0.into(),
            0.into(),
            0.into(),
            1.into(),
        ]);
        let curve = map_curve(&cubic(), &transform);
        let plane = map_surface(&xy_plane(), &transform);
        let contact = points(&plane, &curve).remove(0);
        assert_eq!(
            contact
                .point
                .compare_linear_form(&Vector3::z(), &tiny)
                .unwrap(),
            Ordering::Equal
        );
        assert_eq!(
            contact
                .point
                .compare_linear_form(&Vector3::z(), &Real::zero())
                .unwrap(),
            Ordering::Greater
        );
        let source = map_curve(&lift(parallel(0, 1)), &transform);
        assert!(matches!(
            xy_plane().intersect_curve(&source).unwrap(),
            CurveSurfaceIntersection::None
        ));
        assert!(matches!(
            plane.intersect_curve(&source).unwrap(),
            CurveSurfaceIntersection::Contained
        ));
        let mut builder = ModelBuilder::new();
        builder.vertex(contact.point).unwrap();
        reload(&builder.finish().unwrap(), &format!("tiny-{i}"));
    }
}

pub fn contact_parameters_split_shared_edges_and_replay_solid_operations() {
    let bodies =
        builder::extrude_curve_region(&prior::original_parallel_cap(), 0.into(), 3.into()).unwrap();
    let edge = bodies
        .model()
        .edges()
        .find(|edge| edge.curve().kind() == Curve3Kind::PlanarImage)
        .unwrap();
    let contact = points(&x_plane(1.into()), edge.curve().geometry()).remove(0);
    let split = edge.split(contact.parameter.clone()).unwrap();
    assert!(
        split
            .details()
            .vertex()
            .point()
            .coincides_with(&contact.point)
            .unwrap()
    );
    assert_eq!(split.details().edge_uses().len(), 2);
    let cold = reload(split.model(), "contact-split-cap");
    for edge in cold.edges() {
        let uses = edge.uses().collect::<Vec<_>>();
        assert_eq!(uses.len(), 2);
        assert_ne!(uses[0].direction(), uses[1].direction());
    }
    let original = BodySet::from_solid(cold.solids().next().unwrap());
    let cutter =
        builder::extrude_curve_region(&prior::original_parallel_cap(), 1.into(), 2.into()).unwrap();
    let middle = boolean::intersection(&original, &cutter).unwrap();
    let ends = boolean::difference(&original, &cutter).unwrap();
    assert_eq!(ends.len(), 2);
    for (model, name, zs) in [
        (middle.model(), "contact-middle", [false, true, false]),
        (ends.model(), "contact-ends", [true, false, true]),
    ] {
        let cold = reload(model, name);
        for (z, inside) in [q(1, 2), q(3, 2), q(5, 2)].into_iter().zip(zs) {
            let point = Point3::new(2.into(), 0.into(), z);
            assert_eq!(
                cold.solids()
                    .any(|s| s.classify_point(&point).unwrap() == SolidPointLocation::Inside),
                inside
            );
        }
    }
    let united = boolean::union(&ends, &middle).unwrap();
    let united = reload(united.model(), "contact-reunited");
    let united = BodySet::from_solid(united.solids().next().unwrap());
    assert!(boolean::difference(&united, &original).unwrap().is_empty());
    assert!(boolean::difference(&original, &united).unwrap().is_empty());
    let empty = boolean::difference(&united, &united).unwrap();
    assert!(empty.is_empty());
    reload(empty.model(), "contact-empty");
    assert_eq!(boolean::union(&empty, &united).unwrap().len(), 1);
    let mapped = cold.transformed(&transform()).unwrap();
    let mapped = reload(&mapped, "contact-split-transformed");
    for (y, expected) in [
        (0, SolidPointLocation::Inside),
        (3, SolidPointLocation::Outside),
    ] {
        let query = transform()
            .transform_point3(&Point3::new(2.into(), y.into(), 1.into()))
            .unwrap();
        assert_eq!(
            mapped
                .solids()
                .next()
                .unwrap()
                .classify_point(&query)
                .unwrap(),
            expected
        );
    }
}

pub fn plane_contact_policies_exhaustion_and_recovery_preserve_certainty() {
    let denied = OperationContext::strict(WorkBudget::ZERO)
        .evaluate(|| xy_plane().intersect_curve(&cubic()));
    assert_eq!(denied.result.unwrap_err().limit, 0);
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || xy_plane().intersect_curve(&cubic()));
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
        assert!(matches!(
            outcome.into_unverified().unwrap(),
            CurveSurfaceIntersection::Points(_)
        ));
    }
    assert_eq!(points(&xy_plane(), &cubic()).len(), 1);
}

pub fn plane_contacts_reject_poles_before_empty_or_contained_shortcuts() {
    let source = Curve2::from(
        hypercurve::RationalBezier2::try_new(
            vec![
                Uv::from_values(0, 0),
                Uv::from_values(1, 1),
                Uv::from_values(2, 1),
                Uv::from_values(3, 0),
            ],
            [-1, -1, -1, 1].map(Real::from).to_vec(),
        )
        .unwrap(),
    );
    let curve = lift(source);
    let mut builder = ModelBuilder::new();
    builder.curve(curve.clone()).unwrap();
    let cold = reload(&builder.finish().unwrap(), "pole-definition");
    for curve in [&curve, cold.curves().next().unwrap().geometry()] {
        for plane in [
            xy_plane(),
            Surface::plane(
                Point3::new(0.into(), 0.into(), 1.into()),
                Vector3::x(),
                Vector3::y(),
            )
            .unwrap(),
            x_plane(1.into()),
        ] {
            assert!(matches!(
                plane.intersect_curve(curve),
                Err(GeometryError::PlanarCurve(_))
            ));
        }
        curve.point_at(&Real::zero().into()).unwrap();
        curve.point_at(&Real::one().into()).unwrap();
    }
    // The pole at cbrt(1/2) is outside this consumed range. Its endpoints
    // and complete interval are finite even though the original source is not.
    let finite = curve.subcurve(&Real::zero(), &q(1, 2)).unwrap();
    for (i, finite) in [finite.clone(), finite.reversed().unwrap()]
        .into_iter()
        .enumerate()
    {
        assert!(matches!(
            xy_plane().intersect_curve(&finite).unwrap(),
            CurveSurfaceIntersection::Contained
        ));
        finite.bounds().unwrap();
        let CurveParameterLocation::Parameters(endpoint) = finite
            .parameters_of(&Point3::new(1.into(), 1.into(), 0.into()))
            .unwrap()
        else {
            panic!("finite endpoint inverse");
        };
        assert_eq!(endpoint.len(), 1);
        assert_eq!(
            endpoint[0]
                .compare(&Real::from(if i == 0 { 1 } else { 0 }).into())
                .unwrap(),
            Ordering::Equal
        );
        let contacts = points(&x_plane(q(1, 2)), &finite);
        assert_eq!(contacts.len(), 1);
        assert_eq!(
            contacts[0]
                .point
                .compare_linear_form(&Vector3::x(), &q(1, 2))
                .unwrap(),
            Ordering::Equal
        );
        assert_eq!(contacts[0].kind, CurveSurfaceContactKind::Simple);
        let mut builder = ModelBuilder::new();
        builder.curve(finite).unwrap();
        builder.vertex(contacts[0].point.clone()).unwrap();
        let cold = reload(
            &builder.finish().unwrap(),
            &format!("finite-pole-subcurve-{i}"),
        );
        assert_eq!(
            points(&x_plane(q(1, 2)), cold.curves().next().unwrap().geometry()).len(),
            1
        );
    }
    assert_eq!(points(&x_plane(1.into()), &lift(parallel(0, 1))).len(), 1);
}

pub fn collapsed_planar_spans_keep_parameter_components() {
    let source = Curve2::from(
        hypercurve::RationalBezier2::try_new(vec![Uv::from_values(1, 2); 4], vec![Real::one(); 4])
            .unwrap(),
    );
    let curve = lift(source);
    assert!(matches!(
        x_plane(1.into()).intersect_curve(&curve).unwrap(),
        CurveSurfaceIntersection::Contained
    ));
    assert!(matches!(
        x_plane(2.into()).intersect_curve(&curve).unwrap(),
        CurveSurfaceIntersection::None
    ));
    let source = Curve2::try_nurbs(
        1,
        [(1, 2), (1, 2), (2, 3), (1, 4)]
            .map(|(x, y)| Uv::from_values(x, y))
            .to_vec(),
        vec![Real::one(); 4],
        [2, 2, 3, 5, 7, 7].map(Real::from).to_vec(),
    )
    .unwrap();
    let CurveSurfaceIntersection::Components(components) =
        x_plane(1.into()).intersect_curve(&lift(source)).unwrap()
    else {
        panic!("collapsed range and isolated endpoint");
    };
    assert_eq!(components.overlaps().len(), 1);
    assert_eq!(components.points().len(), 1);
    assert_eq!(
        components.overlaps()[0]
            .start()
            .compare(&Real::zero().into())
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        components.overlaps()[0]
            .end()
            .compare(&q(1, 5).into())
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        components.points()[0]
            .parameter
            .compare(&Real::one().into())
            .unwrap(),
        Ordering::Equal
    );
}

pub fn finite_cone_boundary_contacts_are_not_claimed_as_equation_roots() {
    let cone = Surface::cone(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        (Real::pi() / Real::from(4)).unwrap(),
    )
    .unwrap();
    let lower = Curve3::line(
        Point3::new((-1).into(), 0.into(), (-1).into()),
        Point3::origin(),
    )
    .unwrap();
    for (curve, endpoint) in [
        (lower.clone(), Real::one()),
        (lower.reversed().unwrap(), Real::zero()),
    ] {
        let contacts = points(&cone, &curve);
        assert_eq!(contacts.len(), 1);
        assert_eq!(
            contacts[0].parameter.compare(&endpoint.into()).unwrap(),
            Ordering::Equal
        );
        assert_eq!(contacts[0].kind, CurveSurfaceContactKind::Boundary);
        assert!(
            contacts[0]
                .point
                .coincides_with(&Point3::origin().into())
                .unwrap()
        );
    }
    let upper = Curve3::line(Point3::origin(), Point3::new(1.into(), 0.into(), 1.into())).unwrap();
    assert!(matches!(
        cone.intersect_curve(&upper).unwrap(),
        CurveSurfaceIntersection::Contained
    ));
}
