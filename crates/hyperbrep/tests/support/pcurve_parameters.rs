use hyperbrep::*;
use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Curve2, CurveParameter2, CurveParameterRange2, Point2 as Uv,
    QuadraticBezier2,
};
use std::cmp::Ordering;

#[allow(dead_code)]
#[path = "../../../hypercurve/tests/support/selected_fiber_images.rs"]
mod fibers;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn selected() -> CurveParameter2 {
    BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![
                (-1).into(),
                0.into(),
                0.into(),
                2.into(),
            ])
            .unwrap(),
            BezierParameterInterval::try_new(q(3, 4), q(7, 8)).unwrap(),
        )
        .unwrap(),
    )
    .into()
}

fn sources() -> Vec<Curve2> {
    let quadratic = Curve2::from(QuadraticBezier2::new(
        Uv::from_values(0, 0),
        Uv::new(q(1, 2), 0.into()),
        Uv::from_values(1, 1),
    ))
    .restrict_source(CurveParameterRange2::try_new(q(1, 4).into(), selected()).unwrap())
    .unwrap();
    let value = fibers::source()
        .affine_form(1.into(), 1.into(), 0.into())
        .parameter()
        .unwrap();
    let chord = Curve2::try_line(
        value
            .affine_point(1.into(), 0.into(), 0.into(), 0.into())
            .unwrap(),
        Uv::from_values(10, 0).into(),
    )
    .unwrap();
    vec![quadratic, chord.scalar_chart().unwrap().curve().clone()]
}

fn frame() -> Surface {
    Surface::plane(
        Point3::new(1.into(), 2.into(), 3.into()),
        Vector3::x() * Real::from(2),
        Vector3::y() * Real::from(2),
    )
    .unwrap()
}

fn lifted(source: Curve2) -> Curve3 {
    Curve3::planar_image(
        source,
        Point3::new(1.into(), 2.into(), 3.into()),
        Vector3::x() * Real::from(2),
        Vector3::y() * Real::from(2),
    )
    .unwrap()
}

fn check_map(plane: &Surface, curve: &Curve3) {
    let map = plane.planar_pcurve(curve).unwrap().unwrap();
    let (first, last) = map.curve().parameter_endpoints();
    assert!(first.scalar().is_none() || last.scalar().is_none());
    for (local, expected) in [
        (first, curve.domain().start()),
        (last, curve.domain().end()),
    ] {
        let spatial = map.spatial_parameter_at(local).unwrap();
        assert_eq!(spatial.compare(expected).unwrap(), Ordering::Equal);
        assert_eq!(spatial.try_real().is_some(), local.scalar().is_some());
        let image = curve.point_at(&spatial).unwrap();
        assert_eq!(
            image
                .compare_linear_form(&Vector3::z(), &Real::from(3))
                .unwrap(),
            Ordering::Equal
        );
        assert!(
            image
                .coincides_with(&curve.point_at(expected).unwrap())
                .unwrap()
        );
        // The original spatial frame is x=1+2u, y=2+2v, z=3.
        // Compare its retained source image with a separately lifted pcurve.
        let pcurve_lift = lifted(map.curve().clone());
        let endpoint = if std::ptr::eq(local, first) {
            pcurve_lift.domain().start()
        } else {
            pcurve_lift.domain().end()
        };
        assert!(
            image
                .coincides_with(&pcurve_lift.point_at(endpoint).unwrap())
                .unwrap()
        );
    }
}

pub fn selected_pcurve_maps_compose_and_replay() {
    for (index, source) in sources().into_iter().enumerate() {
        for reverse in [false, true] {
            let source = if reverse {
                source.reversed().unwrap()
            } else {
                source.clone()
            };
            let curve = lifted(source);
            check_map(&frame(), &curve);
            let map = frame().planar_pcurve(&curve).unwrap().unwrap();
            let mut builder = ModelBuilder::new();
            let first = builder
                .vertex(curve.point_at(curve.domain().start()).unwrap())
                .unwrap();
            let last = builder
                .vertex(curve.point_at(curve.domain().end()).unwrap())
                .unwrap();
            let geometry = builder.curve(curve.clone()).unwrap();
            builder
                .edge(&first, &last, &geometry, curve.domain().clone())
                .unwrap();
            builder.pcurve(Pcurve::new(map.curve().clone())).unwrap();
            let model = builder.finish().unwrap();
            let json = model.to_json().unwrap();
            let mut cold = model.clone();
            for _ in 0..3 {
                cold = RawModel::from_json(&cold.to_json().unwrap(), &ArchiveLimits::default())
                    .unwrap()
                    .validate()
                    .unwrap();
                assert_eq!(cold.to_json().unwrap(), json);
                check_map(&frame(), cold.curves().next().unwrap().geometry());
            }
            #[cfg(not(target_arch = "wasm32"))]
            if let Ok(directory) = std::env::var("HYPERBREP_PCURVE_MAP_ARTIFACT_DIR") {
                let directory = std::path::PathBuf::from(directory);
                std::fs::create_dir_all(&directory).unwrap();
                std::fs::write(directory.join(format!("map-{index}-{reverse}.json")), &json)
                    .unwrap();
            }
            #[cfg(target_arch = "wasm32")]
            let _ = index;
            let transform = Matrix4::from_row_major([
                1.into(),
                0.into(),
                0.into(),
                4.into(),
                0.into(),
                1.into(),
                0.into(),
                (-2).into(),
                0.into(),
                0.into(),
                1.into(),
                0.into(),
                0.into(),
                0.into(),
                0.into(),
                1.into(),
            ]);
            let moved = cold.transformed(&transform).unwrap();
            let moved = RawModel::from_json(&moved.to_json().unwrap(), &ArchiveLimits::default())
                .unwrap()
                .validate()
                .unwrap();
            for (original, moved) in cold.vertices().zip(moved.vertices()) {
                assert!(
                    original
                        .point()
                        .transformed(&transform)
                        .unwrap()
                        .coincides_with(moved.point())
                        .unwrap()
                );
            }
        }
    }
}

pub fn pcurve_maps_check_domains_and_recover_after_exhaustion() {
    // The scalar owner supplies bounds directly, including exterior charts.
    // No spatial/planar point or separate coordinate root is needed.
    let fiber = fibers::source()
        .affine_form(1.into(), 1.into(), 0.into())
        .parameter()
        .unwrap();
    for source in [Real::from(3).into(), selected(), fiber] {
        for scale in [-2, 3] {
            for offset in [-5, 5] {
                let value = source
                    .affine_image(&Real::from(scale), &Real::from(offset))
                    .unwrap();
                let (lower, upper) = value.scalar_bounds().unwrap();
                assert_ne!(
                    value.compare(&lower.clone().into()).unwrap(),
                    Ordering::Less
                );
                assert_ne!(
                    value.compare(&upper.clone().into()).unwrap(),
                    Ordering::Greater
                );
                assert_eq!(
                    value
                        .affine_image(&q(1, scale), &q(-offset, scale))
                        .unwrap()
                        .compare(&source)
                        .unwrap(),
                    Ordering::Equal
                );
            }
        }
    }
    let curve = lifted(sources().remove(0));
    let map = frame().planar_pcurve(&curve).unwrap().unwrap();
    for outside in [Real::from(-1), Real::one()] {
        assert!(matches!(
            map.spatial_parameter_at(&outside.into()),
            Err(GeometryError::ParameterOutsideDomain)
        ));
    }
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let result = hypercurve::evaluate_under(policy, || check_map(&frame(), &curve));
        assert_eq!(result.certainty(), hypercurve::CurveCertainty::Certified);
    }
    let model = {
        let mut builder = ModelBuilder::new();
        builder.curve(curve).unwrap();
        builder.finish().unwrap()
    };
    let json = model.to_json().unwrap();
    // This direct rational curve needs no instrumented promotion/refinement.
    // A zero solver allowance is not a bound on every arithmetic operation.
    let direct = OperationContext::strict(WorkBudget::ZERO).evaluate(|| {
        RawModel::from_json(&json, &ArchiveLimits::default())
            .unwrap()
            .validate()
    });
    let cold = direct.result.unwrap().unwrap();
    assert_eq!(direct.usage.refinements, 0);
    assert_eq!(direct.usage.promotions, 0);
    check_map(&frame(), cold.curves().next().unwrap().geometry());
    // Replaying the mixed selected-fiber affine form does require promotion.
    let mut builder = ModelBuilder::new();
    builder.curve(lifted(sources().remove(1))).unwrap();
    let json = builder.finish().unwrap().to_json().unwrap();
    let denied = OperationContext::strict(WorkBudget::ZERO).evaluate(|| {
        RawModel::from_json(&json, &ArchiveLimits::default())
            .unwrap()
            .validate()
    });
    assert_eq!(denied.result.unwrap_err().limit, 0);
    let cold = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    check_map(&frame(), cold.curves().next().unwrap().geometry());
}
