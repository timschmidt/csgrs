#[path = "archive_graph.rs"]
mod archive_graph;
use hyperbrep::*;
use hypercurve::{Curve2, Point2 as Uv};
use std::cmp::Ordering;

#[allow(dead_code)]
#[path = "../../../hypercurve/tests/support/selected_fiber_images.rs"]
mod curve_recipe;

pub fn mixed_fiber_charts_survive_brep_archives_and_transforms() {
    let source = curve_recipe::source();
    for coefficients in [[1, 1, 0], [3, -4, 2], [0, 1, 0]] {
        let [a, b, c] = coefficients;
        let value = source
            .affine_form(a.into(), b.into(), c.into())
            .parameter()
            .unwrap();
        let point = value
            .affine_point(1.into(), 0.into(), 0.into(), 0.into())
            .unwrap();
        let chord = Curve2::try_line(point, Uv::from_values(10, 0).into()).unwrap();
        let chart = chord.scalar_chart().unwrap();
        let curve = Curve3::planar_image(
            chart.curve().clone(),
            Point3::origin(),
            Vector3::new([1.into(), 2.into(), 3.into()]),
            Vector3::new([0.into(), 1.into(), 0.into()]),
        )
        .unwrap();
        let domain = curve.domain().clone();
        let start = curve.point_at(domain.start()).unwrap();
        let end = curve.point_at(domain.end()).unwrap();
        assert!(start.try_coordinates().is_none());
        assert_eq!(
            start
                .compare_linear_form(
                    &Vector3::new([2.into(), (-1).into(), 0.into()]),
                    &Real::zero()
                )
                .unwrap(),
            Ordering::Equal
        );
        let mut builder = ModelBuilder::new();
        let first = builder.vertex(start.clone()).unwrap();
        let last = builder.vertex(end).unwrap();
        let geometry = builder.curve(curve).unwrap();
        builder.edge(&first, &last, &geometry, domain).unwrap();
        let model = builder.finish().unwrap();
        let json = model.to_json().unwrap();
        assert!(json.contains("AffineForm") && json.contains("PolynomialFiber"));
        let restored = RawModel::from_json(&json, &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(restored.to_json().unwrap(), json);
        let raw: serde_json::Value = serde_json::from_str(&json).unwrap();
        let affine_index = raw["selection"]["parameters"]
            .as_array()
            .unwrap()
            .iter()
            .position(|entry| entry.get("AffineForm").is_some())
            .unwrap();
        for mode in 0..2 {
            let mut bad = raw.clone();
            let point = archive_graph::at_mut(
                &mut bad,
                &format!("/selection/parameters/{affine_index}/AffineForm/point/PolynomialFiber"),
            );
            if mode == 0 {
                point["interval"] = serde_json::json!([Real::from(3), Real::from(4)]);
            } else {
                point["parameter"] = serde_json::json!({"Selected": affine_index});
            }
            let error = RawModel::from_json(&bad.to_string(), &ArchiveLimits::default())
                .unwrap()
                .validate()
                .unwrap_err();
            if mode == 0 {
                assert!(matches!(
                    error,
                    PersistenceError::Build(BuildError::Geometry(GeometryError::PlanarCurve(
                        hypercurve::ExactCurveError::Invalid {
                            cause: hypercurve::CurveError::InvalidBezierAlgebraicParameter,
                            ..
                        }
                    )))
                ));
            } else {
                assert!(matches!(error, PersistenceError::InvalidReference { .. }));
            }
            RawModel::from_json(&json, &ArchiveLimits::default())
                .unwrap()
                .validate()
                .unwrap();
        }
        let mut oversized = raw.clone();
        oversized["selection"]["parameters"][affine_index]["AffineForm"]["coefficients"][0] =
            serde_json::json!("invalid scalar behind degree gate");
        assert!(matches!(
            RawModel::from_json(
                &oversized.to_string(),
                &ArchiveLimits {
                    degree: 2,
                    ..ArchiveLimits::default()
                }
            ),
            Err(PersistenceError::ResourceLimit(ArchiveLimitExceeded {
                resource: ArchiveResource::Degree,
                observed: 3,
                limit: 2
            }))
        ));
        assert!(
            restored
                .vertices()
                .next()
                .unwrap()
                .point()
                .coincides_with(&start)
                .unwrap()
        );
        let transform = Matrix4::from_row_major([
            1.into(),
            0.into(),
            0.into(),
            5.into(),
            0.into(),
            1.into(),
            0.into(),
            (-2).into(),
            0.into(),
            0.into(),
            1.into(),
            7.into(),
            0.into(),
            0.into(),
            0.into(),
            1.into(),
        ]);
        let moved = restored.transformed(&transform).unwrap();
        let moved_json = moved.to_json().unwrap();
        let cold = RawModel::from_json(&moved_json, &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(cold.to_json().unwrap(), moved_json);
        assert!(
            start
                .transformed(&transform)
                .unwrap()
                .coincides_with(cold.vertices().next().unwrap().point())
                .unwrap()
        );
        #[cfg(not(target_arch = "wasm32"))]
        if let Some(directory) = std::env::var_os("HYPERBREP_FIBER_IMAGE_ARTIFACT_DIR") {
            std::fs::create_dir_all(&directory).unwrap();
            std::fs::write(
                std::path::PathBuf::from(directory).join(format!("fiber-{a}-{b}-{c}.json")),
                moved_json,
            )
            .unwrap();
        }
    }
}
