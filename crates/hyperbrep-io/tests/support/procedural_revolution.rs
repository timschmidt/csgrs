use hyperbrep::{
    ArchiveLimits, EntityKind, OperationContext, Point3, RawModel, Real, SolidPointLocation,
    SurfaceKind, WorkBudget, builder,
};
use hyperbrep_io::step::{StepBody, StepWriteOptions, WriteErrorKind, write_advanced};

pub fn procedural_revolution_step_rejection_preserves_native_solid(
    profile: &hypercurve::CurveRegion2,
) {
    let shift = hypercurve::Similarity2::try_from_real_affine(
        1.into(),
        0.into(),
        0.into(),
        1.into(),
        3.into(),
        0.into(),
    )
    .unwrap();
    let region = profile.transform_similarity(&shift).unwrap();
    let bodies = builder::revolve_curve_region(&region).unwrap();
    let original = bodies.model().to_json().unwrap();
    let options = StepWriteOptions::new("procedural-revolution.step", "2026-10-18T00:00:00");
    let context = OperationContext::strict(WorkBudget {
        refinements: 100_000,
        promotions: 10_000,
    });
    for _ in 0..2 {
        let model = RawModel::from_json(&original, &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        let input = model
            .solids()
            .map(|solid| StepBody {
                solid,
                name: "selected revolution",
            })
            .collect::<Vec<_>>();
        let error = write_advanced(&input, &options, &context)
            .result
            .unwrap()
            .unwrap_err();
        assert!(
            matches!(
                error.kind,
                WriteErrorKind::UnsupportedSurface(SurfaceKind::Revolution)
            ),
            "{error:?}"
        );
        assert_eq!(error.body, Some(0));
        assert!(matches!(error.entity, Some((EntityKind::Face, _))));
        assert_eq!(model.to_json().unwrap(), original);
        assert_eq!(
            model
                .solids()
                .next()
                .unwrap()
                .classify_point(&Point3::new(
                    (Real::from(7) / Real::from(2)).unwrap(),
                    0.into(),
                    0.into()
                ))
                .unwrap(),
            SolidPointLocation::Inside
        );
    }
}
