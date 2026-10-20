use hyperbrep::{
    ArchiveLimits, Curve3Kind, EntityKind, Model, OperationContext, Point3, RawModel,
    SolidPointLocation, WorkBudget, builder,
};
use hyperbrep_io::step::{StepBody, StepWriteOptions, WriteErrorKind, write_advanced};

pub fn selected_procedural_profiles_fail_step_with_attribution_and_native_recovery(
    profile: &hypercurve::CurveRegion2,
) {
    let bodies = builder::extrude_curve_region(profile, 0.into(), 3.into()).unwrap();
    let replay = |model: &Model| {
        RawModel::from_json(&model.to_json().unwrap(), &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap()
    };
    for model in [bodies.model().clone(), replay(bodies.model())] {
        let options = StepWriteOptions::new("selected-procedural.step", "2026-10-18T00:00:00");
        let context = OperationContext::strict(WorkBudget {
            refinements: 100_000,
            promotions: 10_000,
        });
        let input = model
            .solids()
            .map(|solid| StepBody {
                solid,
                name: "selected procedural profile",
            })
            .collect::<Vec<_>>();
        let error = write_advanced(&input, &options, &context)
            .result
            .unwrap()
            .unwrap_err();
        // The full procedural carrier reaches the STEP boundary before a
        // selected Cartesian vertex. This is the existing exact carrier
        // frontier, not an approximation or a fabricated rational coordinate.
        assert!(
            matches!(
                error.kind,
                WriteErrorKind::UnsupportedCurve(Curve3Kind::PlanarImage)
            ),
            "{error:?}"
        );
        assert_eq!(error.body, Some(0));
        assert!(matches!(error.entity, Some((EntityKind::Edge, _))));
        let cold = replay(&model);
        assert_eq!(cold.to_json().unwrap(), model.to_json().unwrap());
        assert_eq!(
            cold.solids()
                .next()
                .unwrap()
                .classify_point(&Point3::new(
                    (hyperbrep::Real::one() / hyperbrep::Real::from(2)).unwrap(),
                    0.into(),
                    1.into()
                ))
                .unwrap(),
            SolidPointLocation::Inside
        );
    }
}
