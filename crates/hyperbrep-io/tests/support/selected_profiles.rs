use hyperbrep::{Model, OperationContext, Real, WorkBudget, builder};
use hyperbrep_io::step::{StepBody, StepWriteOptions, WriteErrorKind, write_advanced};
use hypercurve::CurveRegion2;

type ProfileSource = fn(bool, bool, bool) -> (CurveRegion2, Real);

pub fn selected_profile_step_vertices_remain_explicitly_unencodable(
    profile: ProfileSource,
    replay: fn(&Model) -> Model,
) {
    let context = OperationContext::strict(WorkBudget {
        refinements: 100_000,
        promotions: 10_000,
    });
    let options = StepWriteOptions::new("selected.step", "2026-10-18T00:00:00");
    for xor in [false, true] {
        for affine in [false, true] {
            let (region, _) = profile(xor, affine, false);
            let original = builder::extrude_curve_region(&region, 0.into(), 3.into()).unwrap();
            for model in [original.model().clone(), replay(original.model())] {
                let bodies = model
                    .solids()
                    .map(|solid| StepBody {
                        solid,
                        name: "selected exact profile",
                    })
                    .collect::<Vec<_>>();
                let error = write_advanced(&bodies, &options, &context)
                    .result
                    .unwrap()
                    .unwrap_err();
                assert!(
                    matches!(error.kind, WriteErrorKind::NonRational),
                    "{error:?}"
                );
                assert!(error.body.is_some());
                assert!(error.entity.is_some());
                // No partial file replaces the exact model on failure.
                assert_eq!(replay(&model).to_json().unwrap(), model.to_json().unwrap());
            }
        }
    }
}
