#[path = "../../hyperreal/tests/support/representations.rs"]
mod scalar_cases;

#[path = "../dev/mercator.rs"]
mod workflow;

#[test]
fn all_484_scale_latitude_representation_pairs_complete_the_projection_workflow() {
    for first in 0..22 {
        for second in 0..22 {
            let report = workflow::exercise([first, second, 128, 2, 0, 0]);
            assert!(report.is_ok(), "recipe pair {first}/{second}: {report:?}");
        }
    }
}
