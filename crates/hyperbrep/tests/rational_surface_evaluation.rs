#[path = "support/rational_surface_evaluation.rs"]
mod support;

#[test]
fn rational_surface_policy_and_admission() {
    support::rational_surface_policy_and_admission();
}

#[test]
fn joint_tensor_images_compose_transform_and_replay() {
    support::joint_tensor_images_compose_transform_and_replay();
}

#[test]
fn selected_rational_extrusions_and_knot_boundaries() {
    support::selected_rational_extrusions_and_knot_boundaries();
}

#[test]
fn nurbs_second_span_and_final_endpoint() {
    support::nurbs_second_span_and_final_endpoint();
}
