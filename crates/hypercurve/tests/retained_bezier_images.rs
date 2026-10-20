#[path = "support/retained_bezier_images.rs"]
mod acceptance;

#[test]
fn selected_quadratic_image_keeps_the_complete_curve_and_original_roots() {
    acceptance::selected_quadratic_image_keeps_the_complete_curve_and_original_roots();
}

#[test]
fn nonmonotone_native_image_preserves_its_entire_parameterized_polynomial() {
    acceptance::nonmonotone_native_image_preserves_its_entire_parameterized_polynomial();
}

#[test]
fn unavailable_higher_degree_views_do_not_replace_selected_evidence() {
    acceptance::unavailable_higher_degree_views_do_not_replace_selected_evidence();
}

#[test]
fn selected_weighted_curve_images_preserve_exact_quotients() {
    acceptance::selected_weighted_curve_images_preserve_exact_quotients();
}
