#[path = "support/retained_rational_images.rs"]
mod support;

#[test]
fn mixed_images_keep_the_base_field_on_both_axes() {
    support::mixed_images_keep_the_base_field_on_both_axes();
}
#[test]
fn source_poles_and_zero_image_components_remain_explicit() {
    support::source_poles_and_zero_image_components_remain_explicit();
}
#[test]
fn rational_image_limits_and_budget_recovery() {
    support::rational_image_limits_and_budget_recovery();
}
