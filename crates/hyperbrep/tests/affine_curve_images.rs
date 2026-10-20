#[path = "support/affine_curve_images.rs"]
mod support;

#[test]
fn affine_images_cover_native_and_retained_families() {
    support::affine_images_cover_native_and_retained_families();
}

#[test]
fn affine_images_preserve_periods_derivatives_and_admission() {
    support::affine_images_preserve_periods_derivatives_and_admission();
}
