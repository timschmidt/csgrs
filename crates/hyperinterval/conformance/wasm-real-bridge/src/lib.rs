#[path = "../../../tests/support/real_dyadic_lifecycle.rs"]
mod lifecycle;

#[unsafe(no_mangle)]
pub extern "C" fn verify_real_dyadic_retry() {
    lifecycle::check_retry();
}
