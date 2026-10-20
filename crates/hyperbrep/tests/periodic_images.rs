#[path = "support/angular_images.rs"]
mod support;

#[test]
fn equivalent_angular_spans_preserve_whole_images_without_merging_turns() {
    support::proofs();
}
