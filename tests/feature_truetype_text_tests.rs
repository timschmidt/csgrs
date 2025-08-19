#![cfg(feature = "truetype-text")]

use csgrs::sketch::Sketch;

#[test]
fn text() {
    // We can't easily test visually, but we can at least test that it doesn't panic
    // and returns some polygons for normal ASCII letters.
    let font_data = include_bytes!("../asar.ttf");
    let text_csg: Sketch<()> = Sketch::text("ABC", font_data, 10.0, None);
    assert!(!text_csg.geometry.is_empty());
}
