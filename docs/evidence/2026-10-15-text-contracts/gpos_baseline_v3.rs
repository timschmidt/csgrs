#[path = "../../../tests/support/text_font.rs"]
mod text_font;
use csgrs::{Real, curve};
fn main() {
    let font = text_font::font(true);
    let face = ttf_parser::Face::parse(&font, 0).unwrap();
    assert!(face.tables().gpos.is_some());
    let unit = Real::try_from(0.3527777).unwrap();
    for (text, x, expected) in [("Q", 50, Some(true)), ("QQ", 850, Some(true)), ("QQ", 1050, Some(false))] {
        let region = curve::truetype_text(text, &font, Real::from(1000));
        let result = curve::contains_xy(&region, Real::from(x) * &unit, Real::from(10) * &unit).unwrap();
        assert_eq!(result, expected);
        println!("{text}: point at ({x},10) font units = {result:?}; QQ should be inside at 1050, outside at 850");
    }
}
