#[path = "../../../tests/support/text_font.rs"]
mod text_font;
use csgrs::{Real, curve};
fn main() {
    let font = text_font::font(true);
    let face = ttf_parser::Face::parse(&font, 0).unwrap();
    let left = face.glyph_index('R').unwrap();
    let right = face.glyph_index(' ').unwrap();
    for lookup in face.tables().gpos.unwrap().lookups {
        for subtable in lookup.subtables.into_iter::<ttf_parser::gpos::PositioningSubtable>() {
            if let ttf_parser::gpos::PositioningSubtable::Pair(ttf_parser::gpos::PairAdjustment::Format2 { coverage, classes, matrix }) = subtable {
                assert!(coverage.get(left).is_some());
                let (a, b) = matrix.get((classes.0.get(left), classes.1.get(right))).unwrap();
                assert_eq!((a.x_advance, b.x_placement), (-200, 0));
                println!("R-space has covered pair adjustment -200; advances 1000 + 1000 => next R starts at 1800");
            }
        }
    }
    for (text, x, y, expected) in [("RR",850,50,true), ("QQ",1050,10,true), ("R R",1850,50,true), ("R R",2050,50,false), ("R\tR",5050,50,true), ("R\nR",50,-950,true), ("R\rR",50,50,true)] {
        let region = curve::truetype_text(text, &font, (Real::from(360_000) / Real::from(127)).unwrap()).unwrap();
        let location = curve::contains_xy(&region, Real::from(x), Real::from(y)).unwrap();
        println!("{text:?} ({x},{y}): {location:?}, expected {expected}");
        assert_eq!(location, Some(expected));
    }
}
