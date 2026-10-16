use csgrs::{Real, curve};
use hypercurve::{CurveRegion2, CurveString2};
use std::{
    panic::{AssertUnwindSafe, catch_unwind},
    path::Path,
};

fn r(value: i32) -> Real {
    value.into()
}
fn unknown_zero() -> Real {
    let s = Real::e().sin();
    let c = Real::e().cos();
    &s * &s + &c * &c - r(1)
}
fn region_record(region: &CurveRegion2) -> String {
    let mut contours = Vec::new();
    for profile in region.boundary_profiles().unwrap() {
        let (material, holes) = profile.linear_contours().unwrap().unwrap();
        for ring in std::iter::once(&material).chain(holes.iter()) {
            let points = ring
                .segments()
                .iter()
                .map(|edge| {
                    format!(
                        "[{},{}]",
                        edge.start().x().to_json(),
                        edge.start().y().to_json()
                    )
                })
                .collect::<Vec<_>>();
            contours.push(format!("[{}]", points.join(",")));
        }
    }
    format!("[{}]\n", contours.join(",\n"))
}
fn strings_record(strings: &[CurveString2]) -> String {
    let records = strings
        .iter()
        .map(|string| {
            let edges = string
                .segments()
                .iter()
                .map(|edge| {
                    format!(
                        "[[{},{}],[{},{}]]",
                        edge.start().x().to_json(),
                        edge.start().y().to_json(),
                        edge.end().x().to_json(),
                        edge.end().y().to_json()
                    )
                })
                .collect::<Vec<_>>();
            format!("[{}]", edges.join(","))
        })
        .collect::<Vec<_>>();
    format!("[{}]\n", records.join(",\n"))
}
fn main() {
    let args = std::env::args().collect::<Vec<_>>();
    let out = Path::new(&args[1]);
    std::fs::create_dir_all(out).unwrap();
    let font = include_bytes!("../../../asar.ttf");
    for (label, text, data, scale) in [
        ("invalid bytes", "A", &b"not a font"[..], r(1)),
        ("invalid bytes on empty", "", &b"not a font"[..], r(1)),
        ("zero scale", "A", &font[..], r(0)),
        ("negative scale", "A", &font[..], r(-1)),
        ("undecided scale", "A", &font[..], unknown_zero()),
        ("missing glyph", "\u{10ffff}", &font[..], r(1)),
        ("empty text", "", &font[..], r(1)),
        ("whitespace", " \n\t", &font[..], r(1)),
    ] {
        match catch_unwind(AssertUnwindSafe(|| curve::truetype_text(text, data, scale))) {
            Ok(region) => println!("truetype {label}: success_empty={}", region.is_empty()),
            Err(_) => println!("truetype {label}: panic"),
        }
    }
    for (label, text, scale) in [
        ("readme-truetype", "CSG", r(72)),
        ("bench-truetype", "csgrs benchmark", r(24)),
        ("truetype-unit", "I", r(1)),
        (
            "truetype-near-unit",
            "I",
            r(1) + (Real::one() / Real::from(2_u8).powi_i64(100).unwrap()).unwrap(),
        ),
    ] {
        let region = curve::truetype_text(text, font, scale);
        let record = region_record(&region);
        std::fs::write(out.join(format!("{label}.json")), record).unwrap();
        println!(
            "fixture {label}: empty={} components={}",
            region.is_empty(),
            region.len()
        );
    }
    let same = std::fs::read(out.join("truetype-unit.json")).unwrap()
        == std::fs::read(out.join("truetype-near-unit.json")).unwrap();
    println!("distinct exact TrueType scales have identical represented coordinates: {same}");
    assert!(same, "reproduction must establish scale rounding");

    let valid = hypercurve::hershey::Font::new(&["MWRMNV RRMVV"], 'A');
    let invalid = hypercurve::hershey::Font::new(&["M"], 'A');
    let degenerate = hypercurve::hershey::Font::new(&["MWRRRR"], 'A');
    let mixed = hypercurve::hershey::Font::new(&["MWRMNV RRMVV", "M"], 'A');
    for (label, text, font, size) in [
        ("malformed record", "A", &invalid, r(1)),
        ("degenerate stroke", "A", &degenerate, r(1)),
        ("missing glyph", "B", &valid, r(1)),
        ("partial text", "ABA", &mixed, r(1)),
        ("zero scale", "A", &valid, r(0)),
        ("negative scale", "A", &valid, r(-1)),
        ("undecided scale", "A", &valid, unknown_zero()),
        ("empty text", "", &valid, r(1)),
        ("whitespace", " \n\t", &valid, r(1)),
    ] {
        match catch_unwind(AssertUnwindSafe(|| curve::hershey_strings(text, font, size))) {
            Ok(strings) => println!("hershey {label}: success_strings={}", strings.len()),
            Err(_) => println!("hershey {label}: panic"),
        }
    }
    for (label, text, font, size) in [
        (
            "readme-hershey",
            "CSG",
            &hypercurve::hershey::fonts::ROWMANS,
            r(1),
        ),
        (
            "bench-hershey",
            "CSGRS",
            &hypercurve::hershey::fonts::FUTURAL,
            r(2),
        ),
        ("custom-hershey", "AA", &valid, r(1)),
        ("negative-hershey", "AA", &valid, r(-1)),
    ] {
        let strings = curve::hershey_strings(text, font, size);
        assert!(!strings.is_empty());
        std::fs::write(out.join(format!("{label}.json")), strings_record(&strings)).unwrap();
        println!(
            "fixture {label}: strings={} edges={}",
            strings.len(),
            strings.iter().map(|s| s.segments().len()).sum::<usize>()
        );
    }
    let strokes = curve::hershey_strings("AA", &valid, r(1));
    assert_eq!(strokes.len(), 4);
    for (stroke, (x0, y0, x1, y1)) in
        strokes
            .iter()
            .zip([(0, 5, -4, -4), (0, 5, 4, -4), (8, 5, 4, -4), (8, 5, 12, -4)])
    {
        assert_eq!(stroke.segments().len(), 1);
        let edge = &stroke.segments()[0];
        assert_eq!(edge.start(), &hypercurve::Point2::new(r(x0), r(y0)));
        assert_eq!(edge.end(), &hypercurve::Point2::new(r(x1), r(y1)));
    }
    println!(
        "independent Hershey coordinates, exact advance and Cartesian orientation passed"
    );
}
