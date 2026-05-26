#![no_main]

use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{RectangularPocket, build_rectangular_rest_material_graph};
use hyperreal::{Rational, Real};
use libfuzzer_sys::fuzz_target;

fn r(value: i64) -> Real {
    Real::new(Rational::new(value))
}

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(r(x), r(y))
}

fn coord(byte: u8) -> i64 {
    i64::from(byte % 64) - 16
}

fn ordered_rect(bytes: &[u8]) -> Option<RectangularPocket> {
    if bytes.len() < 4 {
        return None;
    }
    let x0 = coord(bytes[0]);
    let y0 = coord(bytes[1]);
    let x1 = coord(bytes[2]);
    let y1 = coord(bytes[3]);
    if x0 == x1 || y0 == y1 {
        return None;
    }
    RectangularPocket::new(p(x0.min(x1), y0.min(y1)), p(x0.max(x1), y0.max(y1))).ok()
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 12 {
        return;
    }

    let stock = match ordered_rect(&data[0..4]) {
        Some(stock) => stock,
        None => return,
    };
    let mut cutters = Vec::new();
    for chunk in data[4..].chunks(4).take(8) {
        if let Some(cutter) = ordered_rect(chunk) {
            cutters.push(cutter);
        }
    }
    if cutters.is_empty() {
        return;
    }

    let graph = build_rectangular_rest_material_graph(stock.clone(), cutters, PredicatePolicy::default())
        .expect("integer rectangle rest graph should certify");
    assert!(graph.all_area_certified());
    assert_eq!(
        graph.removed_area() + graph.remaining_area(),
        stock.width() * stock.height()
    );
    for stage in &graph.stages {
        assert!(stage.area_certification.all_satisfied());
        assert_eq!(stage.cuts.len(), stage.before.len());
        assert_eq!(stage.after.len(), graph.stages[stage.cutter_index].after.len());
        for piece in &stage.after {
            assert_ne!(piece.min().x, piece.max().x);
            assert_ne!(piece.min().y, piece.max().y);
        }
    }
});
