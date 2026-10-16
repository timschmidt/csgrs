//! Native representation feasibility only; advance-only layout omits kerning.
//! This is not acceptance of final font layout or parser-source fidelity.
use csgrs::{Real, curve};
use hypercurve::{
    CubicBezier2, Curve2, CurvePath2, CurveRegion2, FillRule, LineSeg2, Point2,
    QuadraticBezier2,
};
use ttf_parser::{Face, OutlineBuilder};

struct Collector {
    scale: Real,
    offset: Real,
    current: Vec<Curve2>,
    paths: Vec<CurvePath2>,
    first: Option<Point2>,
    last: Option<Point2>,
}
impl Collector {
    fn point(&self, x: f32, y: f32) -> Point2 {
        Point2::new(
            (Real::try_from(x).unwrap() + &self.offset) * &self.scale,
            Real::try_from(y).unwrap() * &self.scale,
        )
    }
}
impl OutlineBuilder for Collector {
    fn move_to(&mut self, x: f32, y: f32) {
        assert!(self.current.is_empty() && self.last.is_none());
        let point = self.point(x, y);
        self.first = Some(point.clone());
        self.last = Some(point);
    }
    fn line_to(&mut self, x: f32, y: f32) {
        let point = self.point(x, y);
        let last = self.last.take().unwrap();
        if last != point {
            self.current
                .push(LineSeg2::try_new(last, point.clone()).unwrap().into());
        }
        self.last = Some(point);
    }
    fn quad_to(&mut self, x1: f32, y1: f32, x: f32, y: f32) {
        let point = self.point(x, y);
        let control = self.point(x1, y1);
        self.current.push(
            QuadraticBezier2::new(self.last.take().unwrap(), control, point.clone()).into(),
        );
        self.last = Some(point);
    }
    fn curve_to(&mut self, x1: f32, y1: f32, x2: f32, y2: f32, x: f32, y: f32) {
        let point = self.point(x, y);
        let c1 = self.point(x1, y1);
        let c2 = self.point(x2, y2);
        self.current
            .push(CubicBezier2::new(self.last.take().unwrap(), c1, c2, point.clone()).into());
        self.last = Some(point);
    }
    fn close(&mut self) {
        let first = self.first.take().unwrap();
        let last = self.last.take().unwrap();
        if last != first {
            self.current
                .push(LineSeg2::try_new(last, first).unwrap().into());
        }
        if !self.current.is_empty() {
            self.paths
                .push(CurvePath2::try_new(std::mem::take(&mut self.current)).unwrap());
        }
    }
}
fn main() {
    let args = std::env::args().collect::<Vec<_>>();
    let (text, points) = match args[1].as_str() {
        "readme" => ("CSG", 72),
        "benchmark" => ("csgrs benchmark", 24),
        _ => panic!("unknown original fixture"),
    };
    let face = Face::parse(include_bytes!("../../../asar.ttf"), 0).unwrap();
    let scale = (Real::from(points) * Real::from(127)
        / (Real::from(360) * Real::from(face.units_per_em())))
    .unwrap();
    let mut collector = Collector {
        scale,
        offset: Real::zero(),
        current: Vec::new(),
        paths: Vec::new(),
        first: None,
        last: None,
    };
    for character in text.chars() {
        let gid = face.glyph_index(character).unwrap();
        let outlined = face.outline_glyph(gid, &mut collector);
        assert!(outlined.is_some() || character.is_whitespace());
        assert!(collector.current.is_empty() && collector.last.is_none());
        collector.offset += Real::from(face.glyph_hor_advance(gid).unwrap());
    }
    println!(
        "fixture={} input_paths={} input_curves={} exact_point_unit=127/360 layout=advance-only",
        args[1],
        collector.paths.len(),
        collector
            .paths
            .iter()
            .map(|p| p.curves().len())
            .sum::<usize>()
    );
    let receipt = hypercurve::evaluate_under(hypercurve::PredicatePolicy::STRICT, || {
        CurveRegion2::try_from_boundary_paths(&collector.paths, FillRule::NonZero)
    });
    assert!(receipt.is_certified());
    let region = receipt
        .into_unverified()
        .expect("full exact outline must regularize");
    assert!(!region.is_empty());
    println!("native exact region components={}", region.len());
    let translated = curve::try_translated(&region, Real::from(3), Real::from(-2)).unwrap();
    let composed = translated
        .boolean_region(&CurveRegion2::empty(), hypercurve::BooleanOp::Union)
        .unwrap();
    assert!(!composed.is_empty());
    println!("strict translation and empty-union composition completed");
}
