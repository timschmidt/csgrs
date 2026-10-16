use csgrs::{GeometryContext, Real, curve};
use hypercurve::{BooleanOp, CurveRegion2};

fn r(n: i32) -> Real { n.into() }
fn q(n: i32, d: i32) -> Real { (r(n) / r(d)).unwrap() }
fn rect(x0: Real, y0: Real, x1: Real, y1: Real) -> CurveRegion2 {
    curve::polygon(&[[x0.clone(), y0.clone()], [x1.clone(), y0], [x1, y1.clone()], [x0, y1]]).unwrap()
}
fn main() {
    let square = curve::square(r(4)).unwrap();
    let hole = rect(q(1,2), q(5,4), q(3,2), q(7,4));
    let holed = square.boolean_region(&hole, BooleanOp::Difference).unwrap();
    let disconnected = rect(r(0), r(0), q(5,4), r(4)).boolean_region(
        &rect(q(11,4), r(0), r(4), r(4)), BooleanOp::Union).unwrap();
    let overlap = square.boolean_region(&rect(r(1), r(1), r(2), r(3)), BooleanOp::Difference).unwrap();
    for (name, boundary) in [("hole", holed), ("disconnected", disconnected), ("boundary overlap", overlap)] {
        for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512, GeometryContext::STRICT] {
            match curve::hilbert_strings_with_context(&boundary, 1, r(0), &context) {
                Ok(outcome) => {
                    let segments = outcome.value.iter().flat_map(|s| s.segments()).collect::<Vec<_>>();
                    println!("{name} {context:?}: certainty={:?} runs={} segments={}", outcome.certainty, outcome.value.len(), segments.len());
                    for segment in &segments {
                        println!("  ({}, {}) -> ({}, {})", segment.start().x(), segment.start().y(), segment.end().x(), segment.end().y());
                    }
                    if name == "hole" {
                        assert!(segments.iter().any(|s| s.start().x() == &r(1) && s.start().y() == &r(1) && s.end().y() == &r(3)));
                        println!("  defect: whole first leg crosses the open hole interval y=(5/4,7/4)");
                    } else if name == "disconnected" {
                        assert_eq!(segments.len(), 2);
                        println!("  defect: missing top-row intervals x=[1,5/4] and [11/4,3]");
                    }
                }
                Err(error) => {
                    assert_eq!(name, "boundary overlap");
                    println!("{name} {context:?}: kind={:?} error={error}", error.kind());
                }
            }
        }
    }
    for (order, padding) in [(0,r(0)),(11,r(0)),(1,r(-1)),(1,r(2))] {
        assert!(curve::hilbert_strings(&square, order, padding.clone()).is_empty());
        let error = curve::hilbert_strings_with_context(&square, order, padding, &GeometryContext::STRICT).unwrap_err();
        println!("invalid order={order}: primary_empty=true checked_kind={:?}", error.kind());
    }
    for (size, order, padding) in [(r(8),3,r(1)),(r(2),4,Real::try_from(0.12_f64).unwrap())] {
        let outcome = curve::hilbert_strings_with_context(&curve::square(size).unwrap(),order,padding,&GeometryContext::STRICT).unwrap();
        assert_eq!(outcome.value.len(),1);
        assert_eq!(outcome.value[0].len(), (1 << (2*order))-1);
        println!("original fixture order={order}: certainty={:?} segments={}",outcome.certainty,outcome.value[0].len());
    }
}
