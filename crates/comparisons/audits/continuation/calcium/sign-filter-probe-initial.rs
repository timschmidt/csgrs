use hyperlimit::{Point2, PredicatePolicy, Sign, ring_area_sign};
use hyperreal::{Rational, Real, dispatch_trace};
use serde_json::json;

fn main() {
    for fixture in ["rational", "pi-far", "pi-near"] {
        for n in [3, 4, 8, 16, 32] {
            let left = if fixture == "rational" { Real::from(3) } else { Real::pi() };
            let right = match fixture {
                "rational" => Real::from(2),
                "pi-far" => Real::from(3),
                _ => Real::new(Rational::fraction(103_993, 33_102).unwrap()),
            };
            // Split one straight edge into genuine collinear vertices. Twice
            // the signed area remains left-right, independent of vertex count.
            let mut points = (0..n-1).map(|i| {
                let t = Real::new(Rational::fraction(i as i64, (n-2) as i64).unwrap());
                Point2::new(&left * &t, t)
            }).collect::<Vec<_>>();
            points.push(Point2::new(right, Real::one()));
            for repeat in 0..3 {
                dispatch_trace::reset();
                let result = dispatch_trace::with_recording(|| ring_area_sign(&points, PredicatePolicy::STRICT));
                assert_eq!(result.value(), Some(Sign::Positive));
                let trace = dispatch_trace::take().into_iter().filter(|v| v.layer == "hyperlimit")
                    .map(|v| json!({"operation":v.operation,"path":v.path,"count":v.count})).collect::<Vec<_>>();
                println!("{}", json!({"fixture":fixture,"vertices":n,"repeat":repeat,"outcome":format!("{result:?}"),"trace":trace}));
            }
        }
    }
}
