use csgrs::{GeometryContext, Real, curve};
use std::error::Error;

fn r(value: i32) -> Real {
    Real::from(value)
}

fn main() {
    let closed = [[r(0), r(0)], [r(3), r(0)], [r(0), r(3)], [r(0), r(0)]];
    let open = [[r(0), r(0)], [r(1), r(0)], [r(1), r(1)], [r(0), r(1)]];
    for (name, controls, display) in [
        ("missing controls", &[][..], 16),
        ("unclosed controls", &open[..], 16),
        ("invalid display", &closed[..], 0),
    ] {
        let unchecked = curve::bezier_region(controls, display);
        let error = curve::bezier_region_with_context(controls, display, &GeometryContext::STRICT)
            .expect_err("malformed construction must fail in the checked API");
        println!("{name}: primary_empty={} checked_kind={:?} checked={error}", unchecked.is_empty(), error.kind());
        assert!(unchecked.is_empty(), "baseline should reproduce empty-on-error");
    }
    println!("coincident line Bezier path absent={}", curve::bezier_path(&[[r(0), r(0)], [r(0), r(0)]], 1).is_none());
    for degree in [0, usize::MAX] {
        let absent = curve::bspline_path(&open, degree, 8).is_none();
        let error = curve::bspline_path_with_context(&open, degree, 8, &GeometryContext::STRICT)
            .expect_err("invalid spline degree must fail");
        println!("spline degree={degree}: primary_absent={absent} checked_kind={:?} source={} checked={error}", error.kind(), error.source().is_some());
    }
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let zero = &sine * &sine + &cosine * &cosine - Real::one();
    let unresolved = [[r(0), r(0)], [r(3), r(0)], [r(0), r(3)], [zero, r(0)]];
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512, GeometryContext::STRICT] {
        match curve::bezier_region_with_context(&unresolved, 16, &context) {
            Ok(outcome) => println!("unresolved closure {context:?}: certainty={:?} empty={}", outcome.certainty, outcome.value.is_empty()),
            Err(error) => println!("unresolved closure {context:?}: kind={:?} source={} error={error}", error.kind(), error.source().is_some()),
        }
    }
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let outcome = curve::bezier_region_with_context(&closed, 16, &context).expect("original valid cubic must remain supported");
        println!("valid cubic {context:?}: certainty={:?} empty={} area={:?}", outcome.certainty, outcome.value.is_empty(), outcome.value.signed_area());
        assert!(!outcome.value.is_empty());
    }
    let degenerate = [[r(0), r(0)], [r(1), r(0)], [r(0), r(0)]];
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        match curve::bezier_region_with_context(&degenerate, 16, &context) {
            Ok(outcome) => println!("retraced quadratic {context:?}: certainty={:?} empty={}", outcome.certainty, outcome.value.is_empty()),
            Err(error) => println!("retraced quadratic {context:?}: kind={:?} error={error}", error.kind()),
        }
    }
    let controls = [[r(0), r(0)], [r(1), r(2)], [r(2), r(1)], [r(3), r(0)], [r(4), r(2)]];
    let expected = [(1, 2, 1, 1), (1, 1, 5, 4), (3, 2, 9, 8), (2, 1, 1, 1)];
    for (count, (xn, xd, yn, yd)) in (2..=5).zip(expected) {
        for display in [1, 16, usize::MAX] {
            for (name, path) in [
                ("Bezier", curve::bezier_path(&controls[..count], display).unwrap()),
                ("B-spline", curve::bspline_path(&controls[..count], count - 1, display).unwrap()),
            ] {
                let point = path.curves()[0].point_at(&(r(1) / r(2)).unwrap().into()).unwrap();
                let coordinates = point.represented_coordinates().unwrap();
                assert_eq!(coordinates.0, &(r(xn) / r(xd)).unwrap());
                assert_eq!(coordinates.1, &(r(yn) / r(yd)).unwrap());
                println!("exact {name} degree={} display={display}: midpoint=({xn}/{xd},{yn}/{yd})", count - 1);
            }
        }
    }
}
