use hypercurve::*;

fn p(x: i32, y: i32) -> Point2 { Point2::from_values(x, y) }
fn q(n: i32, d: i32) -> Real { (Real::from(n) / Real::from(d)).unwrap() }
fn exact<T>(value: Classification<T>) -> T {
    match value {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("{reason:?}"),
    }
}
fn main() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let retraced = Curve2::from(QuadraticBezier2::new(p(0, 0), p(2, 0), p(0, 0)));
        let report = retraced.intersect_curve(&retraced, &policy).unwrap();
        assert_eq!(report.certainty, CurveCertainty::Certified);
        let restricted = report.value.overlaps().iter().filter(|overlap| {
            exact(overlap.restrict(
                [Real::zero().into(), q(1, 4).into()],
                [q(3, 4).into(), Real::one().into()], &policy,
            ).unwrap().into_value()).is_some()
        }).count();
        let first = retraced.subcurve(Real::zero().into(), q(1, 4).into(), &policy).unwrap().into_value();
        let second = retraced.subcurve(q(3, 4).into(), Real::one().into(), &policy).unwrap().into_value();
        let fresh = first.intersect_curve(&second, &policy).unwrap();
        assert!(fresh.value.is_complete());
        assert_eq!(fresh.value.overlaps().len(), 1);
        println!("retraced: complete={} whole_overlaps={} restricted_overlaps={} fresh_overlaps={}",
            report.value.is_complete(), report.value.overlaps().len(), restricted, fresh.value.overlaps().len());

        // x=12(2t-1)^2, y=12((2t-1)^3-(2t-1)/4).
        // t=1/4 and t=3/4 are distinct transverse visits to (3,0).
        let nodal = Curve2::from(CubicBezier2::new(p(12, -9), p(-4, 13), p(-4, -13), p(12, 9)));
        let report = nodal.intersect_curve(&nodal, &policy).unwrap();
        println!("nodal: complete={} contacts={} overlaps={}", report.value.is_complete(),
            report.value.contacts().len(), report.value.overlaps().len());
    }
}
