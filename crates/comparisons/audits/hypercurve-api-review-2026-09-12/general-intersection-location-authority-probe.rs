use hypercurve::*;
fn p(x:i32,y:i32)->Point2{Point2::from_values(x,y)}
fn r(x:i32)->Real{x.into()}
fn q(a:i32,b:i32)->Real{(r(a)/r(b)).unwrap()}
fn decided<T>(c:Classification<T>)->T {match c {Classification::Decided(v)=>v, Classification::Uncertain(r)=>panic!("{r:?}")}}
fn selected_intersection_locations_reenter_evaluation_and_subdivision() {
    // x(t) = t^2 meets x = 1/2 at the positive root sqrt(1/2).
    // Spline charts map that same root into the authored interval [2, 5].
    let controls = vec![p(0, 0), p(0, 0), p(1, 0)];
    let root = q(1, 2).sqrt().unwrap();
    let point = Point2::new(q(1, 2), Real::zero());
    let crossing = Curve2::from(
        LineSeg2::try_new(Point2::new(q(1, 2), r(-1)), Point2::new(q(1, 2), r(1))).unwrap(),
    );
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let rational = RationalBezier2::try_new(controls.clone(), vec![Real::one(); 3]).unwrap();
        let knots = vec![r(2), r(2), r(2), r(5), r(5), r(5)];
        let curves = [
            Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 0), p(1, 0))),
            Curve2::from(rational.elevated_to_degree(5).unwrap()),
            Curve2::try_polynomial_bspline(2, controls.clone(), knots.clone(), &policy)
                .unwrap()
                .into_value(),
            Curve2::try_nurbs(2, controls.clone(), vec![Real::one(); 3], knots, &policy)
                .unwrap()
                .into_value(),
        ];
        for (index, original) in curves.into_iter().enumerate() {
            for reversed in [false, true] {
                let curve = if reversed {
                    original.reversed(&policy).unwrap().into_value()
                } else {
                    original.clone()
                };
                let local = if reversed {
                    Real::one() - &root
                } else {
                    root.clone()
                };
                let expected_parameter = if index < 2 {
                    local
                } else {
                    r(2) + r(3) * local
                };
                for swapped in [false, true] {
                    let (first, second) = if swapped {
                        (&crossing, &curve)
                    } else {
                        (&curve, &crossing)
                    };
                    let outcome = first.intersect_curve(second, &policy).unwrap();
                    assert_eq!(outcome.certainty, CurveCertainty::Certified);
                    let result = outcome.value;
                    assert!(result.is_complete(), "{:?}", result.blockers());
                    assert!(result.overlaps().is_empty());
                    assert_eq!(result.contacts().len(), 1);
                    let contact = &result.contacts()[0];
                    let location = if swapped {
                        contact.second()
                    } else {
                        contact.first()
                    };
                    let parameter = decided(location.parameter(&policy).unwrap());
println!("family={index} reversed={reversed} swapped={swapped} local_scalar={} public_scalar={}",location.local_parameter().scalar().is_some(),parameter.scalar().is_some());
                    assert_eq!(
                        parameter
                            .same_value(&expected_parameter.clone().into(), &CurveContext::STRICT)
                            .unwrap(),
                        Classification::Decided(true),
                    );
                    if index < 2 {
                        assert_eq!(&parameter, location.local_parameter());
                    }
                    let evaluated = curve.point_at(&parameter, &policy).unwrap();
                    assert_eq!(evaluated.certainty, CurveCertainty::Certified);
                    assert_eq!(
                        evaluated
                            .value
                            .coincides_with(&point.clone().into(), &CurveContext::STRICT)
                            .value,
                        Classification::Decided(true),
                    );
                    let split = curve.split_at(parameter, &policy).unwrap();
                    assert_eq!(split.certainty, CurveCertainty::Certified);
                    for endpoint in [split.value.0.end(), split.value.1.start()] {
                        assert_eq!(
                            endpoint
                                .coincides_with(&point.clone().into(), &CurveContext::STRICT)
                                .value,
                            Classification::Decided(true),
                        );
                    }
                }
            }
        }
    }
}

fn main(){ selected_intersection_locations_reenter_evaluation_and_subdivision(); }
