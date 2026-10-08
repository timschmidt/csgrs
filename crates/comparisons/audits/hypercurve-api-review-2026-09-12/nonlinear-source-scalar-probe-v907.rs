use hypercurve::{BezierAlgebraicEndpointImage2, BezierAlgebraicParameter2, BezierAlgebraicTangentVector2, BezierArrangementFragment2, BezierArrangementGraph2, BezierParameter2, BezierParameterInterval, BezierParameterPolynomial, BezierSplitFragment2, BezierSubcurve2, Classification, CurveContext, HomogeneousControl2, Point2, QuadraticBezier2, RationalBezier2, Real};
fn decided<T>(value: Classification<T>) -> T {
    match value {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("unexpected source blocker {reason:?}"),
    }
}
fn choose(n: i64, k: i64) -> i64 {
    if n < k { return 0; }
    (0..k).fold(1, |a, j| a * (n - j)) / (1..=k).product::<i64>()
}
fn q(n: i64, d: i64) -> Real { (Real::from(n) / Real::from(d)).unwrap() }
fn main() {
    let mut blocked = 0;
    for (policy_index, policy) in [CurveContext::STRICT, CurveContext::APPROXIMATE_512].into_iter().enumerate() {
        // P(t)=4*t^3-pi, A(t)=(P(t),t*P(t)), B(u)=(P(u^2),u^2*P(u^2)+P(u^2)^2).
        // On the positive branch t=u^2, the matched-x difference remains exactly x^2.
        let mut fragments = vec![BezierArrangementFragment2::new(0, 0, BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()), end: BezierParameter2::Exact(Real::one()),
            curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(Point2::from_values(-1,0), Point2::new(q(-1,2),Real::zero()), Point2::from_values(0,0))),
        })];
        for second in [false, true] {
            let power: i64 = if second { 2 } else { 1 };
            let mut coefficients = vec![Real::zero(); (3 * power + 1) as usize];
            coefficients[0] = -Real::pi(); coefficients[(3 * power) as usize] = Real::from(4);
            let parameter = decided(BezierAlgebraicParameter2::try_isolate(
                decided(BezierParameterPolynomial::try_new_power_basis(coefficients, &policy).unwrap()),
                decided(BezierParameterInterval::try_new(Real::zero(),Real::one(),&policy).unwrap()), &policy).unwrap());
            let controls = (0..=12).map(|k| {
                let basis = |order| q(choose(k, order), choose(12, order));
                let x = -Real::pi() + Real::from(4) * basis(3 * power);
                let mut y = -Real::pi() * basis(power) + Real::from(4) * basis(4 * power);
                if second { y = y + Real::pi() * Real::pi() - Real::from(8) * Real::pi() * basis(3 * power) + Real::from(16) * basis(6 * power); }
                HomogeneousControl2::new(x,y,Real::one())
            }).collect();
            let source = BezierSubcurve2::Rational(decided(RationalBezier2::from_homogeneous_controls(controls,&policy).unwrap()));
            let image = decided(BezierAlgebraicEndpointImage2::from_source_curve(&source,&parameter,&policy).unwrap());
            let point = decided(image.point().unwrap());
            let represented_point = point.x().and_then(|c| c.representation()).is_some() && point.y().and_then(|c| c.representation()).is_some();
            let tangent = decided(image.tangent().unwrap());
            let retained = tangent.retained_parameter().is_some();
            let represented_vector = BezierAlgebraicTangentVector2::from_image(tangent).represented_coordinates().is_some();
            println!("policy={policy_index} second={second} point_represented={represented_point} retained_tangent={retained} vector_represented={represented_vector}");
            assert!(represented_point && retained && !represented_vector);
            fragments.push(BezierArrangementFragment2::new(if second {2} else {1},0,BezierSplitFragment2::RetainedBezier {
                reversed:false,start:BezierParameter2::Algebraic(parameter),end:BezierParameter2::Exact(Real::one()),source_curve:source,start_image:Some(image),end_image:None,
            }));
        }
        let graph = BezierArrangementGraph2::new(fragments).unwrap();
        match graph.traverse_retained_with_tangent_order(&policy) {
            Classification::Decided(value) => {
                assert_eq!(value.chains()[0].fragment_indices(),[0,1]); assert_eq!(value.chains()[1].fragment_indices(),[2]);
                println!("policy={policy_index} decided=true");
            }
            Classification::Uncertain(reason) => { blocked += 1; println!("policy={policy_index} blocked={reason:?}"); }
        }
    }
    println!("complete requests=2 blocked={blocked}");
}
