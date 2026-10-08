use hypercurve::*;
fn p(x: i32, y: i32) -> Point2 { Point2::from_values(x,y) }
fn q(n: i32, d: i32) -> Real { (Real::from(n)/Real::from(d)).unwrap() }
    fn retained_boundary_identity_requires_finite_affine_endpoints() {
        // The line has W(t)=1-2t; the conic has W(t)=(1-2t)^2.
        // Their homogeneous numerators are nonzero at t=1/2, so that
        // parameter is a true pole, even when both fragments share it.
        let sources = [
            BezierSubcurve2::Rational(
                RationalBezier2::try_new(vec![p(0, 0), p(2, 0)], vec![Real::one(), -Real::one()])
                    .unwrap(),
            ),
            BezierSubcurve2::RationalQuadratic(
                RationalQuadraticBezier2::try_new(
                    p(0, 0), p(1, 1), p(2, 0),
                    Real::one(), -Real::one(), Real::one(),
                ).unwrap(),
            ),
        ];
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for (family, source) in sources.iter().enumerate() {
                for (range, (start, end, finite)) in [
                    (Real::zero(), q(1, 4), true),
                    (q(3, 4), Real::one(), true),
                    (Real::zero(), q(1, 2), false),
                    (q(1, 2), Real::one(), false),
                ].into_iter().enumerate() {
                    let fragments = [false, true].map(|reversed| {
                        BezierSplitFragment2::RetainedBezier {
                            source_curve: source.clone(),
                            start: BezierParameter2::Exact(start.clone()),
                            end: BezierParameter2::Exact(end.clone()),
                            reversed,
                            start_image: None,
                            end_image: None,
                        }
                    });
                    assert_eq!(
                        CurveRegionBoundaryLoop2::new(fragments.into(), &policy).is_ok(),
                        finite,
                        "shared parameter identity needs affine endpoints: policy={policy:?}, family={family}, range={range}",
                    );
                }
            }
        }
    }


fn main() { retained_boundary_identity_requires_finite_affine_endpoints(); }
