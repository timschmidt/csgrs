from pathlib import Path
import json,hashlib
A=Path(__file__).resolve().parent;W=A.parent;C=A/'finite-point-image-candidate-v787';B=A/'finite-point-image-base-v787.json';base=json.loads(B.read_text())
def read(n):
 p=C/n
 if not p.exists():
  d=(W/n).read_bytes();p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(d);base[n]=hashlib.sha256(d).hexdigest();B.write_text(json.dumps(base,indent=2)+'\n')
 return p,p.read_text()
p,s=read('hypercurve/src/lib.rs');s=(W/'hypercurve/src/lib.rs').read_text();s=s.replace('mod tests {','''mod tests {
    pub(crate) fn decided<T>(value: crate::Classification<T>) -> T {
        match value {
            crate::Classification::Decided(value) => value,
            crate::Classification::Uncertain(reason) => panic!("expected exact value: {reason:?}"),
        }
    }
''',1);p.write_text(s)
p,s=read('hypercurve/benches/comparative.rs');a=s.index('    const fn decided');b=s.index('fn hypercurve',a);s=s[:a]+'    const '+s[b:];idx=s.index('impl CommonBooleanOp');s=s[:idx]+'''fn decided<T>(value: Classification<T>) -> T {
    match value {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("expected exact benchmark fixture: {reason:?}"),
    }
}

'''+s[idx:];p.write_text(s)
p,s=read('hypercurve/tests/hypercurve_bezier_algebraic_image.rs');a=s.index('#[test]\nfn rational_quadratic_denominator_boundary_is_reported()');b=s.index('\nproptest!',a);s=s[:a]+'''#[test]
fn rational_point_images_require_finite_affine_coordinates() {
    use hypercurve::{BezierAlgebraicEndpointImage2, CurvePoint2, UncertaintyReason};

    // D(t) = (1-2t)^2, with y numerator -1/2 at the pole.
    // There is no affine point to admit, even though the parameter is exact.
    let conic = RationalQuadraticBezier2::try_new(
        p(0, 0), p(1, 1), p(2, 0), r(1), r(-1), r(1),
    ).unwrap();
    let general = RationalBezier2::from(conic.clone());
    let pole = isolate(polynomial(vec![r(-1), r(2)]), interval(q(2, 5), q(3, 5)));
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for _ in 0..2 {
            for result in [
                conic.point_at_algebraic_parameter(&pole, &policy),
                general.point_at_algebraic_parameter(&pole, &policy),
            ] {
                assert!(matches!(result, Ok(Classification::Uncertain(UncertaintyReason::Boundary))));
            }
            for result in [
                BezierAlgebraicEndpointImage2::rational_quadratic(&conic, &pole, &policy),
                BezierAlgebraicEndpointImage2::rational(&general, &pole, &policy),
            ] {
                assert!(matches!(result, Ok(Classification::Uncertain(UncertaintyReason::Boundary))));
            }
        }
        for (parameter_value, expected) in [
            (q(1, 4), Point2::new(r(-1), q(-3, 2))),
            (q(3, 4), Point2::new(r(3), q(-3, 2))),
        ] {
            let parameter = isolate(polynomial(vec![-parameter_value, r(1)]), interval(r(0), r(1)));
            for image in [
                decided(conic.point_at_algebraic_parameter(&parameter, &policy).unwrap()),
                decided(general.point_at_algebraic_parameter(&parameter, &policy).unwrap()),
            ] {
                let point = CurvePoint2::from(image);
                assert!(matches!(point.coincides_with(&CurvePoint2::from(expected.clone()), &policy).value,
                    Classification::Decided(true)));
            }
        }
        // Tangent construction still exposes its separate diagnostic report.
        assert_eq!(conic.tangent_at_algebraic_parameter(&pole, &policy).unwrap().status(),
            BezierAlgebraicImageStatus::XImageFailed);
    }
}
''' + s[b:];p.write_text(s)
p,s=read('hypercurve/tests/hypercurve_bezier_algebraic_parameter.rs');old='''                assert_eq!(image.status(), BezierAlgebraicImageStatus::XImageFailed);
                assert!(image.x().is_none() && image.y().is_none());
                assert!(image.retained_coordinate_polynomials().is_none());''';assert s.count(old)==1;s=s.replace(old,'''                assert!(matches!(image, Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)));''');p.write_text(s)
p,s=read('hypercurve/tests/hypercurve_curve_point.rs');s=s.replace('fn decided<T: std::fmt::Debug>', 'fn decided<T>');s=s.replace('other => panic!("unexpected classification: {other:?}"),','Classification::Uncertain(reason) => panic!("unexpected classification: {reason:?}"),');p.write_text(s)
print('Finished finite/pole regressions and bounded test utility placement')
