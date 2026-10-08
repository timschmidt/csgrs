from pathlib import Path
import json,hashlib
A=Path(__file__).resolve().parent;W=A.parent
r=json.loads((A/'conic-separation-stack-20260928-v602-terminal.json').read_text());assert r['all_processes_reaped'] and r['returncode']==124
manifest=json.loads((A/'parameter-construction-20260928-v598-sources.json').read_text())
for n,sha in manifest.items():assert hashlib.sha256((W/n).read_bytes()).hexdigest()==sha,n
p=W/'hypercurve/src/rational_bezier_general.rs';s=p.read_text();old=s
start=s.index('fn image_parameter_contains_map_interval(');end=s.index('fn next_rational_image_refinement(',start)
s=s[:start]+'''fn image_parameter_matches_map_interval(
    parameter: &BezierParameter2,
    image_interval: &ExactRealInterval,
    policy: &CurveContext,
) -> bool {
    // A selected root can learn an exact scalar without changing carriers.
    // Its point enclosure cannot contain a nonzero map interval; instead
    // that interval must contain the known image value.
    if let Some(value) = parameter.scalar() {
        return matches!(
            compare_reals(&image_interval.lower, value, policy),
            Some(Ordering::Less | Ordering::Equal)
        ) && matches!(
            compare_reals(value, &image_interval.upper, policy),
            Some(Ordering::Less | Ordering::Equal)
        );
    }
    let Ok(Classification::Decided(interval)) = parameter.known_interval(policy) else {
        return false;
    };
    matches!(
        compare_reals(interval.start(), &image_interval.lower, policy),
        Some(Ordering::Less | Ordering::Equal)
    ) && matches!(
        compare_reals(&image_interval.upper, interval.end(), policy),
        Some(Ordering::Less | Ordering::Equal)
    )
}

'''+s[end:]
s=s.replace('image_parameter_contains_map_interval(', 'image_parameter_matches_map_interval(')
marker='    #[test]\n    fn conic_rational_image_separation_refines_past_the_old_limit() {';assert s.count(marker)==1
fixture='''    #[test]
    fn rational_image_selection_reuses_learned_exact_scalar_views() {
        let half = (Real::one() / Real::from(2)).unwrap();
        let quarter = (Real::one() / Real::from(4)).unwrap();
        let margin = (Real::one() / Real::from(16)).unwrap();
        for value in [half.sqrt().unwrap(), Real::pi() * &quarter] {
            for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
                let Classification::Decided(polynomial) = BezierParameterPolynomial::try_new_power_basis(
                    vec![-(&value * &value), Real::zero(), Real::one()],
                    &CurveContext::STRICT,
                ).unwrap() else { panic!("the exact image polynomial must be certified"); };
                let Classification::Decided(interval) = BezierParameterInterval::try_new(
                    half.clone(), Real::one(), &CurveContext::STRICT,
                ).unwrap() else { panic!("the positive image root must be bracketed"); };
                let Classification::Decided(root) = BezierAlgebraicParameter2::try_isolate(
                    polynomial, interval, &CurveContext::STRICT,
                ).unwrap() else { panic!("the positive image root must be isolated"); };
                let parameter = BezierParameter2::Algebraic(root);
                let retained = parameter.clone();
                assert!(parameter.scalar().is_none());
                let image_interval = ExactRealInterval {
                    lower: &value - &margin,
                    upper: &value + &margin,
                };
                assert!(image_parameter_matches_map_interval(&parameter, &image_interval, &policy));
                let exact = BezierParameter2::Exact(value.clone());
                assert_eq!(parameter.same_value(&exact, &CurveContext::STRICT).unwrap(), Classification::Decided(true));
                assert!(matches!(retained, BezierParameter2::Algebraic(_)));
                assert!(retained.scalar().is_some());
                let foreign = ExactRealInterval { lower: Real::zero(), upper: quarter.clone() };
                for parameter in [&exact, &retained] {
                    assert!(image_parameter_matches_map_interval(parameter, &image_interval, &policy));
                    assert!(!image_parameter_matches_map_interval(parameter, &foreign, &policy));
                }
            }
        }
    }

'''
s=s.replace(marker,fixture+marker);assert s!=old;p.write_text(s)
(A/'rational-image-scalar-view-v603-applied.json').write_text(json.dumps(dict(path=str(p),before=hashlib.sha256(old.encode()).hexdigest(),after_unformatted=hashlib.sha256(s.encode()).hexdigest(),diagnostic='conic-separation-stack-20260928-v602-backtrace.txt'),indent=2)+'\n')
print('Applied scalar-view selection fix and independent retained-carrier regression')
