from pathlib import Path
import hashlib,json
A=Path(__file__).resolve().parent;W=A.parent;r=json.loads((A/'rational-image-scalar-view-20260928-v606-terminal.json').read_text());assert r['all_processes_reaped']and r['cases'][-1]['returncode']==101
for n,h in json.loads((A/r['source_manifest']).read_text()).items():assert hashlib.sha256((W/n).read_bytes()).hexdigest()==h,n
p=W/'hypercurve/src/rational_bezier_general.rs';s=p.read_text();old=s
start=s.index('fn real_coefficient_rational_image_parameter(');end=s.index('fn next_rational_image_refinement(',start);part=s[start:end]
part=part.replace('    let mut denominator_sign = None;','    let mut denominator_sign = None;\n    let mut excluded_endpoints = [false; 2];',1)
a=part.index('            let mut containing = image_parameters.iter().filter(');b=part.index('\n        }\n        let next =',a)
part=part[:a]+'''            let inside = [
                matches!(compare_reals(&image_interval.lower, &Real::zero(), &strict), Some(Ordering::Greater | Ordering::Equal)),
                matches!(compare_reals(&image_interval.upper, &Real::one(), &strict), Some(Ordering::Less | Ordering::Equal)),
            ];
            // This root inventory is complete on the unit interval. Once the
            // image enclosure lies there, a single possible owner identifies
            // the image. A scalar witness must not hide another overlapping
            // algebraic root merely because its carrier has a wider interval.
            if inside == [true; 2] {
                let mut possible = image_parameters.iter().filter(|parameter| {
                    image_parameter_may_meet_map_interval(parameter, &image_interval, &strict)
                });
                if let Some(parameter) = possible.next()
                    && possible.next().is_none()
                {
                    return Ok(Classification::Decided(Some(parameter.clone())));
                }
            } else if refinement_steps >= 64 {
                // An exact endpoint can straddle the unit boundary forever.
                // The map enclosure already proves a nonzero denominator;
                // replay N=0 or N-D=0 at the retained source to own that point.
                for endpoint in 0..2 {
                    if inside[endpoint] || excluded_endpoints[endpoint] {
                        continue;
                    }
                    let coefficients = if endpoint == 0 {
                        candidate.numerator.clone()
                    } else {
                        subtract_power_polynomials(&candidate.numerator, &candidate.denominator)
                    };
                    match signed_coefficients_at_parameter(&coefficients, source_parameter, &strict)? {
                        Classification::Decided(RealSign::Zero) => {
                            return Ok(Classification::Decided(Some(BezierParameter2::Exact(Real::from(endpoint as i8)))));
                        }
                        Classification::Decided(RealSign::Positive | RealSign::Negative) => excluded_endpoints[endpoint] = true,
                        Classification::Uncertain(_) => {}
                    }
                }
            }'''+part[b:]
a=part.index('fn image_parameter_matches_map_interval(')
part=part[:a]+'''fn image_parameter_may_meet_map_interval(
    parameter: &BezierParameter2,
    image_interval: &ExactRealInterval,
    policy: &CurveContext,
) -> bool {
    // Both a retained isolator and a learned scalar enclose the same root.
    // Only certified disjointness excludes a candidate; an unavailable
    // comparison must remain possible while the source enclosure refines.
    let Ok(Classification::Decided(interval)) = parameter.known_interval(policy) else {
        return true;
    };
    !matches!(compare_reals(interval.end(), &image_interval.lower, policy), Some(Ordering::Less))
        && !matches!(compare_reals(&image_interval.upper, interval.start(), policy), Some(Ordering::Less))
}

'''
s=s[:start]+part+s[end:];s=s.replace('image_parameter_matches_map_interval(', 'image_parameter_may_meet_map_interval(')
# Use the existing local polynomial subtraction helper's actual name.
assert 'fn subtract_power_polynomials('in s
marker='        let expected = Real::eval_poly(&numerator, &selected_source);\n        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {'
assert s.count(marker)==1
s=s.replace(marker,'''        let expected = Real::eval_poly(&numerator, &selected_source);
        let opposite_source = ((&one + fifth.sqrt().unwrap()) / &two).unwrap();
        let opposite = BezierParameter2::Exact(Real::eval_poly(&numerator, &opposite_source));
        let Ok(Classification::Decided(images)) = candidate.image_parameters.get().unwrap() else {
            unreachable!()
        };
        let mut warmed = 0;
        for image in images {
            match image.same_value(&opposite, &strict).unwrap() {
                Classification::Decided(true) => warmed += 1,
                Classification::Decided(false) => {}
                Classification::Uncertain(_) => panic!("the independent competing image must be decidable"),
            }
        }
        assert_eq!(warmed, 1, "warm the other conjugate before selecting this source image");
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {''',1)
# The original selected-source construction must retain fifth for the opposite root.
s=s.replace('let selected_source = ((&one - fifth.sqrt().unwrap()) / &two).unwrap();','let selected_source = ((&one - fifth.clone().sqrt().unwrap()) / &two).unwrap();',1)
marker='    #[test]\n    fn conic_rational_image_separation_refines_past_the_old_limit() {'
fixture='''    #[test]
    fn rational_image_selection_certifies_unit_endpoints_before_admission() {
        let half = (Real::one() / Real::from(2)).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(polynomial) = BezierParameterPolynomial::try_new_power_basis(
                vec![-half.clone(), Real::zero(), Real::one()], &CurveContext::STRICT,
            ).unwrap() else { panic!("the source polynomial must be certified"); };
            let Classification::Decided(interval) = BezierParameterInterval::try_new(
                half.clone(), Real::one(), &CurveContext::STRICT,
            ).unwrap() else { panic!("the source interval must be certified"); };
            let Classification::Decided(source) = BezierAlgebraicParameter2::try_isolate(
                polynomial.clone(), interval, &CurveContext::STRICT,
            ).unwrap() else { panic!("the positive source root must be isolated"); };
            let source = BezierParameter2::Algebraic(source);
            for value in [-1, 0, 1, 2] {
                // At the selected root of t^2-1/2, the map is exactly value.
                let Classification::Decided(candidate) = conic_parameter_candidate(
                    polynomial.coefficients(),
                    &(vec![Real::from(value) - &half, Real::zero(), Real::one()], vec![Real::one()]),
                    &CurveContext::STRICT,
                ).unwrap() else { panic!("the exact map must be constructible"); };
                assert!(candidate.quotient_matrices.set(None).is_ok());
                let result = real_coefficient_rational_image_parameter(&source, &candidate, &policy).unwrap();
                if (0..=1).contains(&value) {
                    let Classification::Decided(Some(parameter)) = result else {
                        panic!("an exact unit endpoint must be admitted");
                    };
                    assert_eq!(parameter.same_value(&BezierParameter2::Exact(Real::from(value)), &CurveContext::STRICT).unwrap(), Classification::Decided(true));
                } else {
                    assert_eq!(result, Classification::Decided(None));
                }
            }
        }
    }

'''
assert s.count(marker)==1;s=s.replace(marker,fixture+marker);p.write_text(s)
(A/'rational-image-selection-v607-applied.json').write_text(json.dumps(dict(path=str(p),before=hashlib.sha256(old.encode()).hexdigest(),after_unformatted=hashlib.sha256(s.encode()).hexdigest()),indent=2)+'\n')
print('Applied conservative image selection, endpoint replay and opposite-conjugate regression')
