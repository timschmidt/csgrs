from pathlib import Path
import re
p=Path('tests/hypercurve_bspline.rs'); s=p.read_text()
# Evidence can no longer be forged through public constructors. Remove their tests and helper factories.
a=s.index('fn assert_topology_error'); b=s.index('fn assert_point_eq'); s=s[:a]+s[b:]
a=s.index('#[test]\nfn retained_span_weight_evidence_rejects_inconsistent_counts'); b=s.index('#[test]\nfn retained_bspline_span_facts_evidence_native_bounds_and_monotonicity'); s=s[:a]+s[b:]
s=s.replace('    Aabb2, ', '    ').replace('QuadraticBezier2, ', '')
for name in ['RationalBSplineNativeTopologyEvidence2','RationalBezierSpanTopologyEvidence2','RationalBezierSpanTopologyPath2','RationalQuadraticBSplineCurve2','RetainedBSplineSpanFactEvidence2','RetainedBSplineSpanFacts2','RetainedSpanWeightDomainEvidence2','RetainedTopologyStatus']:
 s=re.sub(r'\b'+name+r',\s*','',s,count=1)
s=s.replace('RationalQuadraticBSplineCurve2::try_new(', 'RationalBSplineCurve2::try_new(2,')

def replace_tail(name, start, tail, new_name=None):
 global s
 a=s.index('fn '+name+'('); b=s.find('\n#[test]',a)
 if b<0: b=len(s)
 block=s[a:b]; c=block.index(start)
 block=block[:c]+tail+'\n}\n'
 if new_name: block=block.replace('fn '+name+'(', 'fn '+new_name+'(',1)
 s=s[:a]+block+s[b:]

replace_tail('rational_linear_span_preserves_homogeneous_parameterization', '    let evidence =', '''    let native = extraction.native_subcurves(&policy());
    let BezierSubcurve2::Rational(curve) = &native[0] else {
        panic!("expected the original degree-one rational evaluator");
    };
    assert_eq!(curve.degree(), 1);
    assert_eq!(curve.weights(), &[r(1), r(3)]);
    assert_point_eq(&decided(curve.point_at(&q(1, 2), &policy()).unwrap()), &p(3, 0));''')
replace_tail('singular_rational_linear_elevation_stays_retained', '    let evidence =', '''    let native = extraction.native_subcurves(&policy());
    let BezierSubcurve2::Rational(curve) = &native[0] else { panic!("expected rational span"); };
    assert_eq!(curve.degree(), 1);
    assert_point_eq(curve.start(), &p(0, 0));
    assert_point_eq(curve.end(), &p(4, 0));
    assert!(curve.point_at(&q(1, 2), &policy()).is_err());''', 'rational_linear_span_retains_its_denominator_pole')
s=s.replace('decided(extraction.native_subcurves(&policy()).unwrap())','extraction.native_subcurves(&policy())')
replace_tail('retained_rational_quadratic_spans_promote_to_native_conic_topology', '    let evidence =', '''    let native = extraction.native_subcurves(&policy());
    assert_eq!(native.len(), 1);
    let BezierSubcurve2::RationalQuadratic(curve) = &native[0] else { panic!("expected conic specialization"); };
    assert_point_eq(curve.start(), &p(0, 0));
    assert_point_eq(curve.control(), &p(2, 4));
    assert_point_eq(curve.end(), &p(4, 0));''')
replace_tail('nonuniform_rational_cubic_spans_promote_without_degree_reduction','    let evidence =', '''    let native = extraction.native_subcurves(&policy());
    assert_eq!(native.len(), extraction.spans().len());
    for (span, native) in extraction.spans().iter().zip(&native) {
        let BezierSubcurve2::Rational(curve) = native else { panic!("expected general rational span"); };
        assert_eq!(curve.degree(), 3);
        assert!(std::ptr::eq(curve.homogeneous_controls(), span.curve().homogeneous_controls()));
    }''')
replace_tail('equal_weight_rational_cubic_topology_evidence_names_native_exact_spans','    let evidence =', '''    let native = extraction.native_subcurves(&policy());
    assert_eq!(native.len(), 2);
    assert!(native.iter().all(|span| matches!(span, BezierSubcurve2::Cubic(_))));
    assert_eq!(extraction.spans()[0].knot_interval(), (&r(0), &r(1)));
    assert_eq!(extraction.spans()[1].knot_interval(), (&r(1), &r(2)));''', 'equal_weight_rational_cubic_spans_specialize_to_polynomial_cubics')
s=s.replace('    assert_eq!(span.topology_status(), RetainedTopologyStatus::NativeExact);\n    assert!(span.weight_domain().is_none());\n','')
replace_tail('retained_rational_quadratic_span_facts_include_weight_domain','    let weight_domain =', '''    let span = &facts.span_facts()[0];
    assert_eq!(span.bounds().min(), &p(0, 0));
    assert_eq!(span.bounds().max(), &p(2, 1));
    assert_eq!(span.x_monotonicity(), RetainedSpanAxisMonotonicity::CertifiedMonotone);
    assert_eq!(span.y_monotonicity(), RetainedSpanAxisMonotonicity::HasInteriorExtrema);''','rational_quadratic_span_facts_certify_bounds_and_extrema')
a=s.index('fn retained_rational_quadratic_span_facts_follow_refined_knot_windows'); b=s.index('\n#[test]',a)
block=s[a:b]; c=block.index('    assert!(\n'); block=block[:c]+'}\n'; s=s[:a]+block+s[b:]
s=s.replace('span.topology_status() == RetainedTopologyStatus::NativeExact\n            && span.x_monotonicity()', 'span.x_monotonicity()')
s=s.replace('            && span\n                .weight_domain()\n                .is_some_and(|weights| weights.all_weights_certified_nonzero())','')
s=s.replace('    let topology = decided(extraction.native_topology_evidence(&policy()).unwrap());\n','')
s=s.replace('''    assert_eq!(
        topology.span_evidence()[0].decision_path(),
        RationalBezierSpanTopologyPath2::NativeGeneralRationalSpan
    );''','''    assert_eq!(extraction.spans()[0].curve().degree(), 4);''')
# Access the retained shared evaluator explicitly.
s=s.replace('extraction.refined_control_points().len()', 'extraction.refined_homogeneous_controls().len()')
s=s.replace('    assert_eq!(extraction.refined_weights().len(), 7);\n','')
s=s.replace('extraction.refined_weights(),', 'extraction.refined_homogeneous_controls().iter().map(|control| control.weight().clone()).collect::<Vec<_>>(),')
s=s.replace('match &extraction.spans()[0] {', 'match extraction.spans()[0].native_subcurve(&policy()) {').replace('match &extraction.spans()[1] {', 'match extraction.spans()[1].native_subcurve(&policy()) {')
s=s.replace('let BezierSubcurve2::RationalQuadratic(rational) = rational_span else', 'let BezierSubcurve2::RationalQuadratic(rational) = rational_span.native_subcurve(&policy()) else')
for expr in ['span','rational_span','extraction.spans()[0]','extraction.spans()[1]']:
 for method,new in [('degree()','curve().degree()'),('control_points()','curve().affine_control_points().unwrap()'),('weights()','curve().weights()')]:
  s=re.sub(r'(?<![\w.])'+re.escape(expr+'.'+method),expr+'.'+new,s)
# Native vectors need no uncertainty/report wrappers.
s=s.replace('''fragments.extend(decided(
        decided(upper.extract_bezier_spans(&policy()).unwrap())
            .native_subcurves(&policy())
            .unwrap(),
    ));''','''fragments.extend(decided(upper.extract_bezier_spans(&policy()).unwrap()).native_subcurves(&policy()));''')
s=s.replace('''fragments.extend(decided(
        decided(lower.extract_bezier_spans(&policy()).unwrap())
            .native_subcurves(&policy())
            .unwrap(),
    ));''','''fragments.extend(decided(lower.extract_bezier_spans(&policy()).unwrap()).native_subcurves(&policy()));''')
a=s.index('fn extracted_rational_bspline_spans_feed_conic_region_area'); block=s[a:]; block=block.replace('.spans()\n            .to_vec()', '.native_subcurves(&policy())'); s=s[:a]+block
s=s.replace('fn rational_bspline_rejects_zero_or_uncertain_refined_weights', 'fn affine_authoring_rejects_zero_weights_and_extraction_rejects_infinite_endpoints')
p.write_text(s)

p=Path('tests/hypercurve_nurbs.rs'); s=p.read_text(); s=s.replace('.control_points()', '.homogeneous_controls()'); s=s.replace('view.degree()', 'view.curve().degree()'); s=s.replace('.refined_weights()','.refined_homogeneous_controls()'); s=s.replace('.all(|weight| weight.zero_status()', '.all(|control| control.weight().zero_status()'); p.write_text(s)

for name in ['benches/bspline.rs','benches/api_surface.rs']:
 p=Path(name); s=p.read_text()
 # Only compiler-identified NURBS getter uses; polynomial control nets keep their API.
 log=Path('../hypercurve-api-review-2026-09-12/homogeneous-nurbs-migration-all-targets-1.log').read_text()
 lines=s.splitlines(keepends=True)
 for line in set(int(n) for n in re.findall(re.escape(name)+r':(\d+):\d+',log)):
  if '.control_points()' in lines[line-1]: lines[line-1]=lines[line-1].replace('.control_points()', '.homogeneous_controls()')
 s=''.join(lines).replace('    RationalQuadraticBSplineCurve2, Real,','    Real,')
 s=s.replace('RationalQuadraticBSplineCurve2::try_new(', 'RationalBSplineCurve2::try_new(2,')
 s=s.replace('facts\n                    .span_facts()\n                    .iter()\n                    .filter(|span| span.weight_domain().is_some())\n                    .count()', 'facts.span_facts().len()')
 s=s.replace('.refined_weights()', '.refined_homogeneous_controls()')
 s=s.replace('        let evidence = decided(extraction.native_topology_evidence(&policy)?);\n','')
 s=s.replace('let native = decided(extraction.native_subcurves(&policy)?);','let native = extraction.native_subcurves(&policy);')
 s=s.replace('                + evidence.span_evidence().len()\n                + usize::from(evidence.is_fully_native_exact())\n','')
 s=s.replace('''evidence.span_evidence().len()
                + evidence
                    .span_evidence()
                    .iter()
                    .filter(|span| span.status().is_retained_evidence())
                    .count()''','''extraction.native_subcurves(&policy).len()''')
 s=s.replace('topology_status_checksum','general_native_checksum').replace('rational_cubic_bspline_topology_status','rational_cubic_bspline_general_native')
 p.write_text(s)
