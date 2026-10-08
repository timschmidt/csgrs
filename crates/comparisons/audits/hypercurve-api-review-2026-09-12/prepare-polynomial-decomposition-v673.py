from pathlib import Path
import hashlib,json
A=Path(__file__).resolve().parent;W=A.parent;candidate=A/'polynomial-decomposition-candidate-v673/hypercurve'
paths=['src/bspline.rs','src/polynomial_spline.rs','src/lib.rs']
base={p:hashlib.sha256((W/'hypercurve'/p).read_bytes()).hexdigest()for p in paths}
texts={p:(W/'hypercurve'/p).read_text()for p in paths}
s=texts['src/polynomial_spline.rs']
a=s.index('/// Exact Bezier decomposition retained by a');b=s.index('/// Borrowed polynomial Bezier span',a);s=s[:a]+s[b:]
a=s.index('impl PolynomialSplineBezierDecomposition2 {');b=s.index("impl<'a> PolynomialSplineBezierSpanView2",a);s=s[:a]+s[b:]
a=s.index('fn source_intervals(');b=s.index('\nfn ',a+1);s=s[:a]+s[b+1:]
s=s.replace('''            let intervals = require_classification(
                source_intervals(&extraction, policy)?,
                CurveOperation2::Construction,
            )?;
''','')
s=s.replace('''decomposition.seed_certified(PolynomialSplineBezierDecomposition2 {
                    extraction,
                    intervals,
                });''','decomposition.seed_certified(extraction);')
a=s.index('            let extraction = match map_classified_curve_result(',s.index('pub(crate) fn bezier_decomposition_with_policy'))
b=s.index('\n        })',a)
s=s[:a]+'''            map_classified_curve_result(
                self.data.retained.extract_bezier_spans(attempt),
                CurveOperation2::BezierDecomposition,
            )'''+s[b:]
s=s.replace('PolynomialSplineBezierDecomposition2','PolynomialBSplineBezierExtraction2')
assert 'source_intervals'not in s and 'extraction,'not in s
texts['src/polynomial_spline.rs']=s
s=texts['src/bspline.rs']
needle='''    spans: Vec<BezierSubcurve2>,
    inserted_knot_count: usize,''';assert s.count(needle)==1
s=s.replace(needle,'''    spans: Vec<BezierSubcurve2>,
    intervals: Vec<(Real, Real)>,
    inserted_knot_count: usize,''')
a=s.index('        let spans = match extract_refined_bezier_spans(');b=s.index('\n    }\n}',a)
s=s[:a]+'        extract_refined_bezier_spans(refined, policy)'+s[b:]
a=s.index('    /// Returns how many knots were inserted',s.index('impl PolynomialBSplineBezierExtraction2'))
s=s[:a]+'''    /// Returns source knot intervals corresponding one-to-one with the spans.
    pub fn intervals(&self) -> &[(Real, Real)] {
        &self.intervals
    }

'''+s[a:]
s=s.replace('native_span_fact_evidence(&self.spans, &self.refined_knots, self.degree, policy)','native_span_fact_evidence(&self.spans, &self.intervals, policy)')
a=s.index('fn native_span_fact_evidence(');b=s.index('\nfn subcurve_certified_bounds',a)
body=s[a:b]
x=body.index('    let mut span_index = 0_usize;');y=body.index('        let bounds = match',x)
body=body[:x]+'''    debug_assert_eq!(spans.len(), intervals.len());
    for (span_index, (span, (start, end))) in spans.iter().zip(intervals).enumerate() {
'''+body[y:]
body=body.replace('''    refined_knots: &[Real],
    degree: usize,''','''    intervals: &[(Real, Real)],''')
body=body.replace('refined_knots[knot_index].clone()','start.clone()').replace('refined_knots[knot_index + 1].clone()','end.clone()')
body=body.replace('        span_index += 1;\n','').replace('''    if span_index != spans.len() {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
''','')
s=s[:a]+body+s[b:]
a=s.index('fn extract_refined_bezier_spans(');b=s.index('\nfn extract_refined_rational_spans(',a)
body=s[a:b].replace('refined: &BSplineWorkingCurve','refined: BSplineWorkingCurve').replace('CurveResult<Classification<Vec<BezierSubcurve2>>>','CurveResult<Classification<PolynomialBSplineBezierExtraction2>>')
body=body.replace('    let mut spans = Vec::new();','    let mut spans = Vec::new();\n    let mut intervals = Vec::new();')
body=body.replace('        spans.push(span);','''        spans.push(span);
        // The same positive knot window owns the geometry and its chart.
        intervals.push((refined.knots[knot_index].clone(), refined.knots[knot_index + 1].clone()));''')
body=body.replace('    Ok(Classification::Decided(spans))','''    Ok(Classification::Decided(PolynomialBSplineBezierExtraction2 {
        degree: refined.degree,
        refined_control_points: refined.control_points,
        refined_knots: refined.knots,
        spans,
        intervals,
        inserted_knot_count: refined.inserted_knot_count,
    }))''')
s=s[:a]+body+s[b:];texts['src/bspline.rs']=s
s=texts['src/lib.rs'];assert s.count('PolynomialSplineBezierDecomposition2, ')==1
texts['src/lib.rs']=s.replace('PolynomialSplineBezierDecomposition2, ','')
assert not candidate.exists()
for path,text in texts.items():
 target=candidate/path;target.parent.mkdir(parents=True,exist_ok=True);target.write_text(text)
(A/'polynomial-decomposition-candidate-v673-base.json').write_text(json.dumps(base,indent=2)+'\n')
(A/'polynomial-decomposition-candidate-v673.md').write_text('''# Polynomial spline extraction/decomposition consolidation (isolated, unrun)\n\nPrepared while V667 NURBS qualification owns frozen production. Candidate has three copied files and must not promote until V667 is reaped/committed and every base hash matches. No production edit or build.\n\nRemove PolynomialSplineBezierDecomposition2. Its only additional state, exact source knot intervals, now lives on PolynomialBSplineBezierExtraction2 and is recorded in the same positive knot-window loop that emits each Bezier span. The extraction helper consumes the working control net and returns the complete extraction directly. The retained polynomial spline cache returns this actual result; forwarding wrapper, repeated source_intervals scan and export are removed. Native span facts read those same intervals rather than scanning refined knots and independently matching a second span counter. Their existing policy-dependent interval/bounds validation remains in place. Native low-degree and general polynomial carriers remain unchanged. No alias or compatibility interface.\n\nMeaningful existing validation should cover all polynomial spline, B-spline and public Curve integrations, library spline policy/domain/periodic guards, repeated/discontinuous knots, unclamped intervals, higher-degree spans, cached clones and retained bounds/monotonicity evidence. Add no implementation-mirroring tests. Lower public authoring-carrier consolidation remains a separate future change because finite endpoint admission and cold extraction benchmarks need explicit preservation.\n\nCandidate is UNFORMATTED/UNBUILT/UNPROMOTED. Next free artifact V674.\n''')
print('Prepared isolated candidate:',paths)
