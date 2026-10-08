from pathlib import Path
import hashlib,json,subprocess
A=Path(__file__).resolve().parent;W=A.parent
prerequisite='primitive-tangent-reuse-broad-20260928-v571'
r=json.loads((A/f'{prerequisite}-terminal.json').read_text())
assert r['qualification_complete'] and r['all_processes_reaped']
assert (A/f'{prerequisite}-committed.json').exists()
assert not subprocess.check_output(['git','status','--porcelain=v1'],cwd=W/'hypercurve').strip()
manifest=json.loads((A/r['source_manifest']).read_text())
for name,sha in manifest.items():assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
p=W/'hypercurve/src/bezier_parameter.rs';source=p.read_text()
replacements=[('    /// Constructs a closed interval in Bezier parameter space.\n    pub fn try_new(\n        start: Real,\n        end: Real,\n        policy: &CurveContext,\n    ) -> CurveResult<Classification<Self>> {\n        let in_start = in_closed_unit_interval(&start, policy);\n        let in_end = in_closed_unit_interval(&end, policy);\n        match (in_start, in_end) {\n            (Some(false), _) | (_, Some(false)) => return Err(CurveError::InvalidBezierParameter),\n            (Some(true), Some(true)) => {}\n            _ => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),\n        }\n\n        Self::try_new_ordered(start, end, policy)\n    }\n\n    pub(crate) fn try_new_ordered(\n        start: Real,\n        end: Real,\n        policy: &CurveContext,\n    ) -> CurveResult<Classification<Self>> {\n        match compare_reals(&start, &end, policy) {\n            Some(Ordering::Greater) => Err(CurveError::InvalidBezierRange),\n            Some(_) => Ok(Classification::Decided(Self { start, end })),\n            None => Ok(Classification::Uncertain(UncertaintyReason::Ordering)),\n        }\n    }\n\n', '    /// Certifies ordered exact bounds for a closed finite parameter interval.\n    ///\n    /// Curve operations enforce their own source domains; this interval can\n    /// also retain roots on an exterior continuation of an authored segment.\n    pub fn try_new(\n        start: Real,\n        end: Real,\n        policy: &CurveContext,\n    ) -> CurveResult<Classification<Self>> {\n        match compare_reals(&start, &end, policy) {\n            Some(Ordering::Greater) => Err(CurveError::InvalidBezierRange),\n            Some(_) => Ok(Classification::Decided(Self { start, end })),\n            None => Ok(Classification::Uncertain(UncertaintyReason::Ordering)),\n        }\n    }\n\n'), ('        let interval = match if unit_domain {\n            BezierParameterInterval::try_new(\n                representation.interval.lower.clone(),\n                representation.interval.upper.clone(),\n                policy,\n            )\n        } else {\n            BezierParameterInterval::try_new_ordered(\n                representation.interval.lower.clone(),\n                representation.interval.upper.clone(),\n                policy,\n            )\n        }? {\n', '        if unit_domain {\n            let in_start = in_closed_unit_interval(&representation.interval.lower, policy);\n            let in_end = in_closed_unit_interval(&representation.interval.upper, policy);\n            match (in_start, in_end) {\n                (Some(false), _) | (_, Some(false)) => {\n                    return Err(CurveError::InvalidBezierParameter);\n                }\n                (Some(true), Some(true)) => {}\n                _ => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),\n            }\n        }\n        let interval = match BezierParameterInterval::try_new(\n            representation.interval.lower.clone(),\n            representation.interval.upper.clone(),\n            policy,\n        )? {\n'), ('/// Public construction certifies `[0, 1]` membership and `start <= end`.\n/// Internally, exact corner-extension charts may retain an ordered finite\n/// interval outside the segment domain. `BezierAlgebraicParameter2`\n', '/// Construction certifies `start <= end` for arbitrary finite exact bounds.\n/// The owning curve operation separately validates its parameter domain.\n/// `BezierAlgebraicParameter2`\n'), ("/// Oriented positive-length range in a Bezier segment's `[0, 1]` domain.", '/// Oriented positive-length range of exact Bezier source parameters.')]
for old,new in replacements:
 assert source.count(old)==1,old
 source=source.replace(old,new)
source+='\n'+(A/'finite-parameter-interval-import-v570.rs').read_text().split('\n', 1)[1]
p.write_text(source)
for p in (W/'hypercurve/src').rglob('*.rs'):
 source=p.read_text()
 if 'BezierParameterInterval::try_new_ordered' in source:
  p.write_text(source.replace('BezierParameterInterval::try_new_ordered','BezierParameterInterval::try_new'))
p=W/'hypercurve/tests/hypercurve_bezier_algebraic_parameter.rs';source=p.read_text()
old='    let outside = BezierParameterInterval::try_new(r(-1), q(1, 4), &policy())\n        .expect_err("out-of-domain intervals are invalid");\n    assert_eq!(outside, CurveError::InvalidBezierParameter);\n'
assert source.count(old)==1
source=source.replace(old,'').replace('fn invalid_parameter_intervals_are_rejected()', 'fn reversed_parameter_intervals_are_rejected()')
source+='\n'+(A/'finite-parameter-interval-public-v570.rs').read_text().split('\n', 1)[1];p.write_text(source)
for p in (W/'hypercurve/src').rglob('*.rs'):assert 'try_new_ordered' not in p.read_text(),p
print('Applied ordered-interval constructor consolidation and three domain/pole regressions; run formatting and qualification next.')
