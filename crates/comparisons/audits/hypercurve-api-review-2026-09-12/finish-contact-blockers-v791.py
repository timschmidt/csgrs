from pathlib import Path
import re,json
A=Path(__file__).resolve().parent;C=A/'contact-blockers-candidate-v791'
def sub(s,a,b):
 assert s.count(a)==1,(a[:100],s.count(a));return s.replace(a,b)
p=C/'hypercurve/src/curve_region_boolean.rs';s=p.read_text();a=s.index('                let point =\n                    exact_contact_point_evidence(&rational_line,');b=s.index('\n                chord_parameter(point)',a)
s=s[:a]+'''                let point = match exact_contact_point_evidence(&rational_line, parameter, &self.data.policy)
                    .map_err(|cause| self.invalid(chord_index, cause))? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
                };'''+s[b:]
old='''            exact_contact_point_evidence(curve, &parameter, policy)?.ok_or_else(|| {
                CurveError::Topology(
                    "a selected-fiber rational boundary could not retain its exact point".into(),
                )
            })''';new='''            match exact_contact_point_evidence(curve, &parameter, policy)? {
                Classification::Decided(point) => Ok(point),
                Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
                    "a selected-fiber rational boundary could not retain its exact point: {reason:?}"
                ))),
            }''';s=sub(s,old,new);p.write_text(s)
# Every test fixture previously required Some from the optional construction
# attempt. Require a certified decision directly; do not add compatibility APIs.
def call_end(s,a):
 depth=1;i=a+1
 while depth:
  if s[i]=='"':
   i+=1
   while s[i]!='"' or s[i-1]=='\\':i+=1
  elif s[i]=='(':depth+=1
  elif s[i]==')':depth-=1
  i+=1
 return i
suffix=re.compile(r'\s*\.(?:unwrap\(\)|expect\("(?:\\.|[^"\\])*"\))')
counts={}
for rel in json.loads((A/'contact-blockers-base-v791.json').read_text()):
 p=C/rel;s=p.read_text();edits=[]
 for m in re.finditer(r'(?:crate::rational_bezier_general::)?exact_contact_point_evidence\(',s):
  if s[max(0,m.start()-20):m.start()].endswith('fn '):continue
  e=call_end(s,m.end()-1);one=suffix.match(s,e)
  if not one:continue
  two=suffix.match(s,one.end())
  if not two:continue
  edits.append((m.start(),two.end(),'crate::tests::decided('+s[m.start():one.end()]+')'))
 for a,b,new in reversed(edits):s=s[:a]+new+s[b:]
 if edits:p.write_text(s)
 counts[rel]=len(edits)
# Regression exercises a real affine pole and adjacent valid points using
# represented and independently isolated parameter carriers.
p=C/'hypercurve/src/rational_bezier_general.rs';s=p.read_text();s=s.replace('pub(crate) fn exact_contact_point_evidence(','''/// Retains the exact affine contact point or its construction blocker.
/// An undefined point must not be confused with absent contact evidence.
pub(crate) fn exact_contact_point_evidence(''',1)
a=s.index('mod tests {')+len('mod tests {');s=s[:a]+'''
    #[test]
    fn rational_contact_evidence_preserves_affine_domain_blockers() {
        use crate::tests::decided;
        let q = |n: i32, d: i32| (Real::from(n) / Real::from(d)).unwrap();
        let curve = RationalBezier2::try_new(
            vec![Point2::from_values(0, 0), Point2::from_values(1, 1), Point2::from_values(2, 0)],
            vec![Real::one(), -Real::one(), Real::one()],
        ).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for value in [q(1, 4), q(1, 2), q(3, 4)] {
                let polynomial = decided(BezierParameterPolynomial::try_new_power_basis(vec![-value.clone(), Real::one()], &policy).unwrap());
                let interval = decided(BezierParameterInterval::try_new(Real::zero(), Real::one(), &policy).unwrap());
                let isolated = decided(BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy).unwrap());
                for parameter in [BezierParameter2::Exact(value.clone()), BezierParameter2::Algebraic(isolated)] {
                    let result = exact_contact_point_evidence(&curve, &parameter, &policy).unwrap();
                    if value == q(1, 2) {
                        assert!(matches!(result, Classification::Uncertain(UncertaintyReason::Boundary)));
                    } else {
                        let expected = Point2::new(if value == q(1, 4) { Real::from(-1) } else { Real::from(3) }, q(-3, 2));
                        let point = decided(result);
                        assert!(matches!(point.coincides_with(&CurvePoint2::from(expected), &policy).value, Classification::Decided(true)));
                    }
                }
            }
        }
    }
''' +s[a:];p.write_text(s)
p=C/'hypercurve/src/bezier_offset.rs';s=p.read_text();a=s.index('mod conversion_tests {')+len('mod conversion_tests {');s=s[:a]+'''
    #[test]
    fn contact_fallback_never_replaces_a_certified_pole() {
        use crate::tests::decided;
        let curve = RationalBezier2::try_new(
            vec![Point2::from_values(0, 0), Point2::from_values(1, 1), Point2::from_values(2, 0)],
            vec![Real::one(), -Real::one(), Real::one()],
        ).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let polynomial = decided(BezierParameterPolynomial::try_new_power_basis(vec![Real::from(-1), Real::from(2)], &policy).unwrap());
            let interval = decided(BezierParameterInterval::try_new(Real::zero(), Real::one(), &policy).unwrap());
            let parameter = decided(BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy).unwrap());
            assert!(matches!(rational_point_evidence_at_parameter(&curve, &BezierParameter2::Algebraic(parameter), &policy), Ok(Classification::Uncertain(UncertaintyReason::Boundary))));
        }
    }
''' +s[a:];p.write_text(s)
print('Migrated finite test fixtures',counts)
