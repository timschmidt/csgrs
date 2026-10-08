from pathlib import Path
exec(Path(__file__).with_name('edit-finite-point-image-callers-v787.py').read_text().split("edit('hypercurve/src/rational_bezier_general.rs',general)")[0].split('def general(s):')[0])
import re

def mask(s):
 # Mask literal contents so punctuation in assertion messages is not syntax.
 return re.sub(r'"(?:\\.|[^"\\])*"',lambda m:'"'+' '*(len(m[0])-2)+'"',s)
def start_expr(s,end):
 m=mask(s);depth=0;i=end-1
 while i>=0:
  c=m[i]
  if c in ')]}':depth+=1
  elif c in '([{':
   if depth==0:break
   depth-=1
  elif depth==0 and c in '=;,|':break
  i-=1
 i+=1
 while s[i].isspace():i+=1
 return i

def wrap_calls(s,pattern,helper,after=0,choose=lambda s,a,b:True):
 edits=[]
 for m in re.finditer(pattern,s):
  if m.start()<after:continue
  start=start_expr(s,m.start());end=m.end()
  if not choose(s,start,end):continue
  wrap=helper(s,start,end) if callable(helper) else helper
  edits.append((start,end,f'{wrap}({s[start:end]})'))
 for a,b,new in reversed(edits):s=s[:a]+new+s[b:]
 return s
suffix=r'\([^;\n]*?\)\s*\.(?:unwrap\(\)|expect\("[^"\n]*"\))'
point=r'\.point_at_algebraic_parameter'+suffix

def measures(s):
 s=sub(s,'image.and_then(|image| algebraic_endpoint_interval(image.point()))','''image.and_then(|image| match image.point().ok()? {
                Classification::Decided(point) => algebraic_endpoint_interval(point),
                Classification::Uncertain(_) => None,
            })''')
 for side in ['start','end']:
  old=f'                let Some({side}) = algebraic_endpoint_interval({side}_image.point()) else {{'
  new=f'''                let {side}_point = match {side}_image.point() {{
                    Ok(Classification::Decided(point)) => point,
                    Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
                    Err(_) => return Classification::Uncertain(UncertaintyReason::Boundary),
                }};
                let Some({side}) = algebraic_endpoint_interval({side}_point) else {{''';s=sub(s,old,new)
 return s
# completed measure edits
def regionfix(s):
 s=sub(s,'''            let point = match image.point()? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };''','''            let point = match image.point() {
                Ok(Classification::Decided(point)) => point,
                Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
                Err(_) => return Classification::Uncertain(UncertaintyReason::Boundary),
            };''')
 return wrap_calls(s,point,'crate::tests::decided',after=s.index('mod tests {') if 'mod tests {' in s else s.index('.point_at_algebraic_parameter(',s.index('fn exterior_chord_contact_deflation_preserves_other_contacts')))
# completed region edits
for f in ['bezier_offset','curve_region_boolean','curve_evaluation','curve_region_trim','curve','rational_bezier_general']:
 edit(f'hypercurve/src/{f}.rs',lambda s:wrap_calls(s,point,'crate::tests::decided',after=s.index('mod tests {') if 'mod tests {' in s else s.index('.point_at_algebraic_parameter(',s.index('fn exterior_chord_contact_deflation_preserves_other_contacts'))))
# Test-only ? and .ok()? expressions use the same exact-value expectation,
# except exploratory fixtures which already return Option on blockers.
def offsetfixtures(s):
 s=wrap_calls(s,r'\.point_at_algebraic_parameter\([^;\n]*?\)\?', 'crate::tests::decided',after=s.index('mod tests {') if 'mod tests {' in s else s.index('.point_at_algebraic_parameter(',s.index('fn exterior_chord_contact_deflation_preserves_other_contacts')))
 for m in list(re.finditer(r'\.point_at_algebraic_parameter\([^;\n]*?\)\s*\.ok\(\)\?',s))[::-1]:
  a=start_expr(s,m.start());b=m.end();s=s[:a]+'decided('+s[a:b]+')?'+s[b:]
 return s
edit('hypercurve/src/bezier_offset.rs',offsetfixtures)
def util(s):
 a=s.index('mod tests {') if 'mod tests {' in s else s.index('.point_at_algebraic_parameter(',s.index('fn exterior_chord_contact_deflation_preserves_other_contacts'))+len('mod tests {')
 return s[:a]+'''
    pub(crate) fn decided<T>(value: crate::Classification<T>) -> T {
        match value {
            crate::Classification::Decided(value) => value,
            crate::Classification::Uncertain(reason) => panic!("expected exact value: {reason:?}"),
        }
    }
'''+s[a:]
edit('hypercurve/src/lib.rs',util)
for f in ['hypercurve_curve_region_promotion','hypercurve_curve_point','hypercurve_rational_bezier']:
 edit(f'hypercurve/tests/{f}.rs',lambda s:wrap_calls(s,point,'decided'))
# Integration finite rational image calls only; the polynomial factories are unchanged.
def images(s):
 return wrap_calls(s,point,'decided',choose=lambda s,a,b:s[a:b].lstrip().startswith('conic') and 'fn rational_quadratic_denominator_boundary_is_reported()' not in s[s.rfind('#[test]',0,a):a])
edit('hypercurve/tests/hypercurve_bezier_algebraic_image.rs',images)
# Rational eager endpoint callers and shared source endpoint tests.
endpoint=r'BezierAlgebraicEndpointImage2::(?:rational_quadratic|rational|from_source_curve)\([\s\S]*?\)\s*\.unwrap\(\)'
def wrap_endpoint(s,helper,after=0):
 edits=[]
 for m in re.finditer(endpoint,s):
  if m.start()<after:continue
  edits.append((m.start(),m.end(),f'{helper}({m[0]})'))
 for a,b,new in reversed(edits):s=s[:a]+new+s[b:]
 return s
for f in ['hypercurve_bezier_arrangement','hypercurve_bezier_tangent_order']:
 edit(f'hypercurve/tests/{f}.rs',lambda s:wrap_endpoint(s,'decided'))
def split_tests(s):
 s=wrap_endpoint(s,'decided')
 s=s.replace('match image.point() {','match decided(image.point().unwrap()) {')
 pos=s.index('fn ')
 s=s[:pos]+'''fn decided<T>(value: Classification<T>) -> T {
    match value {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("expected exact value: {reason:?}"),
    }
}

'''+s[pos:];return s
edit('hypercurve/tests/hypercurve_bezier_split_materialization.rs',split_tests)
edit('hypercurve/src/bezier_region.rs',lambda s:wrap_endpoint(s,'crate::tests::decided',s.index('mod tests {') if 'mod tests {' in s else s.index('.point_at_algebraic_parameter(',s.index('fn exterior_chord_contact_deflation_preserves_other_contacts'))))
# Fuzz retains pole coverage without ever constructing a point from a failure.
def fuzz(s):
 s=sub(s,'''            assert_eq!(
                rational_point.status(),
                BezierAlgebraicImageStatus::XImageFailed
            );''','''            assert!(matches!(rational_point, Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)));''')
 s=sub(s,'''        } else if mode == 0 {
            assert_eq!(
                rational_point.status(),''','''        } else if mode == 0 {
            let Classification::Decided(rational_point) = rational_point else {
                panic!("finite rational image must be certified");
            };
            assert_eq!(
                rational_point.status(),''')
 return s
edit('hypercurve/fuzz/fuzz_targets/bezier_algebraic_image.rs',fuzz)
# Benchmark fixtures are exact, and may assert their known construction conditions.
def comparative(s):
 s=wrap_calls(s,point,'decided')
 pos=s.index('fn ')
 s=s[:pos]+'''fn decided<T>(value: Classification<T>) -> T {
    match value {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("expected exact benchmark fixture: {reason:?}"),
    }
}

'''+s[pos:];return s
edit('hypercurve/benches/comparative.rs',comparative)
def editing(s):
 s=wrap_calls(s,point,'|image| expect_decided(image, "exact benchmark endpoint")') if False else s
 # Map the classified successful image before the existing Result callers.
 s=s.replace('.map(CurvePoint2::from)', '.map(|image| CurvePoint2::from(expect_decided(image, "exact benchmark endpoint")))')
 for pattern in [point,r'\.point_at_algebraic_parameter\([^;\n]*?\)\?']:
  edits=[]
  for m in re.finditer(pattern,s):
   a=start_expr(s,m.start());b=m.end();edits.append((a,b,'expect_decided('+s[a:b]+', "exact benchmark endpoint")'))
  for a,b,new in reversed(edits):s=s[:a]+new+s[b:]
 return s
edit('hypercurve/benches/editing.rs',editing)
def algebraic_bench(s):
 return sub(s,'let point = conic.point_at_algebraic_parameter(&midpoint, &policy)?;', 'let point = decided(conic.point_at_algebraic_parameter(&midpoint, &policy)?);')
edit('hypercurve/benches/bezier_algebraic_parameter.rs',algebraic_bench)
print('Migrated finite fixtures, endpoint query consumers, benchmarks and fuzz point admission')
