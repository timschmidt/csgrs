from pathlib import Path
import re

W = Path(__file__).resolve().parent.parent
HC = W / 'hypercurve'

def closing(s, start):
    stack = []
    quote = False
    escape = False
    pairs = {')': '(', ']': '[', '}': '{'}
    for i in range(start, len(s)):
        c = s[i]
        if quote:
            if escape:
                escape = False
            elif c == '\\':
                escape = True
            elif c == '"':
                quote = False
            continue
        if c == '"':
            quote = True
        elif c in '([{':
            stack.append(c)
        elif c in ')]}':
            assert stack.pop() == pairs[c]
            if not stack:
                return i
    raise ValueError('unclosed expression')

def arguments(s):
    result = []
    start = 0
    i = 0
    while i < len(s):
        if s[i] in '([{':
            i = closing(s, i)
        elif s[i] == ',':
            if s[start:i].strip():
                result.append(s[start:i].strip())
            start = i + 1
        i += 1
    if s[start:].strip():
        result.append(s[start:].strip())
    return result

def migrate_decided(s, benchmark=False):
    edits = []
    for m in re.finditer(r'\bdecided\(', s):
        end = closing(s, m.end()-1)
        inner = s[m.end():end].strip().removesuffix(',').strip()
        if re.match(r'(PolynomialBSplineCurve2|RationalBSplineCurve2)::try_new\(', inner) or re.match(r'\w+\.extract_bezier_spans\(', inner):
            if benchmark:
                assert inner.endswith('?')
                inner = inner[:-1] + '.expect("benchmark spline operation remains exact")'
            edits.append((m.start(), end+1, inner+'.into_value()'))
    for a,b,v in reversed(edits):
        s = s[:a]+v+s[b:]
    return s

p = HC/'tests/hypercurve_bspline.rs'
s = p.read_text()
assert s.startswith('mod support;\n\n')
s = migrate_decided(s.removeprefix('mod support;\n\n'))
s = s.replace('PolynomialBSplineCurve2','PolynomialSplineCurve2').replace('RationalBSplineCurve2','NurbsCurve2').replace('extract_bezier_spans','bezier_decomposition')
s = s.replace('CurveContext, CurveError,','CurveContext, CurveError, ExactCurveError, UncertaintyReason,')
edits = []
for m in re.finditer(r'assert_eq!\(', s):
    end = closing(s, m.end()-1)
    args = arguments(s[m.end():end])
    assert len(args) == 2, args
    left, right = args
    if right.startswith('Err(CurveError::'):
        cause = right[len('Err('):-1]
        value = f'assert!(matches!({left}, Err(ExactCurveError::Invalid {{ cause: {cause}, .. }})))'
    elif right.startswith('Ok(Classification::Uncertain('):
        assert 'UncertaintyReason::Boundary' in right
        value = f'assert!(matches!({left}, Err(ExactCurveError::Blocked(blocker)) if blocker.reason() == UncertaintyReason::Boundary))'
    elif any(token in left+right for token in ['r(', 'q(', 'p(', 'Real::', 'Point2::', '.weights(', '_weight(', '.control_points(', '.knot_interval(', '.start()', '.end()', '.bounds()', 'signed_area(']):
        value = f'assert!({left} == {right})'
    else:
        continue
    edits.append((m.start(),end+1,value))
for a,b,v in reversed(edits):
    s = s[:a]+v+s[b:]
s = re.sub(r'other => panic!\("expected ([^"\n]+), got \{other:\?\}"\)',r'_ => panic!("expected \1")',s)
p.write_text(s)

p = HC/'benches/bspline.rs'
s = p.read_text()
# Each former raw-extraction source is now a fresh public-curve factory.
for name in ['spline','rational','rational_cubic','equal_weight_rational_cubic']:
    old = 'let '+name+' = decided('
    assert s.count(old) == 1
    s = s.replace(old,'let '+name+' = || decided(')
    old = f'        let extraction = decided({name}.extract_bezier_spans(&policy)?);'
    count = s.count(old)
    assert count == (2 if name == 'rational_cubic' else 1), (name,count)
    s = s.replace(old,f'        let curve = {name}();\n        let extraction = decided(curve.extract_bezier_spans(&policy)?);')
s = migrate_decided(s,benchmark=True)
s = s.replace('    PolynomialBSplineCurve2, PolynomialSplineCurve2, RationalBSplineCurve2, Real,','    PolynomialSplineCurve2, Real,')
s = s.replace('PolynomialBSplineCurve2','PolynomialSplineCurve2').replace('RationalBSplineCurve2','NurbsCurve2').replace('extract_bezier_spans','bezier_decomposition')
labels = {
 'bspline_bezier_extraction': 'polynomial_spline_cold_construction_decomposition_and_facts',
 'rational_quadratic_bspline_bezier_extraction': 'nurbs_quadratic_cold_construction_decomposition_and_facts',
 'rational_cubic_bspline_bezier_extraction': 'nurbs_cubic_cold_construction_decomposition_and_facts',
 'rational_cubic_bspline_native_subcurves': 'nurbs_equal_weight_cubic_cold_construction_decomposition_and_promotion',
 'rational_cubic_bspline_general_native_evidence': 'nurbs_cubic_cold_construction_decomposition_and_promotion',
}
for old,new in labels.items():
    assert s.count('"'+old+':') == 1
    s = s.replace('"'+old+':','"'+new+':')
p.write_text(s)
print('Migrated all 24 integration cases and five cold benchmark workloads')
