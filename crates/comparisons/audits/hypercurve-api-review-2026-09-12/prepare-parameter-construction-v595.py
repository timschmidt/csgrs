from pathlib import Path
import hashlib, json, re

A = Path(__file__).resolve().parent
W = A.parent
out = A / 'parameter-construction-candidate-v595'
assert not out.exists()
baseline = json.loads((A / 'common-point-incidence-broad-20260928-v593-sources.json').read_text())
inventory = json.loads((A / 'parameter-construction-uses-v588.json').read_text())['files']
names = set(inventory) | {'hypercurve/src/bezier_parameter.rs', 'hypersolve/src/algebraic.rs'}
texts = {}
for name in names:
    data = (W / name).read_bytes()
    assert hashlib.sha256(data).hexdigest() == baseline[name], name
    texts[name] = data.decode()

def once(text, old, new):
    assert text.count(old) == 1, old[:120]
    return text.replace(old, new)

def call(text, start):
    opening = text.index('(', start)
    depth = 1
    begin = opening + 1
    args = []
    i = begin
    while depth:
        c = text[i]
        if c in '([{': depth += 1
        elif c in ')]}': depth -= 1
        elif c == ',' and depth == 1:
            args.append(text[begin:i].strip())
            begin = i + 1
        i += 1
    if text[begin:i-1].strip(): args.append(text[begin:i-1].strip())
    return i, args

# Remove the two public wrappers. Domain admission remains in the importer.
name = 'hypercurve/src/bezier_parameter.rs'
s = texts[name]
start = s.index('    /// Constructs a represented exact Bezier parameter.\n')
end = s.index('    /// Returns a stored `Real` view', start)
s = s[:start] + s[end:]
start = s.index('    fn from_algebraic_root_representation_with_domain(')
at = s.index('        if !representation.is_valid()', start)
s = s[:at] + '''        let admit_exact = |value: Real, policy: &CurveContext| {
            if unit_domain {
                match in_closed_unit_interval(&value, policy) {
                    Some(true) => {}
                    Some(false) => return Err(CurveError::InvalidBezierParameter),
                    None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
                }
            }
            Ok(Classification::Decided(Self::Exact(value)))
        };
''' + s[at:]
pattern = r'return if unit_domain \{\s*Self::exact\((exact(?:\.clone\(\))?), (policy|&strict)\)\s*\} else \{\s*Ok\(Classification::Decided\(Self::Exact\(exact(?:\.clone\(\))?\)\)\)\s*\};'
s, count = re.subn(pattern, lambda m: f'return admit_exact({m[1]}, {m[2]});', s)
assert count == 3, count
marker = '    #[test]\n    fn unit_import_keeps_its_domain_after_generic_interval_construction() {'
s = once(s, marker, (A / 'represented-parameter-domain-tests-v594.rs').read_text() + '\n' + marker)
texts[name] = s

# Finite conic and finite line operations retain explicit membership checks.
name = 'hypercurve/src/rational_bezier_general.rs'
s = texts[name]
for policy, prefix in [('&strict', 'return '), ('policy', '')]:
    old = f'''{prefix}match BezierParameter2::exact(value, {policy}) {{
        '''
    start = s.index(old)
    end = s.index('\n    }', start) + len('\n    }') if not prefix else s.index('\n        };', start) + len('\n        };')
    block = s[start:end]
    assert 'Err(error) => Err(error)' in block
    indent = '        ' if prefix else '    '
    replacement = f'''{prefix}match in_closed_unit_interval(&value, {policy}) {{
{indent}    Some(true) => Ok(Classification::Decided(Some(BezierParameter2::Exact(value)))),
{indent}    Some(false) => Ok(Classification::Decided(None)),
{indent}    None => Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
{indent}}}''' + (';' if prefix else '')
    s = s[:start] + replacement + s[end:]
texts[name] = s

name = 'hypercurve/src/bezier_offset.rs'
s = texts[name]
start = s.index('                match BezierParameter2::exact(line_parameter, policy) {')
end = s.index('\n                }', start) + len('\n                }')
s = s[:start] + '''                match in_closed_unit_interval(&line_parameter, policy) {
                    Some(true) => Some(BezierParameter2::Exact(line_parameter)),
                    Some(false) => None,
                    None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
                }''' + s[end:]
texts[name] = s

# Fixture helpers no longer hide a fallible unit-only constructor.
for name in ['hypercurve/tests/hypercurve_bezier_arrangement.rs',
             'hypercurve/tests/hypercurve_bezier_region.rs',
             'hypercurve/tests/hypercurve_bezier_split_materialization.rs']:
    s = texts[name]
    start = s.index('fn exact(value: Real) -> BezierParameter2 {')
    end = s.index('\n}\n', start) + len('\n}\n')
    s = s[:start] + s[end:]
    s = re.sub(r'\bexact\(', 'BezierParameter2::Exact(', s)
    texts[name] = s

for name in ['hypercurve/benches/curve_region_boolean_batch.rs',
             'hypercurve/tests/hypercurve_analytic_parallel_region.rs']:
    s = texts[name]
    start = s.index('fn exact_parameter(')
    end = s.index('\n}\n', start) + len('\n}\n')
    s = s[:start] + s[end:]
    while 'exact_parameter(' in s:
        start = s.index('exact_parameter(')
        end, args = call(s, start)
        assert len(args) == 2
        s = s[:start] + f'BezierParameter2::Exact(Real::from({args[0]}))' + s[end:]
    texts[name] = s

name = 'hypercurve/tests/hypercurve_bezier_algebraic_parameter.rs'
s = texts[name]
for variable in ['left', 'overlapping', 'close_rational']:
    start = s.index(f'    let {variable} = BezierParameter2::exact(')
    _, args = call(s, s.index('BezierParameter2::exact(', start))
    middle = s.index(f'    let {variable} = match {variable} {{', start)
    end = s.index('\n    };', middle) + len('\n    };')
    s = s[:start] + f'    let {variable} = BezierParameter2::Exact({args[0]});' + s[end:]
texts[name] = s

for name in ['hypercurve/fuzz/fuzz_targets/bezier_arrangement.rs',
             'hypercurve/fuzz/fuzz_targets/bezier_region.rs',
             'hypercurve/fuzz/fuzz_targets/bezier_split_materialization.rs']:
    s = texts[name]
    at = s.index('BezierParameter2::exact(')
    _, args = call(s, at)
    start = s.rfind('let mut parameters = Vec::new();', 0, at)
    end = s.index('parameters.push(parameter);', at)
    end = s.index('}', end) + 1
    assert start >= 0
    mutability = 'mut ' if name.endswith('bezier_split_materialization.rs') else ''
    s = s[:start] + f'let {mutability}parameters = vec![BezierParameter2::Exact({args[0]})];' + s[end:]
    texts[name] = s

for name, s in list(texts.items()):
    # Every remaining exact constructor is a benchmark's decided(...?)/unwrap.
    while 'BezierParameter2::exact(' in s:
        at = s.index('BezierParameter2::exact(')
        assert s[at-len('decided('):at] == 'decided(', (name, s[at-30:at])
        end, args = call(s, at)
        start = at-len('decided(')
        if s[end:].startswith('?)'): end += 2
        elif s[end:].startswith('.unwrap())'): end += len('.unwrap())')
        else: raise AssertionError((name, s[end:end+40]))
        s = s[:start] + f'BezierParameter2::Exact({args[0]})' + s[end:]
    texts[name] = s.replace('BezierParameter2::algebraic(', 'BezierParameter2::Algebraic(')

name = 'hypersolve/src/algebraic.rs'
texts[name] = once(texts[name], '/// Certified unit isolating interval or exact point interval.',
                  '/// Certified finite isolating interval or exact point interval.')

changes = {}
for name, text in sorted(texts.items()):
    assert not re.search(r'(?:BezierParameter2|Self)::(?:exact|algebraic)\(', text), name
    original = (W/name).read_bytes()
    if text.encode() == original: continue
    path = out/name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    changes[name] = dict(before=hashlib.sha256(original).hexdigest(),
                         candidate=hashlib.sha256(text.encode()).hexdigest())
(A/'parameter-construction-candidate-v595.json').write_text(json.dumps(changes, indent=2)+'\n')
print('Prepared unformatted, uncompiled candidates only:', len(changes), 'files; production unchanged')
