"""Preserve the contact-index failure without claiming a full Boolean failure."""
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(name):
    return json.loads((EVIDENCE / name).read_text())


def rows(name):
    return [json.loads(line) for line in (EVIDENCE / name).read_text().splitlines()]


index_source = 'crates/hypercurve/src/curve_region_boolean.rs'
committed = subprocess.check_output(['git', 'show', 'HEAD:' + index_source], cwd=ROOT, text=True)
start = committed.index('#[derive(Default)]\nstruct ContactPointIndex {')
body = committed[start:committed.index('\nfn replace_topology_vertex(', start)]
assert body in (ROOT / index_source).read_text()
body_hash = hashlib.sha256(body.encode()).hexdigest()
contract = read('contact-index-range-baseline/manifest.json')
contract_rows = rows('contact-index-range-baseline/run.jsonl')
assert contract['implementation_sha256'] == body_hash
assert [command['exit_code'] for command in contract['commands']] == [0, 101]
assert len(contract_rows) == 11
assert sum(row['expected_overlap'] != row['retained'] for row in contract_rows) == 7
for row in contract_rows:
    assert 1 in row['candidates']

failed = read('contact-index-producer-baseline/manifest.json')
assert len(failed['commands']) == 1 and failed['commands'][0]['exit_code'] == 1
for name, expected in failed['artifacts'].items():
    assert digest(EVIDENCE / 'contact-index-producer-baseline' / name) == expected

producer = read('contact-index-producer-baseline-corrected/manifest.json')
assert producer['implementation_sha256'] == body_hash
assert [command['exit_code'] for command in producer['commands']] == [0, 101]
assert digest(Path(producer['library']['path'])) == producer['library']['sha256']
producer_rows = rows('contact-index-producer-baseline-corrected/run.jsonl')
assert len(producer_rows) == 2
assert producer_rows[0]['same_point_certified']
assert producer_rows[1]['candidates'] == [1]
assert not producer_rows[1]['same_point_retained']

wide_low = -Fraction(1) - Fraction(1, 2**52)
narrow_low = Fraction(1) - Fraction(1, 2**53)
upper = Fraction(1)
delta = Fraction(1, 2**54)


def polynomial(t):
    return t**5 + t - 2 + delta


assert polynomial(wide_low) < 0
assert polynomial(narrow_low) < 0 < polynomial(upper)
assert list(map(Fraction, producer_rows[0]['wide_box'])) == [wide_low, upper, 0, 0]
assert list(map(Fraction, producer_rows[0]['narrow_box'])) == [narrow_low, upper, 0, 0]
assert Fraction(float(upper - wide_low)) == 2
assert Fraction(float(narrow_low) - float(upper - wide_low)) == -1 > wide_low

artifact_names = [
    'contact-index-range-baseline',
    'contact-index-producer-baseline',
    'contact-index-producer-baseline-corrected',
]
artifacts = {}
for name in artifact_names:
    for path in sorted((EVIDENCE / name).iterdir()):
        if path.is_file():
            artifacts[str(path.relative_to(EVIDENCE))] = digest(path)

record = {
    'decision': 'confirmed exactness defect in the contact candidate index; no repair admitted yet',
    'properties': ['P11', 'P15', 'P17', 'P19', 'P35', 'P36', 'P37'],
    'head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
    'source_file': index_source,
    'implementation_sha256': body_hash,
    'private_contract_cases': {'cases': 11, 'false_omissions': 7, 'passing_controls': 4},
    'producer_case': {
        'polynomial': 'p(t) = t^5 + t - (2 - 2^-54)',
        'independent_witness': 'p is strictly increasing because p\'(t) = 5t^4 + 1 > 0. Exact Fraction arithmetic checks p(1-2^-53) < 0 < p(1); both intervals isolate the same unique real root.',
        'map': 'Public RationalBezier2 line (t, 0), retained Curve2 range from the selected root to 2, start point.',
        'wide_box': ['-1-2^-52', '1', '0', '0'],
        'narrow_box': ['1-2^-53', '1', '0', '0'],
        'equality': 'Production CurvePoint2::coincides_with returns certified true under STRICT.',
        'width_failure': 'The exact width 2+2^-52 rounds to 2. Subtracting that width from 1-2^-53 rounds to -1, above the stored lower endpoint -1-2^-52.',
        'index_result': 'The identical stored point at index 0 is absent; the unboxed control at index 1 remains.',
    },
    'producer_trace': [
        'CurvePoint2::bounds delegates to bounds_with_policy using the active context.',
        'bounds_with_policy calls algebraic_chord_endpoint_bounds_refined with zero refinement steps.',
        'General algebraic retained points may use their source-parameter isolating interval as a conservative box.',
        'BezierParameter2::refined_isolating_interval with zero steps preserves the supplied interval.',
    ],
    'scope': 'The entire committed private index is extracted verbatim. Actual admitted production public constructors, point bounds and certified equality supply its inputs. The private bounds wrapper is represented by the public STRICT wrapper of the same method. No complete public Boolean operation is executed by these probes.',
    'harness_failure_preserved': 'The first actual-producer probe failed compilation (E0277); explicit BezierParameter2::Algebraic conversion fixes the harness. No production source was changed.',
    'production_library': producer['library'],
    'main_runtime_unchanged': True,
    'next_step': 'Evaluate an outward-rounded maximum width, prove its candidate completeness, add production regressions for selected points and valid-box extremes, and measure resource/report effects before admission.',
    'artifacts': artifacts,
}
(EVIDENCE / 'diagnostics-47.json').write_text(json.dumps(record, indent=2) + '\n')
print('Confirmed: 7/11 valid-box failures and one certified-identical production selected-point omission; no full Boolean failure claimed.')
