"""Keep all old runtime candidates, observations and failures immutable."""
import gzip
import hashlib
import json
from pathlib import Path
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PARENT = ROOT / 'docs/evidence/2026-10-14-laurent-cache-eligibility'
COVERAGE = ROOT / 'docs/evidence/2026-10-13-construction-cache-coverage'
BORROWED = ROOT / 'docs/evidence/2026-10-13-borrowed-integer-leaves'
WORK = ROOT / 'target/laurent-cache-miss-classification-v1'
PREP = E / 'preparation-v1'
ENV = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
       'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never'}
GROUPS = ['main', 'borrowed', 'eligibility', 'classified']
ORDERS = [GROUPS, list(reversed(GROUPS))]
SUITES = ['rational', 'quadratic', 'integer']
sys.path.insert(0, str(PARENT))
import eligibility_cost_common_v1 as prior
sys.path.pop(0)
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
INPUTS = ['miss_classification_common_v1.py', 'prepare_v1.py', 'measure_v1.py', 'assess_v1.py',
          'scope-v1.json', 'instrumentation-edits-v1.json', 'cache_trace_v1.rs',
          'classification_control_v1.rs', 'trace_header_v1.rs']


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


def compressed(path, value):
    Path(path).write_bytes(gzip.compress((json.dumps(value, sort_keys=True) + '\n').encode(), mtime=0))


def verify_prior(full=False):
    prior.verify_costs(json.loads((PARENT / 'scalar-cost-preparation-v1/protocol.json').read_text()), full=full)
    assessment = json.loads((PARENT / 'scalar-cost-assessment-v1.json').read_text())
    assert all(sha(PARENT / p) == h for p, h in assessment['artifacts_sha256'].items())
    assert assessment['assessor_sha256'] == sha(PARENT / 'assess_scalar_costs_v1.py')
    interpretation = json.loads((PARENT / 'scalar-cost-interpretation-v1.json').read_text())
    assert interpretation['assessment_sha256'] == sha(PARENT / 'scalar-cost-assessment-v1.json')
    assert interpretation['interpreter_sha256'] == sha(PARENT / 'interpret_scalar_costs_v1.py')
    closure = json.loads((PARENT / 'closure-v1.json').read_text())
    for field, path in [('scalar_cost_result_sha256', 'scalar-costs-v1/result.json'),
                        ('scalar_cost_assessment_sha256', 'scalar-cost-assessment-v1.json'),
                        ('scalar_cost_interpretation_sha256', 'scalar-cost-interpretation-v1.json'),
                        ('next_diagnostic_audit_sha256', 'cache-miss-classification-next-v1.json'),
                        ('synchronization_clarification_sha256', 'source-audit-synchronization-clarification-v1.json')]:
        assert closure[field] == sha(PARENT / path)
    libraries = json.loads((PARENT / 'scalar-costs-v1/libraries.json').read_text())
    binaries = json.loads((PARENT / 'scalar-costs-v1/binaries.json').read_text())
    assert all(sha(row['path']) == row['sha256'] for group in libraries.values() for row in group.values())
    assert all(sha(row['path']) == row['sha256'] for row in binaries.values())
    audit = json.loads((PARENT / 'cache-miss-classification-audit-v1.json').read_text())
    assert all(sha(ROOT / p) == h for p, h in audit['reviewed_evidence_sha256'].items())
    assert all(sha(Path(audit['parent_source']) / p) == h for p, h in audit['source_sha256'].items())
    return audit


def verify(prep, full=False):
    verify_prior(full=full)
    assert all(sha(E / p) == h for p, h in prep['inputs_sha256'].items())
    assert all(sha(ROOT / p) == h for p, h in prep['prior_inputs_sha256'].items())
    for suffix, key in [('source-hashes.json.gz', 'source_hashes_sha256'),
                        ('source.patch.gz', 'source_patch_sha256'),
                        ('source-delta.tar.gz', 'source_delta_archive_sha256')]:
        assert sha(PREP / suffix) == prep[key]
    mapping = json.loads(gzip.decompress((PREP / 'source-hashes.json.gz').read_bytes()))
    selected = mapping if full else {p: h for p, h in mapping.items()
                                     if p in prep['main_source_sha256'] or p in prep['delta_sha256']}
    source = Path(prep['source'])
    assert all(sha(source / p) == h for p, h in selected.items())
    if full:
        assert {str(p.relative_to(source)) for p in source.rglob('*') if p.is_file()} == set(mapping)
    for suite, row in prep['probes'].items():
        assert sha(ROOT / row['original_path']) == row['original_sha256']
        assert sha(PREP / (suite + '_probe.rs')) == row['sha256']
        original = (ROOT / row['original_path']).read_bytes()
        suffix = original[original.index(row['marker'].encode()):]
        if suite == 'rational':
            assert suffix.count(b".chain([b'\\n'])") == 1
            suffix = suffix.replace(b".chain([b'\\n'])", b'.chain(*b"\\n")')
        assert (PREP / (suite + '_probe.rs')).read_bytes() == (E / 'trace_header_v1.rs').read_bytes().rstrip(b'\n') + b'\n\n' + suffix
    assert all(sha(row['path']) == row['sha256'] for group in prep['reused_libraries'].values() for row in group.values())
