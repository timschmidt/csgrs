"""Guard the closed parent diagnostic and isolate new memo observations."""
import gzip
import hashlib
import json
from pathlib import Path
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PARENT = ROOT / 'docs/evidence/2026-10-14-laurent-cache-miss-classification'
WORK = ROOT / 'target/local-memo-leaf-coverage-v2'
PREP = E / 'preparation-v2'
ENV = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
       'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never'}
GROUPS = ['borrowed', 'diagnostic']
ORDERS = [GROUPS, list(reversed(GROUPS))]
SUITES = ['rational', 'quadratic', 'integer']
INPUTS = ['leaf_memo_common_v2.py', 'prepare_v2.py', 'measure_v2.py', 'assess_v2.py', 'scope-v2.json',
          'instrumentation-edits-v2.json', 'memo_leaf_trace_v2.rs', 'memo_observer_v2.rs',
          'memo_leaf_tests_v2.rs', 'fallback_control_v2.rs', 'trace_header_v2.rs']
sys.path.insert(0, str(PARENT))
import miss_classification_common_v2 as prior
sys.path.pop(0)
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


def compressed(path, value):
    Path(path).write_bytes(gzip.compress((json.dumps(value, sort_keys=True) + '\n').encode(), mtime=0))


def verify_failed_v1(full=False):
    failure = json.loads((E / 'failure-v1.json').read_text())
    assert all(sha(E / p) == h for p, h in failure['old_artifacts_sha256'].items())
    mapping = json.loads(gzip.decompress((E / failure['source_hashes_path']).read_bytes()))
    prep = json.loads((E / 'preparation-v1/protocol.json').read_text())
    selected = mapping if full else {p: h for p, h in mapping.items()
                                     if p in prep['main_source_sha256'] or p in prep['delta_sha256']}
    source = Path(failure['source'])
    assert all(sha(source / p) == h for p, h in selected.items())
    if full:
        assert {str(p.relative_to(source)) for p in source.rglob('*') if p.is_file()} == set(mapping)
    assert all(sha(r['path']) == r['sha256'] for r in failure['binaries'].values())
    assert all(sha(r['path']) == r['sha256'] for g in failure['libraries'].values() for r in g.values())
    assert not (E / 'diagnostic-v1/result.json').exists()
    assert not (E / 'diagnostic-v1/samples.json').exists()


def verify_parent(full=False):
    verify_failed_v1(full=full)
    parent = json.loads((PARENT / 'preparation-v2/protocol.json').read_text())
    prior.verify(parent, full=full)
    assessment = json.loads((PARENT / 'assessment-v2.json').read_text())
    assert assessment['assessor_sha256'] == sha(PARENT / 'assess_v2.py')
    assert all(sha(PARENT / p) == h for p, h in assessment['artifacts_sha256'].items())
    interpretation = json.loads((PARENT / 'interpretation-v1.json').read_text())
    assert interpretation['assessment_sha256'] == sha(PARENT / 'assessment-v2.json')
    assert interpretation['interpreter_sha256'] == sha(PARENT / 'interpret_v1.py')
    closure = json.loads((PARENT / 'closure-v1.json').read_text())
    for key, name in [('result_sha256', 'diagnostic-v2/result.json'), ('assessment_sha256', 'assessment-v2.json'),
                      ('interpretation_sha256', 'interpretation-v1.json'), ('failed_v1_sha256', 'failure-v1.json')]:
        assert closure[key] == sha(PARENT / name)
    libraries = json.loads((PARENT / 'diagnostic-v2/libraries.json').read_text())
    binaries = json.loads((PARENT / 'diagnostic-v2/binaries.json').read_text())
    assert all(sha(r['path']) == r['sha256'] for g in libraries.values() for r in g.values())
    assert all(sha(r['path']) == r['sha256'] for r in binaries.values())
    audit = json.loads((PARENT / 'normalizer-work-audit-v1.json').read_text())
    assert all(sha(ROOT / p) == h for p, h in audit['reviewed_evidence_sha256'].items())
    assert all(sha(Path(audit['parent_source']) / p) == h for p, h in audit['source_sha256'].items())
    return parent


def verify(prep, full=False):
    verify_parent(full=full)
    assert all(sha(E / p) == h for p, h in prep['inputs_sha256'].items())
    assert all(sha(ROOT / p) == h for p, h in prep['prior_inputs_sha256'].items())
    for name, key in [('source-hashes.json.gz', 'source_hashes_sha256'), ('source.patch.gz', 'source_patch_sha256'),
                      ('source-delta.tar.gz', 'source_delta_archive_sha256')]:
        assert sha(PREP / name) == prep[key]
    mapping = json.loads(gzip.decompress((PREP / 'source-hashes.json.gz').read_bytes()))
    selected = mapping if full else {p: h for p, h in mapping.items()
                                     if p in prep['main_source_sha256'] or p in prep['delta_sha256']}
    source = Path(prep['source'])
    assert all(sha(source / p) == h for p, h in selected.items())
    if full:
        assert {str(p.relative_to(source)) for p in source.rglob('*') if p.is_file()} == set(mapping)
    for suite, row in prep['probes'].items():
        path = ROOT / row['parent_path']
        assert sha(path) == row['parent_sha256']
        raw = path.read_bytes()
        suffix = raw[raw.index(row['marker'].encode()):]
        probe = PREP / (suite + '_probe.rs')
        assert sha(probe) == row['sha256']
        assert probe.read_bytes() == (E / 'trace_header_v2.rs').read_bytes().rstrip(b'\n') + b'\n\n' + suffix
    assert all(sha(r['path']) == r['sha256'] for g in prep['reused_libraries'].values() for r in g.values())
