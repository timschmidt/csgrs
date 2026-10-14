"""Guard isolated lookup-eligibility sources and the complete prior evidence chain."""
import gzip
import hashlib
import json
from pathlib import Path
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PARENT = ROOT / 'docs/evidence/2026-10-13-borrowed-integer-leaves'
WORK = ROOT / 'target/laurent-cache-eligibility-v1'
PREP = E / 'preparation-v1'
ENV = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
       'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never'}
sys.path.insert(0, str(PARENT))
import rational_trace_common_v2 as prior
sys.path.pop(0)
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
INPUTS = ['eligibility_common_v1.py', 'prepare_v1.py', 'qualify_v1.py', 'scope-v1.json',
          'runtime-edits-v1.json', 'eligibility_tests_v1.rs', 'source-audit-v1.json',
          'source-audit-searches-v1.json']


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


def compressed(path, value):
    Path(path).write_bytes(gzip.compress((json.dumps(value, sort_keys=True) + '\n').encode(), mtime=0))


def verify_prior(full=False):
    prior.verify(json.loads((PARENT / 'rational-trace-preparation-v2/protocol.json').read_text()), full=full)
    assessment = json.loads((PARENT / 'rational-trace-assessment-v3.json').read_text())
    assert assessment['original_runner_exit_code'] == 1 and assessment['new_samples_or_replacement_runs'] == 0
    assert all(sha(PARENT / p) == h for p, h in assessment['artifacts_sha256'].items())
    assert assessment['assessor_sha256'] == sha(PARENT / 'assess_rational_trace_v3.py')
    assert assessment['failure_record_sha256'] == sha(PARENT / 'rational-trace-failure-v2.json')
    assert assessment['post_observation_assessment_scope_sha256'] == sha(PARENT / 'rational-trace-assessment-scope-v3.json')
    assert not (PARENT / 'rational-trace-v1/result.json').exists()
    assert not (PARENT / 'rational-trace-v2/result.json').exists()
    closure = json.loads((PARENT / 'rational-trace-closure-v3.json').read_text())
    for key, filename in [('assessment_sha256', 'rational-trace-assessment-v3.json'),
                          ('interpretation_sha256', 'rational-trace-interpretation-v1.json'),
                          ('next_audit_sha256', 'rational-cache-eligibility-next-v1.json')]:
        assert closure[key] == sha(PARENT / filename)
    for p, row in closure['new_driver_artifacts'].items():
        assert sha(prior.TRACE_WORK / p) == row['sha256']
    base = json.loads((PARENT / 'preparation-v1/protocol.json').read_text())
    audit = json.loads((E / 'source-audit-v1.json').read_text())
    assert audit['parent_preparation_sha256'] == sha(PARENT / 'preparation-v1/protocol.json')
    assert all(sha(Path(base['source']) / p) == h for p, h in audit['source_sha256'].items())
    assert all(sha(ROOT / p) == h for p, h in audit['evidence_sha256'].items())
    return base


def verify(prep, full=False):
    base = verify_prior(full=full)
    assert all(sha(E / p) == h for p, h in prep['inputs_sha256'].items())
    assert all(sha(ROOT / p) == h for p, h in prep['prior_inputs_sha256'].items())
    for group, variant in prep['variants'].items():
        for suffix, key in [('source-hashes.json.gz', 'source_hashes_sha256'),
                            ('source.patch.gz', 'source_patch_sha256'),
                            ('source-delta.tar.gz', 'source_delta_archive_sha256')]:
            assert sha(PREP / (group + '-' + suffix)) == variant[key]
        mapping = json.loads(gzip.decompress((PREP / (group + '-source-hashes.json.gz')).read_bytes()))
        selected = mapping if full else {p: h for p, h in mapping.items()
                                         if p in base['main_source_sha256'] or p in variant['delta_sha256']}
        source = Path(variant['source'])
        assert all(sha(source / p) == h for p, h in selected.items()), group
        if full:
            assert {str(p.relative_to(source)) for p in source.rglob('*') if p.is_file()} == set(mapping)
