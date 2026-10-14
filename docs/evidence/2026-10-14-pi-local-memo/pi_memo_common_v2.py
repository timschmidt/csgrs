"""Guard the Pi-only runtime prototype and all closed parent evidence."""
import gzip
import hashlib
import json
from pathlib import Path
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
DIAGNOSTIC = ROOT / 'docs/evidence/2026-10-14-local-memo-leaf-coverage'
BASE = ROOT / 'docs/evidence/2026-10-14-laurent-cache-eligibility'
PARENT = ROOT / 'docs/evidence/2026-10-13-borrowed-integer-leaves'
WORK = ROOT / 'target/pi-local-memo-v2'
PREP = E / 'preparation-v2'
ENV = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
       'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never'}
INPUTS = ['pi_memo_common_v2.py', 'prepare_v2.py', 'qualify_v2.py', 'assess_qualification_v2.py',
          'scope-v2.json', 'runtime-edits-v2.json', 'pi_memo_tests_v2.rs']
NEW_TESTS = {'computable::node::pi_memo_tests::' + name for name in [
    'pi_replay_preserves_budgets_and_sharing_in_both_passes',
    'other_leaves_and_pi_valued_composites_keep_polynomial_replay',
    'opaque_replay_keeps_owners_and_exhausted_expansion_fallback']}
LAYOUT_FIELDS = ['rational', 'coefficient', 'polynomial', 'old_memo_value', 'candidate_memo_value',
                 'old_keyed_entry', 'candidate_keyed_entry', 'closed_coefficient', 'closed_polynomial',
                 'computable', 'node']
sys.path.insert(0, str(DIAGNOSTIC))
import leaf_memo_common_v2 as prior
sys.path.pop(0)
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


def compressed(path, value):
    Path(path).write_bytes(gzip.compress((json.dumps(value, sort_keys=True) + '\n').encode(), mtime=0))


def verify_parent(full=False):
    failure = json.loads((E / 'preparation-failure-v1.json').read_text())
    assert all(sha(E / p) == h for p, h in failure['old_inputs_sha256'].items())
    assert not (ROOT / 'target/pi-local-memo-v1').exists()
    assert not (E / 'preparation-v1').exists()
    assert not (E / 'qualification-v1').exists()
    previous = json.loads((DIAGNOSTIC / 'preparation-v2/protocol.json').read_text())
    prior.verify(previous, full=full)
    audit = json.loads((DIAGNOSTIC / 'pi-memo-source-audit-v1.json').read_text())
    assert audit['audit_script_sha256'] == sha(DIAGNOSTIC / 'audit_pi_memo_v1.py')
    assert all(sha(ROOT / p) == h for p, h in audit['evidence_sha256'].items())
    assert all(sha(Path(audit['parent_source']) / p) == h for p, h in audit['source_sha256'].items())
    closure = json.loads((DIAGNOSTIC / 'closure-v1.json').read_text())
    for key, name in [('result_sha256', 'diagnostic-v2/result.json'), ('assessment_sha256', 'assessment-v2.json'),
                      ('interpretation_sha256', 'interpretation-v1.json'), ('failed_v1_sha256', 'failure-v1.json')]:
        assert closure[key] == sha(DIAGNOSTIC / name)
    assessment = json.loads((DIAGNOSTIC / 'assessment-v2.json').read_text())
    assert assessment['assessor_sha256'] == sha(DIAGNOSTIC / 'assess_v2.py')
    assert all(sha(DIAGNOSTIC / p) == h for p, h in assessment['artifacts_sha256'].items())
    interpretation = json.loads((DIAGNOSTIC / 'interpretation-v1.json').read_text())
    assert interpretation['interpreter_sha256'] == sha(DIAGNOSTIC / 'interpret_v1.py')
    binaries = json.loads((DIAGNOSTIC / 'diagnostic-v2/binaries.json').read_text())
    libraries = json.loads((DIAGNOSTIC / 'diagnostic-v2/libraries.json').read_text())
    assert all(sha(r['path']) == r['sha256'] for r in binaries.values())
    assert all(sha(r['path']) == r['sha256'] for g in libraries.values() for r in g.values())
    return json.loads((BASE / 'preparation-v1/protocol.json').read_text())


def verify(prep, full=False):
    verify_parent(full=full)
    assert all(sha(E / p) == h for p, h in prep['inputs_sha256'].items())
    assert all(sha(ROOT / p) == h for p, h in prep['prior_inputs_sha256'].items())
    variant = prep['variants']['candidate']
    for suffix, key in [('source-hashes.json.gz', 'source_hashes_sha256'),
                        ('source.patch.gz', 'source_patch_sha256'),
                        ('source-delta.tar.gz', 'source_delta_archive_sha256')]:
        assert sha(PREP / suffix) == variant[key]
    mapping = json.loads(gzip.decompress((PREP / 'source-hashes.json.gz').read_bytes()))
    selected = mapping if full else {p: h for p, h in mapping.items()
                                     if p in prep['main_source_sha256'] or p in variant['delta_sha256']}
    source = Path(variant['source'])
    assert all(sha(source / p) == h for p, h in selected.items())
    if full:
        assert {str(p.relative_to(source)) for p in source.rglob('*') if p.is_file()} == set(mapping)
    inventory = prep['parent_default_inventory']
    assert sha(inventory['binary']['path']) == inventory['binary']['sha256']
    for stream in ['stdout', 'stderr']:
        assert sha(PREP / ('parent-default-list.' + stream + '.gz')) == inventory[stream + '_sha256']
