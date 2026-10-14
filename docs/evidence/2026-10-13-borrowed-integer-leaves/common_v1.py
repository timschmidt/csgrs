"""Keep the borrowed-integer candidate and all earlier experiments immutable."""
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OLD = ROOT / 'docs/evidence/2026-10-13-integer-conversion-allocation'
PARENT = ROOT / 'docs/evidence/2026-10-13-combined-native-admission'
WORK = ROOT / 'target/borrowed-integer-leaves-v1'
PREP = E / 'preparation-v1'
ENV = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
       'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never'}
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
spec = importlib.util.spec_from_file_location('_integer_allocation_guard', OLD / 'common_v1.py')
prior = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prior)


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


def compressed(path, value):
    Path(path).write_bytes(gzip.compress((json.dumps(value, sort_keys=True) + '\n').encode(), mtime=0))


def verify_prior(full=False):
    out = OLD / 'run-v1'
    prior.verify(json.loads((out / 'protocol.json').read_text()), full=full)
    result = json.loads((out / 'result.json').read_text())
    assert result['complete']
    assert all(sha(out / p) == h for p, h in result['artifact_sha256'].items())
    binary = json.loads((out / 'binary.json').read_text())
    assert sha(binary['path']) == binary['sha256']
    assessment = json.loads((OLD / 'assessment-v1.json').read_text())
    assert assessment['status'] == 'signed-word-reconstruction-allocation-mechanism-confirmed'
    assert assessment['result_sha256'] == sha(out / 'result.json')
    assert assessment['comparison_sha256'] == sha(OLD / 'direct-constructor-comparisons-v1.json')


def verify(prep, full=False):
    verify_prior(full=full)
    assert all(sha(E / p) == h for p, h in prep['inputs_sha256'].items())
    assert all(sha(ROOT / p) == h for p, h in prep['prior_inputs_sha256'].items())
    mapping_path = PREP / 'source-hashes.json.gz'
    assert sha(mapping_path) == prep['source_hashes_sha256']
    assert sha(PREP / 'source.patch.gz') == prep['source_patch_sha256']
    assert sha(PREP / 'source-delta.tar.gz') == prep['source_delta_archive_sha256']
    mapping = json.loads(gzip.decompress(mapping_path.read_bytes()))
    selected = mapping if full else {p: h for p, h in mapping.items()
                                     if p in prep['main_source_sha256'] or p in prep['delta_sha256']}
    assert all(sha(Path(prep['source']) / p) == h for p, h in selected.items())
    if full:
        assert {str(p.relative_to(prep['source'])) for p in Path(prep['source']).rglob('*') if p.is_file()} == set(mapping)
