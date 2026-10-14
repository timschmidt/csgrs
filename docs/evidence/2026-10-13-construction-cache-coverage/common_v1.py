"""Guards for diagnostic instrumentation of the rejected full-cache build."""
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OLD = ROOT / 'docs/evidence/2026-10-13-native-without-persistent-certificates'
PARENT = ROOT / 'docs/evidence/2026-10-13-combined-native-admission'
WORK = ROOT / 'target/construction-cache-coverage-v1'
PREP = E / 'preparation-v1'
ENV = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
       'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never'}
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
spec = importlib.util.spec_from_file_location('_cache_free_guard', OLD / 'common_v1.py')
prior = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prior)


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


def compressed(path, value):
    Path(path).write_bytes(gzip.compress((json.dumps(value, sort_keys=True) + '\n').encode(), mtime=0))


def verify(prep, full=False):
    prior.verify(json.loads((OLD / 'preparation-v1/protocol.json').read_text()), full=full)
    assert all(sha(E / p) == h for p, h in prep['inputs_sha256'].items())
    assert all(sha(ROOT / p) == h for p, h in prep['prior_inputs_sha256'].items())
    assert sha(PREP / 'source-hashes.json.gz') == prep['source_hashes_sha256']
    mapping = json.loads(gzip.decompress((PREP / 'source-hashes.json.gz').read_bytes()))
    selected = mapping if full else {p: h for p, h in mapping.items()
                                     if p in prep['main_source_sha256'] or p in prep['delta_sha256']}
    assert all(sha(Path(prep['source']) / p) == h for p, h in selected.items())
    parent_mapping = json.loads(gzip.decompress((PARENT / 'preparation-v1/candidate-source-hashes.json.gz').read_bytes()))
    selected_parent = parent_mapping if full else {p: h for p, h in parent_mapping.items()
                                                  if p in prep['main_source_sha256'] or p in prep['delta_sha256']}
    assert all(sha(Path(prep['parent_source']) / p) == h for p, h in selected_parent.items())
    for row in prep['prior_libraries'].values():
        assert sha(row['path']) == row['sha256']
    for row in prep['prior_binaries'].values():
        assert sha(row['path']) == row['sha256']
    if full:
        assert {str(p.relative_to(prep['source'])) for p in Path(prep['source']).rglob('*') if p.is_file()} == set(mapping)
        assert {str(p.relative_to(prep['parent_source'])) for p in Path(prep['parent_source']).rglob('*') if p.is_file()} == set(parent_mapping)
