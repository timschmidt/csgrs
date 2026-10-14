"""Integrity helpers for a hardware diagnostic using unchanged libraries."""
import hashlib
import importlib.util
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OLD = ROOT / 'docs/evidence/2026-10-13-native-without-persistent-certificates'
COVERAGE = ROOT / 'docs/evidence/2026-10-13-construction-cache-coverage'
WORK = ROOT / 'target/construction-hardware-attribution-v1'
ENV = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
       'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never'}
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
spec = importlib.util.spec_from_file_location('_coverage_guard', COVERAGE / 'common_v1.py')
coverage = importlib.util.module_from_spec(spec)
spec.loader.exec_module(coverage)


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


def verify_prior(full=False):
    coverage.verify(json.loads((COVERAGE / 'preparation-v1/protocol.json').read_text()), full=full)
    for directory in [COVERAGE / 'diagnostic-v1', OLD / 'construction-trace-v1']:
        result = json.loads((directory / 'result.json').read_text())
        assert result['complete']
        assert all(sha(directory / p) == digest for p, digest in result['artifact_sha256'].items())
        for name in ['libraries.json', 'binaries.json']:
            def check(row):
                if 'path' in row and 'sha256' in row:
                    assert sha(row['path']) == row['sha256']
                else:
                    for child in row.values():
                        check(child)
            check(json.loads((directory / name).read_text()))


def verify(protocol, full=False):
    verify_prior(full=full)
    assert all(sha(p) == digest for p, digest in protocol['inputs_sha256'].items())
    assert all(sha(p) == digest for p, digest in protocol['system_reference_sha256'].items())
    for group in protocol['libraries'].values():
        for row in group.values():
            assert sha(row['path']) == row['sha256']
