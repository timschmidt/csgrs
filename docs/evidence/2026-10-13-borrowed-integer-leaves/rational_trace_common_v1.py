"""Reuse four frozen scalar libraries for an original-workload count diagnostic."""
import gzip
import importlib.util
import json
from pathlib import Path

from cost_common_v1 import E, ROOT, PARENT, COST_PREP, WORK, ENV, sha, dump, verify_costs

COVERAGE = ROOT / 'docs/evidence/2026-10-13-construction-cache-coverage'
TRACE_WORK = WORK / 'rational-trace-v1'
TRACE_PREP = E / 'rational-trace-preparation-v1'
GROUPS = ['main', 'full-cache', 'candidate', 'cache-coverage']
ORDERS = [GROUPS, list(reversed(GROUPS))]
INPUTS = ['rational_trace_common_v1.py', 'prepare_rational_trace_v1.py',
          'trace_rational_v1.py', 'assess_rational_trace_v1.py',
          'rational_trace_header_v1.rs', 'rational-trace-scope-v1.json']
spec = importlib.util.spec_from_file_location('_rational_trace_coverage_guard', COVERAGE / 'common_v1.py')
coverage = importlib.util.module_from_spec(spec)
spec.loader.exec_module(coverage)


def prior_guard(full=False):
    verify_costs(json.loads((COST_PREP / 'protocol.json').read_text()), full=full)
    coverage.verify(json.loads((COVERAGE / 'preparation-v1/protocol.json').read_text()), full=full)
    for base in [E / 'library-costs-v1', COVERAGE / 'diagnostic-v1']:
        result = json.loads((base / 'result.json').read_text())
        assert result['complete']
        assert all(sha(base / p) == h for p, h in result['artifact_sha256'].items())
        binaries = json.loads((base / 'binaries.json').read_text())
        assert all(sha(r['path']) == r['sha256'] for r in binaries.values())
        libraries = json.loads((base / 'libraries.json').read_text())
        groups = libraries.values() if base.parent == E else [libraries]
        assert all(sha(r['path']) == r['sha256'] for group in groups for r in group.values())


def rows(raw):
    return [json.loads(line) for line in raw.splitlines() if line.startswith(b'{')]


def identity(raw):
    return [r for r in rows(raw) if 'fixture' in r or 'witness' in r]


def verify(prep, full=False):
    prior_guard(full=full)
    assert all(sha(E / p) == h for p, h in prep['inputs_sha256'].items())
    assert all(sha(ROOT / p) == h for p, h in prep['prior_inputs_sha256'].items())
    assert sha(TRACE_PREP / 'probe.rs') == prep['probe_sha256']
    original = (PARENT / 'probes-v1/rational_probe.rs').read_bytes()
    suffix = original[original.index(b'fn fixture('):]
    generated = (TRACE_PREP / 'probe.rs').read_bytes()
    assert generated == (E / 'rational_trace_header_v1.rs').read_bytes().rstrip(b'\n') + b'\n\n' + suffix
    assert identity(gzip.decompress((E / 'library-costs-v1/sample-rational-time-1-main.stdout.gz').read_bytes())) == prep['identity']
    for group in prep['libraries'].values():
        assert all(sha(r['path']) == r['sha256'] for r in group.values())
