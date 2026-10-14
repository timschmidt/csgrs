"""Guard all prior evidence while using unchanged constructor libraries."""
import hashlib
import importlib.util
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
HARDWARE = ROOT / 'docs/evidence/2026-10-13-construction-hardware-attribution'
WORK = ROOT / 'target/integer-conversion-allocation-v1'
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
spec = importlib.util.spec_from_file_location('_hardware_guard', HARDWARE / 'common_v2.py')
hardware = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hardware)


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


def verify_prior(full=False):
    out = HARDWARE / 'run-v2'
    hardware.verify(json.loads((out / 'protocol.json').read_text()), full=full)
    result = json.loads((out / 'result.json').read_text())
    assert result['complete']
    assert all(sha(out / p) == h for p, h in result['artifact_sha256'].items())
    assert all(sha(r['path']) == r['sha256'] for r in json.loads((out / 'binaries.json').read_text()).values())
    obj = json.loads((out / 'c-object.json').read_text())
    assert sha(obj['path']) == obj['sha256']
    assert all(sha(p) == h for p, h in json.loads((out / 'c-dependencies.json').read_text()).items())
    decision = json.loads((HARDWARE / 'decision-v1.json').read_text())
    for key in ['input_sha256', 'derived_sha256']:
        assert all(sha(HARDWARE / p) == h for p, h in decision[key].items())


def verify(protocol, full=False):
    verify_prior(full=full)
    for key in ['inputs_sha256', 'source_reference_sha256']:
        assert all(sha(p) == h for p, h in protocol[key].items())
    assert all(sha(r['path']) == r['sha256'] for r in protocol['libraries'].values())
