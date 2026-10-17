"""Freeze current production inputs plus missing private-cache acceptance probes."""
import gzip
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]
OUT = W / 'baseline-preparation-v1'
SOURCE = W / 'baseline-source-v1'
PARENT = ROOT / 'docs/evidence/2026-10-17-native-track-cost-attribution'
def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()
def read(p): return json.loads(Path(p).read_text())
def dump(p, v): Path(p).write_text(json.dumps(v, indent=2) + '\n')
assert not OUT.exists() and not SOURCE.exists()
head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
assert head.startswith('da8b64c19')
subprocess.run(['git', 'diff', '--quiet'], cwd=ROOT, check=True)
subprocess.run(['git', 'diff', '--cached', '--quiet'], cwd=ROOT, check=True)
parent = read(PARENT / 'checkpoint-v1.json')
assert all(sha(ROOT / p) == h for p, h in parent['evidence_files_sha256'].items())
assert sha(ROOT / 'docs/properties-status.json') == parent['ledger_after_sha256']
with gzip.open(ROOT / 'docs/evidence/2026-10-17-kicad-native-strokes/reproduction-v1/source-hashes.json.gz', 'rt') as f:
    names = json.load(f)
inputs = {p: sha(ROOT / p) for p in names}
assert len(inputs) == 2835
OUT.mkdir(); SOURCE.mkdir()
for name, digest in inputs.items():
    dest = SOURCE / name
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ROOT / name, dest)
    assert sha(dest) == digest
modules = {'circle_support_cache_tests': 'cache_contracts_v1.rs', 'circle_support_cost_probe': 'cost_probe_v1.rs'}
source_file = SOURCE / 'crates/hypercurve/src/curve_intersection.rs'
for module, probe in modules.items():
    with source_file.open('a') as f:
        f.write(f'\n#[cfg(test)]\n#[path = "{module}.rs"]\nmod {module};\n')
    shutil.copy2(W / probe, SOURCE / 'crates/hypercurve/src' / (module + '.rs'))
formatted = subprocess.run(['cargo', 'fmt', '--all'], cwd=SOURCE, capture_output=True)
(OUT / 'format.stdout').write_bytes(formatted.stdout); (OUT / 'format.stderr').write_bytes(formatted.stderr)
assert formatted.returncode == 0
final_names = list(inputs) + ['crates/hypercurve/src/' + m + '.rs' for m in modules]
final = {p: sha(SOURCE / p) for p in final_names}
changed = {p: h for p, h in final.items() if inputs.get(p) != h}
assert len(changed) == 3
assert source_file.read_bytes().startswith((ROOT / 'crates/hypercurve/src/curve_intersection.rs').read_bytes())
assert all(sha(ROOT / p) == h for p, h in inputs.items())
for name, value in [('main-input-hashes.json.gz', inputs), ('source-hashes.json.gz', final)]:
    with gzip.open(OUT / name, 'wt') as f: json.dump(value, f, sort_keys=True)
dump(OUT / 'result.json', {'complete': True, 'base_commit': head, 'source': str(SOURCE),
    'source_inputs': len(final), 'source_hashes_sha256': sha(OUT / 'source-hashes.json.gz'),
    'main_input_hashes_sha256': sha(OUT / 'main-input-hashes.json.gz'), 'changes': changed,
    'production_implementation_unchanged': True, 'main_sources_modified': False,
    'input_sha256': {str(p): sha(p) for p in [Path(__file__), W / 'contract-v1.md', *[W / p for p in modules.values()]]}})
print(json.dumps({'source_inputs': len(final), 'baseline_production_unchanged': True}), flush=True)
