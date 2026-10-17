"""Reuse the successful native artifacts without reopening their Cargo target."""
import gzip
import hashlib
import json
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]
OLD = W.parent / 'circle-support-index'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, v):
    Path(p).write_text(json.dumps(v, indent=2) + '\n')

checkpoint = ROOT / 'docs/evidence/2026-10-17-circle-support-index/checkpoint-v1.json'
prior = read(checkpoint)
assert all(sha(ROOT / p) == h for p, h in prior['evidence_files_sha256'].items())
q = OLD / 'candidate-qualification-v2'
assert read(q / 'result.json')['cargo_target_closed']
commands = read(q / 'commands.json')
build = next(r for r in commands if r['name'] == 'native-build')
assert build['exit_code'] == 0
cli = [b for b in build['binaries'].values() if b['target']['name'] == 'hyperdrc' and not b['profile']['test']]
tests = [b for b in build['binaries'].values() if b['target']['name'] == 'hyperdrc' and b['target']['kind'] == ['lib'] and b['profile']['test']]
assert len(cli) == len(tests) == 1
assert all(sha(b['retained']) == b['sha256'] for b in [cli[0], tests[0]])
with gzip.open(OLD / 'candidate-preparation-v2/source-hashes.json.gz', 'rt') as f:
    source = json.load(f)
paths = ['crates/hyperdrc/src/conversion.rs', 'crates/hyperdrc/src/parquet_report.rs', 'crates/hyperdrc/src/sqlite_report.rs']
assert all(sha(ROOT / p) == source[p] for p in paths)
dump(W / 'reused-native-artifacts-v1.json', {
    'complete': True, 'native_build_command': build['command'], 'native_build_exit_code': 0,
    'checkpoint_sha256': sha(checkpoint), 'closed_target_reopened': False,
    'cli': cli[0], 'lib_tests': tests[0], 'source': str(OLD / 'candidate-source-v2'),
    'relevant_test_sources_identical_to_current_main': {p: source[p] for p in paths},
    'cache_prototype_isolated': True, 'cache_admitted': False,
})
script = (OLD / 'project_reports_v1.py').read_text()
script = script.replace("q = W / 'candidate-qualification-v2'", "q = W.parent / 'circle-support-index/candidate-qualification-v2'")
script = script.replace("assert read(q / 'result.json')['command_failures'] == [] and read(q / 'result.json')['cargo_target_closed']",
                        "assert read(q / 'result.json')['cargo_target_closed']\nassert read(W / 'reused-native-artifacts-v1.json')['complete']")
script = script.replace("cli = next(r for r in commands if r['name'] == 'native-cli')",
                        "cli = next(r for r in commands if r['name'] == 'native-build')\nassert cli['exit_code'] == 0")
script = script.replace("W / 'baseline-preparation-v1/main-input-hashes.json.gz'",
                        "W.parent / 'circle-support-index/baseline-preparation-v1/main-input-hashes.json.gz'")
script = script.replace("'purpose': 'Preserve report values", "'qualification_incomplete': True, 'purpose': 'Preserve report values")
with (W / 'project_reports_v1.py').open('x') as f:
    f.write(script)
print('Verified existing CLI and test executable; report comparison prepared.', flush=True)
