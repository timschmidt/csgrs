"""Freeze a test-only temporary-directory repair against current main."""
import gzip
import hashlib
import json
import re
import shutil
import subprocess
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]
OLD = W.parent / 'circle-support-index'
O = W / 'temp-preparation-v1'
S = W / 'temp-candidate-source-v1'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, v):
    Path(p).write_text(json.dumps(v, indent=2) + '\n')

base = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
assert base == '8982f85b09740ab89309e3be9b6a970b65c50ec8'
assert read(W / 'temp-reproduction-v1/result.json')['expected_file_failures_reproduced'] == 5
subprocess.run(['git', 'diff', '--quiet'], cwd=ROOT, check=True)
with gzip.open(OLD / 'baseline-preparation-v1/main-input-hashes.json.gz', 'rt') as f:
    names = json.load(f)
before = {p: sha(ROOT / p) for p in names}
O.mkdir(exist_ok=False)
S.mkdir(exist_ok=False)
for p in names:
    target = S / p
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ROOT / p, target)
paths = ['crates/hyperdrc/src/conversion.rs', 'crates/hyperdrc/src/parquet_report.rs', 'crates/hyperdrc/src/sqlite_report.rs']
old_texts = {p: (S / p).read_text() for p in paths}
conversion = old_texts[paths[0]]
conversion, count = re.subn(r'PathBuf::from\(format!\("/tmp/([^"]+)"\)\)', r'std::env::temp_dir().join(format!("\1"))', conversion)
assert count == 8
# The success and directory-error cases run in one test process concurrently.
marker = '    fn convert_reports_directory_creation_failure() {'
prefix, tail = conversion.split(marker)
assert tail.count('hyperdrc-conv-input-{process_id}') == 1
tail = tail.replace('hyperdrc-conv-input-{process_id}', 'hyperdrc-conv-failure-input-{process_id}')
cleanup = 'std::fs::remove_file(format!("/tmp/hyperdrc-output-file-{process_id}.txt"))'
assert cleanup in tail
tail = tail.replace(cleanup, 'std::fs::remove_file(&request.output_dir)')
(S / paths[0]).write_text(prefix + marker + tail)
for p, extension in zip(paths[1:], ['parquet', 'sqlite']):
    text = old_texts[p]
    assert text.count('    use std::path::PathBuf;\n') == 1
    text = text.replace('    use std::path::PathBuf;\n', '')
    text = text.replace('let path = PathBuf::from(format!(\n            "/tmp/hyperdrc-report-{}.' + extension + '",',
                        'let path = std::env::temp_dir().join(format!(\n            "hyperdrc-report-{}.' + extension + '",')
    assert 'PathBuf' not in text and '"/tmp/' not in text
    (S / p).write_text(text)
with (O / 'format.stdout').open('wb') as out, (O / 'format.stderr').open('wb') as err:
    formatted = subprocess.run(['cargo', 'fmt', '--all'], cwd=S, stdout=out, stderr=err)
assert formatted.returncode == 0
after = {p: sha(S / p) for p in names}
changed = {p: after[p] for p in names if after[p] != before[p]}
assert set(changed) == set(paths)
for p in paths:
    old = old_texts[p]
    new = (S / p).read_text()
    assert old.split('#[cfg(test)]', 1)[0] == new.split('#[cfg(test)]', 1)[0]
    # No assertion line or adjacent fixture field is edited.
    assert [line.strip() for line in old.splitlines() if 'assert' in line or 'expect' in line] == [
        line.strip() for line in new.splitlines() if 'assert' in line or 'expect' in line]
    snapshot = O / 'original-tests' / p
    snapshot.parent.mkdir(parents=True, exist_ok=True)
    snapshot.write_text(old)
with gzip.open(O / 'main-input-hashes.json.gz', 'wt') as f:
    json.dump(before, f, sort_keys=True)
with gzip.open(O / 'source-hashes.json.gz', 'wt') as f:
    json.dump(after, f, sort_keys=True)
dump(O / 'result.json', {
    'complete': True, 'base_commit': base, 'source': str(S), 'source_inputs': len(after),
    'source_hashes_sha256': sha(O / 'source-hashes.json.gz'),
    'main_input_hashes_sha256': sha(O / 'main-input-hashes.json.gz'), 'changes': changed,
    'production_prefixes_unchanged': True, 'assertion_lines_unchanged': True,
    'cache_index_included': False, 'source_changes_applied': False, 'runner_sha256': sha(__file__),
})
print(json.dumps({'complete': True, 'test_only_paths': paths, 'cache_index_included': False}), flush=True)
