"""Isolate the tested root-entry omission on admitted production source."""
import argparse
import difflib
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'laurent-root-production'
WORK = ROOT / 'target/laurent-root-production'
SOURCE = WORK / 'source'
BASE = ROOT / 'target/contact-index-directed-width'
TRIAL = ROOT / 'target/laurent-root-memo-omission/candidate/source'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
base_record = EVIDENCE / 'contact-index-directed-width/sources.json'
native_record = EVIDENCE / 'native-exactness-gate/sources.json'
trial_record = EVIDENCE / 'laurent-root-memo-omission/candidate-sources.json'
base = read(base_record)
native = read(native_record)
trial = read(trial_record)
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--resume-after-input-audit', action='store_true')
args = parser.parse_args()
assert (EVIDENCE / 'diagnostics-65.json').is_file()
assert len(read(EVIDENCE / 'laurent-root-native-validation/status.json')) == 6
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
assert all(digest(BASE / 'source' / name) == value for name, value in base['source_hashes'].items())
assert all(digest(TRIAL / name) == value for name, value in trial['source_hashes'].items())
if args.resume_after_input_audit:
    assert (OUT / 'preparation-failure.json').is_file()
    assert not (OUT / 'sources.json').exists()
    assert all(digest(SOURCE / name) == value for name, value in base['source_hashes'].items())
else:
    OUT.mkdir()
    WORK.mkdir()
    for folder in ['source', 'target']:
        with (OUT / ('copy-' + folder + '.log')).open('w') as log:
            subprocess.run(['cp', '-a', '--reflink=auto', str(BASE / folder), str(WORK / folder)], stdout=log, stderr=subprocess.STDOUT, check=True)

# Check admitted tracked Rust/build/fixture inputs against HEAD, independently of
# the inherited SHA-256 record. Documentation is retained from the frozen source.
head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
tree = subprocess.check_output(['git', 'ls-tree', '-r', '-z', head], cwd=ROOT)
entries = {}
for entry in tree.split(b'\0'):
    if entry:
        metadata, name = entry.split(b'\t', 1)
        mode, kind, oid = metadata.decode().split()
        if kind == 'blob':
            entries[name.decode()] = oid
prefixes = ('crates/', 'src/', 'tests/', 'benchmarks/', 'examples/', 'demos/', '.cargo/')
selected = {
    name: oid for name, oid in entries.items()
    if (name.startswith(prefixes) or name in ['Cargo.toml', 'Cargo.lock', '.rustfmt.toml'])
    and Path(name).suffix in {'.rs', '.toml', '.lock', '.kicad_pcb'}
}
mismatches = []
restored = []
for name, oid in selected.items():
    path = SOURCE / name
    if not path.is_file():
        # Restore exact committed inputs omitted by the inherited source export.
        data = subprocess.check_output(['git', 'cat-file', 'blob', oid], cwd=ROOT)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        restored.append(name)
    data = path.read_bytes()
    actual = hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest()
    if actual != oid:
        mismatches.append({'path': name, 'reason': 'bytes differ', 'head_blob': oid, 'source_blob': actual})
(OUT / 'committed-input-audit.json').write_text(json.dumps({
    'head': head, 'tracked_rust_build_and_board_inputs_checked': len(selected),
    'mismatches': mismatches, 'restored_missing_inputs': restored,
}, indent=2) + '\n')
assert not mismatches, mismatches[:10]

algebra = 'crates/hyperreal/src/computable/node/algebra.rs'
module = '\n#[cfg(test)]\nmod normal_form_memo_frontier_probe;\n'
original = (SOURCE / algebra).read_text()
baseline_trial = (ROOT / 'target/laurent-memo-sharing/baseline/source' / algebra).read_text()
assert baseline_trial.endswith(module)
assert baseline_trial[:-len(module)] == original
candidate = (TRIAL / algebra).read_text()
assert candidate.endswith(module)
(SOURCE / algebra).write_text(candidate[:-len(module)])
environment = os.environ | {
    'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_NET_OFFLINE': 'true',
    'CARGO_TARGET_DIR': str(WORK / 'target'),
}
with (OUT / 'prepare-format.log').open('w') as log:
    subprocess.run(['cargo', 'fmt', '--all'], cwd=SOURCE, env=environment, stdout=log, stderr=subprocess.STDOUT, check=True)
changed = {name: digest(SOURCE / name) for name, value in base['source_hashes'].items() if digest(SOURCE / name) != value}
assert set(changed) == {algebra}, changed
# Include restored committed inputs and a fixture absent from an older inventory.
fixture = 'crates/hyperdrc/tests/fixtures/native-circle-via.kicad_pcb'
additional = {
    name: digest(SOURCE / name) for name in sorted(set(selected) | {fixture})
    if name not in base['source_hashes']
}
record = {
    'scope': 'Only unused final root normal-form memo insertion is omitted on admitted production source. No native-stroke prototype or test-only frontier instrumentation is included.',
    'head': head,
    'base_source_record': str(base_record), 'base_source_record_sha256': digest(base_record),
    'main_source_record': str(native_record), 'main_source_record_sha256': digest(native_record),
    'trial_source_record': str(trial_record), 'trial_source_record_sha256': digest(trial_record),
    'changed_source_hashes': changed, 'additional_source_hashes': additional,
    'source_files': len(base['source_hashes']) + len(additional),
    'only_modified_runtime_source': algebra,
    'frozen_documentation': 'Documentation is inherited from the previously admitted contact-index source; this input audit compares tracked Rust, Cargo/build configuration and board fixtures against HEAD.',
    'main_runtime_unchanged': True,
}
(OUT / 'sources.json').write_text(json.dumps(record, indent=2) + '\n')
with tarfile.open(OUT / 'source-delta.tar.gz', 'w:gz') as archive:
    archive.add(SOURCE / algebra, arcname=algebra)
with tarfile.open(OUT / 'restored-inputs.tar.gz', 'w:gz') as archive:
    for name in additional:
        archive.add(SOURCE / name, arcname=name)
(OUT / 'candidate.patch').write_text(''.join(difflib.unified_diff(
    original.splitlines(True), (SOURCE / algebra).read_text().splitlines(True),
    'a/' + algebra, 'b/' + algebra,
)))
shutil.copyfile(__file__, OUT / 'prepare.py')
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
print('Prepared production-only root-entry omission; main runtime unchanged.', flush=True)
