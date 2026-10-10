"""Remove only frontier instrumentation from the tested shared-expansion source."""
import difflib
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'laurent-shared-production'
WORK = ROOT / 'target/laurent-shared-production'
SOURCE = WORK / 'source'
FOCUSED = EVIDENCE / 'laurent-shared-expansion'
BASELINE = ROOT / 'target/laurent-memo-index/baseline/source'
TRIAL = ROOT / 'target/laurent-shared-expansion/candidate/source'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
assert read(EVIDENCE / 'diagnostics-74.json')['decision'].startswith('advance shared compound expansion')
focused_status = read(EVIDENCE / 'laurent-shared-expansion/status.json')
assert len(focused_status) == 4 and all(row['exit_code'] == 0 for row in focused_status)
base_record = EVIDENCE / 'contact-index-directed-width/sources.json'
main_record = EVIDENCE / 'native-exactness-gate/sources.json'
restoration_record = EVIDENCE / 'laurent-root-production/sources.json'
trial_record = FOCUSED / 'candidate-sources.json'
base, native, restored = read(base_record), read(main_record), read(restoration_record)
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
frontier_hashes = {}
for variant, directory in [('baseline', BASELINE), ('candidate', TRIAL)]:
    record = read(FOCUSED / (variant + '-sources.json'))
    assert digest(Path(record['source_hashes_file'])) == record['source_hashes_file_sha256']
    frontier_hashes[variant] = json.loads(gzip.decompress(Path(record['source_hashes_file']).read_bytes()))
    assert all(digest(directory / name) == value for name, value in frontier_hashes[variant].items())
OUT.mkdir()
WORK.mkdir()
for src, dst, label in [(BASELINE, SOURCE, 'source'), (ROOT / 'target/laurent-shared-expansion/candidate/target', WORK / 'target', 'build-cache')]:
    with (OUT / ('copy-' + label + '.log')).open('w') as log:
        subprocess.run(['cp', '-a', '--reflink=auto', str(src), str(dst)], stdout=log, stderr=subprocess.STDOUT, check=True)
algebra = 'crates/hyperreal/src/computable/node/algebra.rs'
helper = 'crates/hyperreal/src/computable/node/normal_form_memo_frontier_probe.rs'
module = '\n#[cfg(test)]\nmod normal_form_memo_frontier_probe;\n'
original = (BASELINE / algebra).read_text()
candidate = (TRIAL / algebra).read_text()
assert original.endswith(module) and candidate.endswith(module)
original = original[:-len(module)]
candidate = candidate[:-len(module)]
assert original == subprocess.check_output(['git', 'show', 'HEAD:' + algebra], cwd=ROOT, text=True)
(SOURCE / algebra).write_text(original)
(SOURCE / helper).unlink()

# The previous audit restored exact committed non-Rust inputs; verify them again.
head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
tree = subprocess.check_output(['git', 'ls-tree', '-r', '-z', head], cwd=ROOT)
selected = {}
for entry in tree.split(b'\0'):
    if not entry:
        continue
    metadata, name = entry.split(b'\t', 1)
    _, kind, oid = metadata.decode().split()
    name = name.decode()
    if kind == 'blob' and (name.startswith(('crates/', 'src/', 'tests/', 'benchmarks/', 'examples/', 'demos/', '.cargo/')) or name in ['Cargo.toml', 'Cargo.lock', '.rustfmt.toml']) and Path(name).suffix in {'.rs', '.toml', '.lock', '.kicad_pcb'}:
        selected[name] = oid
mismatches = []
for name, oid in selected.items():
    path = SOURCE / name
    if not path.is_file():
        mismatches.append({'path': name, 'reason': 'missing'})
        continue
    data = path.read_bytes()
    actual = hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest()
    if actual != oid:
        mismatches.append({'path': name, 'reason': 'bytes differ', 'head_blob': oid, 'source_blob': actual})
(OUT / 'committed-input-audit.json').write_text(json.dumps({
    'head': head, 'tracked_rust_build_and_board_inputs_checked': len(selected), 'mismatches': mismatches,
    'inherited_exact_input_restoration': str(restoration_record),
    'restored_missing_inputs_this_trial': [],
}, indent=2) + '\n')
assert not mismatches, mismatches[:10]
baseline_hashes = base['source_hashes'] | restored['additional_source_hashes']
assert all(digest(SOURCE / name) == value for name, value in baseline_hashes.items())
(SOURCE / algebra).write_text(candidate)
environment = os.environ | {
    'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_NET_OFFLINE': 'true',
    'CARGO_TARGET_DIR': str(WORK / 'target'),
}
with (OUT / 'prepare-format.log').open('w') as log:
    subprocess.run(['cargo', 'fmt', '--all', '--', '--check'], cwd=SOURCE, env=environment, stdout=log, stderr=subprocess.STDOUT, check=True)
changed = {name: digest(SOURCE / name) for name, value in baseline_hashes.items() if digest(SOURCE / name) != value}
assert set(changed) == {algebra}
assert (SOURCE / algebra).read_text() + module == (TRIAL / algebra).read_text()
assert all(digest(SOURCE / name) == value for name, value in frontier_hashes['candidate'].items() if name not in [algebra, helper])
assert not (SOURCE / helper).exists()
record = {
    'scope': 'Shared compound expansion with static local-memo consumers on admitted production source, with only the test-only frontier module removed from the focused candidate. No native-stroke prototype or diagnostic Rust is included.',
    'head': head,
    'base_source_record': str(base_record), 'base_source_record_sha256': digest(base_record),
    'main_source_record': str(main_record), 'main_source_record_sha256': digest(main_record),
    'trial_source_record': str(trial_record), 'trial_source_record_sha256': digest(trial_record),
    'restoration_source_record': str(restoration_record), 'restoration_source_record_sha256': digest(restoration_record),
    'changed_source_hashes': changed, 'additional_source_hashes': restored['additional_source_hashes'],
    'source_files': len(baseline_hashes), 'only_modified_runtime_source': algebra,
    'removed_test_only_source': helper, 'removed_test_only_declaration': module,
    'candidate_runtime_identical_to_focused_trial': True,
    'build_cache_origin': str(ROOT / 'target/laurent-shared-expansion/candidate/target'),
    'build_cache_is_not_validation_evidence': True,
    'main_runtime_unchanged': True,
}
(OUT / 'sources.json').write_text(json.dumps(record, indent=2) + '\n')
with tarfile.open(OUT / 'source-delta.tar.gz', 'w:gz') as archive:
    archive.add(SOURCE / algebra, arcname=algebra)
(OUT / 'candidate.patch').write_text(''.join(difflib.unified_diff(original.splitlines(True), candidate.splitlines(True), 'a/' + algebra, 'b/' + algebra)))
shutil.copyfile(__file__, OUT / 'prepare.py')
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
print('Prepared shared expansion without diagnostic Rust:', len(baseline_hashes), 'source files; main runtime unchanged.', flush=True)
