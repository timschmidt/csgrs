"""Factor only compound recognition out of the rejected static-consumer routine."""
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
OUT = EVIDENCE / 'laurent-shared-expansion'
WORK = ROOT / 'target/laurent-shared-expansion/candidate'
PREVIOUS = ROOT / 'target/laurent-memo-consumer/candidate'
BASE = ROOT / 'target/laurent-memo-index/baseline'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
base_record = EVIDENCE / 'laurent-memo-index/baseline-sources.json'
parent_record = EVIDENCE / 'laurent-memo-consumer/candidate-sources.json'
base, parent = read(base_record), read(parent_record)
hashes = {}
for name, record, directory in [('baseline', base, BASE), ('parent', parent, PREVIOUS)]:
    assert digest(Path(record['source_hashes_file'])) == record['source_hashes_file_sha256']
    hashes[name] = json.loads(gzip.decompress(Path(record['source_hashes_file']).read_bytes()))
    assert all(digest(directory / 'source' / path) == value for path, value in hashes[name].items())
native = read(Path(base['main_source_record']))
assert read(EVIDENCE / 'diagnostics-73.json')['decision'].startswith('not retained:')
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
OUT.mkdir()
WORK.mkdir(parents=True)
for folder in ['source', 'target']:
    with (OUT / ('copy-' + folder + '.log')).open('w') as log:
        subprocess.run(['cp', '-a', '--reflink=auto', str(PREVIOUS / folder), str(WORK / folder)], stdout=log, stderr=subprocess.STDOUT, check=True)
name = 'crates/hyperreal/src/computable/node/algebra.rs'
path = WORK / 'source' / name
original = (BASE / 'source' / name).read_text()
before = path.read_text()
first = before.index('    fn pi_laurent_polynomial_with_consumer<')
last = before.index('    pub(crate) fn bounded_laurent_rational', first)
start = before.index('            let expanded = (|| match &self.internal.approximation {', first, last)
end = before.index('\n        };\n        if shared {', start, last)
compound = before[start:end]
assert 'C::' not in compound and 'consumer' not in compound.replace('with_consumer', '')
assert compound.count('expanded.or_else') == 1
body = '\n'.join(line[4:] if line.startswith('    ') else line for line in compound.split('\n'))
helper = '''    // The caller has already checked and charged this node's budgets.
    // Preserve the compound fallback before the caller records the raw memo value.
    fn pi_laurent_compound_polynomial(
        &self,
        budget: usize,
        remaining_work: &mut usize,
        memo: &mut Option<Vec<(usize, Option<PiLaurentPolynomial>)>>,
    ) -> Option<PiLaurentPolynomial> {
''' + body + '\n    }\n\n'
after = before[:start] + '            self.pi_laurent_compound_polynomial(budget, remaining_work, memo)' + before[end:last] + helper + before[last:]
assert after.count('fn pi_laurent_compound_polynomial(') == 1
path.write_text(after)
env = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_NET_OFFLINE': 'true', 'CARGO_TARGET_DIR': str(WORK / 'target')}
with (OUT / 'prepare-format.log').open('w') as log:
    subprocess.run(['cargo', 'fmt', '--all', '--', '--check'], cwd=WORK / 'source', env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
current = {filename: digest(WORK / 'source' / filename) for filename in hashes['baseline']}
changes = [filename for filename, value in current.items() if hashes['baseline'][filename] != value]
assert changes == [name]
assert [filename for filename, value in current.items() if hashes['parent'][filename] != value] == [name]
compressed = OUT / 'candidate-source-hashes.json.gz'
compressed.write_bytes(gzip.compress((json.dumps(current, indent=2) + '\n').encode(), mtime=0))
record = {
    'scope': 'Static local-memo consumers with only compound expansion and its existing opaque fallback factored into a nongeneric helper. Cheap recognizers, original root memo, budgets and closed-only borrowing remain unchanged.',
    'base_source_record': str(base_record), 'base_source_record_sha256': digest(base_record),
    'parent_source_record': str(parent_record), 'parent_source_record_sha256': digest(parent_record),
    'main_source_record': base['main_source_record'],
    'source_hashes_file': str(compressed), 'source_hashes_file_sha256': digest(compressed),
    'source_files': len(current), 'only_modified_sources': changes,
    'baseline_source_directory': str(BASE / 'source'), 'candidate_source_directory': str(WORK / 'source'),
    'main_runtime_unchanged': True,
}
(OUT / 'candidate-sources.json').write_text(json.dumps(record, indent=2) + '\n')
shutil.copyfile(base_record, OUT / 'baseline-sources.json')
with tarfile.open(OUT / 'candidate-source-delta.tar.gz', 'w:gz') as archive:
    archive.add(path, arcname=name)
for label, source in [('candidate', original), ('factor-only', before)]:
    (OUT / (label + '.patch')).write_text(''.join(difflib.unified_diff(source.splitlines(True), after.splitlines(True), 'a/' + name, 'b/' + name)))
binary = EVIDENCE / 'laurent-consumer-production-cost/candidate-time'
manifest = read(EVIDENCE / 'laurent-consumer-production-cost/manifest.json')
assert digest(binary) == manifest['binaries']['candidate-time']['sha256']
command = ['nm', '-S', '-C', str(binary)]
symbols = subprocess.check_output(command, text=True)
selected = [line for line in symbols.splitlines() if 'pi_laurent_polynomial_with_consumer' in line or 'pi_laurent_polynomial_with_memo' in line or 'PiLaurentConsumer' in line]
(OUT / 'previous-consumer-symbols.json').write_text(json.dumps({
    'command': command, 'binary_sha256': digest(binary), 'symbols': selected,
    'scope': 'Emitted symbol inventory from the rejected production timing binary; symbol sizes are not inclusive call costs or a causal performance explanation.',
}, indent=2) + '\n')
shutil.copyfile(__file__, OUT / 'prepare.py')
assert all(digest(ROOT / name) == value for name, value in native['source_hashes'].items())
print('Prepared shared compound expansion; prior candidates and main runtime unchanged.', flush=True)
