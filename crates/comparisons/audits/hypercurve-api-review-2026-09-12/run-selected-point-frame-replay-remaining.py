from pathlib import Path
import hashlib, json, subprocess, time

audit = Path('/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12')
prefix = 'selected-point-frame-replay'
stem = prefix + '-remaining-circle-domains'
destination = audit / (stem + '.json')
assert not destination.exists()
items = [json.loads(line) for line in (audit / (prefix + '-test-build.jsonl')).read_text().splitlines()]
library = Path(next(f for x in items if x.get('reason') == 'compiler-artifact' and x['target']['name'] == 'hypercurve' and not x['profile']['test'] for f in x['filenames'] if f.endswith('.rlib')))
source = audit / 'finite-circle-native-poles-public.rs'
executable = audit / stem
hashfile = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
source_sha, library_sha = hashfile(source), hashfile(library)
command = ['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc', '--edition=2024', '-O', str(source), '--extern', 'hypercurve=' + str(library), '-L', 'dependency=' + str(library.parent), '-o', str(executable)]
with (audit / (stem + '-compile.log')).open('w') as log:
    result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=120)
assert result.returncode == 0
start = time.monotonic()
with (audit / (stem + '.log')).open('w') as log:
    try:
        code = subprocess.run([str(executable)], stdout=log, stderr=subprocess.STDOUT, timeout=120).returncode
    except subprocess.TimeoutExpired:
        code = 124
record = dict(status='remaining closure probe, separate from passing qualification', commit=None, returncode=code, elapsed_seconds=time.monotonic() - start, source=source.name, source_sha256=source_sha, normal_library=str(library), normal_library_sha256=library_sha, executable=executable.name, executable_sha256=hashfile(executable), command=command, log=stem + '.log')
for line in reversed((audit / (stem + '.log')).read_text().splitlines()):
    try:
        counts = json.loads(line)
    except json.JSONDecodeError:
        continue
    if isinstance(counts, dict):
        record['counts'] = counts
        break
assert hashfile(source) == source_sha and hashfile(library) == library_sha
destination.write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps(record, indent=2))
