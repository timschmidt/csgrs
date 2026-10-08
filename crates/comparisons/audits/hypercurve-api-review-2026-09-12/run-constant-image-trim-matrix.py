from pathlib import Path
import hashlib, json, subprocess, time
root = Path('/home/tim/Documents/GitHub/workspace')
audit = root / 'hypercurve-api-review-2026-09-12'
items = [json.loads(line) for line in (audit / 'constant-image-trim-test-build.jsonl').read_text().splitlines()]
assert items[-1].get('success')
lib = next(x for x in items if x.get('reason') == 'compiler-artifact' and x['target']['name'] == 'hypercurve' and not x['profile']['test'])
rlib = next(p for p in lib['filenames'] if p.endswith('.rlib'))
binary = audit / 'constant-image-trim-matrix'
command = ['rustc', '--edition=2024', '-O', str(audit / 'constant-image-trim-matrix.rs'), '--extern', 'hypercurve=' + rlib, '-L', 'dependency=' + str(Path(rlib).parent), '-o', str(binary)]
r = subprocess.run(command, capture_output=True, text=True)
assert r.returncode == 0, r.stderr
start = time.monotonic()
with (audit / 'constant-image-trim-matrix.log').open('w') as out:
    try:
        r = subprocess.run([str(binary)], stdout=out, stderr=subprocess.STDOUT, timeout=180)
        code = r.returncode
    except subprocess.TimeoutExpired:
        code = 124
record = {'returncode': code, 'elapsed_seconds': time.monotonic() - start, 'library': rlib,
          'sha256': hashlib.sha256(Path(rlib).read_bytes()).hexdigest(), 'log': 'constant-image-trim-matrix.log'}
if code == 0:
    record.update(json.loads((audit / 'constant-image-trim-matrix.log').read_text()))
(audit / 'constant-image-trim-matrix.json').write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps(record), flush=True)
if code:
    print((audit / 'constant-image-trim-matrix.log').read_text()[-10000:], flush=True)
raise SystemExit(code)
