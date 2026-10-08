from pathlib import Path
import subprocess, json, hashlib, time, sys

root = Path('/home/tim/Documents/GitHub/workspace')
audit = root / 'hypercurve-api-review-2026-09-12'
stem, library = sys.argv[1:]
source = audit / 'finite-region-self-domain-public.rs'
library = Path(library)
executable = audit / stem
command = ['rustc', '--edition=2024', '-O', str(source), '--extern',
           'hypercurve=' + str(library), '-L',
           'dependency=' + str(root / 'hypercurve/target/release/deps'),
           '-o', str(executable)]
compile_result = subprocess.run(command, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, timeout=120)
(audit / (stem + '-compile.log')).write_text(compile_result.stdout)
if compile_result.returncode:
    print(compile_result.stdout)
    raise SystemExit(compile_result.returncode)
start = time.monotonic()
with (audit / (stem + '.log')).open('w') as output:
    try:
        code = subprocess.run([str(executable)], stdout=output,
                              stderr=subprocess.STDOUT, timeout=120).returncode
    except subprocess.TimeoutExpired:
        code = 124
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
data = dict(returncode=code, elapsed_seconds=time.monotonic() - start,
            source=source.name, source_sha256=sha(source),
            normal_library=str(library), normal_library_sha256=sha(library),
            executable_sha256=sha(executable), log=stem + '.log', command=command,
            commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'],
                cwd=root / 'hypercurve', text=True).strip())
(audit / (stem + '.json')).write_text(json.dumps(data, indent=2) + '\n')
print(json.dumps(data, indent=2))
print((audit / (stem + '.log')).read_text()[-5500:])
raise SystemExit(code)
