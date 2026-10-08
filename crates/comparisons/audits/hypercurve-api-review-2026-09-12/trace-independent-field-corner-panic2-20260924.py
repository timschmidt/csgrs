from pathlib import Path
import hashlib,json,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;root=Path('/tmp/hypercurve-independent-field-corners-v1-20260924');P='independent-field-corners-20260924-v1'
manifest=json.loads((A/f'{P}-sources.json').read_text());terminal=json.loads((A/f'{P}-terminal.json').read_text());binary=A/f'{P}-libtest';script=A/f'{P}-panic2.gdb'
def verify():
 for name,sha in manifest.items():
  assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
  assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
 assert hashlib.sha256(binary.read_bytes()).hexdigest()==terminal['binary_sha256']
verify();command=['gdb','--batch','-x',str(script),'--args',str(binary),'--exact','bezier_region::tests::independent_field_corner_edits_preserve_normalized_sets','--nocapture','--test-threads=1','--color','never'];start=time.monotonic()
with (A/f'{P}-panic2-stack.log').open('w') as out:
 code=subprocess.run(command,cwd=root/'hypercurve',stdout=out,stderr=subprocess.STDOUT,timeout=30).returncode
verify();output=(A/f'{P}-panic2-stack.log').read_text();print(output[-9000:])
(A/f'{P}-panic2-terminal.json').write_text(json.dumps(dict(command=command,returncode=code,elapsed_seconds=time.monotonic()-start,binary_sha256=terminal['binary_sha256'],script_sha256=hashlib.sha256(script.read_bytes()).hexdigest(),all_sources_unchanged=True,driver_reaped=True,inferior_killed='killed' in output),indent=2)+'\n')
