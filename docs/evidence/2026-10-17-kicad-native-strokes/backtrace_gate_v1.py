"""Locate the query behind the scalar hot spots; exclude the run from timing admission."""
import hashlib,json,subprocess,time
from pathlib import Path
W=Path(__file__).resolve().parent;O=W/'gate-backtrace-v1';O.mkdir(exist_ok=False)
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
p=json.loads((W/'gate-observation-v2/result.json').read_text())['processes'][0]
assert Path('/proc',str(p['pid']),'cmdline').read_bytes().decode().split('\0')==p['args']
assert sha(p['args'][0])==p['exe_sha256']
command=['gdb','-q','-nx','-batch','-iex','set debuginfod enabled off','-iex','set auto-load safe-path /nonexistent',
    '-p',str(p['pid']),'-ex','set pagination off','-ex','thread apply all bt 48','-ex','detach']
start=time.monotonic()
with (O/'stdout').open('wb') as out,(O/'stderr').open('wb') as err:
    r=subprocess.run(command,cwd=O,stdout=out,stderr=err,timeout=30)
(O/'result.json').write_text(json.dumps({'complete':True,'runner_sha256':sha(__file__),'command':command,
    'exit_code':r.returncode,'elapsed_seconds':time.monotonic()-start,'stdout_sha256':sha(O/'stdout'),
    'stderr_sha256':sha(O/'stderr'),'exe_sha256':p['exe_sha256'],
    'diagnostic_only_not_performance_admission':True},indent=2)+'\n')
print(json.dumps({'complete':True,'exit_code':r.returncode}))
