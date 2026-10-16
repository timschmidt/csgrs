"""Bounded hostile-header and broad PWN probes through the completed WASM build."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import resource
import subprocess
import time

ROOT=Path('/home/tim/Documents/GitHub/workspace/csgrs')
E=ROOT/'docs/evidence/2026-10-16-wasm-format-failures'
WORK=ROOT/'target/properties/wasm-format-failures'
OUT=WORK/'import-resources-v1'
PROBE=E/'import_resources_v1.cjs'
MODULE=WORK/'wasm-target-v1/wasm-bindings-node/csgrs.js'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


def dump(path,value):
    Path(path).write_text(json.dumps(value,indent=2)+'\n')


qualified=json.loads((WORK/'wasm-validation-v1/result.json').read_text())
assert qualified['complete'] and not qualified['unexpected_exits'] and qualified['real_js_and_compiled_typescript_consumers_passed']
assert all(sha(path)==record['sha256'] for path,record in qualified['artifacts'].items())
OUT.mkdir(exist_ok=False)
command=['node',str(PROBE),str(MODULE)]
reader=Path('/home/tim/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/stl_io-0.8.6/src/lib.rs')
protocol={'runner_sha256':sha(__file__),'probe_sha256':sha(PROBE),'command':command,
          'qualified_artifacts':qualified['artifacts'],'reader_source':str(reader),'reader_source_sha256':sha(reader),
          'reader_fact':'as_indexed_triangles uses growing vectors and deliberately does not reserve from the untrusted size_hint; a read error stops iteration.',
          'elapsed_limit_seconds':120,'address_space_limit_bytes':64*1024**3,
          'timing_scope':'Correctness/resource probe while unrelated native qualification may be compiling; no performance comparison.',
          'public_internal_work_or_heap_bound_added':False,'completed_targets_modified':False}
dump(OUT/'protocol.json',protocol)
env=os.environ.copy()
env.pop('NODE_OPTIONS',None)
env['TMPDIR']=str(OUT)
def limits():
    resource.setrlimit(resource.RLIMIT_AS,(64*1024**3,64*1024**3))
    resource.setrlimit(resource.RLIMIT_CORE,(0,0))
start=time.monotonic()
with (OUT/'stdout').open('wb') as stdout,(OUT/'stderr').open('wb') as stderr:
    p=subprocess.run(['/usr/bin/time','-v','-o',str(OUT/'resources'),'timeout','--signal=TERM','--kill-after=10s','120s',*command],
                     cwd=ROOT,env=env,stdout=stdout,stderr=stderr,preexec_fn=limits,start_new_session=True)
record={'exit_code':p.returncode,'elapsed_seconds':time.monotonic()-start,
        **{suffix+'_sha256':sha(OUT/suffix) for suffix in ['stdout','stderr','resources']}}
dump(OUT/'command.json',record)
assert all(sha(path)==row['sha256'] for path,row in qualified['artifacts'].items())
assert sha(PROBE)==protocol['probe_sha256'] and sha(__file__)==protocol['runner_sha256']
assert p.returncode==0
observations=[json.loads(line) for line in (OUT/'stdout').read_text().splitlines()]
assert len(observations)==8
assert observations[0]['repeats']==128 and observations[0]['bytes']==84
assert [row['fixture'] for row in observations[-4:]]==['disconnected','nested-hole','inverted','double-winding']
dump(OUT/'result.json',{'complete':True,'observations':observations,'wasm_artifacts_unchanged':True,
                      'native_pwn_domain_retained':True,'global_resource_bound_claimed':False})
print(json.dumps({'exit_code':p.returncode,'observations':len(observations),'hostile_header_typed_failures':128}),flush=True)
