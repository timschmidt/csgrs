"""Run after reproduce.py finishes, without other builds or measurements."""
import hashlib,json,os,subprocess,time
from pathlib import Path
out=Path(__file__).resolve().parent
work=Path('/tmp/drc-line-fragment-reversal-fix')
source=work/'source'
record=json.loads((out/'sources.json').read_text())
def verify():
    assert all(hashlib.sha256((source/p).read_bytes()).hexdigest()==h for p,h in record['source_hashes'].items())
environment={**os.environ,'CARGO_TARGET_DIR':str(work/'target'),'CCACHE_DIR':str(work/'ccache')}
commands=[
 ('curve-string-integration',['cargo','test','--locked','-p','hypercurve','--release','--test','hypercurve_curve_string','--','--test-threads=2']),
 ('curve-string-unit-matched',['cargo','test','--locked','-p','hypercurve','--release','--lib','tests::curve_string_','--','--test-threads=2']),
 ('line-side-unit',['cargo','test','--locked','-p','hypercurve','--release','--lib','line_side_','--','--test-threads=2']),
]
status=[]
for name,command in commands:
    verify();start=time.monotonic()
    with (out/(name+'.log')).open('w') as log:
        code=subprocess.run(command,cwd=source,env=environment,stdout=log,stderr=subprocess.STDOUT).returncode
    verify()
    status.append(dict(name=name,command=command,exit_code=code,elapsed_seconds=time.monotonic()-start))
    (out/'consumer-status.json').write_text(json.dumps(status,indent=2)+'\n')
    print(name,code,flush=True)
    if code:raise SystemExit(code)
