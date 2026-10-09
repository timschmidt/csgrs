"""Run the focused fixture, existing fixture and shared drill-index checks."""
from pathlib import Path
import hashlib,json,subprocess,sys,time
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False)
source=lambda:{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in Path('crates/hyperdrc').rglob('*.rs') if 'target' not in p.parts}
old=source()
commands=[('clippy',['cargo','clippy','--locked','-p','hyperdrc','--all-features','--all-targets','--','-D','warnings']),('focused',['cargo','test','--locked','-p','hyperdrc','--release','--lib','checks::fixture_access_regressions','--','--nocapture','--test-threads=1']),('existing-ipc',['cargo','test','--locked','-p','hyperdrc','--release','--lib','testpoint_','--','--test-threads=1']),('existing-thermal',['cargo','test','--locked','-p','hyperdrc','--release','--lib','thermal_copper_area_','--','--test-threads=1']),('existing-drills',['cargo','test','--locked','-p','hyperdrc','--release','--lib','checks::drill_native_regressions','--','--test-threads=1'])]
status=[]
for name,cmd in commands:
 start=time.time()
 with (out/(name+'.log')).open('w') as log:code=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT).returncode
 status.append(dict(name=name,command=cmd,exit_code=code,elapsed_seconds=time.time()-start));(out/'status.json').write_text(json.dumps(status,indent=2)+'\n');print(name,code,flush=True)
 if code:break
(out/'sources.json').write_text(json.dumps(dict(source_hashes=old,unchanged=old==source()),indent=2)+'\n')
raise SystemExit(status[-1]['exit_code'])
