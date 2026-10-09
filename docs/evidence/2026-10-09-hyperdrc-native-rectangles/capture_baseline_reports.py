"""Capture the full fixture-check reports from the verified previous library."""
import gzip, hashlib, json, pathlib, shutil, subprocess, time
root=pathlib.Path.cwd();record=pathlib.Path(__file__).resolve().parent
saved=pathlib.Path('/tmp/drc-native-rectangles-baseline-libraries')
out=record/'baseline-reports';out.mkdir()
inputs=pathlib.Path('/tmp/drc-native-rectangles-report-inputs');reports=pathlib.Path('/tmp/drc-native-rectangles-baseline-reports')
info=lambda p: dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
prior=record.parent/'2026-10-09-hyperdrc-fixture-access'
old=json.loads((prior/'cost/manifest.json').read_text())
assert info(saved/'libhyperdrc.rlib')==next(v for p,v in old['binaries'].items() if p.endswith('.rlib'))
artifacts=[json.loads(line) for line in (prior/'cost/library-build.jsonl').read_text().splitlines()]
def rlib(name):
 return next(pathlib.Path(f) for a in artifacts if a.get('reason')=='compiler-artifact' and a['target']['name']==name for f in a['filenames'] if f.endswith('.rlib'))
linked={'hyperdrc':saved/'libhyperdrc.rlib','clap':rlib('clap'),'serde_json':rlib('serde_json')}
command=['rustc','--edition=2024','-O',str(record/'full_report_replay.rs'),'-L',f'dependency={root}/target/release/deps']
for name,path in linked.items():command+=['--extern',f'{name}={path}']
binary=saved/'full-report-replay';command+=['-o',str(binary)]
with (out/'build.log').open('w') as log:subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
archives={}
for label,name in [('cparti-fpga','CPArti FPGA dev board.zip'),('hvp109a','HVP109A.zip')]:
 archive=pathlib.Path('crates/hyperdrc/docs')/name;archives[str(archive)]=info(archive)
 destination=inputs/label;destination.mkdir(parents=True)
 subprocess.run(['unzip','-q',str(archive),'-d',str(destination)],check=True)
shutil.copyfile(record/'fixture.ipc',inputs/'fixture.ipc')
input_files={str(p.relative_to(inputs)):info(p) for p in sorted(inputs.rglob('*')) if p.is_file()}
run=['timeout','600s',str(binary),str(inputs),str(reports)];start=time.time()
with (out/'run.log').open('w') as log:code=subprocess.run(run,stdout=log,stderr=subprocess.STDOUT).returncode
result={}
for path in reports.glob('*.json'):
 raw=path.read_bytes();parsed=json.loads(raw);(out/(path.name+'.gz')).write_bytes(gzip.compress(raw,mtime=0))
 result[path.stem]={**info(path),'violations':parsed['violation_count'],'summary':parsed['summary']}
(out/'manifest.json').write_text(json.dumps(dict(scope='Verified reused baseline DRC library from 5e94512ea, built and recorded by the fixture-access campaign. Twelve checks (four fixture-smoke, four dense-pad and four thermal/IPC fixture checks) on both full boards plus the retained synthetic IPC sidecar; the full boards are unchanged.',build_command=command,command=run,exit_code=code,elapsed_seconds=time.time()-start,linked_libraries={str(p):info(p) for p in linked.values()},binary=info(binary),driver=info(record/'full_report_replay.rs'),archives=archives,input_files=input_files,reports=result),indent=2)+'\n')
shutil.copyfile(prior/'cost/manifest.json',out/'reused-cost-manifest.json');shutil.copyfile(prior/'cost/library-build.jsonl',out/'reused-library-build.jsonl')
print('baseline complete reports',code,flush=True)
