import gzip,hashlib,json,os,shutil,subprocess,sys,time
from pathlib import Path
repo=Path.cwd();root=repo/'docs/evidence/2026-10-09-hyperdrc-native-linear-strokes'
out=Path(sys.argv[1]).resolve();out.mkdir(parents=True,exist_ok=False)
consumers=json.loads((root/'area-compaction-consumers-trial2/manifest.json').read_text())
assert consumers['source_unchanged'] and all(r['exit_code']==0 for r in consumers['status'])
variants={
 'baseline':dict(source=Path('/tmp/drc-point-equality-regressions/source'),target=Path('/tmp/drc-point-equality-regressions/target'),evidence=root/'point-equality-regressions'),
 'candidate':dict(source=Path('/tmp/drc-area-compaction-final/source'),target=Path('/tmp/drc-area-compaction-final/target'),evidence=root/'area-compaction-final-source'),
}
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
info=lambda p:dict(bytes=p.stat().st_size,sha256=digest(p))
for name,v in variants.items():
 v['hashes']=json.loads((v['evidence']/'sources.json').read_text())['source_hashes']
 status=json.loads((v['evidence']/'status.json').read_text())
 assert status[-1]['name']==('curve-doc' if name=='baseline' else 'real-clippy') and all(r['exit_code']==0 for r in status)
fixtures=json.loads(Path(consumers["fixture_recovery_record"]).read_text())
def verify():
 assert all(digest(Path(name))==value["sha256"] for name,value in fixtures["inputs"].items())
 for v in variants.values():assert all(digest(v['source']/name)==h for name,h in v['hashes'].items())
verify()
previous=repo/'docs/evidence/2026-10-09-hyperdrc-native-pad-primitives'
input_record=json.loads((previous/'report-replay/manifest.json').read_text())
inputs=Path(input_record['trials'][0]['command'][-2])
inventory=lambda:{str(p.relative_to(inputs)):info(p) for p in sorted(inputs.rglob('*')) if p.is_file()}
assert inventory()==input_record['input_files']
for name,expected in input_record['archives'].items():assert info(Path(name))==expected
probe_dir=out/'probe_sources';probe_dir.mkdir()
for p in (previous/'cost').glob('*.rs'):shutil.copy2(p,probe_dir/p.name)
shutil.copy2(previous/'full_report_replay.rs',probe_dir/'full_report_replay.rs')
shutil.copy2(__file__,out/'reproduce.py')
commands=[]
def run(label,command,source=None,environment=None):
 verify();start=time.monotonic()
 with (out/(label+'.log')).open('w') as log:code=subprocess.run(command,cwd=source or repo,env=environment,stdout=log,stderr=subprocess.STDOUT).returncode
 commands.append(dict(name=label,command=command,exit_code=code,wall_seconds=time.monotonic()-start))
 (out/'commands.json').write_text(json.dumps(commands,indent=2)+'\n')
 verify();assert code==0,label
 print(label,'passed',flush=True)
 return (out/(label+'.log')).read_text()
for name,v in variants.items():
 environment={**os.environ,'CARGO_HOME':'/tmp/csgrs-properties-cargo','CARGO_NET_OFFLINE':'true','CARGO_TARGET_DIR':str(v['target']),'CCACHE_DIR':'/tmp/csgrs-properties-ccache'}
 text=run(name+'-library-build',['cargo','build','--locked','--release','-p','hyperdrc','--lib','--message-format=json'],v['source'],environment)
 artifacts=[]
 for line in text.splitlines():
  try:a=json.loads(line)
  except ValueError:continue
  if a.get('reason')=='compiler-artifact':artifacts.append(a)
 libraries={}
 for crate in ['hyperdrc','hyperlimit','hypercurve','serde_json','clap']:
  libraries[crate]=next(Path(f) for a in artifacts if a['target']['name']==crate for f in a['filenames'] if f.endswith('.rlib'))
 v['libraries']=libraries;v['binaries']={}
 for kind,source,crates in [
  ('operation',probe_dir/'operation_probe.rs',['hyperdrc','hyperlimit','hypercurve','serde_json']),
  ('report',probe_dir/'full_report_replay.rs',['hyperdrc','clap','serde_json']),
 ]:
  binary=Path('/tmp')/('drc-area-compaction-'+name+'-'+kind)
  command=['rustc','--edition=2024','-O',str(source),'-L','dependency='+str(v['target']/'release/deps')]
  for crate in crates:command+=['--extern',crate+'='+str(libraries[crate])]
  command+=['-o',str(binary)]
  run(name+'-'+kind+'-build',command)
  v['binaries'][kind]=binary
costs=[]
for index in range(1,6):
 for name in (('baseline','candidate') if index%2 else ('candidate','baseline')):
  text=run(f'{name}-operation-{index}',['taskset','-c','0','/usr/bin/time','-v',str(variants[name]['binaries']['operation'])])
  rows=[]
  for line in text.splitlines():
   if line.startswith('operation_'):
    label,iterations,nanos,witness=line.split('\t');rows.append(dict(name=label,iterations=int(iterations),elapsed_ns=int(nanos),witness=witness))
  assert len(rows)==231
  costs.append(dict(variant=name,index=index,rows=rows))
identity=lambda trial:[{k:v for k,v in row.items() if k!='elapsed_ns'} for row in trial['rows']]
identities=identity(costs[0]);assert all(identity(t)==identities for t in costs)
canonical=(json.dumps(identities,sort_keys=True,separators=(',',':'))+'\n').encode()
(out/'rows.json').write_bytes(canonical)
historical=json.loads((repo/'docs/evidence/2026-10-09-hyperdrc-kicad-pad-sources/cost/rows.json').read_text())
assert len(historical)==231
deltas=[dict(index=i,historical=a,current=b) for i,(a,b) in enumerate(zip(historical,identities)) if a!=b]
(out/'historical-row-deltas.json').write_text(json.dumps(deltas,indent=2)+'\n')
report_trials=[]
for index in (1,2):
 for name in (('baseline','candidate') if index==1 else ('candidate','baseline')):
  destination=out/f'{name}-reports-{index}'
  run(f'{name}-reports-{index}',['taskset','-c','0','/usr/bin/time','-v',str(variants[name]['binaries']['report']),str(inputs),str(destination)])
  report_trials.append(dict(variant=name,index=index))
reports={}
for board in ['cparti-fpga','hvp109a']:
 paths=[out/f"{t['variant']}-reports-{t['index']}"/(board+'.json') for t in report_trials]
 raw=paths[0].read_bytes();assert all(p.read_bytes()==raw for p in paths)
 data=json.loads(raw);assert len(data['violations'])==data['violation_count']
 for t,p in zip(report_trials,paths):
  (out/f"{t['variant']}-reports-{t['index']}-{board}.json.gz").write_bytes(gzip.compress(p.read_bytes(),mtime=0))
 reports[board]=dict(bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest(),complete_bytes_identical_across_both_variants_and_repetitions=True,violation_count=data['violation_count'],summary=data['summary'])
assert inventory()==input_record['input_files'];verify()
manifest=dict(fixture_recovery_record=consumers['fixture_recovery_record'],additional_fixture_inputs=fixtures['inputs'],scope='Isolated scalar normal-form reconstruction and loop-area consumers: all 231 existing capability rows in five fresh CPU-0-pinned processes per variant, followed by two fresh complete twelve-check/two-board report serializations per variant. Exact rows and complete report bytes compare without normalization. This does not establish a broad speedup or admit the provisional native-source changes.',baseline_revision='45fcbd3e3',only_modified_sources=json.loads((root/'area-compaction-isolated-trial2/sources.json').read_text())['only_modified_sources']+['crates/hyperreal/tests/gmp_api_coverage.rs'],commands=commands,cost_trials=costs,canonical_rows=info(out/'rows.json'),historical_row_deltas=len(deltas),reports=reports,input_files=input_record['input_files'],archives=input_record['archives'],source_unchanged=True,variants={name:dict(source_record=str(v['evidence']/'sources.json'),source_directory=str(v['source']),libraries={str(p):info(p) for p in v['libraries'].values()},binaries={kind:dict(path=str(p),**info(p)) for kind,p in v['binaries'].items()}) for name,v in variants.items()},probe_sources={p.name:info(p) for p in probe_dir.iterdir()})
(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('production capability/report replay complete',flush=True)
