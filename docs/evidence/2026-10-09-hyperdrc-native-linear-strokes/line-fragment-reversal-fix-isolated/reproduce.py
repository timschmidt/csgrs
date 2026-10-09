import hashlib,json,os,shutil,subprocess,tarfile,time
from pathlib import Path
repo=Path.cwd()
root=repo/'docs/evidence/2026-10-09-hyperdrc-native-linear-strokes'
baseline=Path('/tmp/drc-linear-conic-frame-fix-isolated')
base=json.loads((root/'conic-frame-fix-isolated/sources.json').read_text())
work=Path('/tmp/drc-line-fragment-reversal-fix')
out=root/'line-fragment-reversal-fix-isolated'
work.mkdir(exist_ok=False);out.mkdir(exist_ok=False)
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert all(digest(baseline/'source'/p)==h for p,h in base['source_hashes'].items())
for name in ['source','target']:
 subprocess.run(['cp','-a','--reflink=auto',str(baseline/name),str(work/name)],check=True)
source=work/'source';target=work/'target'
changed=['crates/hypercurve/src/segment.rs','crates/hypercurve/tests/line_fragment_orientation.rs']
name=changed[0]
original=(source/name).read_text()
assert original==subprocess.check_output(['git','show','HEAD:'+name],text=True)
start='    /// [`Self::classify_point`] under an explicit predicate policy.\n'
end='    /// Classifies whether a point lies on this finite line segment.\n'
live=(repo/name).read_text()
replacement=live[live.index(start):live.index(end)]
fixed=original[:original.index(start)]+replacement+original[original.index(end):]
assert fixed!=original
(source/name).write_text(fixed)
shutil.copyfile(repo/changed[1],source/changed[1])
paths=sorted(set(base['source_hashes'])|set(changed))
hashes=lambda:{p:digest(source/p) for p in paths}
initial=hashes()
assert {p for p,h in base['source_hashes'].items() if initial[p]!=h}=={changed[0]}
record=dict(production_baseline_revision='0e9fd3e21',base_source_record=str(root/'conic-frame-fix-isolated/sources.json'),only_modified_sources=changed,source_hashes=initial,unchanged=True)
(out/'sources.json').write_text(json.dumps(record,indent=2)+'\n')
with tarfile.open(out/'source-delta.tar.gz','w:gz') as archive:
 for name in changed:archive.add(source/name,arcname=name)
shutil.copyfile(__file__,out/'reproduce.py')
environment={**os.environ,'CARGO_TARGET_DIR':str(target),'CCACHE_DIR':str(work/'ccache')}
commands=[
 ('format',['cargo','fmt','--all','--','--check']),
 ('public-consumers',['cargo','test','--locked','-p','hypercurve','--release','--test','line_fragment_orientation','--test','hypercurve_fragments','--test','hypercurve_split','--test','hypercurve_contour','--test','hypercurve_offset','--test','hypercurve_curve_region_boolean','--test','line_constant_coordinates','--','--test-threads=2']),
 ('segment-unit',['cargo','test','--locked','-p','hypercurve','--release','--lib','segment::','--','--test-threads=2']),
 ('fragment-unit',['cargo','test','--locked','-p','hypercurve','--release','--lib','fragment::','--','--test-threads=2']),
 ('curve-string-unit',['cargo','test','--locked','-p','hypercurve','--release','--lib','curve_string::','--','--test-threads=2']),
 ('prepared-unit',['cargo','test','--locked','-p','hypercurve','--release','--lib','prepared::','--','--test-threads=2']),
 ('curve-clippy',['cargo','clippy','--locked','-p','hypercurve','--release','--all-targets','--','-D','warnings']),
 ('build-board-smoke',['cargo','test','--locked','-p','hyperdrc','--release','--lib','--no-run','--message-format=json']),
]
status=[]
def run(name,command):
 assert hashes()==initial
 started=time.monotonic()
 with (out/(name+'.log')).open('w') as log:code=subprocess.run(command,cwd=source,env=environment,stdout=log,stderr=subprocess.STDOUT).returncode
 unchanged=hashes()==initial
 status.append(dict(name=name,command=command,exit_code=code,elapsed_seconds=time.monotonic()-started))
 (out/'status.json').write_text(json.dumps(status,indent=2)+'\n')
 record['unchanged']=unchanged;(out/'sources.json').write_text(json.dumps(record,indent=2)+'\n')
 assert unchanged
 print(name,code,flush=True)
 if code:raise SystemExit(code)
for name,command in commands:run(name,command)
executables=[];libraries=[]
for line in (out/'build-board-smoke.log').read_text().splitlines():
 try:a=json.loads(line)
 except ValueError:continue
 if a.get('reason')!='compiler-artifact':continue
 if a.get('target',{}).get('name')=='hyperdrc' and a.get('executable'):executables.append(a['executable'])
 if a.get('target',{}).get('name')=='hypercurve':libraries.extend(Path(p) for p in a['filenames'] if p.endswith('.rlib'))
assert len(executables)==len(libraries)==1
probe=root/'line-fragment-reversal-counterexample/probe.rs'
binary=work/'public-counterexample'
run('counterexample-build',['rustc','--edition=2024','-O',str(probe),'-L','dependency='+str(libraries[0].parent),'--extern','hypercurve='+str(libraries[0]),'-o',str(binary)])
run('counterexample-replay',[str(binary)])
run('full-board-smoke',['/usr/bin/time','-v','-o',str(out/'full-board-smoke-time.log'),executables[0],'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite','--exact','--test-threads=1','--nocapture'])
log=(out/'full-board-smoke.log').read_text()
assert 'skipping complex project fixture test' not in log
assert log.count('hyperdrc: finished emit report')==2
assert all(digest(baseline/'source'/p)==h for p,h in base['source_hashes'].items())
assert (repo/changed[1]).read_bytes()==(source/changed[1]).read_bytes()
print('isolated line reversal fix passed',flush=True)
