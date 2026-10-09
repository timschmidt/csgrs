import hashlib,json,os,shutil,subprocess,tarfile,time
from pathlib import Path
repo=Path.cwd();root=repo/'docs/evidence/2026-10-09-hyperdrc-native-linear-strokes'
baseline=Path('/tmp/drc-line-fragment-reversal-fix')
base=json.loads((root/'line-fragment-reversal-fix-isolated/sources.json').read_text())
work=Path('/tmp/drc-circular-support-guard-fix');out=root/'circular-support-guard-fix-isolated'
work.mkdir(exist_ok=False);out.mkdir(exist_ok=False)
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert all(digest(baseline/'source'/p)==h for p,h in base['source_hashes'].items())
for folder in ['source','target']:subprocess.run(['cp','-a','--reflink=auto',str(baseline/folder),str(work/folder)],check=True)
source=work/'source';target=work/'target';name='crates/hypercurve/src/arc_bezier.rs'
original=(source/name).read_text();assert original==subprocess.check_output(['git','show','HEAD:'+name],text=True)
fixed=(repo/name).read_text()
start='    // Degree elevation and exact subdivision retain the authoritative circle\n'
stop='        && matches!(\n'
def guard(text):
 a=text.index(start);return text[a:text.index(stop,a)]
assert guard(original)!=guard(fixed)
pre_fix=fixed.replace(guard(fixed),guard(original),1)
(source/name).write_text(pre_fix)
paths=sorted(base['source_hashes']);hashes=lambda:{p:digest(source/p) for p in paths}
environment={**os.environ,'CARGO_TARGET_DIR':str(target),'CCACHE_DIR':str(work/'ccache')}
old_out=out/'counterexample-before';old_out.mkdir()
old_hashes=hashes();assert {p for p,h in base['source_hashes'].items() if old_hashes[p]!=h}=={name}
(old_out/'sources.json').write_text(json.dumps(dict(production_baseline_revision='7a9771f4f',only_new_test_code=True,source_hashes=old_hashes),indent=2)+'\n')
with tarfile.open(old_out/'source-delta.tar.gz','w:gz') as a:a.add(source/name,arcname=name)
command=['cargo','test','--locked','-p','hypercurve','--release','--lib','circular_support_does_not_promote_a_retraced_chart_to_one_arc','--','--exact','--test-threads=1']
# Libtest's exact name includes its module path.
command[command.index('circular_support_does_not_promote_a_retraced_chart_to_one_arc')]='arc_bezier::tests::circular_support_does_not_promote_a_retraced_chart_to_one_arc'
with (old_out/'test.log').open('w') as log:code=subprocess.run(command,cwd=source,env=environment,stdout=log,stderr=subprocess.STDOUT).returncode
assert hashes()==old_hashes
(old_out/'status.json').write_text(json.dumps(dict(command=command,exit_code=code,source_unchanged=True),indent=2)+'\n')
assert code==101 and 'test result: FAILED. 0 passed; 1 failed' in (old_out/'test.log').read_text()
print('original guard counterexample reproduced',flush=True)
(source/name).write_text(fixed)
initial=hashes()
assert {p for p,h in base['source_hashes'].items() if initial[p]!=h}=={name}
record=dict(production_baseline_revision='7a9771f4f',base_source_record=str(root/'line-fragment-reversal-fix-isolated/sources.json'),only_modified_sources=[name],source_hashes=initial,unchanged=True)
(out/'sources.json').write_text(json.dumps(record,indent=2)+'\n')
with tarfile.open(out/'source-delta.tar.gz','w:gz') as a:a.add(source/name,arcname=name)
shutil.copyfile(__file__,out/'reproduce.py')
commands=[
 ('format',['cargo','fmt','--all','--','--check']),
 ('arc-unit',['cargo','test','--locked','-p','hypercurve','--release','--lib','arc_bezier::tests::','--','--test-threads=2']),
 ('injectivity-certificate',['cargo','test','--locked','-p','hypercurve','--release','--lib','injectivity_certificates_own_their_domain_and_circle_chart','--','--test-threads=2']),
 ('rational-bezier-unit',['cargo','test','--locked','-p','hypercurve','--release','--lib','rational_bezier_general::tests::','--','--test-threads=2']),
 ('native-region-unit',['cargo','test','--locked','-p','hypercurve','--release','--lib','native_region_tests::','--','--test-threads=2']),
 ('public-consumers',['cargo','test','--locked','-p','hypercurve','--release','--test','hypercurve_curve_region_boolean','--test','hypercurve_bezier_fit_offset','--test','hypercurve_curve_region_promotion','--','--test-threads=2']),
 ('curve-clippy',['cargo','clippy','--locked','-p','hypercurve','--release','--all-targets','--','-D','warnings']),
 ('build-board-smoke',['cargo','test','--locked','-p','hyperdrc','--release','--lib','--no-run','--message-format=json']),
]
status=[]
def run(label,cmd):
 assert hashes()==initial
 start=time.monotonic()
 with (out/(label+'.log')).open('w') as log:code=subprocess.run(cmd,cwd=source,env=environment,stdout=log,stderr=subprocess.STDOUT).returncode
 unchanged=hashes()==initial
 status.append(dict(name=label,command=cmd,exit_code=code,elapsed_seconds=time.monotonic()-start))
 (out/'status.json').write_text(json.dumps(status,indent=2)+'\n')
 record['unchanged']=unchanged;(out/'sources.json').write_text(json.dumps(record,indent=2)+'\n')
 assert unchanged
 print(label,code,flush=True)
 if code:raise SystemExit(code)
for label,cmd in commands:run(label,cmd)
executables=[]
for line in (out/'build-board-smoke.log').read_text().splitlines():
 try:a=json.loads(line)
 except ValueError:continue
 if a.get('reason')=='compiler-artifact' and a.get('target',{}).get('name')=='hyperdrc' and a.get('executable'):executables.append(a['executable'])
assert len(executables)==1
run('full-board-smoke',['/usr/bin/time','-v','-o',str(out/'full-board-smoke-time.log'),executables[0],'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite','--exact','--test-threads=1','--nocapture'])
log=(out/'full-board-smoke.log').read_text();assert 'skipping complex project fixture test' not in log and log.count('hyperdrc: finished emit report')==2
assert all(digest(baseline/'source'/p)==h for p,h in base['source_hashes'].items())
assert (repo/name).read_bytes()==(source/name).read_bytes()
print('isolated circular support guard passed',flush=True)
