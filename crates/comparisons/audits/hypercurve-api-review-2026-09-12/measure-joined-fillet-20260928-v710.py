from pathlib import Path
import hashlib,json,os,signal,statistics,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='joined-fillet-comparison-20260928-v710'
assert (A/'overlap-orientation-20260928-v709-committed.json').exists()
inputs=json.loads((A/'joined-fillet-timing-inputs-v710.json').read_text());test=inputs['test'];cases=[];sources={};binaries={}
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
for label,parent in [('historical','finite-domain-ownership-20260927-v527'),('current','native-path-shims-20260928-v707')]:
 r=json.loads((A/f'{parent}-terminal.json').read_text());assert r['qualification_complete'] and r['all_processes_reaped']
 sources[label]=(Path(r['source_directory']),json.loads((A/r['source_manifest']).read_text()))
 found=[b['binaries']['hypercurve']for b in r['builds']if 'hypercurve'in b.get('binaries',{})];assert len(found)==1;binaries[label]=found[0]
 def fixture(name):
  s=(sources[label][0]/'hypercurve/src/curve_fillet.rs').read_text();start=s.index('fn '+name+'(');brace=s.index('{',start);level=1;end=brace+1
  while level:level+=(s[end]=='{')-(s[end]=='}');end+=1
  return s[start:end]
 for name,sha in inputs['fixtures'][parent].items():assert hashlib.sha256(fixture(name).encode()).hexdigest()==sha
 if label=='historical':old_test=fixture(test.split('::')[-1])
 else:new_test=fixture(test.split('::')[-1])
# The sole test-body change expands failure diagnostics; the success path's
# geometry inputs, operation sequence, correctness assertions and replay match.
start=old_test.index('                assert!(matches!(\n');end=old_test.index('                for excluded',start)
new_start=new_test.index('                let unconstrained =\n');new_end=new_test.index('                for excluded',new_start)
assert old_test[:start]==new_test[:new_start] and old_test[end:]==new_test[new_end:]
assert old_test[start:end].count('source.fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy)')==new_test[new_start:new_end].count('source.fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy)')==1
for n in ['joined_parallel_path','same','q']:assert len({v[n]for v in inputs['fixtures'].values()})==1
current=json.loads((A/'overlap-orientation-20260928-v709-terminal.json').read_text());manifest=json.loads((A/current['source_manifest']).read_text())
def verify():
 for root,files in sources.values():
  for name,sha in files.items():assert digest(root/name)==sha,(root,name)
 for name,sha in manifest.items():
  for root in [W,Path(current['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
 for label,binary in binaries.items():assert digest(Path(binary['path']))==binary['sha256'],label
report=dict(binaries=binaries,cases=cases,qualification_complete=False,all_processes_reaped=False,scope='Three serial paired runs; geometry test inputs/assertions match, only failure diagnostics differ. Current pinned binary predates the verified orientation-only rename.')
def save():(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));verify();save();code=0
try:
 for pair,order in enumerate([['historical','current'],['current','historical'],['historical','current']]):
  for label in order:
   log=A/f'{prefix}-{pair}-{label}.log';command=[binaries[label]['path'],'--exact',test,'--nocapture','--test-threads=1'];start=time.monotonic()
   with log.open('w')as out:
    process=subprocess.Popen(command,cwd=sources[label][0]/'hypercurve',stdout=out,stderr=subprocess.STDOUT,env=env,start_new_session=True)
    try:rc=process.wait(timeout=90)
    except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait();rc=124
    except BaseException:os.killpg(process.pid,signal.SIGKILL);process.wait();raise
   passed=rc==0 and 'test result: ok. 1 passed;'in log.read_text();cases.append(dict(pair=pair,label=label,command=command,log=log.name,elapsed_seconds=time.monotonic()-start,returncode=rc,passed=passed));save();print(label,pair,round(cases[-1]['elapsed_seconds'],3),'passed',passed,flush=True)
   if not passed:raise RuntimeError(label+' comparison failed')
 report['medians']={label:statistics.median(c['elapsed_seconds']for c in cases if c['label']==label)for label in binaries};report['qualification_complete']=True
except Exception as error:code=1;report['failure']=str(error);print(str(error),flush=True)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
