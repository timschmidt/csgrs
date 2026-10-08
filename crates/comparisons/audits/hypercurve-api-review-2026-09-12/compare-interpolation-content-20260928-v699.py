from pathlib import Path
import hashlib,json,os,signal,subprocess,time,statistics
A=Path(__file__).resolve().parent;W=A.parent;prefix='interpolation-content-comparison-20260928-v699'
name='bezier_offset::structural_overlap_trace_regression::monotone_reparameterization_preserves_structural_pair_crossings_strict'
records={label:json.loads((A/f'{run}-terminal.json').read_text())for label,run in [('baseline','projection-overlap-20260928-v695'),('candidate','interpolation-content-20260928-v697')]}
for label,run in [('baseline','projection-overlap-20260928-v695'),('candidate','interpolation-content-20260928-v697')]:assert json.loads((A/f'{run}-reaped.json').read_text())['outer_exit_code']==0
manifests={label:json.loads((A/record['source_manifest']).read_text())for label,record in records.items()}
assert {n for n in manifests['baseline']if manifests['baseline'][n]!=manifests['candidate'][n]}=={'hypersolve/src/curve_resultant.rs'}
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
binaries={label:next(build['binaries']['hypercurve']for build in record['builds']if 'hypercurve'in build['binaries'])for label,record in records.items()}
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
report=dict(cases=[],all_processes_reaped=False,comparison_complete=False,workload=name,protocol='Three serial paired runs, alternating pair order. Compare full distributions and median; no additional build or concurrent qualification.')
def verify():
 for label,manifest in manifests.items():
  for n,sha in manifest.items():
   assert digest(Path(records[label]['source_directory'])/n)==sha,(label,n)
  assert digest(Path(binaries[label]['path']))==binaries[label]['sha256'],label
 for n,sha in manifests['baseline'].items():assert digest(W/n)==sha,n
 for n,sha in manifests['candidate'].items():assert digest(A/'build-workspace-20260925'/n)==sha,n
 assert digest(A/'interpolation-content-candidate-v696/hypersolve/src/curve_resultant.rs')==manifests['candidate']['hypersolve/src/curve_resultant.rs']
def save():(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
verify();save();code=0
try:
 for pair in range(3):
  for label in (['baseline','candidate']if pair%2==0 else ['candidate','baseline']):
   log=A/f'{prefix}-{pair+1}-{label}.log';start=time.monotonic()
   with log.open('w')as out:
    process=subprocess.Popen([binaries[label]['path'],'--exact',name,'--nocapture','--test-threads=1'],cwd=Path(records[label]['source_directory'])/'hypercurve',stdout=out,stderr=subprocess.STDOUT,env=env,start_new_session=True)
    try:rc=process.wait(timeout=180)
    except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait();rc=124
    except BaseException:os.killpg(process.pid,signal.SIGKILL);process.wait();raise
   case=dict(pair=pair+1,label=label,returncode=rc,elapsed_seconds=time.monotonic()-start,log=log.name,passed=rc==0 and 'test result: ok. 1 passed;'in log.read_text())
   report['cases'].append(case);save();print(label,pair+1,rc,round(case['elapsed_seconds'],3),flush=True);assert case['passed']
 report['summary']={label:dict(samples=[c['elapsed_seconds']for c in report['cases']if c['label']==label],median_seconds=statistics.median(c['elapsed_seconds']for c in report['cases']if c['label']==label))for label in records}
 report['comparison_complete']=True
 print(json.dumps(report['summary']),flush=True)
except Exception as error:code=1;report['failure']=str(error);print('comparison failed',type(error).__name__,flush=True)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
