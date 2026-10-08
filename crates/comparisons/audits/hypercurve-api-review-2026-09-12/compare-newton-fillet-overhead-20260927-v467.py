from pathlib import Path
import hashlib,json,os,re,signal,statistics,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='newton-fillet-paired-20260927-v467'
labels={'before':'normalized-root-replay-20260927-v454','after':'certified-root-newton-20260927-v462'}
inputs={};binaries={}
for label,source in labels.items():
 r=json.loads((A/f'{source}-terminal.json').read_text());assert r['qualification_complete']and r['all_processes_reaped']
 manifest=json.loads((A/r['source_manifest']).read_text());archive=Path(r['source_directory'])
 for name,sha in manifest.items():assert hashlib.sha256((archive/name).read_bytes()).hexdigest()==sha,(label,name)
 selected=next(b for b in r['builds']if '--lib'in b['command']);binary=Path(selected['binary']['path']);assert hashlib.sha256(binary.read_bytes()).hexdigest()==selected['binary']['sha256']
 inputs[label]=dict(source_manifest=r['source_manifest'],source_directory=str(archive),binary=selected['binary']);binaries[label]=binary
manifest=json.loads((A/inputs['after']['source_manifest']).read_text())
def verify():
 for name,sha in manifest.items():assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
 for label,record in inputs.items():assert hashlib.sha256(binaries[label].read_bytes()).hexdigest()==record['binary']['sha256']
def save():(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
report=dict(inputs=inputs,cases=[],all_processes_reaped=False,complete=False)
names=['curve::curve_fillet::tests::joined_path_selects_and_replays_a_continuous_fillet_family','curve::curve_fillet::tests::nonlinear_linear_fillet_components_retain_unique_contacts_and_tangents']
verify();save();code=0
try:
 for repetition in range(3):
  for name in names:
   for label in (['before','after']if repetition%2==0 else ['after','before']):
    log=A/f'{prefix}-case-{len(report["cases"]):03}.log';started=time.monotonic()
    with log.open('w')as out:
     process=subprocess.Popen([str(binaries[label]),'--exact',name,'--nocapture','--test-threads=1'],cwd=W,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
     try:returncode=process.wait(timeout=60)
     except BaseException:os.killpg(process.pid,signal.SIGKILL);process.wait();raise
    row=dict(label=label,name=name,repetition=repetition,returncode=returncode,elapsed_seconds=time.monotonic()-started,log=log.name);report['cases'].append(row);save();print(row,flush=True)
    assert returncode==0 and re.search(r'test result: ok\. 1 passed;',log.read_text())
 report['medians']={name:{label:statistics.median(c['elapsed_seconds']for c in report['cases']if c['name']==name and c['label']==label)for label in labels}for name in names}
 report['complete']=True
except Exception as error:code=1;report['failure']=str(error)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
