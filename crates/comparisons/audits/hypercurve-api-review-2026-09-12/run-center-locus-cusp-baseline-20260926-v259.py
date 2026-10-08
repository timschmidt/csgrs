from pathlib import Path
import hashlib,json,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='center-locus-cusp-baseline-20260926-v259';prior=json.loads((A/'center-locus-cusp-baseline-20260926-v258-terminal.json').read_text());manifest=json.loads((A/prior['source_manifest']).read_text());guard={n:s for n,s in json.loads((A/prior['workspace_guard']).read_text()).items()if not n.startswith('fiber-probe/')};archive=Path(prior['source_directory']);binary=Path(prior['binary']['path'])
def verify():
 for n,s in guard.items():assert hashlib.sha256((W/n).read_bytes()).hexdigest()==s,n
 for n,s in manifest.items():assert hashlib.sha256((archive/n).read_bytes()).hexdigest()==s,n
 assert hashlib.sha256(binary.read_bytes()).hexdigest()==prior['binary']['sha256']
verify();name='bezier_offset::conversion_tests::selected_parallel_normal_circle_accepts_a_center_locus_cusp';names=subprocess.check_output([str(binary),'--list'],text=True);assert name+': test' in names
cmd=[str(binary),'--exact',name,'--nocapture','--test-threads=1'];log=A/f'{prefix}.log';start=time.monotonic()
with log.open('w')as out:code=subprocess.run(cmd,cwd=archive/'hypercurve',stdout=out,stderr=subprocess.STDOUT,timeout=30).returncode
verify();text=log.read_text();assert code==101 and 'a center-locus cusp retains a regular source frame' in text and '1 failed' in text
report=dict(source_manifest=prior['source_manifest'],source_directory=prior['source_directory'],workspace_guard=prior['workspace_guard'],binary=prior['binary'],command=cmd,name=name,log=log.name,returncode=code,elapsed_seconds=time.monotonic()-start,expected_failure_reproduced=True,all_processes_reaped=True,all_sources_unchanged=True)
(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n');print(text[-2500:]);print('Expected constructor rejection reproduced.')
