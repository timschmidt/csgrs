from pathlib import Path
import hashlib, json, re, subprocess
A=Path(__file__).resolve().parent
W=A.parent
prefix='retained-fillet-components-full-20260926-v219'
report=json.loads((A/f'{prefix}-terminal.json').read_text())
assert report['qualification_complete'] and report['all_processes_reaped']
assert report['hypercurve_build_returncode']==0
assert len(report['checks'])==4 and all(row['returncode']==0 for row in report['checks'])
assert len(report['cases'])==len(report['selection'])==report['hypercurve_passed']
assert all(row['returncode']==0 for row in report['cases'])
for row in report['cases']:
 assert re.search(r'test result: ok\. 1 passed; 0 failed; 0 ignored;', (A/row['log']).read_text()),row['name']
manifest=json.loads((A/report['source_manifest']).read_text())
for name,sha in manifest.items():
 for root in (W,Path(report['source_directory']),Path(report['build_source_directory'])):
  assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(str(root),name)
for binary in report['binaries'].values():
 assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest()==binary['sha256']
repos={}
for repo in sorted(W.iterdir()):
 if (repo/'.git').exists():
  repos[repo.name]=dict(head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),
   status=subprocess.check_output(['git','status','--short'],cwd=repo,text=True))
  assert not subprocess.check_output(['git','diff','--cached','--name-only'],cwd=repo),repo.name
  if repo.name!='hypercurve':assert not repos[repo.name]['status'],repo.name
for repo,head in report['parents'].items():assert repos[repo]['head']==head
files=['src/bezier_offset.rs','src/curve.rs','src/curve_intersection.rs','src/curve_parameter_component.rs','src/curve_support_intersection.rs']
changed=set(subprocess.check_output(['git','diff','--name-only','HEAD','-z'],cwd=W/'hypercurve').decode().strip('\0').split('\0'))
changed.update(subprocess.check_output(['git','ls-files','--others','--exclude-standard','-z'],cwd=W/'hypercurve').decode().strip('\0').split('\0'))
changed.discard('')
assert changed==set(files),changed
subprocess.run(['git','diff','--check'],cwd=W/'hypercurve',check=True)
qualified=dict(parent=report['parents']['hypercurve'],qualification=f'{prefix}-terminal.json',
 all_processes_reaped=True,outer_session_id=67104,outer_exit_code=0,
 source_manifest=report['source_manifest'],files={file:manifest['hypercurve/'+file] for file in files},repositories=repos)
(A/f'{prefix}-qualified.json').write_text(json.dumps(qualified,indent=2)+'\n')
print('Qualified',len(manifest),'inputs;',len(report['binaries']),'binaries;',len(report['cases']),'release cases;',len(repos),'repositories audited.')
print('Ready to stage:',', '.join(files))
