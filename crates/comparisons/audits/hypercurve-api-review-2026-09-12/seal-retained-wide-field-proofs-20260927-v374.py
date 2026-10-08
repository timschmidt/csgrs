from pathlib import Path
import hashlib,json,subprocess,sys
A=Path(__file__).resolve().parent;W=A.parent;prefix='retained-wide-field-proofs-20260927-v374';repo=W/'hyperreal'
review=json.loads((A/f'{prefix}-reviewed.json').read_text());record=review['repos']['hyperreal'];paths=list(record['paths']);mode=sys.argv[1]
def git(*args,cwd=repo):return subprocess.check_output(['git',*args],cwd=cwd)
manifest=json.loads((A/review['source_manifest']).read_text());terminal=json.loads((A/review['qualification']).read_text())
for name,sha in manifest.items():
 for root in [W,Path(terminal['source_directory']),A/'build-workspace-20260925']:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
if mode=='staged':
 assert git('rev-parse','HEAD').decode().strip()==record['parent']
 assert git('diff','--cached','--name-only').decode().splitlines()==paths
 assert not git('diff','--name-only').strip()
 for p,sha in record['paths'].items():assert hashlib.sha256(git('show',':'+p)).hexdigest()==sha,p
 (A/f'{prefix}-staged.json').write_text(json.dumps(dict(parent=record['parent'],paths=record['paths'],qualification=review['qualification'],verified_source_files=len(manifest)),indent=2)+'\n')
 print('Index verified for',len(paths),'paths')
elif mode=='committed':
 head=git('rev-parse','HEAD').decode().strip();assert git('rev-parse','HEAD^').decode().strip()==record['parent']
 assert git('show','--pretty=format:','--name-only','HEAD').decode().strip().splitlines()==paths
 assert not git('status','--porcelain=v1').strip()
 for p,sha in record['paths'].items():assert hashlib.sha256(git('show','HEAD:'+p)).hexdigest()==sha,p
 previous=json.loads((A/f'{prefix}-repositories-before.json').read_text());current={}
 for name,old in previous.items():
  h=git('rev-parse','HEAD',cwd=W/name).decode().strip();status=git('status','--porcelain=v1',cwd=W/name).decode();assert not status,name
  assert h==(head if name=='hyperreal' else old['head']),name
  current[name]=dict(head=h,status=status)
 (A/f'{prefix}-repositories-after.json').write_text(json.dumps(current,indent=2)+'\n')
 (A/f'{prefix}-committed.json').write_text(json.dumps(dict(commit=head,parent=record['parent'],paths=record['paths'],qualification=review['qualification'],verified_source_files=len(manifest),clean_repositories=len(current)),indent=2)+'\n')
 print('Committed blobs verified:',head,';',len(current),'clean repositories')
else:raise ValueError(mode)
