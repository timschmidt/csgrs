from pathlib import Path
import hashlib,json,subprocess,sys
A=Path(__file__).resolve().parent;W=A.parent;prefix='known-fillet-centers-20260928-v665'
review=json.loads((A/f'{prefix}-reviewed.json').read_text());mode=sys.argv[1]
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
manifest=json.loads((A/review['source_manifest']).read_text());terminal=json.loads((A/review['qualification']).read_text())
for name,sha in manifest.items():
 for root in [W,Path(terminal['source_directory']),A/'build-workspace-20260925']:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
if mode=='staged':
 for repo,record in review['repos'].items():
  paths=list(record['paths']);assert git(repo,'rev-parse','HEAD').decode().strip()==record['parent']
  assert git(repo,'diff','--cached','--name-only').decode().splitlines()==paths
  assert not git(repo,'diff','--name-only').strip()
  for p,sha in record['paths'].items():assert hashlib.sha256(git(repo,'show',':'+p)).hexdigest()==sha,(repo,p)
 (A/f'{prefix}-staged.json').write_text(json.dumps(dict(repos=review['repos'],qualification=review['qualification'],verified_source_files=len(manifest)),indent=2)+'\n')
 print('Index verified for',sum(len(r['paths'])for r in review['repos'].values()),'paths')
elif mode=='committed':
 commits={}
 for repo,record in review['repos'].items():
  head=git(repo,'rev-parse','HEAD').decode().strip();assert git(repo,'rev-parse','HEAD^').decode().strip()==record['parent']
  assert git(repo,'show','--pretty=format:','--name-only','HEAD').decode().strip().splitlines()==list(record['paths'])
  assert not git(repo,'status','--porcelain=v1').strip()
  for p,sha in record['paths'].items():assert hashlib.sha256(git(repo,'show','HEAD:'+p)).hexdigest()==sha,(repo,p)
  commits[repo]=dict(commit=head,**record)
 previous=json.loads((A/f'{prefix}-repositories-before.json').read_text());current={}
 for name,old in previous.items():
  head=git(name,'rev-parse','HEAD').decode().strip();status=git(name,'status','--porcelain=v1').decode();assert not status,name
  assert head==(commits[name]['commit']if name in commits else old['head']),name
  current[name]=dict(head=head,status=status)
 (A/f'{prefix}-repositories-after.json').write_text(json.dumps(current,indent=2)+'\n')
 (A/f'{prefix}-committed.json').write_text(json.dumps(dict(repos=commits,qualification=review['qualification'],verified_source_files=len(manifest),clean_repositories=len(current)),indent=2)+'\n')
 print('Committed blobs verified:',{k:v['commit']for k,v in commits.items()},';',len(current),'clean repositories')
else:raise ValueError(mode)
