from pathlib import Path
import hashlib,json,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='overlap-orientation-20260928-v709'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_checks'] and r['qualification_complete'] and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2047
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['checks'])==6 and all(c['returncode']==0 for c in r['checks'])
promotion=json.loads((A/'overlap-orientation-promotion-v708.json').read_text());changed=set(promotion['promoted']);assert len(changed)==12
prior=json.loads((A/'native-path-shims-20260928-v707-terminal.json').read_text());old_manifest=json.loads((A/prior['source_manifest']).read_text())
assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:assert digest(A/promotion['candidate']/name)==manifest[name]==promotion['promoted'][name]
for name in manifest:
 if name.endswith('.rs'):assert b'RationalBezierOverlapOrientation2'not in (W/name).read_bytes(),name
subprocess.run(['python3',str(A/'verify-overlap-orientation-v708.py'),str(W)],check=True)
previous=json.loads((A/'native-path-shims-20260928-v707-repositories-after.json').read_text());repositories={};repos={}
for name,old in previous.items():
 head=git(name,'rev-parse','HEAD').decode().strip();status=git(name,'status','--porcelain=v1').decode();assert head==old['head'],name
 paths=sorted(n.removeprefix(name+'/')for n in changed if n.startswith(name+'/'))
 if paths:
  assert status==''.join(' M '+p+'\n'for p in paths),(name,status)
  assert not git(name,'diff','--cached','--name-only').strip();subprocess.run(['git','diff','--check'],cwd=W/name,check=True)
  repos[name]=dict(parent=head,paths={p:manifest[name+'/'+p]for p in paths})
 else:assert not status,name
 repositories[name]=dict(head=head,status=status)
(A/f'{prefix}-repositories-before.json').write_text(json.dumps(repositories,indent=2)+'\n')
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),checks=6,repos=repos,scope='One generic orientation type for all exact overlap representations; source transform is identifier/ownership/formatting only',unresolved='Full implementation goal remains active; no performance claim or additional geometry regression execution for this mechanical migration.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
(A/f'{prefix}-commit.txt').write_text('''Use one generic curve overlap orientation vocabulary

Rename RationalBezierOverlapOrientation2 to CurveOverlapOrientation2 and move its definition/export to the common curve intersection API. Same and Reversed already describe rational spans, retained circles, analytic parallels and generic curve intersections equally. Update all controlled callers and remove the old name without a compatibility alias.

Validation: exact source transformation checked for all twelve files, including formatter effects; no geometry or evidence logic changed. All-target Clippy under all/no-default features, formatting, three affected fuzz targets, documentation and downstream Hyperbrep Clippy pass. Existing mathematical tests are retained.
''')
print('Reviewed twelve mechanical source migrations, six checks, 2047 sources and 30 repositories')
