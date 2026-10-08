from pathlib import Path
import hashlib,json,statistics,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='composition-evidence-20260928-v711';archive=A/'source-archives'/prefix;mirror=A/'build-workspace-20260925'
assert not archive.exists()
assert json.loads((A/'joined-fillet-comparison-20260928-v710-reaped.json').read_text())['outer_exit_code']==0
prior=json.loads((A/'overlap-orientation-20260928-v709-terminal.json').read_text());old_manifest=json.loads((A/prior['source_manifest']).read_text())
record='hypercurve/benchmarks/checkpoints/2026-09-28-exact-composition-replay.json';changed={'hypercurve/PERFORMANCE.md',record};manifest={}
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
data=json.loads((W/record).read_text());assert len(data['workloads'])==2
for workload in data['workloads']:
 receipt=json.loads((A/workload['receipt']).read_text());assert receipt['all_processes_reaped'] and all(c['passed']and c['returncode']==0 for c in receipt['cases'])and len(receipt['cases'])==6
 for c in receipt['cases']:assert 'test result: ok. 1 passed;'in (A/c['log']).read_text()
 for label,dataset in workload['datasets'].items():
  samples=[c['elapsed_seconds']for c in receipt['cases']if c['label']==label];assert samples==dataset['samples_seconds']and statistics.median(samples)==dataset['median_seconds']
  source=json.loads((A/(dataset['source_snapshot']+'-terminal.json')).read_text());assert digest(A/source['source_manifest'])==dataset['source_manifest_sha256']
  binary=next(b['binaries']['hypercurve']for b in source['builds']if 'hypercurve'in b['binaries']);assert dataset['binary_filename']==Path(binary['path']).name and dataset['binary_sha256']==binary['sha256']==digest(Path(binary['path']))
  for name,sha in json.loads((A/source['source_manifest']).read_text()).items():assert digest(Path(source['source_directory'])/name)==sha
for name in sorted(set(old_manifest)|{record}):
 bytes=(W/name).read_bytes();manifest[name]=hashlib.sha256(bytes).hexdigest()
 if name not in changed:assert manifest[name]==old_manifest[name],name
 target=archive/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(bytes)
 target=mirror/name;target.parent.mkdir(parents=True,exist_ok=True)
 if not target.exists()or target.read_bytes()!=bytes:target.write_bytes(bytes)
assert len(manifest)==2048
for name,sha in manifest.items():
 for root in [W,archive,mirror]:assert digest(root/name)==sha,(root,name)
previous=json.loads((A/'overlap-orientation-20260928-v709-repositories-after.json').read_text());repositories={};repos={}
for name,old in previous.items():
 head=git(name,'rev-parse','HEAD').decode().strip();status=git(name,'status','--porcelain=v1').decode();assert head==old['head'],name
 if name=='hypercurve':
  assert status==' M PERFORMANCE.md\n?? benchmarks/checkpoints/2026-09-28-exact-composition-replay.json\n',status
  assert not git(name,'diff','--cached','--name-only').strip();subprocess.run(['git','diff','--check'],cwd=W/name,check=True)
  repos[name]=dict(parent=head,paths={p.removeprefix('hypercurve/'):manifest[p]for p in sorted(changed)})
 else:assert not status,name
 repositories[name]=dict(head=head,status=status)
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
terminal=dict(normal_production_documentation=True,source_manifest=f'{prefix}-sources.json',source_directory=str(archive),all_processes_reaped=True,qualification_complete=True,checks=[],cases=[],builds=[],evidence_runs=12,note='Documentation only; audited prior exact test logs, timings, source snapshots and executable hashes. No repeated build/test.')
(A/f'{prefix}-terminal.json').write_text(json.dumps(terminal,indent=2)+'\n')
(A/f'{prefix}-repositories-before.json').write_text(json.dumps(repositories,indent=2)+'\n')
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=terminal['source_manifest'],validated_files=len(manifest),repos=repos,scope='Record matched exact-composition performance measurements and resolve the historical joined-fillet timing debt for this unchanged workload')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
(A/f'{prefix}-commit.txt').write_text('''Record matched exact composition replay measurements

Preserve all twelve passing runs, source snapshot identifiers and four executable hashes for joined continuous-fillet replay and degree-six offset overlap. The joined regression now measures 0.314 seconds median against 3.018 seconds for the historical passing baseline; document the 68-request workload and its unchanged exactness assertions. Record the separate 47.878-to-45.834-second common-content interpolation comparison.

The evidence describes full regression workloads and does not claim uniform speedups or isolate every intervening fillet change. Documentation-only validation audits original logs, timings, source hashes and pinned binaries; no redundant build or test rerun.
''')
print('Reviewed two documentation files, twelve retained passing timing runs, four binaries and 2048 current source files')
