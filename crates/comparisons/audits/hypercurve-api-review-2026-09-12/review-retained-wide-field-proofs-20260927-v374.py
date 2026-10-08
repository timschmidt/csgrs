from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='retained-wide-field-proofs-20260927-v374'
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['qualification_complete'] and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text())
for n,h in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert hashlib.sha256((root/n).read_bytes()).hexdigest()==h,(root,n)
expected={(target,name)for target,names in r['expected_cases'].items()for name in names};actual=set();targets={}
for b in r['builds']:
 assert b['returncode']==0
 binary=Path(b['binary']['path']);assert hashlib.sha256(binary.read_bytes()).hexdigest()==b['binary']['sha256']
 cmd=b['command'];target=b['crate'] if '--lib' in cmd else cmd[cmd.index('--test')+1];targets[binary.name]=target
for c in r['cases']:
 assert c['returncode']==0;actual.add((targets[c['binary']],c['name']))
assert expected==actual and len(r['cases'])==len(actual)
assert len(r['builds'])==9 and len(r['checks'])==13
assert all(c['returncode']==0 for c in r['checks'])
counts={}
for label in ['hyperreal-full-lib','hypersolve-full-lib','hyperreal-integration','hypersolve-integration']:
 text=(A/f'{prefix}-{label}.log').read_text();counts[label]=[int(n)for n in re.findall(r'test result: ok\. (\d+) passed;',text)]
repo=W/'hyperreal';paths=['src/computable/node/quadratic_tower.rs','src/computable/node/representation.rs','src/real/arithmetic/quadratic_tower_sign.rs']
def git(*args,cwd=repo):return subprocess.check_output(['git',*args],cwd=cwd)
assert git('rev-parse','HEAD').decode().strip()=='b1beafe77ca073324770e95634df1ce09b00f2af'
assert git('diff','--name-only').decode().splitlines()==paths
assert not git('diff','--cached','--name-only').strip()
assert git('status','--porcelain=v1').decode()==''.join(' M '+p+'\n'for p in paths)
previous=json.loads((A/'initial-endpoint-root-replay-20260927-v362-repositories-after.json').read_text());repositories={}
for name,old in previous.items():
 head=git('rev-parse','HEAD',cwd=W/name).decode().strip();status=git('status','--porcelain=v1',cwd=W/name).decode();assert head==old['head'],name
 if name!='hyperreal':assert not status,name
 repositories[name]=dict(head=head,status=status)
(A/f'{prefix}-repositories-before.json').write_text(json.dumps(repositories,indent=2)+'\n')
record=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=49937,validated_files=len(manifest),selected_geometry_tests=len(actual),suite_pass_counts=counts,checks=len(r['checks']),repos={'hyperreal':dict(parent=repositories['hyperreal']['head'],paths={p:manifest['hyperreal/'+p]for p in paths})},known_unresolved=r['known_unresolved'])
(A/f'{prefix}-reviewed.json').write_text(json.dumps(record,indent=2)+'\n')
(A/f'{prefix}-commit.txt').write_text('Retain exact quadratic-field proofs across wide coefficients\n\nRemove coefficient-size gates from exact quadratic-field arithmetic, retained\nreductions and compaction. Wide intermediate values can cancel exactly and\nlater scalar queries can reuse their shared rational coefficients instead of\nreplaying numerical construction histories. Clone a reduction only when the\ncache will publish it; add no new public scalar representation.\n\nKeep the explicit binary-shift allocation guard, measuring new expansion after\ncancellation rather than rejecting already represented wide coefficients.\nCover wide nested-radical cancellation, tiny signed perturbations, exported\nproofs and small shifts; existing huge-shift and concurrent-cache cases pass.\n\nValidation: Hyperreal and Hypersolve unit/integration suites, 232 selected\nHypercurve release tests, all-target Clippy, formatting, editing fuzz, denied-\nwarning docs and Hyperbrep checks. The two full extended-fillet re-offset\nperformance cases remain unresolved and are excluded from this qualification.\n')
print('Reviewed',len(manifest),'sources,',len(actual),'geometry tests, suite counts',counts,'and',len(r['checks']),'checks')
