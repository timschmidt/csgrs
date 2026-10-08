from pathlib import Path
import hashlib,json,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='selected-generator-order-20260927-v348'
qualification=json.loads((A/f'{prefix}-terminal.json').read_text())
assert qualification['qualification_complete'] and qualification['all_processes_reaped']
manifest=json.loads((A/f'{prefix}-sources.json').read_text())
for name,sha in manifest.items():
 for root in [W,Path(qualification['source_directory']),A/'build-workspace-20260925']:
  assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
targets={}
for build in qualification['builds']:
 assert build['returncode']==0
 binary=Path(build['binary']['path']);assert hashlib.sha256(binary.read_bytes()).hexdigest()==build['binary']['sha256']
 command=build['command'];target='hypercurve' if '--lib' in command else command[command.index('--test')+1]
 targets[binary.name]=target
actual={(targets[c['binary']],c['name']) for c in qualification['cases']}
expected={(target,name) for target,names in qualification['expected_cases'].items() for name in names}
assert actual==expected,(expected-actual,actual-expected)
assert len(actual)==len(qualification['cases']) and all(c['returncode']==0 for c in qualification['cases'])
assert len(qualification['builds'])==7
assert len(qualification['checks'])==6 and all(c['returncode']==0 for c in qualification['checks'])
repo=W/'hypercurve'
def git(*args):return subprocess.check_output(['git',*args],cwd=repo)
assert git('rev-parse','HEAD').decode().strip()=='68e1f2b01bc9741dd77e410e9f9ba6b4099c9499'
assert git('diff','--name-only').decode().splitlines()==['src/bezier_offset.rs']
assert not git('diff','--cached','--name-only').strip()
assert git('status','--porcelain=v1').decode()==' M src/bezier_offset.rs\n'
record=dict(source_manifest=f'{prefix}-sources.json',qualification=f'{prefix}-terminal.json',outer_session_reaped=24412,validated_files=len(manifest),passed_tests=len(actual),static_checks=len(qualification['checks']),reused_qualification='selected-generator-order-20260927-v347-terminal.json',reused_outer_session_reaped=56650,reused_tests=4,repos={'hypercurve':{'parent':git('rev-parse','HEAD').decode().strip(),'paths':{'src/bezier_offset.rs':manifest['hypercurve/src/bezier_offset.rs']}}},known_unresolved_not_requalified=qualification['known_unresolved_not_requalified'])
(A/f'{prefix}-reviewed.json').write_text(json.dumps(record,indent=2)+'\n')
(A/f'{prefix}-commit.txt').write_text('Reuse coefficient generators when ordering local roots\n\nCompare a selected polynomial root directly with an existing coefficient\nfield generator, retaining exact boundary separation and the defining\npolynomial sign inside its isolator. Share the existing-axis import and\navoid rebuilding the field or replaying a higher-degree native relation.\n\nCover close distinct roots, equality, both orientations and policies, and\nduplicate selected axes. The extended nonzero parallel-loop chamfer now\nfinishes in 113 seconds after previously exceeding 240 seconds.\n\nValidation: '+str(len(actual))+' selected release tests across seven binaries; all-target\nClippy with all features and without defaults, formatting, editing fuzz\ncompilation, denied-warning docs, and Hyperbrep all-target/all-feature check.\n')
print('Reviewed',len(manifest),'source inputs,',len(actual),'passing tests and',len(qualification['checks']),'checks; ready to stage one file.')
