from pathlib import Path
import difflib, hashlib, json, os, shutil, subprocess, time
A=Path(__file__).resolve().parent
W=A.parent
prefix='selected-frame-import-20260925-v94-candidate'
prior_prefix='selected-frame-import-20260925-v92-candidate'
prior=json.loads((A/f'{prior_prefix}-terminal.json').read_text())
assert prior['all_processes_reaped'] and prior['all_sources_unchanged']
assert len(prior['cases'])==252
old_test='direct_bezier_pair_fillet_materializes_both_incident_extensions'
new_test='direct_bezier_pair_fillet_retains_both_incident_extensions'
assert [(c['name'],c['returncode']) for c in prior['cases'] if c['returncode']!=0]==[(old_test,101)]
guard=json.loads((A/prior['workspace_guard']).read_text())
old_manifest=json.loads((A/prior['source_manifest']).read_text())
old_root=A/'source-archives'/prior_prefix
root=A/'source-archives'/prefix
assert not root.exists()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for name, expected in old_manifest.items():
    assert sha(old_root/name)==expected,name
    assert sha(W/name)==guard[name],name
    destination=root/name
    destination.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(old_root/name,destination)
public_file='hypercurve/tests/hypercurve_curve.rs'
shutil.copy2(W/public_file,root/public_file)
manifest={name:sha(root/name) for name in guard}
assert [name for name in manifest if manifest[name]!=old_manifest[name]]==[public_file]
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/'hypercurve',text=True).strip()
assert head==prior['parent']
public_path='tests/hypercurve_curve.rs'
old=subprocess.check_output(['git','show',f'HEAD:{public_path}'],cwd=W/'hypercurve',text=True)
new=(W/public_file).read_text()
patch=(A/'selected-frame-import-20260925-v92.patch').read_text()+''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='a/'+public_path,tofile='b/'+public_path))
(A/'selected-frame-import-20260925-v94.patch').write_text(patch)
build_root=A/'build-workspace-20260925'
for name in manifest:
    source,destination=root/name,build_root/name
    if not destination.exists() or source.read_bytes()!=destination.read_bytes():shutil.copy2(source,destination)
    assert source.stat().st_ino!=destination.stat().st_ino
repo=build_root/'hypercurve'
# Copied snapshots preserve earlier mtimes; explicitly invalidate the changed test.
os.utime(repo/public_path,None)
def verify():
    assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/'hypercurve',text=True).strip()==head
    for name,expected in manifest.items():
        assert sha(root/name)==expected,name
        assert sha(build_root/name)==expected,name
        assert sha(W/name)==guard[name],name
    for binary in prior['binaries'].values():assert sha(Path(binary['path']))==binary['sha256']
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
toolchain=Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin')
report=dict(parent=head,selected_files=['src/bezier_offset.rs',public_path],workspace_changed=prior['workspace_changed'],workspace_guard=prior['workspace_guard'],source_manifest=f'{prefix}-sources.json',source_directory=str(root),build_source_directory=str(build_root),patch='selected-frame-import-20260925-v94.patch',reused_from=f'{prior_prefix}-terminal.json',unchanged_inputs_except=[public_file],checks=[],binaries=prior['binaries'].copy(),cases=[dict(c,reused_from=f'{prior_prefix}-terminal.json') for c in prior['cases'] if c['returncode']==0],selection=[(t,new_test if n==old_test else n) for t,n in prior['selection']],all_processes_reaped=False)
def save():
    verify()
    report['all_sources_unchanged']=True
    (A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
def run(label,command,limit):
    log=A/f'{prefix}-{label}.log'
    start=time.monotonic()
    with log.open('w') as out:
        try:code=subprocess.run(command,cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=limit).returncode
        except subprocess.TimeoutExpired:code='timeout'
    verify()
    return dict(label=label,command=command,returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name,ignored='1 ignored' in log.read_text())
checks=[('fmt',[str(toolchain/'rustfmt'),'--edition','2024','--check',*report['selected_files']],60)]
checks.extend((f'clippy-{i}',[str(toolchain/'cargo'),'clippy','--all-targets',feature,'--locked','--offline','--','-D','warnings'],1200) for i,feature in enumerate(['--all-features','--no-default-features']))
for label,command,limit in checks:
    row=run(label,command,limit)
    report['checks'].append(row)
    print(label,row['returncode'],flush=True)
    if row['returncode']!=0:
        report['all_processes_reaped']=True
        save()
        print((A/row['log']).read_text()[-2500:],flush=True)
        raise SystemExit(1)
    save()
command=[str(toolchain/'cargo'),'test','--release','--all-features','--test','hypercurve_curve','--no-run','--message-format=json','--locked','--offline']
with (A/f'{prefix}-build.jsonl').open('w') as out,(A/f'{prefix}-build.log').open('w') as err:
    code=subprocess.run(command,cwd=repo,env=env,stdout=out,stderr=err,timeout=1200).returncode
report['build_returncode']=code
verify()
if code:
    report['all_processes_reaped']=True
    save()
    raise SystemExit(1)
artifacts=[json.loads(line) for line in (A/f'{prefix}-build.jsonl').read_text().splitlines()]
artifact=next(row for row in artifacts if row.get('reason')=='compiler-artifact' and row['target']['name']=='hypercurve_curve' and row.get('executable'))
assert not artifact['fresh']
binary=A/f'{prefix}-hypercurve_curve'
shutil.copy2(artifact['executable'],binary)
report['binaries']['hypercurve_curve']=dict(path=str(binary),sha256=sha(binary))
report['active_case']=['hypercurve_curve',new_test]
save()
row=run('geometric-fillet-oracle',[str(binary),'--exact',new_test,'--nocapture','--test-threads=1','--color','never'],75)
row.update(target='hypercurve_curve',name=new_test)
report['cases'].append(row)
report.pop('active_case')
report['all_processes_reaped']=True
save()
print('geometric fillet oracle',row['returncode'],(A/row['log']).read_text()[-2500:],flush=True)
print('Qualified cases:',len(report['cases']),'; unchanged cases reused:',len(report['cases'])-1,'; every child reaped',flush=True)
raise SystemExit(int(row['returncode']!=0))
