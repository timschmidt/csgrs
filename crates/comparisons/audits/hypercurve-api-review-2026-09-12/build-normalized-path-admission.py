from pathlib import Path
import io,json,os,shutil,subprocess,tarfile,time,hashlib,sys
w=Path('/home/tim/Documents/GitHub/workspace');a=w/'hypercurve-api-review-2026-09-12';root=Path('/tmp/hypercurve-region-admission-qualification');prefix='normalized-path-admission-candidate'+sys.argv[1]
if not root.exists():
 root.mkdir()
 commits={}
 for name in ['hypercurve','hyperbrep','hypersolve']:
  commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=w/name,text=True).strip();commits[name]=commit;dest=root/name;dest.mkdir()
  with tarfile.open(fileobj=io.BytesIO(subprocess.check_output(['git','archive',commit],cwd=w/name))) as archive:archive.extractall(dest,filter='data')
 for name in ['hyperreal','hyperlattice','hyperlimit','hypertri']:(root/name).symlink_to(Path('/tmp/hypercurve-pruning-qualification')/name,target_is_directory=True)
 (a/'normalized-path-admission-snapshot.json').write_text(json.dumps(commits,indent=2)+'\n')
changed=subprocess.check_output(['git','diff','--name-only'],cwd=w/'hypercurve',text=True).splitlines();selected=[name for name in changed if name!='src/bezier_offset.rs'];assert selected and all(name.startswith(('src/', 'tests/')) for name in selected),selected
archive=a/(prefix+'-sources');archive.mkdir()
for name in selected:
 shutil.copy2(w/'hypercurve'/name,root/'hypercurve'/name);dest=archive/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(w/'hypercurve'/name,dest)
manifest={name:hashlib.sha256((root/'hypercurve'/name).read_bytes()).hexdigest() for name in selected}
(a/(prefix+'-source.patch')).write_bytes(subprocess.check_output(['git','diff','--',*selected],cwd=w/'hypercurve'))
settings=json.loads((a/'finite-point-inverse-domains-build-settings.json').read_text());env=dict(os.environ,**settings);records=[]
for kind,args in [('check',['check','--all-targets','--no-default-features','--locked','--offline']),('test-build',['test','--release','--all-features','--locked','--offline','--lib','--test','hypercurve_curve_region_promotion','--test','hypercurve_exact_closure','--test','hypercurve_curve_region_boolean','--test','hypercurve_curve_region_boolean_fuzz','--test','hypercurve_svg','--test','hypercurve_bbox','--test','hypercurve_pcb_boolean_regressions','--no-run','--message-format=json'])]:
 stem=prefix+'-'+kind;cmd=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo',*args];start=time.monotonic()
 with (a/(stem+'.log')).open('w') as err,(a/(stem+'.jsonl')).open('w') as out:code=subprocess.run(cmd,cwd=root/'hypercurve',env=env,stdout=out if kind=='test-build' else err,stderr=err,timeout=900).returncode
 row=dict(command=cmd,returncode=code,elapsed_seconds=time.monotonic()-start);records.append(row);(a/(stem+'.json')).write_text(json.dumps(row,indent=2)+'\n');print(kind,code,round(row['elapsed_seconds'],2),flush=True)
 if code: print((a/(stem+'.log')).read_text()[-8000:],flush=True);break
for name,sha in manifest.items():assert hashlib.sha256((w/'hypercurve'/name).read_bytes()).hexdigest()==sha==hashlib.sha256((root/'hypercurve'/name).read_bytes()).hexdigest()
(a/(prefix+'.json')).write_text(json.dumps(dict(builds=records,source_files=manifest,excluded_working_source='src/bezier_offset.rs',excluded_scope='Unfinished mapped-point consolidation; this snapshot uses the committed offset implementation.'),indent=2)+'\n')
assert all(row['returncode']==0 for row in records)
