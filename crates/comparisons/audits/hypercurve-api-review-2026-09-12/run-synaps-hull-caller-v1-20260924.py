from pathlib import Path
import base64,hashlib,json,os,shutil,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;repo=W/'synaps-cad';prefix='synaps-hull-caller-20260924-v1'
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
rustc='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc'
bindings=json.loads((A/'synaps-hull-caller-20260924-before-sources.json').read_text())
for name,old in bindings.items():
    digest=hashlib.sha256((W/name).read_bytes()).hexdigest()
    assert name=='synaps-cad/src/compiler/evaluator/booleans.rs' or digest==old,name
    bindings[name]=digest
for p in [A/'synaps-hull-caller-20260924-render.rs',*sorted((A/'synaps-hull-caller-20260924-visual').glob('*.scad'))]:
    bindings[str(p.relative_to(W))]=hashlib.sha256(p.read_bytes()).hexdigest()
(A/f'{prefix}-sources.json').write_text(json.dumps(bindings,indent=2)+'\n')
def verify():
    for name,sha in bindings.items(): assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
verify();checks=[];rows=[]
def finish(**extra):
    verify();(A/f'{prefix}-terminal.json').write_text(json.dumps(dict(checks=checks,cases=rows,all_sources_unchanged=True,all_processes_reaped=True,**extra),indent=2)+'\n')
command=[cargo,'clippy','--all-targets','--all-features','--locked','--offline','--','-D','warnings']
log=A/f'{prefix}-clippy.log'
with log.open('w') as out:code=subprocess.run(command,cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=1200).returncode
verify();checks.append(dict(command=command,returncode=code,log=log.name))
if code:
    finish(tests_executed=0);print('Clippy failed:',log.read_text()[-6000:],flush=True);raise SystemExit(1)
print('Clippy passed all targets/features',flush=True)
env['CARGO_TARGET_DIR']=str(repo/'target')
artifacts={}
for label,command in [('lib',[cargo,'build','--lib','--all-features','--locked','--offline','--message-format=json']),('tests',[cargo,'test','--lib','--all-features','--no-run','--locked','--offline','--message-format=json'])]:
    with (A/f'{prefix}-{label}-build.jsonl').open('w') as out,(A/f'{prefix}-{label}-build.log').open('w') as err:
        code=subprocess.run(command,cwd=repo,env=env,stdout=out,stderr=err,timeout=1200).returncode
    verify();checks.append(dict(command=command,returncode=code,log=f'{prefix}-{label}-build.log'))
    if code:
        finish(tests_executed=0);print('Build failed:',label,(A/f'{prefix}-{label}-build.log').read_text()[-6000:],flush=True);raise SystemExit(1)
    for line in (A/f'{prefix}-{label}-build.jsonl').read_text().splitlines():
        row=json.loads(line)
        if row.get('reason')=='compiler-artifact' and row['target']['name']=='synaps_cad' and row['target']['kind']==['lib']:
            assert not row['fresh'];artifacts[label]=row
    assert label in artifacts
    print('Built:',label,flush=True)
binary=A/f'{prefix}-libtest';shutil.copy2(artifacts['tests']['executable'],binary)
(A/f'{prefix}-artifacts.json').write_text(json.dumps(artifacts,indent=2)+'\n')
names=[line[:-6] for line in subprocess.check_output([str(binary),'--list'],text=True).splitlines() if line.endswith(': test')]
for index,suffix in enumerate(['planar_hull_uses_exact_curve_vertices_without_inventing_thickness','failed_offset_remains_an_explicit_failed_shape']):
    matches=[name for name in names if name.rsplit('::',1)[-1]==suffix];assert len(matches)==1
    log=A/f'{prefix}-test-{index}.log';start=time.monotonic()
    with log.open('w') as out:
        try:code=subprocess.run([str(binary),'--exact',matches[0],'--test-threads=1','--nocapture'],cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=120).returncode
        except subprocess.TimeoutExpired:code='timeout'
    rows.append(dict(name=matches[0],returncode=code,passed=code==0 and '1 passed;' in log.read_text(),elapsed_seconds=time.monotonic()-start,log=log.name))
    print(rows[-1],flush=True)
    if not rows[-1]['passed']:finish();raise SystemExit(1)
lib=Path(next(path for path in artifacts['lib']['filenames'] if path.endswith('.rlib')))
probe=A/f'{prefix}-render'
command=[rustc,'--edition=2024',str(A/'synaps-hull-caller-20260924-render.rs'),'-L',f'dependency={lib.parent}','--extern',f'synaps_cad={lib}','-o',str(probe)]
build=subprocess.run(command,capture_output=True,text=True,timeout=180)
(A/f'{prefix}-render-build.log').write_text(build.stdout+build.stderr);assert build.returncode==0,build.stderr
visual=[]
for source in sorted((A/'synaps-hull-caller-20260924-visual').glob('*.scad')):
    log=source.with_suffix('.synaps.log');start=time.monotonic()
    with log.open('w') as out:
        try:code=subprocess.run([str(probe),str(source)],cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=120).returncode
        except subprocess.TimeoutExpired:code='timeout'
    row=dict(fixture=source.name,synaps_returncode=code,elapsed_seconds=time.monotonic()-start,synaps_log=str(log.relative_to(A)))
    if code!=0:
        visual.append(row);finish(visual=visual);print('Visual fixture failed:',row,log.read_text()[-3000:],flush=True);raise SystemExit(1)
    source.with_suffix('.synaps.png').write_bytes(base64.b64decode(source.with_suffix('.synaps.b64').read_text()))
    svg=source.with_suffix('.openscad.svg');reference=subprocess.run(['/usr/bin/openscad','-o',str(svg),str(source)],capture_output=True,text=True,timeout=120,env=dict(env,QT_QPA_PLATFORM='offscreen'))
    source.with_suffix('.openscad.log').write_text(reference.stdout+reference.stderr);assert reference.returncode==0,reference.stderr
    import cairosvg
    cairosvg.svg2png(url=str(svg),write_to=str(source.with_suffix('.openscad.png')),output_width=512,output_height=512)
    row['openscad_returncode']=reference.returncode;visual.append(row);print('Rendered both engines:',source.name,flush=True)
verify();finish(visual=visual,visual_inspection_pending=True,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),probe_sha256=hashlib.sha256(probe.read_bytes()).hexdigest(),library_sha256=hashlib.sha256(lib.read_bytes()).hexdigest())
print('Terminal; tests and renders reaped. Images await inspection.',flush=True)
