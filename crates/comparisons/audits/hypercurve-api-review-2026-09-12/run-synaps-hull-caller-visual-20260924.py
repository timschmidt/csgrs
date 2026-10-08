from pathlib import Path
import base64,hashlib,json,os,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;repo=W/'synaps-cad';prefix='synaps-hull-caller-20260924-v2'
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
rustc='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc'
bindings=json.loads((A/f'{prefix}-sources.json').read_text())
artifacts=json.loads((A/f'{prefix}-artifacts.json').read_text())
def verify():
    for name,sha in bindings.items():assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
verify()
def finish(**extra):
    verify();(A/'synaps-hull-caller-20260924-visual-terminal.json').write_text(json.dumps(dict(all_sources_unchanged=True,all_processes_reaped=True,**extra),indent=2)+'\n')
lib=Path(next(path for path in artifacts['lib']['filenames'] if path.endswith('.rlib')))
probe=A/f'{prefix}-render'
command=[rustc,'--edition=2024',str(A/'synaps-hull-caller-20260924-render.rs'),'-L',f'dependency={repo/"target/debug/deps"}','--extern',f'synaps_cad={lib}','-o',str(probe)]
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
verify();finish(visual=visual,visual_inspection_pending=True,probe_sha256=hashlib.sha256(probe.read_bytes()).hexdigest(),library_sha256=hashlib.sha256(lib.read_bytes()).hexdigest())
print('Terminal; all renders reaped. Images await inspection.',flush=True)
