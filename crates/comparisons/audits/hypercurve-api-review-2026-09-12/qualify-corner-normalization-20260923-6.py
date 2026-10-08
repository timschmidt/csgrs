from pathlib import Path
import concurrent.futures
import hashlib
import json
import os
import shutil
import subprocess
import time

a=Path(__file__).resolve().parent
w=a.parent/'hypercurve'
r=Path('/tmp/hypercurve-closure-2026-09-23/hypercurve')
prefix='corner-normalization-20260923-fix6'
files=['src/bezier_region.rs','src/bezier_offset.rs','src/curve_region_boolean.rs','src/error.rs']
working={name:hashlib.sha256((w/name).read_bytes()).hexdigest() for name in files}
for name in files:
    content=((a/'single-loop-corner-candidate3.rs').read_bytes() if name=='src/bezier_region.rs'
             else subprocess.check_output(['git','show','c980fed:'+name],cwd=w) if name=='src/error.rs'
             else (w/name).read_bytes())
    (r/name).write_bytes(content)
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true','src/bezier_region.rs'],cwd=r,check=True)
manifest=[dict(file=str(p.relative_to(r.parent)),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
          for repo in sorted(r.parent.iterdir()) if repo.is_dir()
          for p in sorted(repo.rglob('*')) if p.is_file() and 'target' not in p.parts]
(a/(prefix+'-sources.json')).write_text(json.dumps(dict(working=working,isolated=manifest),indent=2)+'\n')
for name in files: (a/(prefix+'-'+Path(name).name)).write_bytes((r/name).read_bytes())
def verify():
    for name,digest in working.items(): assert hashlib.sha256((w/name).read_bytes()).hexdigest()==digest,name
    for row in manifest: assert hashlib.sha256((r.parent/row['file']).read_bytes()).hexdigest()==row['sha256'],row['file']

env=dict(os.environ,**json.loads((a/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
cmd=[cargo,'check','--all-targets','--no-default-features','--locked','--offline']
start=time.monotonic()
with (a/(prefix+'-check.log')).open('w') as out:
    code=subprocess.run(cmd,cwd=r,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=900).returncode
verify();print('check',code,round(time.monotonic()-start,2),flush=True)
if code:
    print((a/(prefix+'-check.log')).read_text()[-5000:],flush=True);raise SystemExit(code)
cmd=[cargo,'test','--release','--all-features','--lib','--no-run','--message-format=json','--locked','--offline']
start=time.monotonic()
with (a/(prefix+'-build.jsonl')).open('w') as out,(a/(prefix+'-build.log')).open('w') as err:
    code=subprocess.run(cmd,cwd=r,env=env,stdout=out,stderr=err,timeout=900).returncode
verify();print('build',code,round(time.monotonic()-start,2),flush=True)
if code:
    print((a/(prefix+'-build.log')).read_text()[-5000:],flush=True);raise SystemExit(code)
for line in (a/(prefix+'-build.jsonl')).read_text().splitlines():
    item=json.loads(line)
    if item.get('reason')=='compiler-artifact' and item.get('executable') and item['target']['name']=='hypercurve':
        assert not item['fresh'];binary=a/(prefix+'-libtest');shutil.copy2(item['executable'],binary);break
else: raise AssertionError('missing library')
names=[
 'curve_region_boolean::certified_successor_tests::curved_face_windings_preserve_crossings_tangencies_overlaps_and_nested_holes',
 'bezier_region::single_loop_corner_publication_tests::selected_circle_corner_candidates_publish_normalized_single_loops',
 'bezier_region::tests::general_nonrepresented_chord_and_retained_rational_arc_complete_the_fillet_kernel',
 'bezier_region::tests::independent_oblique_chord_pair_fillet_crosses_a_rational_line_exactly',
 'bezier_region::tests::independent_oblique_chord_pair_fillet_crosses_algebraic_chords_exactly',
]
listing=subprocess.check_output([str(binary),'--list'],cwd=r,text=True)
def run(name):
    assert name+': test' in listing,name
    cmd=[str(binary),'--exact',name,'--test-threads=1','--nocapture','--color','never']
    log=a/(prefix+'-'+name.split('::')[-1]+'.log');start=time.monotonic()
    with log.open('w') as out:
        try: code=subprocess.run(cmd,cwd=r,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=75).returncode
        except subprocess.TimeoutExpired: code='timeout'
    row=dict(name=name,command=cmd,returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
    print(json.dumps(row),flush=True)
    if code: print(log.read_text()[-4000:],flush=True)
    return row
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool: rows=list(pool.map(run,names))
verify();(a/(prefix+'-runs.json')).write_text(json.dumps(rows,indent=2)+'\n')
