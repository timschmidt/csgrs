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
prefix='corner-normalization-20260923-diagnostic4'
files=['src/bezier_region.rs','src/bezier_offset.rs','src/curve_region_boolean.rs','src/error.rs']
working={name:hashlib.sha256((w/name).read_bytes()).hexdigest() for name in files}
for name in files:
    content=((a/'single-loop-corner-candidate2.rs').read_bytes() if name=='src/bezier_region.rs'
             else subprocess.check_output(['git','show','c980fed:'+name],cwd=w) if name=='src/error.rs'
             else (w/name).read_bytes())
    (r/name).write_bytes(content)
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true','src/bezier_region.rs'],cwd=r,check=True)

file=r/'src/bezier_offset.rs'
s=file.read_text()
start=s.index('    fn has_certified_concentric_source_tangency(')
end=s.index('    fn selected_parallel_normal_rational_intersections_internal(',start)
b=s[start:end]
b=b.replace('        let (Some(frame), Some(target)) = (', '        eprintln!("tangent certificate frame={} target={}", self.data.frame.selected_radial().is_some(), other.retained_circular_conic().is_some());\n        let (Some(frame), Some(target)) = (')
b=b.replace('        if policy.strict_predicate_pass(|| real_sign(&discriminant, policy))', '        eprintln!("tangent circle discriminant {:?}", policy.strict_predicate_pass(|| real_sign(&discriminant, policy)));\n        if policy.strict_predicate_pass(|| real_sign(&discriminant, policy))')
b=b.replace('        let Classification::Decided(center) = parent.center_point_evidence(policy)? else {', '        eprintln!("tangent source center evidence enter");\n        let Classification::Decided(center) = parent.center_point_evidence(policy)? else {\n            eprintln!("tangent source center evidence uncertain");')
b=b.replace('        Ok(policy.strict_predicate_pass(|| {\n            center.same_point', '        eprintln!("tangent center equality {:?}",policy.strict_predicate_pass(|| center.same_point(&CurvePoint2::from(target.center.clone()), policy)));\n        Ok(policy.strict_predicate_pass(|| {\n            center.same_point')
b=b.replace('        let tangent_root = circle_polynomial.len() == 3', '        eprintln!("tangent polynomial size {}",circle_polynomial.len());\n        let tangent_root = circle_polynomial.len() == 3')
b=b.replace('        if tangent_root {', '        eprintln!("tangent certificate result {tangent_root}");\n        if tangent_root {',1)
b=b.replace('        let candidates = match recursive_projective_polynomial_parameters(', '        eprintln!("circle root solve enter");\n        let candidates = match recursive_projective_polynomial_parameters(')
b=b.replace('        let mut contacts = Vec::with_capacity(candidates.len());', '        eprintln!("circle root solve done count={}",candidates.len());\n        let mut contacts = Vec::with_capacity(candidates.len());')
b=b.replace('            let location = match policy.strict_predicate_pass(|| {', '            eprintln!("circle location enter");\n            let location = match policy.strict_predicate_pass(|| {')
b=b.replace('            let tangent_cross_sign = if tangent_root {', '            eprintln!("circle location done {location:?}");\n            let tangent_cross_sign = if tangent_root {')
s=s[:start]+b+s[end:]
file.write_text(s)
file=r/'src/error.rs'
s=file.read_text().replace('    pub(crate) const fn blocked(\n','    #[track_caller]\n    pub(crate) fn blocked(\n')
s=s.replace('        Self::Blocked(ExactCurveBlocker::new(operation, family, reason))','        eprintln!("BLOCK {operation:?} {family:?} {reason:?} {}",std::panic::Location::caller());\n        Self::Blocked(ExactCurveBlocker::new(operation, family, reason))')
file.write_text(s)

manifest=[dict(file=str(p.relative_to(r.parent)),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
          for repo in sorted(r.parent.iterdir()) if repo.is_dir()
          for p in sorted(repo.rglob('*')) if p.is_file() and 'target' not in p.parts]
(a/(prefix+'-sources.json')).write_text(json.dumps(dict(working=working,isolated=manifest),indent=2)+'\n')
for name in files: (a/(prefix+'-'+Path(name).name)).write_bytes((r/name).read_bytes())
def verify():
    for name,digest in working.items(): assert hashlib.sha256((w/name).read_bytes()).hexdigest()==digest,name
    for row in manifest: assert hashlib.sha256((r.parent/row['file']).read_bytes()).hexdigest()==row['sha256'],row['file']

env=dict(os.environ,HYPERCURVE_DEBUG_RATIONAL_BLOCKER='1',**json.loads((a/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
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
names=['bezier_region::tests::general_nonrepresented_chord_and_retained_rational_arc_complete_the_fillet_kernel']
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
