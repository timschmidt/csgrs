from pathlib import Path
import hashlib
import json
import os
import shutil
import subprocess
import time

a = Path(__file__).resolve().parent
w = a.parent/'hypercurve'
r = Path('/tmp/hypercurve-region-admission-qualification/hypercurve')
prefix = 'contact-local-fillet-attempt1'
archive = a/(prefix+'-libraries')
archive.mkdir()
env = dict(os.environ, **json.loads((a/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=w, text=True).strip()
working = {name: hashlib.sha256((w/name).read_bytes()).hexdigest() for name in ['src/bezier_offset.rs', 'src/bezier_region.rs', 'tests/hypercurve_curve_region_promotion.rs', 'src/curve_region_boolean.rs', 'src/curve.rs', 'src/curve_corner_chain.rs']}
for name in ['src/curve_region_boolean.rs', 'tests/hypercurve_curve_region_promotion.rs']:
    (r/name).write_bytes(subprocess.check_output(['git', 'show', 'HEAD:'+name], cwd=w))
records = []

def manifest():
    return [dict(file=str(p.relative_to(r.parent)), sha256=hashlib.sha256(p.read_bytes()).hexdigest())
            for repo in sorted(r.parent.iterdir()) if repo.is_dir()
            for p in sorted(repo.rglob('*')) if p.is_file() and 'target' not in p.parts]

def verify(rows):
    for name,digest in working.items(): assert hashlib.sha256((w/name).read_bytes()).hexdigest()==digest,name
    for row in rows: assert hashlib.sha256((r.parent/row['file']).read_bytes()).hexdigest()==row['sha256'],row['file']

def save():
    (a/(prefix+'-runs.json')).write_text(json.dumps(dict(head=head, working=working, runs=records),indent=2)+'\n')

def command(label, cmd, timeout, rows):
    start=time.monotonic()
    with (a/(prefix+'-'+label+'.log')).open('w') as out:
        try: code=subprocess.run(cmd,cwd=r,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=timeout).returncode
        except subprocess.TimeoutExpired: code='timeout'
    records.append(dict(label=label,command=cmd,returncode=code,elapsed_seconds=time.monotonic()-start))
    save();verify(rows)
    print(label,code,round(time.monotonic()-start,2),flush=True)
    print((a/(prefix+'-'+label+'.log')).read_text()[-3000:],flush=True)
    return code

def build(variant, rows):
    cmd=[cargo,'test','--release','--all-features','--lib','--no-run','--message-format=json','--locked','--offline']
    start=time.monotonic()
    with (a/(prefix+'-'+variant+'-build.jsonl')).open('w') as out, (a/(prefix+'-'+variant+'-build.log')).open('w') as err:
        code=subprocess.run(cmd,cwd=r,env=env,stdout=out,stderr=err,timeout=900).returncode
    records.append(dict(label=variant+'-build',command=cmd,returncode=code,elapsed_seconds=time.monotonic()-start));save();verify(rows)
    print(variant+'-build',code,round(time.monotonic()-start,2),flush=True)
    if code:
        print((a/(prefix+'-'+variant+'-build.log')).read_text()[-5000:],flush=True)
        raise SystemExit(1)
    for line in (a/(prefix+'-'+variant+'-build.jsonl')).read_text().splitlines():
        item=json.loads(line)
        if item.get('reason')=='compiler-artifact' and item.get('executable') and item['target']['name']=='hypercurve':
            assert item['fresh'] is False
            binary=archive/(variant+'-libtest');shutil.copy2(item['executable'],binary)
            digest=hashlib.sha256(binary.read_bytes()).hexdigest()
            records[-1].update(binary=str(binary),sha256=digest);save()
            return binary
    raise AssertionError('missing libtest')

name='curve::tests::fillet_center_contacts_keep_source_orientation_across_support_cusps'
for variant in ['parent','candidate']:
    for file in ['src/bezier_offset.rs', 'src/curve.rs', 'src/bezier_region.rs', 'src/curve_corner_chain.rs']:
        (r/file).write_bytes((a/('contact-local-fillet-'+variant+'-'+Path(file).name)).read_bytes())
    rows=manifest();(a/(prefix+'-'+variant+'-sources.json')).write_text(json.dumps(rows,indent=2)+'\n')
    if variant=='candidate':
        assert command('check',[cargo,'check','--all-targets','--no-default-features','--locked','--offline'],900,rows)==0
    binary=build(variant,rows)
    listing=subprocess.check_output([str(binary),'--list'],cwd=r,text=True);assert name+': test' in listing
    code=command(variant+'-regression',[str(binary),'--exact',name,'--nocapture','--test-threads=1','--color','never'],90,rows)
    if variant=='parent': assert code==101,'parent must fail the independent source tangent oracle'
    else: assert code==0,'candidate must pass both policies, source carriers, and traversals'

held_out=['selected_parallel_normal_circle_intersects_genuinely_analytic_parallel_in_one_fiber','independent_oblique_chord_pair_fillets_extend_on_infinite_supports','selected_circle_and_analytic_parallel_extend_on_full_supports','pair_native_boolean_algebraic_chord_corner_publishes_a_third_generation_fillet']
args=[str(binary),'--test-threads=2','--color','never']
for suffix in held_out:
    matches=[line.removesuffix(': test') for line in listing.splitlines() if line.endswith(suffix+': test')]
    assert len(matches)==1,(suffix,matches)
    args.extend(['--skip',matches[0]])
assert command('candidate-library',args,360,rows)==0
