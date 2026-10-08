from pathlib import Path
import hashlib,json,os,subprocess,time
a=Path('/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12')
r=Path('/tmp/hypercurve-circle-projection-baseline')
p='finite-circle-projection-parent'
t=Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin')
settings=dict(CARGO_BUILD_JOBS='2',RUSTC=str(t/'rustc'),RUSTDOC=str(t/'rustdoc'),CARGO_TARGET_DIR='/tmp/hypercurve-pruning-qualification/hyperbrep/target',CCACHE_DIR='/tmp/hypercurve-pruning-qualification/ccache',CCACHE_TEMPDIR='/tmp/hypercurve-pruning-qualification/ccache-tmp')
manifest=[]
for name in ['hypercurve','hyperbrep','hyperlattice','hyperlimit','hyperreal','hypersolve','hypertri']:
    for path in sorted((r/name).rglob('*')):
        if path.is_file() and 'target' not in path.parts:
            manifest.append(dict(file=str(path.relative_to(r)),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
(a/(p+'-sources.json')).write_text(json.dumps(manifest,indent=2)+'\n')
cmd=[str(t/'cargo'),'test','--release','--all-features','--locked','--offline','--lib','--no-run','--message-format=json']
start=time.monotonic()
with (a/(p+'-build.log')).open('w') as err,(a/(p+'-build.jsonl')).open('w') as out:
    result=subprocess.run(cmd,cwd=r/'hypercurve',env=dict(os.environ,**settings),stdout=out,stderr=err,timeout=900)
build=dict(command=cmd,returncode=result.returncode,elapsed_seconds=time.monotonic()-start,settings=settings)
(a/(p+'-build.json')).write_text(json.dumps(build,indent=2)+'\n')
assert result.returncode==0,(a/(p+'-build.log')).read_text()[-6000:]
artifacts=[json.loads(l) for l in (a/(p+'-build.jsonl')).read_text().splitlines()]
b=Path(next(x['executable'] for x in artifacts if x.get('reason')=='compiler-artifact' and x['target']['name']=='hypercurve' and x.get('executable')))
sha=hashlib.sha256(b.read_bytes()).hexdigest()
records=[]
for test in ['selected_axis_projection_owns_finite_ranges_and_incident_roots','selected_projection_replays_selected_boundaries_under_the_original_policy']:
    command=[str(b),'--exact','bezier_offset::conversion_tests::'+test,'--nocapture','--test-threads=1']
    start=time.monotonic();log=p+'-'+test+'.log'
    with (a/log).open('w') as out:
        result=subprocess.run(command,stdout=out,stderr=subprocess.STDOUT,timeout=120)
    body=(a/log).read_text()
    records.append(dict(command=command,returncode=result.returncode,elapsed_seconds=time.monotonic()-start,log=log,binary_sha256=sha))
    print(body,flush=True)
    assert result.returncode==101 and '0 passed; 1 failed;' in body,records[-1]
for row in manifest: assert hashlib.sha256((r/row['file']).read_bytes()).hexdigest()==row['sha256'],row['file']
archive=a/(p+'-libtest');archive.write_bytes(b.read_bytes())
assert hashlib.sha256(archive.read_bytes()).hexdigest()==sha
(a/(p+'-results.json')).write_text(json.dumps(dict(build=build,tests=records,source_count=len(manifest),source_hashes_verified=True),indent=2)+'\n')
print('Both parent defects reproduced; artifact archived; source hashes unchanged',flush=True)
