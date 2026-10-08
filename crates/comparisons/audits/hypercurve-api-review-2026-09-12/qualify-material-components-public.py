from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import hashlib,json,shutil,subprocess,time,re
p=Path(__file__).resolve().parent;r=Path('/tmp/hypercurve-region-admission-qualification');w=p.parent;prefix='material-components-candidate1'
for file in ['isolated','working']:
    base=r if file=='isolated' else w
    for row in json.loads((p/(prefix+'-'+file+'-sources.json')).read_text()):assert hashlib.sha256((base/row['file']).read_bytes()).hexdigest()==row['sha256'],row['file']
artifacts=[json.loads(x) for x in (p/(prefix+'-hypercurve-test-build.jsonl')).read_text().splitlines()]
item=next(x for x in artifacts if x.get('reason')=='compiler-artifact' and x['target']['name']=='hypercurve' and not x['profile']['test']);lib=next(Path(x) for x in item['filenames'] if x.endswith('.rlib'));archive=p/(prefix+'-libraries');shutil.copy2(lib,archive/lib.name);lib=archive/lib.name
parent=p/'normalized-traversal-candidate3-libraries/libhypercurve-bacb55d5ec235376.rlib'
jobs=[]
for name,source,library,args,timeout in [('material-components-policy-public','material-components-policy-public.rs',lib,[],60),('million-edge-circle-admission-parent','million-edge-circle-admission-public.rs',parent,['1000000'],90),('million-edge-circle-admission-candidate','million-edge-circle-admission-public.rs',lib,['1000000'],90)]:
    cmd=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc','--edition=2024','-O',str(p/source),'--extern','hypercurve='+str(library),'-L','dependency=/tmp/hypercurve-pruning-qualification/hyperbrep/target/release/deps','-o',str(p/name)];subprocess.run(cmd,check=True)
    jobs.append(dict(name=name,command=[str(p/name),*args],timeout=timeout,library=str(library),library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),binary_sha256=hashlib.sha256((p/name).read_bytes()).hexdigest(),source_sha256=hashlib.sha256((p/source).read_bytes()).hexdigest(),compile_command=cmd))
# The completed binary records all other cases. Re-run the selected group with
# its million-edge stress case held out explicitly, rather than hiding a timeout.
binary=next(archive.glob('hyperdrc-*'));jobs.append(dict(name='material-components-drc-geometry-completed',command=[str(binary),'geometry::tests::','--test-threads=2','--skip','circle_polygon_normalizes_invalid_segment_counts'],timeout=60,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest()))
def run(job):
    start=time.monotonic();timed_out=False
    with (p/(job['name']+'.log')).open('w') as log:
        try:code=subprocess.run(job['command'],cwd=r/'hyperdrc',stdout=log,stderr=subprocess.STDOUT,timeout=job['timeout']).returncode
        except subprocess.TimeoutExpired:code=124;timed_out=True
    row=dict(**job,returncode=code,timed_out=timed_out,elapsed_seconds=time.monotonic()-start)
    (p/(job['name']+'.json')).write_text(json.dumps(row,indent=2)+'\n');print(job['name'],code,round(row['elapsed_seconds'],2),flush=True);print((p/(job['name']+'.log')).read_text()[-2400:],flush=True);return row
# Keep the two million-edge constructions sequential for an equal resource scope.
results=[]
for job in jobs:results.append(run(job))
(p/'material-components-public-results.json').write_text(json.dumps(results,indent=2)+'\n')
for file in ['isolated','working']:
    base=r if file=='isolated' else w
    for row in json.loads((p/(prefix+'-'+file+'-sources.json')).read_text()):assert hashlib.sha256((base/row['file']).read_bytes()).hexdigest()==row['sha256'],row['file']
print('source manifests unchanged',flush=True)
