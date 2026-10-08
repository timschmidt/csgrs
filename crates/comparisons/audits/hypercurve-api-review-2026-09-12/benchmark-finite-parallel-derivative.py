from pathlib import Path
import hashlib,json,statistics,subprocess,time

audit=Path('/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12')
source=audit/'finite-parallel-derivative-bench.rs'
parent=json.loads((audit/'finite-parallel-pruning-qualification.json').read_text())
items=[json.loads(line) for line in (audit/'finite-parallel-affine-test-build.jsonl').read_text().splitlines()]
candidate=Path(next(f for x in items if x.get('reason')=='compiler-artifact' and x['target']['name']=='hypercurve' and not x['profile']['test'] for f in x['filenames'] if f.endswith('.rlib')))
libraries={'parent':(Path(parent['archived_library']),Path(parent['normal_library']).parent),'candidate':(candidate,candidate.parent)}
assert hashlib.sha256(libraries['parent'][0].read_bytes()).hexdigest()==parent['normal_library_sha256']
executables={}; records=[]
for name,(library,deps) in libraries.items():
    exe=audit/('finite-parallel-derivative-bench-'+name)
    command=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc','--edition=2024','-O',str(source),'--extern','hypercurve='+str(library),'-L','dependency='+str(deps),'-o',str(exe)]
    with (audit/(exe.name+'-compile.log')).open('w') as out:
        result=subprocess.run(command,stdout=out,stderr=subprocess.STDOUT,timeout=120)
    assert result.returncode==0,(audit/(exe.name+'-compile.log')).read_text()
    executables[name]=exe
    records.append(dict(kind='build',name=name,command=command,library=str(library),library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),executable_sha256=hashlib.sha256(exe.read_bytes()).hexdigest()))
for trial,order in enumerate([['parent','candidate'],['candidate','parent'],['parent','candidate']]):
    for name in order:
        exe=executables[name]; start=time.monotonic()
        result=subprocess.run([str(exe)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=120)
        (audit/(exe.name+f'-trial{trial}.log')).write_text(result.stdout)
        assert result.returncode==0,result.stdout
        rows=[json.loads(line) for line in result.stdout.splitlines()]
        assert len(rows)==4 and all(row['calls']==100000 for row in rows)
        records.append(dict(kind='run',trial=trial,name=name,elapsed_seconds=time.monotonic()-start,results=rows))
        print(name,trial,rows,flush=True)
summary=[]
for case in ['quadratic','cubic','rational','zero_distance_rational']:
    measurements={name:[row['elapsed_ns'] for record in records if record['kind']=='run' and record['name']==name for row in record['results'] if row['case']==case] for name in libraries}
    medians={name:statistics.median(values) for name,values in measurements.items()}
    summary.append(dict(case=case,calls=100000,elapsed_ns=measurements,median_ns=medians,parent_over_candidate=medians['parent']/medians['candidate']))
report=dict(source=source.name,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),records=records,summary=summary,scope='Local native-parameter derivative microbenchmark only: same public source, direct stable compiler, warm source cache per run, three alternating process trials. Other-session system load is uncontrolled. No general geometry throughput claim.')
(audit/'finite-parallel-affine-derivative-benchmark.json').write_text(json.dumps(report,indent=2)+'\n')
print('benchmark complete',summary,flush=True)
