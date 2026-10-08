from pathlib import Path
import hashlib,json,subprocess,time,statistics,resource
root=Path('/home/tim/Documents/GitHub/workspace'); audit=root/'hypercurve-api-review-2026-09-12'; prefix='analytic-point-field-replay-comparison'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
source=audit/'region-carrier-evidence-matrix.rs'; source_sha=sha(source)
old=json.loads((audit/'region-carrier-evidence-matrix.json').read_text());new=json.loads((audit/'analytic-point-field-replay-region-carrier-matrix.json').read_text());assert old['counts']==new['counts']
cases=[('before',audit/'region-carrier-evidence-matrix',old['normal_library_sha256'])]+[(f'after-{i}',audit/'analytic-point-field-replay-region-carrier-matrix',new['normal_library_sha256']) for i in range(1,4)]
rows=[]
for label,binary,library_sha in cases:
    binary_sha=sha(binary); log=audit/(prefix+'-'+label+'.log'); before=resource.getrusage(resource.RUSAGE_CHILDREN); start=time.monotonic()
    with log.open('w') as out:
        try:
            result=subprocess.run([str(binary)],stdout=out,stderr=subprocess.STDOUT,timeout=360)
            code=result.returncode
        except subprocess.TimeoutExpired:
            code=124
    elapsed=time.monotonic()-start; after=resource.getrusage(resource.RUSAGE_CHILDREN)
    counts=None
    for line in reversed(log.read_text().splitlines()):
        try:item=json.loads(line)
        except json.JSONDecodeError:continue
        if isinstance(item,dict): counts=item;break
    row={'label':label,'executable':str(binary),'executable_sha256':binary_sha,'normal_library_sha256':library_sha,'returncode':code,'elapsed_seconds':elapsed,'user_cpu_seconds':after.ru_utime-before.ru_utime,'system_cpu_seconds':after.ru_stime-before.ru_stime,'counts':counts,'log':log.name}
    assert sha(binary)==binary_sha and sha(source)==source_sha
    rows.append(row);print(json.dumps(row),flush=True)
    (audit/(prefix+'.json')).write_text(json.dumps({'source':str(source),'source_sha256':source_sha,'runs':rows,'complete':len(rows)==len(cases)},indent=2)+'\n')
    assert code==0 and counts==old['counts'],row
summary={'source':str(source),'source_sha256':source_sha,'runs':rows,'complete':True,'before_seconds':rows[0]['elapsed_seconds'],'after_median_seconds':statistics.median(row['elapsed_seconds'] for row in rows[1:]),'scope':'One unchanged public API matrix; separate sequential processes, preserved old executable versus final qualified executable. Not a general throughput, allocation, or memory benchmark.'}
(audit/(prefix+'.json')).write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k!='runs'}),flush=True)
