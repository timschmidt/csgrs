from pathlib import Path
import subprocess,time,json,os
root=Path('/home/tim/Documents/GitHub/workspace')
audit=root/'hypercurve-api-review-2026-09-12'
results=[]
for name,manifest in [('fuzz','fuzz/Cargo.toml'),('ui','examples/hypercurve_ui/Cargo.toml')]:
    cmd=['cargo','check','--manifest-path',manifest,'--all-targets','--locked','--offline']
    start=time.monotonic()
    log=audit/('analytic-point-field-replay-'+name+'-check.log')
    with log.open('w') as out:
        try:
            result=subprocess.run(cmd,cwd=root/'hypercurve',env=dict(os.environ,CARGO_BUILD_JOBS='2'),stdout=out,stderr=subprocess.STDOUT,timeout=300)
            code=result.returncode
        except subprocess.TimeoutExpired:
            code=124
    row={'target':name,'command':cmd,'returncode':code,'elapsed_seconds':time.monotonic()-start,'log':log.name}
    results.append(row)
    print(row,flush=True)
    if code: print(log.read_text()[-4000:],flush=True)
(audit/'analytic-point-field-replay-caller-checks.json').write_text(json.dumps(results,indent=2)+'\n')
raise SystemExit(int(any(row['returncode'] for row in results)))
