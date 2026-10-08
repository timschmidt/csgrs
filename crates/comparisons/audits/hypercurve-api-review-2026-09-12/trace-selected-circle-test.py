from pathlib import Path
import os,subprocess,signal,json,time
p=Path('/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12')
cmd=['gdb','--quiet','--batch','-ex','set pagination off','-ex','set confirm off','-ex','set debuginfod enabled off','-ex','run','-ex','thread apply all bt 48','-ex','quit','--args','/home/tim/Documents/GitHub/workspace/hypercurve/target/release/deps/hypercurve_curve_intersection-e3a50a789eb6a53e','selected_circle_tangency_reuses_independent_affine_line_charts','--exact','--nocapture','--test-threads=1']
start=time.monotonic()
with (p/'selected-circle-affine-identity-attempt1-stack.log').open('w') as out:
    process=subprocess.Popen(cmd,stdout=out,stderr=subprocess.STDOUT,env=dict(os.environ,DEBUGINFOD_URLS='',XDG_CACHE_HOME='/tmp/finite-circle-gdb-cache'))
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            process.kill();process.wait()
print('returncode',process.returncode,'elapsed',time.monotonic()-start)
print((p/'selected-circle-affine-identity-attempt1-stack.log').read_text()[-22000:])
(p/'selected-circle-affine-identity-attempt1-stack.json').write_text(json.dumps({'command':cmd,'returncode':process.returncode,'elapsed_seconds':time.monotonic()-start},indent=2)+'\n')
