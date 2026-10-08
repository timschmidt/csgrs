from pathlib import Path
import os,subprocess,signal,json,time
p=Path('/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12')
cmd=['gdb','--quiet','--batch','-ex','set pagination off','-ex','set confirm off','-ex','set debuginfod enabled off','-ex','run','-ex','thread apply all bt 48','-ex','quit','--args',str(p/'finite-circle-domain-stages')]
start=time.monotonic()
with (p/'finite-circle-domain-stack-local.log').open('w') as out:
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
print((p/'finite-circle-domain-stack-local.log').read_text()[-22000:])
(p/'finite-circle-domain-stack-local.json').write_text(json.dumps({'command':cmd,'returncode':process.returncode,'elapsed_seconds':time.monotonic()-start},indent=2)+'\n')
