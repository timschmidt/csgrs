from pathlib import Path
import hashlib,json,os,signal,subprocess,time
p=Path(__file__).resolve().parent
binary=p/'chord-normal-frame-20260923-focused1-libtest'
assert hashlib.sha256(binary.read_bytes()).hexdigest()=='8831d6bc538776a74bcc07dedaf6a9c4cb649b568e2a044a8a6112d6cf403361'
root=Path('/tmp/hypercurve-chord-normal-frame-2026-09-23')
bindings=json.loads((p/'chord-normal-frame-20260923-focused1-sources.json').read_text())
def verify():
    for name,sha in bindings.items():assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
verify()
log=p/'chord-normal-frame-20260923-stack1.log'
with log.open('w') as out:
    process=subprocess.Popen(['/usr/bin/gdb','--quiet','--nx','--args',str(binary),'--exact','bezier_offset::conversion_tests::chord_normal_recursive_frame_retains_center_and_oriented_unit_normal','--test-threads=1','--nocapture'],stdin=subprocess.PIPE,stdout=out,stderr=subprocess.STDOUT,text=True,start_new_session=True,cwd=root/'hypercurve')
    process.stdin.write('set pagination off\nset confirm off\nset debuginfod enabled off\nset print frame-arguments none\nset print elements 4\nrun\n')
    process.stdin.flush()
    time.sleep(20)
    os.killpg(process.pid,signal.SIGINT)
    try:
        process.communicate('thread apply all bt 55\nkill\nquit\n',timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGKILL)
        process.communicate()
        raise
verify()
assert process.returncode==0
text=log.read_text()
assert 'running 1 test' in text and '#0 ' in text
print(text[-24000:],flush=True)
(p/'chord-normal-frame-20260923-stack1-terminal.json').write_text(json.dumps(dict(exit_code=process.returncode,all_processes_reaped=True,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),all_sources_unchanged=True),indent=2)+'\n')
