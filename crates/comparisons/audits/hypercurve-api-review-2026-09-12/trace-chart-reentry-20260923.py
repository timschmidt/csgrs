from pathlib import Path
import hashlib, json, os, signal, subprocess, time

audit = Path(__file__).resolve().parent
binary = audit / 'nonph-order-retain-chart-20260923-probe'
command = ['gdb', '-nx', '-q', '--batch', '-ex', 'set debuginfod enabled off',
           '-ex', 'set pagination off', '-ex', 'set confirm off', '-ex', 'run',
           '-ex', 'thread apply all bt 55', '-ex', 'kill', '--args', str(binary)]
start = time.monotonic()
with (audit / 'nonph-order-retain-chart-20260923-gdb.log').open('w') as out:
    child = subprocess.Popen(command, stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        child.wait(timeout=20)
    except subprocess.TimeoutExpired:
        child.send_signal(signal.SIGINT)
        try:
            child.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGKILL)
            child.wait()
result = dict(command=command, returncode=child.returncode, elapsed_seconds=time.monotonic()-start,
              binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
(audit / 'nonph-order-retain-chart-20260923-gdb.json').write_text(json.dumps(result, indent=2)+'\n')
print(result)
print((audit / 'nonph-order-retain-chart-20260923-gdb.log').read_text())
