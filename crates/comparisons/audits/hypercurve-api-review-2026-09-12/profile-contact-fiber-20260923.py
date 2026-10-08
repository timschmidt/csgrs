import os
import signal
import subprocess
import sys
from pathlib import Path
binary=Path(sys.argv[1]).resolve()
log=Path(sys.argv[2]).resolve()
with log.open("w") as output:
    process=subprocess.Popen(["gdb", "--quiet", "--batch", "-ex", "set debuginfod enabled off", "-ex", "set pagination off", "-ex", "set confirm off", "-ex", "run", "-ex", "thread apply all bt 55", "-ex", "kill", "--args", str(binary), *sys.argv[3:]], stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
print("gdb exit",process.returncode)
