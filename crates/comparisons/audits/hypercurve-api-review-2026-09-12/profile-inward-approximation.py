import hashlib
import os
import signal
import subprocess
from pathlib import Path

binary = Path("stationary-ph-offset-mixed-axis-approximate-baseline").resolve()
assert hashlib.sha256(binary.read_bytes()).hexdigest() == "22855bae7b3f78031978e929788d546d20e922a7f17619aeb0fca0ff95959e7e"
# Verified in this immutable binary's APPROXIMATE_512_CONSUMED call_once:
# mov fs:0,rax; lea -0x550(rax),rax. This is the Cell<bool> address.
commands = Path("inward-approximation-gdb.commands")
commands.write_text("\n".join([
    "set debuginfod enabled off", "set pagination off", "set confirm off",
    "set language c", "start", "set $consumed = (unsigned char*)($fs_base-0x550)",
    "watch -l *$consumed", "condition 2 *$consumed != 0", "commands 2", "silent",
    'printf "first observed approximation consumption\\n"', "bt 50", "quit", "end",
    "continue", "thread apply all bt 50", "",
]))
with Path("stationary-ph-inward-approximation-stack.log").open("w") as output:
    process = subprocess.Popen(["gdb", "--quiet", "--batch", "-x", str(commands), "--args", str(binary)], stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        process.wait(timeout=55)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
print("gdb exit", process.returncode)
