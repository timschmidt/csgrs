import hashlib
import os
import signal
import subprocess
from pathlib import Path

binary = Path("stationary-ph-offset-refined-chord-envelope").resolve()
assert hashlib.sha256(binary.read_bytes()).hexdigest() == "90af7a6b840a771625ba7041e6200a62b3e085248b33be529c9a99a2f618e811"
# The immutable binary disassembly verifies the two point pointers in r15/r14
# at this cold-fallback site, and byte-8 tags 4=ChordPair, 8=AnalyticParallel
# through represented_point_evidence_coordinates' dispatch jump table.
symbol = "_RNvNtCs19gMNq2dmyq_10hypercurve13bezier_offset52algebraic_chord_point_coordinate_order_by_refinement"
commands = Path("mixed-point-axis-order-gdb.commands")
commands.write_text("\n".join([
    "set debuginfod enabled off", "set pagination off", "set confirm off",
    "set language c", f"break *'{symbol}'+0x2b0", "commands", "silent",
    'printf "cold-axis-order tags=(%u,%u) axis=%u\\n", *(unsigned char*)($r15+8), *(unsigned char*)($r14+8), $ebp',
    "bt 5", "continue", "end", "run", "thread apply all bt 30", "",
]))
with Path("mixed-point-axis-order-input-profile.log").open("w") as output:
    process = subprocess.Popen(["gdb", "--quiet", "--batch", "-x", str(commands), "--args", str(binary)], stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
print("gdb exit", process.returncode)
