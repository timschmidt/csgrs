import json
import random
import statistics
import subprocess
from pathlib import Path

exe = "/tmp/realistic-audit/target/release/performance"
cases = [(op,n) for op in ("clone","cached") for n in (0,8,32,128)]
cases += [(op,n) for op in ("sqrt_cold","cos_cold") for n in (64,256,1024)]
cases += [("real_format",32),("real_format",128)]
results = {}
rng = random.Random(753)

def run(lib,op,arg,iters):
    return float(subprocess.check_output(["taskset","-c","6",exe,lib,op,str(arg),str(iters)],text=True))

for op,arg in cases:
    iters = {}
    for lib in ("realistic","hyper"):
        pilot = run(lib,op,arg,100)
        iters[lib] = max(100,min(1_000_000,round(30_000_000/pilot)))
    pairs = []
    for _ in range(11):
        a = run("realistic",op,arg,iters["realistic"])
        b = run("hyper",op,arg,iters["hyper"])
        bb = run("hyper",op,arg,iters["hyper"])
        aa = run("realistic",op,arg,iters["realistic"])
        pairs.append([(a+aa)/2,(b+bb)/2])
    ratios = [a/b for a,b in pairs]
    boots = sorted(statistics.median(rng.choices(ratios,k=len(ratios))) for _ in range(5000))
    row = {
        "realistic_ns":statistics.median(a for a,b in pairs),
        "hyper_ns":statistics.median(b for a,b in pairs),
        "realistic_over_hyper":statistics.median(ratios),
        "bootstrap_ratio_95pct":[boots[125],boots[4874]],
        "iterations":iters,"pairs":pairs,
    }
    results[f"{op}/{arg}"] = row
    print(f"{op}/{arg}: realistic={row['realistic_ns']:.2f} ns hyper={row['hyper_ns']:.2f} ns ratio={row['realistic_over_hyper']:.3f} CI={row['bootstrap_ratio_95pct']}",flush=True)
    Path("/tmp/realistic-performance-results.json").write_text(json.dumps(results,indent=2))
