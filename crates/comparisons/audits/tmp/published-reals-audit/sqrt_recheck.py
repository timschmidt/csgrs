import json, random, statistics, subprocess
from pathlib import Path

paths = {"baseline":"/tmp/published-reals-hyper-sqrt-baseline", "candidate":"/tmp/published-reals-audit/target/release/performance"}
rng = random.Random(923)
results = {}
def run(version, precision, n):
    return float(subprocess.check_output(["taskset","-c","6",paths[version],"hyper","sqrt_cold",str(precision),str(n)],text=True))
for precision in [16,32,56,57,59,60,64,96,128,140,256,1024]:
    pilot = run("baseline", precision, 100)
    n = max(100, min(100_000, round(30_000_000 / pilot)))
    pairs=[]
    for _ in range(41):
        a=run("baseline",precision,n); b=run("candidate",precision,n)
        bb=run("candidate",precision,n); aa=run("baseline",precision,n)
        pairs.append([(a+aa)/2,(b+bb)/2])
    ratios=[b/a for a,b in pairs]
    boots=sorted(statistics.median(rng.choices(ratios,k=len(ratios))) for _ in range(5000))
    row={"baseline_ns":statistics.median(a for a,b in pairs),"candidate_ns":statistics.median(b for a,b in pairs),"ratio":statistics.median(ratios),"ci":[boots[125],boots[4874]],"iterations":n,"pairs":pairs}
    results[precision]=row
    print(precision,{k:v for k,v in row.items() if k!='pairs'},flush=True)
    Path('/tmp/published-reals-sqrt59-final-results.json').write_text(json.dumps(results,indent=2))
