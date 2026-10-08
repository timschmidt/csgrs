import json,random,statistics,subprocess
from pathlib import Path
rng=random.Random(2930)
binary='/tmp/interval-computable-audit/target/release/performance'
def run(lib,case,p,n):
    return float(subprocess.check_output(['taskset','-c','6',binary,lib,case,str(p),str(n)],text=True))
rows={}
for case,p in [('clone',64),('cached',64)]+[(c,p) for c in ['sqrt','inverse'] for p in [16,64,128]]+[('mixed',16)]:
    ns={lib:max(10,min(200000,round(20_000_000/run(lib,case,p,10)))) for lib in ['donor','hyper']}
    pairs=[]
    for _ in range(11):
        a=run('donor',case,p,ns['donor']);b=run('hyper',case,p,ns['hyper'])
        bb=run('hyper',case,p,ns['hyper']);aa=run('donor',case,p,ns['donor'])
        pairs.append([(a+aa)/2,(b+bb)/2])
    ratios=[b/a for a,b in pairs]
    boots=sorted(statistics.median(rng.choices(ratios,k=11)) for _ in range(5000))
    row={'donor_ns':statistics.median(a for a,b in pairs),'hyper_ns':statistics.median(b for a,b in pairs),'ratio':statistics.median(ratios),'ci':[boots[125],boots[4874]],'iterations':ns,'pairs':pairs}
    rows[f'{case}/{p}']=row
    print(case,p,{k:v for k,v in row.items() if k!='pairs'},flush=True)
    Path('/tmp/interval-computable-performance-results.json').write_text(json.dumps(rows,indent=2))
