"""Retain both scalar regression outcomes, including every failure."""
from pathlib import Path
import json,subprocess
E=Path(__file__).resolve().parent
T=E.parents[2]/'target/properties/polynomial-quotient-reduction/factor-baseline-v1'
rows=[]
for name,command in [('units',[str(T/'units'),'ring_quotient_reduction_tests','--test-threads=2']),('public',[str(T/'public'),'--test-threads=2'])]:
 print(json.dumps({'started':name,'command':command}),flush=True)
 result=subprocess.run(command,timeout=120)
 rows.append({'name':name,'command':command,'exit_code':result.returncode})
p=E/'factor-baseline-results-v1.json';assert not p.exists();p.write_text(json.dumps(rows,indent=2)+'\n')
raise SystemExit(int(any(r['exit_code'] for r in rows)))
