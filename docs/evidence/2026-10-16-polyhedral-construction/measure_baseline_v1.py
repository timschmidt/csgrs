import json, subprocess
from pathlib import Path
E=Path(__file__).resolve().parent
out=E/'baseline-cost-v2'
rows=[]
for name,cmd in json.loads((E/'baseline-cost-v2-children.json').read_text()):
    result=subprocess.run(cmd,capture_output=True,text=True,timeout=120)
    (out/(name+'.stdout')).write_text(result.stdout)
    (out/(name+'.stderr')).write_text(result.stderr)
    rows.append({'name':name,'command':cmd,'exit_code':result.returncode})
    print(name,result.returncode,flush=True)
(out/'measurements.json').write_text(json.dumps(rows,indent=2)+'\n')
assert all(r['exit_code']==0 for r in rows)
