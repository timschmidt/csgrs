"""Run both preserved baseline contracts, retaining every test failure."""
from pathlib import Path
import json, subprocess
E=Path(__file__).resolve().parent
T=Path('/run/user/1000/csgrs-quotient-factor-proofs/scalar-zero-baseline-v1')
commands=[['units', [str(T/'units'), 'ring_quotient_tests', '--test-threads=2']], ['public', [str(T/'public'), '--test-threads=2']]]
rows=[]
for name, command in commands:
    print(json.dumps({'started':name, 'command':command}), flush=True)
    result=subprocess.run(command, timeout=120)
    rows.append({'name':name, 'command':command, 'exit_code':result.returncode})
out=E/'scalar-zero-baseline-results-v1.json'; assert not out.exists()
out.write_text(json.dumps(rows,indent=2)+'\n')
raise SystemExit(int(any(row['exit_code'] for row in rows)))
