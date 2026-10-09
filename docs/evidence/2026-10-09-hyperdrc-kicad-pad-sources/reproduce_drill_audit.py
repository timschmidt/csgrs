"""Replay exact full-board drill source/spacing witnesses against complete candidate reports.

Usage: python3 reproduce_drill_audit.py COST_DIRECTORY ORACLE_DIRECTORY REPORT_DIRECTORY NEW_OUTPUT_DIRECTORY
Generate the oracle directory with drill_oracle.py first. This audit changes no
production source and uses the already frozen candidate release libraries.
"""
from pathlib import Path
import hashlib,json,subprocess,sys
cost,oracle,reports,out=map(Path,sys.argv[1:5]);out.mkdir(parents=True,exist_ok=False)
record=Path(__file__).resolve().parent
info=lambda p:dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
m=json.loads((cost/'manifest.json').read_bytes());sources=m['source_hashes'];assert all(info(Path(p))['sha256']==h for p,h in sources.items())
artifacts=[json.loads(l) for l in (cost/'library-build.jsonl').read_text().splitlines()]
def lib(name):return next(Path(f) for a in artifacts if a.get('reason')=='compiler-artifact' and a['target']['name']==name for f in a['filenames'] if f.endswith('.rlib'))
binary=out/'drill-probe';command=['rustc','--edition=2024','-O',str(record/'drill_probe.rs'),'-L',f'dependency={Path.cwd()}/target/release/deps']
linked={name:lib(name) for name in ['hyperdrc','hyperlimit','serde_json']}
for name,path in linked.items():command+=['--extern',f'{name}={path}']
command+=['-o',str(binary)]
with (out/'build.log').open('w') as log:result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
assert result.returncode==0
results=[]
for label in ['cparti-fpga','hvp109a']:
    run=[str(binary),str(oracle/(label+'.json')),str(reports/'trial-1'/(label+'.json'))]
    with (out/(label+'.json')).open('w') as stdout,(out/(label+'.log')).open('w') as stderr:
        code=subprocess.run(run,stdout=stdout,stderr=stderr).returncode
    results.append(dict(label=label,command=run,exit_code=code,oracle=info(oracle/(label+'.json')),report=info(reports/'trial-1'/(label+'.json')),result=json.loads((out/(label+'.json')).read_bytes()) if code==0 else None))
manifest=dict(scope='Exact independent rational/quadratic-surd source positions, diameters and plating for every parsed drill on both full boards, and complete reported clearance-pair multiset at 0.25 mm. Oval/rectangular drills use the existing conservative larger-axis circular proxy; routed-slot geometry remains open.',build_command=command,results=results,source_hashes=sources,source_unchanged=all(info(Path(p))['sha256']==h for p,h in sources.items()),oracle_generator=info(record/'drill_oracle.py'),source_oracle_dependency=info(record/'full_board_oracle.py'),probe_source=info(record/'drill_probe.rs'),binaries={str(binary):info(binary)},libraries={str(p):info(p) for p in linked.values()})
(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps(results,indent=2));assert manifest['source_unchanged'] and all(r['exit_code']==0 for r in results)
