"""Four new capability cost rows; five fresh processes, no speedup claim.
Usage: python3 reproduce_capability_cost.py COST_OUTPUT OUTPUT
"""
import hashlib,json,re,statistics,subprocess,sys
from pathlib import Path
record=Path(__file__).resolve().parent
cost,out=map(Path,sys.argv[1:3]);out.mkdir(parents=True,exist_ok=False)
info=lambda p:dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
artifacts=[json.loads(line) for line in (cost/'library-build.jsonl').read_text().splitlines()]
def rlib(name):
    return next(Path(f) for a in artifacts if a.get('reason')=='compiler-artifact' and a['target']['name']==name for f in a['filenames'] if f.endswith('.rlib'))
source=record/'capability_probe.rs';binary=out/'probe'
command=['rustc','--edition=2024','-O',str(source),'-L',f'dependency={Path.cwd()}/target/release/deps']
linked={name:rlib(name) for name in ['hyperdrc','hypercurve','hyperlimit','serde_json']}
for name,path in linked.items():command+=['--extern',f'{name}={path}']
command+=['-o',str(binary)]
with (out/'build.log').open('w') as log: subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
board=Path('/tmp/drc-native-rectangles-report-inputs/cparti-fpga/CPArti FPGA dev board.kicad_pcb')
observations=[]
for trial in range(1,6):
    run=['taskset','-c','0','/usr/bin/time','-v',str(binary),str(board)]
    with (out/f'trial-{trial}.json').open('w') as stdout,(out/f'trial-{trial}.log').open('w') as stderr:
        subprocess.run(run,stdout=stdout,stderr=stderr,check=True)
    rows=json.loads((out/f'trial-{trial}.json').read_bytes());assert len(rows)==4
    observations.append(dict(rows=rows,peak_rss_kib=int(re.search(r'Maximum resident set size \(kbytes\): (\d+)',(out/f'trial-{trial}.log').read_text())[1])))
identities=lambda o:[{k:v for k,v in r.items() if k!='elapsed_ns'} for r in o['rows']]
assert all(identities(o)==identities(observations[0]) for o in observations)
(out/'rows.json').write_text(json.dumps(identities(observations[0]),sort_keys=True,separators=(',',':'))+'\n')
result=dict(scope='Candidate capability cost. Five sequential fresh release processes pinned to CPU 0 after other work exits. Board parsing occurs outside timers but contributes to RSS. No matched speedup, allocation-count, library-size, or total resource-bound claim.',command=command,run_command=run,source=info(source),board=info(board),binary=info(binary),linked_libraries={str(p):info(p) for p in linked.values()},observations=observations,rows=info(out/'rows.json'),medians_ns={r['name']:statistics.median(o['rows'][i]['elapsed_ns']/o['rows'][i]['iterations'] for o in observations) for i,r in enumerate(observations[0]['rows'])})
(out/'manifest.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result['medians_ns']),flush=True)
