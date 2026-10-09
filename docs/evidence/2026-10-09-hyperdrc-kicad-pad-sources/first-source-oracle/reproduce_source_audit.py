"""Measure loader capability changes and audit complete source geometry against both libraries.

Usage: python3 reproduce_source_audit.py COST_DIRECTORY BASELINE_LIBRARIES_JSON ORACLE_DIRECTORY NEW_OUTPUT_DIRECTORY
Run after all builds/tests/cost jobs exit. Baseline libraries must come from the
recorded base revision; their hashes are checked. Rebuild that revision in an
isolated checkout to reproduce on another host. Do not overlap Cargo builds.
"""
from pathlib import Path
import hashlib
import json
import re
import statistics
import subprocess
import sys

cost, baseline_path, oracle, out = map(Path,sys.argv[1:5])
out.mkdir(parents=True,exist_ok=False)
record=Path(__file__).resolve().parent
info=lambda p:dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
load=lambda p:json.loads(p.read_bytes())
manifest=load(cost/'manifest.json'); sources=manifest['source_hashes']
current=lambda:{p:info(Path(p))['sha256'] for p in sources}
assert manifest['source_unchanged'] and manifest['fixture_exit_code']==0 and current()==sources
baseline=load(baseline_path)
for data in baseline.values():assert info(Path(data['path']))=={k:v for k,v in data.items() if k!='path'}
artifacts=[json.loads(s) for s in (cost/'library-build.jsonl').read_text().splitlines()]
def artifact(name):return next(Path(f) for a in artifacts if a.get('reason')=='compiler-artifact' and a['target']['name']==name for f in a['filenames'] if f.endswith('.rlib'))
names=['hyperdrc','hypercurve','hyperlimit','serde_json']
libraries={'baseline':{n:Path(baseline[n]['path']) for n in names},'candidate':{n:artifact(n) for n in names}}
identities={};result={};builds=[]
for variant,linked in libraries.items():
    folder=out/variant;folder.mkdir()
    binaries={}
    for stem in ['source_probe','full_board_probe']:
        binary=folder/stem
        command=['rustc','--edition=2024','-O',str(record/(stem+'.rs'))]
        if variant=='baseline':command+=['-L',f'dependency={baseline_path.parent}']
        command+=['-L',f'dependency={Path.cwd()}/target/release/deps']
        for name,path in linked.items():command+=['--extern',f'{name}={path}']
        command+=['-o',str(binary)]
        with (folder/(stem+'-build.log')).open('w') as log:
            code=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT).returncode
        builds.append(dict(command=command,exit_code=code))
        (out/'builds.json').write_text(json.dumps(builds,indent=2)+'\n')
        assert code==0
        binaries[stem]=binary
    trials=[]
    for trial in range(1,6):
        command=['taskset','-c','0','/usr/bin/time','-v',str(binaries['source_probe']),str(out/'inputs')]
        with (folder/f'trial-{trial}.jsonl').open('w') as stdout,(folder/f'trial-{trial}.log').open('w') as stderr:
            code=subprocess.run(command,stdout=stdout,stderr=stderr).returncode
        rows=[json.loads(s) for s in (folder/f'trial-{trial}.jsonl').read_text().splitlines()]
        assert code==0 and len(rows)==18
        identity=[{k:v for k,v in row.items() if k!='elapsed_ns'} for row in rows]
        if trial==1:identities[variant]=identity
        else:assert identity==identities[variant]
        trials.append(dict(command=command,exit_code=code,rows=rows,peak_rss_kib=int(re.search(r'Maximum resident set size \(kbytes\): (\d+)',(folder/f'trial-{trial}.log').read_text())[1])))
    (folder/'rows.json').write_text(json.dumps(identities[variant],sort_keys=True,separators=(',',':'))+'\n')
    full=[]
    for label in ['cparti-fpga','hvp109a']:
        command=['taskset','-c','0','/usr/bin/time','-v',str(binaries['full_board_probe']),str(oracle/(label+'.json'))]
        with (folder/(label+'.json')).open('w') as stdout,(folder/(label+'.log')).open('w') as stderr:
            code=subprocess.run(command,stdout=stdout,stderr=stderr).returncode
        assert code==0
        audit=load(folder/(label+'.json'))
        full.append(dict(label=label,command=command,result={k:v if not isinstance(v,list) else len(v) for k,v in audit.items()},artifact=info(folder/(label+'.json'))))
    result[variant]=dict(trials=trials,medians_ns={r['name']:statistics.median(t['rows'][i]['elapsed_ns']/r['iterations'] for t in trials) for i,r in enumerate(trials[0]['rows'])},binaries={str(p):info(p) for p in binaries.values()},libraries={str(p):info(p) for p in linked.values()},complete_boards=full)
    print(variant,'completed',flush=True)
deltas=[]
for before,after in zip(identities['baseline'],identities['candidate']):
    assert before['name']==after['name'] and before['input']==after['input'] and before['iterations']==after['iterations']
    if before!=after:deltas.append(dict(before=before,after=after))
(out/'deltas.json').write_text(json.dumps(deltas,indent=2)+'\n')
result=dict(scope='Five sequential fresh processes per library, CPU 0; 18 complete fixed input strings and 50 loader iterations per row. Witness construction and required-marker checks are outside the loader timing. Different answers preclude a speedup claim. RSS and probe bytes only; no allocation-count or library-size optimization claim. Complete board source oracle validates primitive positions, exact areas and envelopes; custom local geometry is excluded.',variants=result,deltas=len(deltas),source_hashes=sources,source_unchanged=current()==sources,probe_sources={str(record/(name+'.rs')):info(record/(name+'.rs')) for name in ['source_probe','full_board_probe']},oracle_sources={str(p):info(p) for p in oracle.glob('*.json')},oracle_generator=info(record/'full_board_oracle.py'))
(out/'manifest.json').write_text(json.dumps(result,indent=2)+'\n')
assert result['source_unchanged']
assert all(board['result']['all_admitted'] for board in result['variants']['candidate']['complete_boards']), 'Candidate source oracle requires review'
