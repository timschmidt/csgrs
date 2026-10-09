"""Run after the candidate validation and board process have exited.

Usage: python3 reproduce_normal_form_ownership_cost.py NEW_OUTPUT_DIRECTORY
Baseline libraries were copied before the trial from the source-frozen build
in current-correctness-candidate. On another host, check out that record's base
revision, overlay its source archive, and rebuild the recorded Cargo command
in an isolated target directory; update the baseline library metadata path.
"""
import hashlib
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import statistics
import subprocess
import sys
import time

root=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('output',type=Path)
parser.add_argument('--candidate-evidence',type=Path,default=root/'normal-form-ownership-candidate')
parser.add_argument('--candidate-source',type=Path,default=Path.cwd())
parser.add_argument('--baseline-evidence',type=Path,default=root/'current-correctness-candidate')
parser.add_argument('--baseline-metadata',type=Path,default=Path('/tmp/drc-native-ownership-baseline-libraries/libraries.json'))
parser.add_argument('--expected-source-change',action='append',help='Expected changed source path; repeat for a storage/dependency trial. Defaults to algebra.rs alone.')
args=parser.parse_args()
candidate=args.candidate_evidence.resolve();baseline=args.baseline_evidence.resolve()
candidate_source=args.candidate_source.resolve()
out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
baseline_metadata=args.baseline_metadata.resolve()
base=json.loads(baseline_metadata.read_text())
sources=json.loads((candidate/'sources.json').read_text())
old_sources=json.loads((baseline/'sources.json').read_text())
changed=[name for name,h in sources['source_hashes'].items() if old_sources['source_hashes'].get(name)!=h]
expected=args.expected_source_change or ['crates/hyperreal/src/computable/node/algebra.rs']
assert changed==sorted(expected)
statuses=json.loads((candidate/'status.json').read_text())
assert statuses[-1]['name']=='full-board-smoke'
assert all(r['exit_code']==0 for r in statuses[:-1])
assert statuses[-1]['exit_code'] in (0,101)
def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def verify():
    assert all(digest(candidate_source/name)==h for name,h in sources['source_hashes'].items())
    assert all(digest(r['path'])==r['sha256'] for r in base['libraries'])
verify()
shutil.copy2(root/'normal_form_ownership_probe.rs',out/'probe.rs')
shutil.copy2(baseline_metadata,out/'baseline-libraries.json')
shutil.copy2(__file__,out/'reproduce.py')
libraries={'baseline':Path(next(r['path'] for r in base['libraries'] if r['crate']=='hyperreal'))}
for line in (candidate/'build-board-smoke.log').read_text().splitlines():
    try:a=json.loads(line)
    except ValueError:continue
    if a.get('reason')=='compiler-artifact' and a.get('target',{}).get('name')=='hyperreal':
        libraries['candidate']=Path(next(f for f in a['filenames'] if f.endswith('.rlib')))
assert set(libraries)=={'baseline','candidate'}
commands=[];binaries={};trials=[]
def run(name,command):
    verify();start=time.monotonic()
    with (out/(name+'.log')).open('w') as stderr, (out/(name+'.jsonl')).open('w') as stdout:
        code=subprocess.run(command,stdout=stdout,stderr=stderr).returncode
    commands.append(dict(name=name,command=command,exit_code=code,wall_seconds=time.monotonic()-start))
    (out/'commands.json').write_text(json.dumps(commands,indent=2)+'\n')
    verify()
    assert code==0, name
    return (out/(name+'.jsonl')).read_text()
for name,library in libraries.items():
    for mode in ('time','memory'):
        binary=out/(name+'-'+mode)
        command=['rustc','--edition=2024','-O','-D','warnings',str(out/'probe.rs'),'-L','dependency='+str(library.parent),'--extern','hyperreal='+str(library),'-o',str(binary)]
        if mode=='memory':command+=['--cfg','feature="count-alloc"']
        run('build-'+name+'-'+mode,command)
        binaries[name,mode]=binary
        run('preflight-'+name+'-'+mode,['taskset','-c','0',str(binary),'42'])
for mode,count in [('time',4096),('memory',1000)]:
    for index in range(1,6):
        order=('baseline','candidate') if index%2 else ('candidate','baseline')
        for name in order:
            label=f'{mode}-{index}-{name}'
            command=['taskset','-c','0','/usr/bin/time','-v',str(binaries[name,mode]),str(count)]
            text=run(label,command)
            rows=[json.loads(line) for line in text.splitlines()]
            assert len(rows)==48
            witnesses=[r for r in rows if 'witness' in r or 'fixture' in r]
            assert len(witnesses)==24
            rss=int(re.search(r'Maximum resident set size \(kbytes\): (\d+)',(out/(label+'.log')).read_text())[1])
            trials.append(dict(mode=mode,index=index,variant=name,rows=rows,maximum_rss_kib=rss))
            print(label,'passed',flush=True)
for mode in ('time','memory'):
    matching=[t for t in trials if t['mode']==mode]
    first=[r for r in matching[0]['rows'] if 'witness' in r or 'fixture' in r]
    assert all([r for r in t['rows'] if 'witness' in r or 'fixture' in r]==first for t in matching)
summaries=[]
for mode in ('time','memory'):
    names=[r['name'] for r in trials[0]['rows'] if 'name' in r]
    for name in names:
        values={v:[next(r for r in t['rows'] if r.get('name')==name) for t in trials if t['mode']==mode and t['variant']==v] for v in ('baseline','candidate')}
        fields=('nanos',) if mode=='time' else ('allocation_calls','allocated_bytes','retained_bytes_delta','peak_bytes_delta')
        stats={v:{field:dict(samples=[r[field] for r in rows],median=statistics.median(r[field] for r in rows)) for field in fields} for v,rows in values.items()}
        summaries.append(dict(mode=mode,name=name,variants=stats))
binary_records={}
for (name,mode),binary in binaries.items():
    section_output=subprocess.check_output(['size','-A',str(binary)],text=True)
    (out/(name+'-'+mode+'-size.log')).write_text(section_output)
    binary_records[name+'-'+mode]=dict(bytes=binary.stat().st_size,sha256=digest(binary),sections=section_output)
manifest=dict(scope='Five fresh processes per variant/mode, alternating order, CPU 0. Timing binaries have no custom allocator; memory binaries count requested Rust bytes, excluding allocator metadata and native C heaps. Six independently derived identities; construction, first/warm/cloned public exact-rational-normal-form queries. The input receipt records cases already reduced by construction. This is a focused matched probe, not full application or cross-library admission.',source_change=changed,baseline_source=str(baseline/'sources.json'),candidate_source=str(candidate/'sources.json'),candidate_source_directory=str(candidate_source),source_unchanged=True,probe_sha256=digest(out/'probe.rs'),libraries={name:dict(path=str(p),bytes=p.stat().st_size,sha256=digest(p)) for name,p in libraries.items()},compiler=subprocess.check_output(['rustc','-Vv'],text=True),host=os.uname()._asdict() if hasattr(os.uname(),'_asdict') else tuple(os.uname()),cpuinfo=Path('/proc/cpuinfo').read_text().split('\n\n')[0],commands=commands,binaries=binary_records,trials=trials,summaries=summaries)
(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('paired normal-form cost probe complete',flush=True)
