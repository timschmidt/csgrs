"""Compare five fixed pairs of completed valid OBJ exports, retaining unfavorable samples."""
import hashlib
import json
import os
from pathlib import Path
import resource
import statistics
import subprocess
import time

ROOT = Path('/home/tim/Documents/GitHub/workspace/csgrs')
E24 = ROOT / 'docs/evidence/2026-10-16-physical-build-requalification'
WORK = Path('/dev/shm/csgrs-obj-index-contracts')
OUT = WORK / 'cost-v1'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


print('Waiting for candidate checks and physical-build checkpoint before fixed OBJ cost pairs.',flush=True)
while not (WORK / 'candidate-validation-v1/result.json').exists() or not (E24 / 'checkpoint-v1.json').exists():
    time.sleep(1)
qualification = json.loads((WORK / 'candidate-validation-v1/result.json').read_text())
assert qualification['complete'] and not qualification['unexpected_exits']
protocol_before = json.loads((WORK / 'candidate-validation-v1/protocol.json').read_text())
binaries = {variant: WORK / ('obj-cost-'+variant+'-v1') for variant in ['baseline','candidate']}
assert all(sha(path) == qualification['probe_binaries_sha256'][str(path)] for path in binaries.values())
assert sha(protocol_before['cost_probe']) == protocol_before['cost_probe_sha256']
OUT.mkdir(exist_ok=False)
orders = [['baseline','candidate'] if pair % 2 == 0 else ['candidate','baseline'] for pair in range(5)]
protocol = {'runner_sha256':sha(__file__),'orders':orders,'binaries':{v:str(p) for v,p in binaries.items()},
            'binary_sha256':{v:sha(p) for v,p in binaries.items()},
            'probe_sha256':protocol_before['cost_probe_sha256'],
            'qualification_sha256':sha(WORK/'candidate-validation-v1/result.json'),
            'hardware':subprocess.check_output(['lscpu'],text=True),
            'timed_contract':'One complete to_obj String per iteration; every String retained until timing ends. One warmup, native strict topology/volume validation, input construction, disk writes and exact output comparison are outside timing.',
            'cases':{'empty':65536,'unit':4096,'disconnected_256':32},
            'cache_condition':'Warm repeated export of immutable input, fresh process per variant and pair. No cold-export claim.',
            'memory_scope':'Retained String capacities only; not all allocations, native heap bytes or an internal memory bound.',
            'wall_limit_seconds_per_process':120,'address_space_limit_bytes_per_process':8*1024**3,
            'application_speedup_claimed':False,'optimization_claimed':False,'runtime_admitted':False}
dump(OUT/'protocol.json',protocol)
rows = []


def limits():
    resource.setrlimit(resource.RLIMIT_AS,(8*1024**3,8*1024**3))
    resource.setrlimit(resource.RLIMIT_CORE,(0,0))


for pair, order in enumerate(orders):
    for variant in order:
        name = f'pair-{pair+1}-{variant}'
        directory = OUT/name
        directory.mkdir(exist_ok=False)
        assert sha(binaries[variant]) == protocol['binary_sha256'][variant]
        command = [str(binaries[variant]),str(directory)]
        start=time.monotonic()
        with (OUT/(name+'.stdout')).open('wb') as stdout,(OUT/(name+'.stderr')).open('wb') as stderr:
            result=subprocess.run(['/usr/bin/time','-v','-o',str(OUT/(name+'.resources')),
                'timeout','--signal=TERM','--kill-after=10s','120s',*command],cwd=WORK,
                env=os.environ|{'TMPDIR':str(WORK/'candidate-scratch-v1')},stdout=stdout,stderr=stderr,
                preexec_fn=limits,start_new_session=True)
        raw={'pair':pair+1,'variant':variant,'command':command,'exit_code':result.returncode,
             'elapsed_seconds':time.monotonic()-start,
             **{s+'_sha256':sha(OUT/(name+'.'+s)) for s in ['stdout','stderr','resources']}}
        dump(OUT/(name+'.json'),raw)
        assert result.returncode==0,raw
        measured=[json.loads(line) for line in (OUT/(name+'.stdout')).read_text().splitlines()]
        assert {r['case']:r['iterations'] for r in measured} == protocol['cases']
        for row in measured:
            output=directory/(row['case']+'.obj')
            assert row['all_outputs_identical'] and output.stat().st_size==row['output_bytes']
            row |= {'pair':pair+1,'variant':variant,'output_path':str(output),'output_sha256':sha(output)}
        rows.extend(measured)
        dump(OUT/'rows.json',rows)
        print(name+' completed',flush=True)
summary=[]
for case in protocol['cases']:
    sample={v:sorted([r for r in rows if r['case']==case and r['variant']==v],key=lambda r:r['pair']) for v in binaries}
    expected=Path(sample['baseline'][0]['output_path']).read_bytes()
    assert all(Path(r['output_path']).read_bytes()==expected for group in sample.values() for r in group)
    assert len({r['retained_string_capacity_bytes'] for group in sample.values() for r in group})==1
    ratios=[candidate['nanoseconds']/baseline['nanoseconds'] for baseline,candidate in zip(sample['baseline'],sample['candidate'])]
    summary.append({'case':case,'complete_output_bytes_equal_in_all_processes':True,
                    'output_bytes':len(expected),'output_sha256':hashlib.sha256(expected).hexdigest(),
                    'retained_string_capacity_bytes':sample['baseline'][0]['retained_string_capacity_bytes'],
                    'baseline_nanoseconds':[r['nanoseconds'] for r in sample['baseline']],
                    'candidate_nanoseconds':[r['nanoseconds'] for r in sample['candidate']],
                    'paired_ratios':ratios,'median_paired_ratio':statistics.median(ratios),
                    'slower_candidate_pairs':sum(r>1 for r in ratios)})
assert sha(__file__)==protocol['runner_sha256']
assert all(sha(p)==protocol['binary_sha256'][v] for v,p in binaries.items())
record={'complete':True,'fresh_processes':10,'completed_rows':len(rows),'summary':summary,
        'binary_bytes':{v:p.stat().st_size for v,p in binaries.items()},
        'all_exact_output_bytes_equal':True,'all_retained_string_capacities_equal':True,
        'application_speedup_claimed':False,'statistical_significance_claimed':False,
        'cold_export_measured':False,'total_allocations_measured':False,
        'runtime_admitted':False,'full_plan_complete':False}
dump(OUT/'result.json',record)
print(json.dumps(record),flush=True)
