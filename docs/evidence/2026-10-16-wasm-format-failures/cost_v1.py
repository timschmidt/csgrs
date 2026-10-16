"""Matched completed exports and candidate-only complete pipelines/error receipts."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import resource
import statistics
import subprocess
import sys
import time

ROOT=Path('/home/tim/Documents/GitHub/workspace/csgrs')
E=ROOT/'docs/evidence/2026-10-16-wasm-format-failures'
WORK=ROOT/'target/properties/wasm-format-failures'
OLD=ROOT/'target/properties/export-projection-contracts'
OUT=WORK/'cost-v1'
PROBE=E/'cost_probe_v1.cjs'
FIXTURE=E/'cost_fixture_v1.cjs'
MODULES={'baseline':OLD/'wasm-target-v1/wasm-bindings-node/csgrs.js',
         'candidate':WORK/'wasm-target-v1/wasm-bindings-node/csgrs.js'}


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


def read(path):return json.loads(Path(path).read_text())
def dump(path,value):Path(path).write_text(json.dumps(value,indent=2)+'\n')

native=read(WORK/'native-validation-v1/result.json')
wasm=read(WORK/'wasm-validation-v1/result.json')
old=read(OLD/'wasm-validation-v1/result.json')
assert native['complete'] and not native['unexpected_exits']
assert wasm['complete'] and not wasm['unexpected_exits'] and wasm['real_js_and_compiled_typescript_consumers_passed']
assert old['all_commands_passed']
artifacts=old['artifacts']|wasm['artifacts']
assert all(sha(path)==record['sha256'] for path,record in artifacts.items())
files={str(path):sha(path) for path in [Path(__file__),PROBE,FIXTURE,E/'cost-contract-v1.md',
    WORK/'native-validation-v1/result.json',WORK/'wasm-validation-v1/result.json',
    OLD/'wasm-validation-v1/result.json',WORK/'preparation-v1/source-hashes.json.gz']}
OUT.mkdir(exist_ok=False)
protocol={'runner_probe_fixture_contract_and_qualification_sha256':files,
          'artifacts_read_only':artifacts,'modules':{key:str(path) for key,path in MODULES.items()},
          'node':subprocess.check_output(['node','--version'],text=True).strip(),
          'kernel':platform.platform(),'cpu':subprocess.check_output(['lscpu'],text=True),
          'meminfo':Path('/proc/meminfo').read_text(),'loadavg_initial':Path('/proc/loadavg').read_text(),
          'native_and_wasm_qualification_complete':True,'cargo_builds_invoked':False,
          'matched_pairs':5,'candidate_only_repeats':5,'encodings':['binary','ascii'],'cubes':[0,1,256],
          'first_iterations':1,'warm_iterations_small':64,'warm_iterations_256_cubes':4,
          'typed_errors_per_iteration':8,'elapsed_limit_seconds_per_process':120,
          'address_space_limit_bytes_per_process':64*1024**3,
          'timing_contract':(E/'cost-contract-v1.md').read_text(),
          'optimization_claimed':False,'rust_allocator_instrumented':False,
          'public_resource_bound_added':False,'external_host_load_controlled':False}
dump(OUT/'protocol.json',protocol)
env=os.environ.copy()
env.pop('NODE_OPTIONS',None)
env['TMPDIR']=str(OUT)

def limits():
    resource.setrlimit(resource.RLIMIT_AS,(64*1024**3,64*1024**3))
    resource.setrlimit(resource.RLIMIT_CORE,(0,0))

commands=[]
rows=[]
references={}

def run(mode,encoding,cubes,repeat,variant):
    assert all(sha(path)==digest for path,digest in files.items())
    name=f'{mode}-{encoding}-{cubes}-{repeat}-{variant}'
    directory=OUT/name
    directory.mkdir(exist_ok=False)
    command=['node','--expose-gc',str(PROBE),str(MODULES[variant]),str(directory),mode,encoding,str(cubes)]
    start=time.monotonic()
    loadavg_before=Path('/proc/loadavg').read_text().strip()
    with (directory/'stdout').open('wb') as stdout,(directory/'stderr').open('wb') as stderr:
        p=subprocess.run(['/usr/bin/time','-v','-o',str(directory/'resources'),
                          'timeout','--signal=TERM','--kill-after=10s','120s',*command],
                         cwd=ROOT,env=env,stdout=stdout,stderr=stderr,
                         preexec_fn=limits,start_new_session=True)
    record={'name':name,'command':command,'exit_code':p.returncode,
            'elapsed_seconds':time.monotonic()-start,'loadavg_before':loadavg_before,
            'loadavg_after':Path('/proc/loadavg').read_text().strip(),
            'files':{path.name:{'sha256':sha(path),'bytes':path.stat().st_size}
                     for path in sorted(directory.iterdir()) if path.is_file()}}
    commands.append(record)
    dump(OUT/'commands.json',commands)
    if p.returncode!=0:
        dump(OUT/'result.json',{'complete':False,'failed':name,'completed_processes':len(commands),
                              'all_runs_retained':True,'optimization_claimed':False})
        print(json.dumps(record),flush=True)
        sys.exit(101)
    observed=[json.loads(line) for line in (directory/'stdout').read_text().splitlines()]
    assert len(observed)==2 and [row['phase'] for row in observed]==['first','warm']
    rss=int(re.search(r'Maximum resident set size \(kbytes\):\s+(\d+)',(directory/'resources').read_text()).group(1))
    for row in observed:
        assert row['mode']==mode and row['encoding']==encoding and row['cubes']==cubes
        assert row['operations']>0 and row['elapsed_ns']>0
        row.update({'repeat':repeat,'variant':variant,'process':name,'peak_rss_kib':rss,
                    'ns_per_operation':row['elapsed_ns']/row['operations']})
        rows.append(row)
    key=(mode,encoding,cubes)
    if mode!='errors':
        required=['input.json','output.data']+(['input.stl'] if mode=='pipeline' else [])
        if key in references:
            for filename in required:
                assert (directory/filename).read_bytes()==(references[key]/filename).read_bytes()
        else:references[key]=directory
    else:
        records=read(directory/'errors.json')
        assert len(records)==8
    dump(OUT/'rows.json',rows)
    print(json.dumps({'process':name,'exit_code':0,'rows':len(observed),'elapsed_seconds':record['elapsed_seconds']}),flush=True)

for encoding in ['binary','ascii']:
    for cubes in [0,1,256]:
        for repeat in range(5):
            for variant in (['baseline','candidate'] if repeat%2==0 else ['candidate','baseline']):
                run('export',encoding,cubes,repeat,variant)
for encoding in ['binary','ascii']:
    for cubes in [0,1,256]:
        for repeat in range(5):run('pipeline',encoding,cubes,repeat,'candidate')
for repeat in range(5):run('errors','none',0,repeat,'candidate')
assert len(commands)==95 and len(rows)==190
assert all(sha(path)==digest for path,digest in files.items())
assert all(sha(path)==record['sha256'] for path,record in artifacts.items())
summaries=[]
for mode,encoding,cubes in sorted({(row['mode'],row['encoding'],row['cubes']) for row in rows}):
    for phase in ['first','warm']:
        group=[row for row in rows if (row['mode'],row['encoding'],row['cubes'],row['phase'])==(mode,encoding,cubes,phase)]
        summary={'mode':mode,'encoding':encoding,'cubes':cubes,'phase':phase,'variants':{}}
        for variant in sorted({row['variant'] for row in group}):
            selected=[row for row in group if row['variant']==variant]
            summary['variants'][variant]={
                'samples':len(selected),'ns_per_operation_median':statistics.median(row['ns_per_operation'] for row in selected),
                'ns_per_operation_min':min(row['ns_per_operation'] for row in selected),
                'ns_per_operation_max':max(row['ns_per_operation'] for row in selected),
                'peak_rss_kib_median':statistics.median(row['peak_rss_kib'] for row in selected),
                'peak_rss_kib_max':max(row['peak_rss_kib'] for row in selected),
                'retained_memory_medians':{key:statistics.median(row['retained'][key] for row in selected) for key in selected[0]['retained']},
                'after_memory_maxima':{key:max(row['after'][key] for row in selected) for key in selected[0]['after']},
                'output_bytes':selected[0]['output_bytes'],'output_count':selected[0]['output_count']}
        if mode=='export':
            baseline={row['repeat']:row for row in group if row['variant']=='baseline'}
            candidate={row['repeat']:row for row in group if row['variant']=='candidate'}
            ratios=[candidate[i]['ns_per_operation']/baseline[i]['ns_per_operation'] for i in range(5)]
            summary['paired_candidate_over_baseline_ratios']=ratios
            summary['paired_ratio_median']=statistics.median(ratios)
        summaries.append(summary)
size_changes={}
for filename in ['csgrs.js','csgrs.d.ts','csgrs_bg.wasm','csgrs_bg.wasm.d.ts']:
    before=MODULES['baseline'].parent/filename
    after=MODULES['candidate'].parent/filename
    size_changes[filename]={'baseline_bytes':before.stat().st_size,'candidate_bytes':after.stat().st_size,
                            'delta_bytes':after.stat().st_size-before.stat().st_size}
dump(OUT/'summary.json',{'groups':summaries,'artifact_size_changes':size_changes})
result={'complete':True,'processes':len(commands),'rows':len(rows),'all_exit_codes_zero':True,
        'matched_export_processes':60,'candidate_pipeline_processes':30,'candidate_error_processes':5,
        'complete_inputs_and_outputs_byte_equal':True,'independent_integer_oracle_passed':True,
        'strict_parsed_geometry_pipeline_receipts_preserved':True,'typed_failures_never_returned_models':True,
        'artifacts_and_probes_unchanged':True,'optimization_claimed':False,
        'rust_allocator_instrumented':False,'global_resource_bounds_claimed':False,
        'summary_sha256':sha(OUT/'summary.json'),'commands_sha256':sha(OUT/'commands.json'),'rows_sha256':sha(OUT/'rows.json')}
dump(OUT/'result.json',result)
print(json.dumps(result),flush=True)
