"""Absolute resource observations for newly exact fractional text exports."""
import hashlib
import json
import os
from pathlib import Path
import resource
import statistics
import subprocess
import time

ROOT = Path('/home/tim/Documents/GitHub/workspace/csgrs')
E = ROOT / 'docs/evidence/2026-10-16-export-projection-contracts'
WORK = ROOT / 'target/properties/export-projection-contracts'
OUT = WORK / 'fractional-cost-v1'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


matched = json.loads((WORK / 'cost-v1/result.json').read_text())
assert matched['complete'] and matched['sample_rows'] == 180
OUT.mkdir(exist_ok=False)
bindings = json.loads((WORK / 'candidate-validation-v2/probe-bindings.json').read_text())
deps = WORK / 'candidate-target-v2/release/deps'
libraries = {str(path):sha(path) for path in deps.iterdir() if path.is_file() and path.suffix in {'.rlib','.rmeta','.so'}}
for record in bindings.values():
    assert sha(record['retained']) == record['sha256']
    libraries[record['retained']] = record['sha256']
dump(OUT / 'library-hashes.json', libraries)
protocol = {'runner_sha256':sha(__file__),'probe_sha256':sha(E / 'fractional_cost_v1.rs'),
            'contract_sha256':sha(E / 'fractional-cost-contract-v1.md'),'bindings':bindings,
            'dependencies':str(deps),'library_hashes_sha256':sha(OUT / 'library-hashes.json'),
            'preceding_matched_result_sha256':sha(WORK / 'cost-v1/result.json'),
            'hardware':subprocess.check_output(['lscpu'],text=True),
            'rustc':subprocess.check_output(['rustc','-vV'],text=True),
            'processes':5,'elapsed_limit_seconds':120,'address_space_limit_bytes':8*1024**3,
            'optimization_claimed':False,'matched_baseline_comparison':False,
            'completed_targets_modified':False}
dump(OUT / 'protocol.json',protocol)
rows = []
env = os.environ | {'TMPDIR':str(OUT),'RUST_BACKTRACE':'0'}


def limits():
    resource.setrlimit(resource.RLIMIT_AS,(8*1024**3,8*1024**3))
    resource.setrlimit(resource.RLIMIT_CORE,(0,0))


def run(name, command, directory):
    start=time.monotonic()
    with (directory/'stdout').open('wb') as stdout, (directory/'stderr').open('wb') as stderr:
        p=subprocess.run(['/usr/bin/time','-v','-o',str(directory/'resources'),
                          'timeout','--signal=TERM','--kill-after=10s','120s',*command],
                         cwd=ROOT,env=env,stdout=stdout,stderr=stderr,preexec_fn=limits,start_new_session=True)
    row={'name':name,'command':command,'exit_code':p.returncode,'elapsed_seconds':time.monotonic()-start,
         'directory':str(directory),**{key+'_sha256':sha(directory/key) for key in ['stdout','stderr','resources']}}
    rows.append(row)
    dump(OUT/'commands.json',rows)
    assert p.returncode==0,name


compile_dir=OUT/'compile'
compile_dir.mkdir()
binary=compile_dir/'fractional-cost'
command=['rustc','--edition=2024','-C','opt-level=3','-C','debug-assertions=off','-D','warnings','-L','dependency='+str(deps)]
for crate,record in bindings.items():
    command+=['--extern',crate+'='+record['retained']]
run('compile',command+[str(E/'fractional_cost_v1.rs'),'-o',str(binary)],compile_dir)
binary_record={'path':str(binary),'sha256':sha(binary),'bytes':binary.stat().st_size}
samples=[]
files={}
for repeat in range(5):
    directory=OUT/f'repeat-{repeat}'
    directory.mkdir()
    assert sha(binary)==binary_record['sha256']
    run(f'repeat-{repeat}',[str(binary),str(directory)],directory)
    observations=[json.loads(line) for line in (directory/'stdout').read_text().splitlines()]
    assert len(observations)==8 and all(row['completion']=='verified_exact_binary64_coordinate_projection' for row in observations)
    assert [row['iterations'] for row in observations]==[1,64]*4
    for row in observations:
        samples.append(row|{'repeat':repeat})
    for path in directory.glob('*.txt'):
        if path.name in files:
            assert path.read_bytes()==Path(files[path.name]['path']).read_bytes()
        else:
            files[path.name]={'path':str(path),'sha256':sha(path),'bytes':path.stat().st_size}
    dump(OUT/'samples.json',samples)
    print(f'Completed fractional observation {repeat+1}/5.',flush=True)
summaries=[]
for fixture in sorted({row['fixture'] for row in samples}):
    for phase in ['first','warm']:
        group=[row for row in samples if row['fixture']==fixture and row['phase']==phase]
        assert len(group)==5
        summary={'fixture':fixture,'phase':phase,'observations':len(group),
                 'median_ns_per_export':statistics.median(row['elapsed_ns']/row['iterations'] for row in group),
                 'allocation_calls_per_export':statistics.median(row['allocation_calls']/row['iterations'] for row in group),
                 'allocated_bytes_per_export':statistics.median(row['allocated_bytes']/row['iterations'] for row in group),
                 'retained_bytes_per_export':statistics.median(row['retained_delta_bytes']/row['iterations'] for row in group),
                 'maximum_additional_live_bytes_observed':max(row['peak_additional_live_bytes'] for row in group),
                 'output_bytes':group[0]['output_bytes'],'output_capacity':group[0]['output_capacity']}
        assert all(row['output_bytes']==summary['output_bytes'] and row['output_capacity']==summary['output_capacity'] for row in group)
        summaries.append(summary)
assert all(sha(path)==digest for path,digest in libraries.items())
assert sha(__file__)==protocol['runner_sha256'] and sha(E/'fractional_cost_v1.rs')==protocol['probe_sha256']
assert sha(E/'fractional-cost-contract-v1.md')==protocol['contract_sha256']
dump(OUT/'result.json',{'complete':True,'runtime_processes':5,'sample_rows':len(samples),
                      'library_inputs_unchanged':True,'all_repeated_complete_outputs_equal':True,
                      'binary':binary_record,'outputs':files,'summaries':summaries,
                      'matched_baseline_comparison':False,'optimization_claimed':False})
