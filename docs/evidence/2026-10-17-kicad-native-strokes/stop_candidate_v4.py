"""Stop only the identified rejected candidate; retain cancellation as incomplete."""
import hashlib,json,os,signal,time
from pathlib import Path
W=Path(__file__).resolve().parent; O=W/'candidate-v4-cancellation-v1'; O.mkdir(exist_ok=False)
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
observation=json.loads((W/'gate-observation-v2/result.json').read_text())
assert len(observation['processes'])==1
test=observation['processes'][0]; pid=test['pid']; root=Path('/proc')/str(pid)
assert (root/'cmdline').read_bytes().decode().split('\0')==test['args']
assert sha(test['args'][0])==test['exe_sha256']
assert any(t['cpu_seconds']>120 for t in test['tasks'])
ancestors=[]; current=pid; runner=None
while current>1:
    p=Path('/proc')/str(current)
    stat=(p/'stat').read_text();tail=stat[stat.rindex(')')+2:].split()
    args=(p/'cmdline').read_bytes().decode().split('\0')
    ancestors.append({'pid':current,'args':args,'stat':stat})
    if len(args)>1 and args[1].endswith('target/properties/kicad-native-strokes/candidate_v4.py'):
        runner=current;break
    current=int(tail[1])
assert runner is not None
record={'runner_sha256':sha(__file__),'observation_sha256':sha(W/'gate-observation-v2/result.json'),
    'reason':'Revised candidate exceeded 120 seconds worker CPU before profiling; layer-sanity took 247.806 seconds and native copper-neck work remains expensive. Report-state tests pass. Full gate is cancelled incomplete.',
    'ancestors':ancestors,'intended_signals':[{'pid':runner,'signal':'SIGINT'},{'pid':pid,'signal':'SIGTERM'}],
    'test_assertion_completed':False,'full_board_admitted':False,'remaining_commands_attempted':False}
(O/'protocol.json').write_text(json.dumps(record,indent=2)+'\n')
os.kill(runner,signal.SIGINT)
os.kill(pid,signal.SIGTERM)
observed=[]
for _ in range(50):
    states=[]
    for a in ancestors:
        try:
            stat=(Path('/proc')/str(a['pid'])/'stat').read_text();tail=stat[stat.rindex(')')+2:].split()
            states.append({'pid':a['pid'],'state':tail[0]})
        except FileNotFoundError:states.append({'pid':a['pid'],'state':'absent'})
    observed.append(states)
    if all(r['state'] in ['absent','Z'] for r in states):break
    time.sleep(.1)
assert all(r['state'] in ['absent','Z'] for r in observed[-1])
files={str(p):sha(p) for p in (W/'candidate-validation-v4').iterdir() if p.is_file()}
(O/'result.json').write_text(json.dumps({'complete':True,'signals_sent':record['intended_signals'],
    'final_process_states':observed[-1],'terminal_validation_files_sha256':files,
    'cargo_target_closed':True,'full_board_result':'cancelled incomplete; no passing gate or completed assertion credited',
    'subsequent_original_gates_and_suite':'not attempted on rejected v3; retained for next candidate'},indent=2)+'\n')
print(json.dumps({'complete':True,'closed_process_states':observed[-1],'candidate_admitted':False}))
