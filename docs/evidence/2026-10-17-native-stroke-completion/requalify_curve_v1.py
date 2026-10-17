"""Complete the same all-features corpus with build and execution contained separately."""
import sys
sys.dont_write_bytecode=True
from requalification_common_v1 import Validation,read,WORK
prior=read(WORK/'candidate-validation-v4/result.json')
assert prior['complete'] and prior['unexpected_exits']==['curve-all']
failed=next(r for r in read(WORK/'candidate-validation-v4/commands.json') if r['name']=='curve-all')
assert failed['exit_code']==124 and failed['expected_exit_code']==0
assert ' ... FAILED' not in (WORK/'candidate-validation-v4/curve-all.stdout').read_text()
v=Validation('candidate',__file__)
v.run('curve-all-build',['cargo','test','--offline','--locked','-p','hypercurve','--release','--all-features','--no-run','--message-format=json'])
v.run('curve-all',['cargo','test','--no-fail-fast','--offline','--locked','-p','hypercurve','--release','--all-features','--','--test-threads=1'])
sys.exit(v.finish({'purpose':'Complete the same all-features suite after v4 combined build/test reached external timeout',
    'original_partial_attempt':'candidate-validation-v4/curve-all',
    'original_workload_gates_unchanged':True,'production_source_bytes_unchanged':True,
    'test_command_unchanged':True,'generated_case_configuration_changed':False,'compiler_and_execution_timeout_seconds_each':1800}))
