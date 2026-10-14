import gzip,hashlib,json,re,subprocess
from pathlib import Path
root=Path.cwd();e=root/'docs/evidence/2026-10-14-sampled-sphere-hull';out=e/'normal-image-asan-v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
protocol=json.loads((out/'protocol.json').read_text()); failure=json.loads((out/'failure.json').read_text()); commands=json.loads((out/'commands.json').read_text())
assert not (out/'result.json').exists() and not failure['complete']
assert [x['name'] for x in commands]==['control-format','control-build','control-expected-failure','default-tests']
assert [x['exit_code'] for x in commands]==[0,0,66,66]
assert failure['failed_command']==commands[-1]
for x in commands:
 for stream in ['stdout','stderr']:assert sha(out/(x['name']+'.'+stream))==x[stream+'_sha256']
assert all(sha(e/p)==h for p,h in protocol['inputs_sha256'].items())
mapping=json.loads(gzip.decompress((e/'sphere-normal-image-v1/source-hashes.json.gz').read_bytes()));roots=json.loads(gzip.decompress((out/'root-source-hashes.json.gz').read_bytes()))
assert len(mapping)==10136 and len(roots)==10129
assert all(sha(Path(protocol['source'])/p)==h for p,h in mapping.items())
assert all(sha(root/p)==h for p,h in roots.items())
for name in ['binaries.json','libraries.json']:
 assert all(sha(x['path'])==x['sha256'] for x in json.loads((e/'sphere-normal-image-v1'/name).read_text()).values())
raw=(out/'default-tests.stderr').read_text();stdout=(out/'default-tests.stdout').read_text()
assert 'LeakSanitizer has encountered a fatal error.' in raw
assert 'LeakSanitizer does not work under ptrace' in raw
assert 'ERROR: AddressSanitizer:' not in raw and 'ERROR: LeakSanitizer: detected memory leaks' not in raw
counts=[tuple(map(int,x)) for x in re.findall(r'^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;',stdout,re.M)]
assert counts==[(242,0,0,0,0)]
executed=Path('target/normal-image-asan-v1/target/x86_64-unknown-linux-gnu/release/deps/hyperlimit-9162eeee8c99e3a1').resolve()
control=json.loads((out/'control-binary.json').read_text());assert sha(Path(control['path']))==control['sha256']
closure={
 'complete':False,'trial_closed':True,'driver_terminal_exit_code':1,
 'commands_completed':4,'commands_matching_expected_exit':3,
 'deliberate_control_detected_expected_heap_buffer_overflow':True,
 'failed_command':'default-tests','failed_command_exit_code':66,
 'completed_test_summaries':[{'passed':242,'failed':0,'ignored':0,'measured':0,'filtered_out':0}],
 'failure':'The first default test binary completed 242 tests successfully, then LeakSanitizer reported a fatal scanning error at shutdown. The run stopped; later default binaries, all-feature tests and the original hull replay did not execute.',
 'diagnostic_classification':'LeakSanitizer infrastructure failure; the diagnostic gives a generic ptrace warning. Its exact cause has not yet been demonstrated. No leak report or library invalid-access diagnostic was produced.',
 'executed_test_binary':{'path':str(executed),'sha256':sha(executed),'bytes':executed.stat().st_size},
 'executed_control_binary':control,
 'frozen_source_files_rechecked':len(mapping),'root_source_files_rechecked':len(roots),
 'all_frozen_sources_original_libraries_and_binaries_unchanged':True,
 'artifacts_sha256':{p.name:sha(p) for p in sorted(out.iterdir()) if p.is_file()},
 'driver_stdout_sha256':sha(e/'normal-image-asan-driver-v1.stdout'),
 'driver_stderr_sha256':sha(e/'normal-image-asan-driver-v1.stderr'),
 'next_step':'A separately frozen standalone ASan/LSan control will compare sandboxed and separately authorized unrestricted execution. Preserve this failed run; do not relabel it, disable leak checks in place, or resume it.',
 'runtime_admitted':False,'plan_complete':False,
}
closure['closure_reader_source_sha256']=sha(Path(__file__))
closure['closure_reader_failure']='normal-image-asan-closure-reader-failure-v1.json'
p=e/'normal-image-asan-closure-v1.json';assert not p.exists();p.write_text(json.dumps(closure,indent=2)+'\n')
p=root/'docs/properties-status.json';ledger=json.loads(p.read_text());base=json.loads(subprocess.check_output(['git','show','9025de62:docs/properties-status.json']))
assert {k:v for k,v in ledger.items() if k!='current_work'}=={k:v for k,v in base.items() if k!='current_work'}
s=ledger['current_work']['sampled_sphere_hull'];prefix='docs/evidence/2026-10-14-sampled-sphere-hull/'
s.update({'status':'normal-form-image-filter-lsan-shutdown-failure-preserved',
 'sphere_normal_image_asan_executed':True,'sphere_normal_image_asan_complete':False,
 'sphere_normal_image_asan_failure':prefix+'normal-image-asan-v1/failure.json',
 'sphere_normal_image_asan_closure':prefix+'normal-image-asan-closure-v1.json',
 'sphere_normal_image_asan_partial_tests_passed':242,
 'sphere_normal_image_asan_failure_classification':closure['diagnostic_classification'],
 'sphere_normal_image_runtime_admitted':False,'plan_complete':False})
p.write_text(json.dumps(ledger,indent=2)+'\n')
print(json.dumps({'trial_closed':True,'complete':False,'closure_sha256':sha(e/'normal-image-asan-closure-v1.json')}))
