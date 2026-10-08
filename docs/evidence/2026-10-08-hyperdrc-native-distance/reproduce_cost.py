"""Run after validation and all build/test sessions finish; candidate-only cost."""
import hashlib,json,pathlib,re,statistics,subprocess,sys
root=pathlib.Path.cwd()
validation=pathlib.Path(sys.argv[1])
out=pathlib.Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=False)
log=(validation/'test.log').read_text()
test=root/re.search(r'Running unittests src/lib.rs \(([^)]+)\)',log)[1]
fixture=root/re.search(r'Executable benches/fixture_smoke.rs \(([^)]+)\)',(validation/'fixture-build.log').read_text())[1]
observations=[]
for i in range(5):
 cmd=['taskset','-c','0','/usr/bin/time','-v',str(test),'checks::distance::native_regressions::prepared_boundary_reuses_large_exact_segment_field','--exact','--nocapture']
 with (out/f'field-{i+1}.log').open('w') as f:code=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT).returncode
 assert code==0
 data=(out/f'field-{i+1}.log').read_text()
 values=re.search(r'native_boundary_field squares=(\d+) probes=(\d+) prepare_us=(\d+) query_us=(\d+) bytes=(\d+) sha256=([a-f0-9]+)',data).groups()
 row=dict(zip(['squares','probes','prepare_us','query_us','bytes','sha256'],[*(int(v) for v in values[:5]),values[5]]))
 row['peak_rss_kib']=int(re.search(r'Maximum resident set size \(kbytes\): (\d+)',data)[1]);observations.append(row)
cmd=['taskset','-c','0','/usr/bin/time','-v',str(fixture)]
with (out/'fixture.log').open('w') as f:code=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT).returncode
fixture_data=(out/'fixture.log').read_text()
manifest={'scope':'Candidate-only cost of native DRC metrics and retained full fixture smoke. No matched-baseline, speedup, allocation-count or library-size claim.', 'field_command':['taskset','-c','0','/usr/bin/time','-v',str(test.relative_to(root)),'checks::distance::native_regressions::prepared_boundary_reuses_large_exact_segment_field','--exact','--nocapture'], 'field_runs':observations,'medians':{k:statistics.median(r[k] for r in observations) for k in ['prepare_us','query_us']},'fixture_command':['taskset','-c','0','/usr/bin/time','-v',str(fixture.relative_to(root))],'fixture_exit_code':code,'binaries':{str(p.relative_to(root)):{'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in [test,fixture]},'rustc':subprocess.run(['rustc','-Vv'],capture_output=True,text=True,check=True).stdout,'cargo':subprocess.run(['cargo','-V'],capture_output=True,text=True,check=True).stdout,'sources':json.loads((validation/'sources.json').read_text())['source_hashes'],'run_conditions':'Five sequential fresh field processes and one fixture process pinned to CPU 0 after this task validation/build/test sessions had exited; compilation excluded. Field timer includes 64 probe constructions and exact queries, keeps 2,000 squares and the unchanged 2-second query gate. Prepare timer includes filled field construction and native boundary preparation. Whole-process RSS includes fixture input construction and test/runtime overhead.'}
(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps({k:manifest[k] for k in ['medians','fixture_exit_code']},indent=2))
raise SystemExit(code)
