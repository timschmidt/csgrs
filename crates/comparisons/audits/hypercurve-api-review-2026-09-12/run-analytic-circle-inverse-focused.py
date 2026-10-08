from pathlib import Path
import hashlib,json,subprocess,sys,time
root=Path('/home/tim/Documents/GitHub/workspace');audit=root/'hypercurve-api-review-2026-09-12';stem=sys.argv[1]
assert (audit/(stem+'.exit')).read_text().strip()=='0'
binaries={}
for line in (audit/(stem+'.jsonl')).read_text().splitlines():
 row=json.loads(line)
 if row.get('reason')=='compiler-artifact' and row.get('executable') and row['profile']['test']:
  binaries[row['target']['name']]=row['executable']
results=[]
for target,filter in [
 ('hypercurve_analytic_parallel_region','analytic_parallel_circle_tangency_retains_zero_cross_evidence'),
 ('hypercurve_analytic_parallel_region','analytic_parallel_intersects_independently_parameterized_circles_exactly'),
 ('hypercurve','bezier_offset::conversion_tests::parallel_circle_intersections_retain_exceptional_inverse_fibers'),
]:
 command=[binaries[target],filter,'--exact','--test-threads=1'];name=stem+'-'+filter.split('::')[-1];start=time.monotonic()
 with (audit/(name+'.log')).open('w') as output:
  try:code=subprocess.run(command,cwd=root/'hypercurve',stdout=output,stderr=subprocess.STDOUT,timeout=90).returncode
  except subprocess.TimeoutExpired:code=124
 record={'target':target,'filter':filter,'command':command,'sha256':hashlib.sha256(Path(binaries[target]).read_bytes()).hexdigest(),'returncode':code,'seconds':time.monotonic()-start,'log':name+'.log'}
 (audit/(name+'.exit')).write_text(str(code)+'\n');results.append(record);print(filter,code,record['seconds'],flush=True);print((audit/(name+'.log')).read_text()[-5000:],flush=True)
(audit/(stem+'-focused-results.json')).write_text(json.dumps(results,indent=2)+'\n')
