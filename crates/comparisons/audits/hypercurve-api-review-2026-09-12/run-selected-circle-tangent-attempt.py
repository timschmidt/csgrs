from pathlib import Path
import subprocess,time,json,hashlib,sys
p=Path('/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12')
attempt=sys.argv[1]
rows=[]
for target,binary,name in [
 ('hypercurve_curve_intersection','hypercurve_curve_intersection-e3a50a789eb6a53e','selected_circle_tangency_reuses_retained_normal_evidence'),
 ('lib','hypercurve-6e6572a45c5b177b','selected_fiber_circle_overlap_does_not_prove_source_parameter_identity'),
 ('lib','hypercurve-6e6572a45c5b177b','selected_fiber_rational_overlap_replays_certified_policy'),
 ('lib','hypercurve-6e6572a45c5b177b','curve_intersection::curve_support_intersection::circle_dispatch_tests'),
]:
 b=p.parent/'hypercurve/target/release/deps'/binary
 stem='selected-circle-tangent-identity-'+attempt+'-'+name.split('::')[-1]
 cmd=[str(b),name,'--test-threads=2','--color','never','--nocapture']
 start=time.monotonic()
 with (p/(stem+'.log')).open('w') as out:
  try:r=subprocess.run(cmd,stdout=out,stderr=subprocess.STDOUT,timeout=90);code=r.returncode
  except subprocess.TimeoutExpired:code=124
 row={'target':target,'filter':name,'command':cmd,'returncode':code,'elapsed_seconds':time.monotonic()-start,'log':stem+'.log','binary':str(b),'sha256':hashlib.sha256(b.read_bytes()).hexdigest()}
 rows.append(row)
 (p/('selected-circle-tangent-identity-'+attempt+'-focused.json')).write_text(json.dumps(rows,indent=2)+'\n')
 print(row,flush=True)
 print((p/(stem+'.log')).read_text()[-5000:],flush=True)
 if code:break
