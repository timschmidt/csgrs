from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import hashlib,json,re,subprocess,time,sys,shutil
p=Path('/home/tim/Documents/GitHub/workspace/hypercurve-api-review-2026-09-12');prefix='normalized-path-admission-candidate'+sys.argv[1]
entries=[json.loads(l) for l in (p/(prefix+'-test-build.jsonl')).read_text().splitlines()]
jobs=[(x['target']['name'],x['executable']) for x in entries if x.get('executable') and x.get('profile',{}).get('test')]
assert len(jobs)==6
skip=['selected_parallel_normal_circle_intersects_genuinely_analytic_parallel_in_one_fiber','independent_oblique_chord_pair_fillets_extend_on_infinite_supports','selected_circle_and_analytic_parallel_extend_on_full_supports','pair_native_boolean_algebraic_chord_corner_publishes_a_third_generation_fillet']
archive=p/(prefix+'-libraries');archive.mkdir()
for name,binary in jobs:shutil.copy2(binary,archive/Path(binary).name)
library=Path(next(f for x in entries if x.get('reason')=='compiler-artifact' and x['target']['name']=='hypercurve' and not x['profile']['test'] for f in x['filenames'] if f.endswith('.rlib')));shutil.copy2(library,archive/library.name)
def run(job):
 name,binary=job;cmd=[binary,'--test-threads=4','--color','never'];stem=prefix+'-'+name
 if name=='hypercurve':
  for test in skip:cmd+=['--skip',test]
 start=time.monotonic()
 with (p/(stem+'.log')).open('w') as out:
  try:code=subprocess.run(cmd,stdout=out,stderr=subprocess.STDOUT,timeout=300).returncode
  except subprocess.TimeoutExpired:code=124
 text=(p/(stem+'.log')).read_text();states=re.findall(r'^test ([^\n]+?) \.\.\. (ok|FAILED|ignored[^\n]*)$',text,re.M)
 row=dict(target=name,command=cmd,returncode=code,elapsed_seconds=time.monotonic()-start,binary_sha256=hashlib.sha256(Path(binary).read_bytes()).hexdigest(),passed=[n for n,s in states if s=='ok'],failed=[n for n,s in states if s=='FAILED'],ignored=[n for n,s in states if s.startswith('ignored')],summaries=re.findall(r'^test result:.*$',text,re.M))
 (p/(stem+'.json')).write_text(json.dumps(row,indent=2)+'\n');print(name,code,len(row['passed']),row['failed'],round(row['elapsed_seconds'],2),flush=True);return row
rows=[]
with ThreadPoolExecutor(max_workers=2) as pool:
 for future in as_completed([pool.submit(run,j) for j in sorted(jobs,key=lambda j:j[0]!='hypercurve')]):rows.append(future.result())
(p/(prefix+'-tests.json')).write_text(json.dumps(rows,indent=2)+'\n')
