from pathlib import Path
import concurrent.futures,hashlib,json,subprocess,time
a=Path(__file__).resolve().parent
r=Path('/tmp/hypercurve-closure-2026-09-23/hypercurve')
prefix='single-loop-corner-20260923-parent-replay1'
binary=a/'single-loop-corner-20260923-attempt1-libraries/parent-libtest'
bound=json.loads((a/'single-loop-corner-20260923-attempt1-runs.json').read_text())
parent=next(x for x in bound['runs'] if x['label']=='parent-build')
assert hashlib.sha256(binary.read_bytes()).hexdigest()==parent['sha256']
text=(a/'single-loop-corner-20260923-attempt1-candidate-library.log').read_text()
names=[line.split()[1] for line in text.splitlines() if line.startswith('test ') and line.endswith(' ... FAILED')]
names += ['bezier_region::tests::independent_oblique_chord_pair_fillet_crosses_a_rational_line_exactly', 'bezier_region::tests::independent_oblique_chord_pair_fillet_crosses_algebraic_chords_exactly']
rows=json.loads((a/'single-loop-corner-20260923-attempt1-candidate-sources.json').read_text())
def verify():
 for row in rows: assert hashlib.sha256((r.parent/row['file']).read_bytes()).hexdigest()==row['sha256'],row['file']
def run(name):
 log=a/(prefix+'-'+name.split('::')[-1]+'.log');cmd=[str(binary),'--exact',name,'--test-threads=1','--nocapture','--color','never'];start=time.monotonic()
 with log.open('w') as out:
  try: rc=subprocess.run(cmd,cwd=r,stdout=out,stderr=subprocess.STDOUT,timeout=60).returncode
  except subprocess.TimeoutExpired: rc='timeout'
 row=dict(name=name,command=cmd,returncode=rc,elapsed_seconds=time.monotonic()-start,log=log.name,binary_sha256=parent['sha256'])
 print(json.dumps(row),flush=True);print(log.read_text()[-2200:],flush=True);return row
verify();result=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
 for row in pool.map(run,names):
  result.append(row);(a/(prefix+'-runs.json')).write_text(json.dumps(result,indent=2)+'\n')
verify()
