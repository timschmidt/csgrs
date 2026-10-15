"""Close the slice/projection qualification without changing prior evidence or ledger fields."""
import csv, gzip, hashlib, json, re, statistics, subprocess
from pathlib import Path
E=Path(__file__).resolve().parent
ROOT=E.parents[2]
MEMO={}
def sha(path):
 path=Path(path);s=path.stat();key=(s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns)
 if key not in MEMO:
  h=hashlib.sha256()
  with path.open('rb') as f:
   for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
  MEMO[key]=h.hexdigest()
 return MEMO[key]
def dump(path,value):path.write_text(json.dumps(value,indent=2)+'\n')
def span(raw):
 start=re.search(r'"current_work"\s*:\s*',raw).end()
 value,end=json.JSONDecoder().raw_decode(raw[start:])
 return start,start+end,value

def main():
 assert not (E/'closure-v1.json').exists()
 prior=ROOT/'docs/evidence/2026-10-15-planar-operation-failures/closure-v1.json'
 previous=json.loads(prior.read_text())
 assert all(sha(ROOT/p)==h for p,h in previous['pending_native_sha256'].items())
 protected=dict(previous['preserved_executables_sha256'])
 phases={};totals={}
 for name in ['baseline-v1','baseline-v2','candidate-v1','candidate-v2','candidate-v3','candidate-v4','native-v1','renderer-v1']:
  folder=E/name;result=json.loads((folder/'result.json').read_text());proto=json.loads((folder/'protocol.json').read_text())
  assert sha(E/'validate_v1.py')==proto['runner_sha256']
  phases[name]={'passed':result['passed'],'commands':len(result['commands']),'result_sha256':sha(folder/'result.json')}
  total=[0,0,0]
  for row in result['commands']:
   assert row['sources_unchanged']
   for suffix in ['stdout','stderr','resources']:assert sha(folder/(row['name']+'.'+suffix))==row[suffix+'_sha256']
   protected.update(row['binaries_sha256'])
   tests=re.findall(r'test result: \w+\. (\d+) passed; (\d+) failed; (\d+) ignored',(folder/(row['name']+'.stdout')).read_text())
   counts=[sum(int(t[i]) for t in tests) for i in range(3)]
   if tests:totals[name+'/'+row['name']]=counts
   total=[a+b for a,b in zip(total,counts)]
  phases[name]['tests_passed_failed_ignored']=total
 for name in ['candidate-v4','native-v1','renderer-v1']:assert phases[name]['passed']
 for name in ['baseline-v1','baseline-v2','candidate-v3']:assert not phases[name]['passed']
 for phase in ['baseline-v1','baseline-v2']:
  exe=ROOT/'target/slice-projection-contract'/phase/'regressions';protected[str(exe.relative_to(ROOT))]=sha(exe)
 images={str(p.relative_to(ROOT)):sha(p) for p in sorted((ROOT/'target/slice-projection-contract/renderer-v1/images').glob('*.png'))}
 assert len(images)==73
 protected.update(images)
 exe=ROOT/'target/slice-projection-contract/native-v1/release/examples/readme_renders'
 protected[str(exe.relative_to(ROOT))]=sha(exe)
 cost=json.loads((E/'cost-v1/result.json').read_text());assert cost['passed'] and len(cost['commands'])==82
 samples=[r['sample'] for r in cost['commands'] if 'sample' in r]
 assert len(samples)==72 and all(s['completed']==s['iterations']==128 for s in samples)
 for row in cost['commands']:
  assert row['sources_unchanged'] and row['exit_code']==0
  for suffix,h in row['streams_sha256'].items():assert sha(E/'cost-v1'/(row['name']+'.'+suffix))==h
 for path,row in json.loads((E/'cost-v1/artifacts.json').read_text()).items():protected[path]=row['sha256']
 cost_summary={}
 for shape in ['slice_middle','slice_bottom','slice_top','flatten']:
  cost_summary[shape]={}
  for cache in ['warm','varied','disabled']:
   groups={label:[r['sample'] for r in cost['commands'] if r['name'].endswith('-'+shape+'-'+cache+'-'+label)] for label in ['baseline','candidate']}
   assert all(len(g)==3 for g in groups.values())
   summary={label:{metric:statistics.median(s[metric] for s in group)/128 for metric in ['nanos','allocations','requested_bytes','retained_byte_delta','peak_live_bytes_above_start','output_bytes']} for label,group in groups.items()}
   summary['latency_percent_change']=100*(summary['candidate']['nanos']/summary['baseline']['nanos']-1)
   cost_summary[shape][cache]=summary
 binary_sizes=json.loads((E/'cost-v1/library_sizes.json').read_text())
 for label,base in [('baseline','target/planar-operation-failures/cost-build-v1/release'),('candidate','target/slice-projection-contract/native-v1/release')]:
  metadata=(E/'build-identity-v2'/(label+'-libcsgrs.rlib.stdout')).read_text()
  native=re.search(r'^\d+ (hypermesh-\S+) hash ',metadata,re.M).group(1)
  for p in [ROOT/base/'deps'/('lib'+native+'.rlib'),ROOT/'target/slice-projection-contract/cost-v1'/label]:
   binary_sizes[str(p.relative_to(ROOT))]=p.stat().st_size;protected[str(p.relative_to(ROOT))]=sha(p)
 rows=list(csv.DictReader((E/'native-v1/projection-benchmarks.stdout').read_text().splitlines()))
 assert len(rows)==9 and {r['case'] for r in rows}=={'slice_and_flatten','slice','flatten'}
 assert all(r['work_units']=='528' and r['output_size']==('2' if r['case']=='slice_and_flatten' else '1') for r in rows)
 implementation=json.loads((E/'implementation-v1.json').read_text())
 assert all(sha(ROOT/p)==h for p,h in implementation['files_sha256'].items())
 identity=json.loads((E/'build-identity-v2/result.json').read_text());assert identity['passed']
 qualification={
  'implementation_commit':'07258a70d','baseline_reproduction_commit':'490d46ce7',
  'owning_layers':['Hypermesh oriented section intersections and directed chain decomposition','Hypercurve signed nonzero winding and regularization','CSGRS modeling composition, typed failures and retained operation certainty'],
  'properties':['P4','P8','P13','P17','P18','P19','P20','P25','P27','P35','P37','P39'],
  'result':'Qualified slice/projection correctness repair; full properties goal remains active.',
  'meaning':'Holes, nested and duplicated winding, opposite traversals, disconnected components, shared on-plane edges, unindexed position rows and qualified uncertainty survive supported section composition. Primary errors do not become successful empty geometry.',
  'migration':'flatten and slice_z now return Result. Corrected topology and traversal/subdivision/serialization changes are intentional, not a performance-only patch. Existing example and benchmark fixtures and policies are unchanged.',
  'semantic_evidence':'25 modeling regressions; exact rational areas, membership, native source-edge parameter replay, determinant volumes and separate manifold checks. Face permutations and cache disabled/warm/evicted/mutated/concurrent cases retain mathematical meaning.',
  'native_validation':totals,'phase_results':phases,
  'projection_benchmark_rows':rows,'renderer_images':len(images),
  'cost_probe':{'commands':82,'fresh_process_samples':72,'completed_timed_operations':9216,'dump_variants_per_revision':512,'statistics_per_operation':cost_summary,'actual_serialized_rows':json.loads((E/'cost-v1/exact-rows.json').read_text()),'binary_bytes':binary_sizes,'limitations':'Instrumented allocations and shared unpinned host. Fresh and seeded dependency archives have different crate hashes, despite the same core sources, toolchain, actual dependency names/ABI and Cargo compilation units; cause not established. No optimization, bit-identical-dependency or corpus-wide speedup claim.'},
  'remaining':['End-to-end construction provenance is not established; native section source indices/parameters refer to the call input and the planar result does not expose these mesh-source records.','No internal total-work, retained-byte or cancellation bound; trials use external 1800-second wall and 8 GiB per-process address-space limits.','Clean external release/package admission and full CI matrix remain pending. No push performed.','Thirty-five pre-existing native files remain byte-identical and unadmitted. Seven previously reproduced Hypercircuit snapshot/golden failures remain unresolved; they were not re-executed by this slice.','Seven existing Hypermesh tests and one existing all-feature CSGRS test remain ignored under their original conditions.'],
  'implementation_files_sha256':implementation['files_sha256'],'pending_native_sha256':previous['pending_native_sha256'],'full_plan_complete':False,
 }
 dump(E/'qualification-v1.json',qualification)
 ledger=ROOT/'docs/properties-status.json';raw=ledger.read_text();before=sha(ledger)
 reference=subprocess.check_output(['git','show','9025de62:docs/properties-status.json'],cwd=ROOT,text=True)
 start,end,current=span(raw);rstart,rend,_=span(reference)
 assert (raw[:start],raw[end:])==(reference[:rstart],reference[rend:])
 current['updated_on']='2026-10-15'
 current['slice_projection_contract']={'status':'qualified correctness repair; broader resource/provenance/release admission remains pending','implementation_commit':'07258a70d','evidence':'docs/evidence/2026-10-15-slice-projection-contract/qualification-v1.json','closure':'docs/evidence/2026-10-15-slice-projection-contract/closure-v1.json','baseline':'16 tests: 6 pass, 10 fail; expanded 21 tests: 8 pass, 13 fail','regressions':'25 modeling tests pass; candidate-v3 unindexed-input failure retained and corrected in candidate-v4','validation':'14 native commands and full renderer pass; root test totals 338/96/231; Hypermesh default/all-feature totals 237/238; Clippy and docs deny warnings; 73 rendered images','cost':'82 commands, 9216 completed timed operations; exact semantic oracles, raw unfavorable samples and actual serialization differences retained. Rebuild artifact discrepancy documented; no optimization claim.','remaining':qualification['remaining'],'full_plan_complete':False}
 current['plan_complete']=False
 replacement=json.dumps(current,indent=2).replace('\n','\n  ')
 ledger.write_text(raw[:start]+replacement+raw[end:])
 updated=ledger.read_text();ustart,uend,_=span(updated)
 assert (updated[:ustart],updated[uend:])==(reference[:rstart],reference[rend:])
 a=json.loads(updated);b=json.loads(reference);a.pop('current_work');b.pop('current_work');assert a==b
 print('Verifying preserved artifacts and prior evidence.',flush=True)
 assert all(sha(ROOT/p)==h for p,h in previous['evidence_files_sha256'].items())
 assert all(sha(ROOT/p)==h for p,h in protected.items())
 linked=0
 for manifest in E.glob('linked-compiler-duplicates-v*.json.gz'):
  data=json.loads(gzip.decompress(manifest.read_bytes()))
  for name,row in data.items():
   path=Path(name);origin=Path(row['retained_original'])
   assert sha(path)==sha(origin)==row['sha256']
   assert path.stat().st_ino==origin.stat().st_ino
  linked+=len(data)
 evidence={str(p.relative_to(ROOT)):sha(p) for p in sorted(E.rglob('*')) if p.is_file() and '__pycache__' not in p.parts}
 dump(E/'closure-v1.json',{'qualification_sha256':sha(E/'qualification-v1.json'),'closure_runner_sha256':sha(Path(__file__)),'previous_closure_sha256':sha(prior),'preserved_artifacts_sha256':protected,'evidence_files_sha256':evidence,'pending_native_sha256':previous['pending_native_sha256'],'linked_compiler_paths_verified':linked,'ledger_before_sha256':before,'ledger_after_sha256':sha(ledger),'only_current_work_changed_in_ledger':True,'full_plan_complete':False})
 print(json.dumps({'evidence_files_including_closure':len(evidence)+1,'protected_artifacts':len(protected),'linked_compiler_paths':linked,'pending_native_files_unchanged':35,'full_plan_complete':False}),flush=True)
if __name__=='__main__':main()
