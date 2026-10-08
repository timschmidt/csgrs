from pathlib import Path
import concurrent.futures, hashlib, json, os, shutil, subprocess, time
w=Path('/home/tim/Documents/GitHub/workspace')
a=Path(__file__).resolve().parent
r=Path('/tmp/hypercurve-region-admission-qualification')
prefix='private-region-factories-candidate2'
files=['src/bezier_region.rs','tests/hypercurve_bezier_region.rs']
changed=subprocess.check_output(['git','diff','--name-only'],cwd=w/'hypercurve',text=True).splitlines()
assert sorted(x for x in changed if x!='src/bezier_offset.rs')==sorted(files),changed
working=[dict(file='hypercurve/'+name,sha256=hashlib.sha256((w/'hypercurve'/name).read_bytes()).hexdigest()) for name in [*files,'src/bezier_offset.rs']]
for name in files: (r/'hypercurve'/name).write_bytes((w/'hypercurve'/name).read_bytes())
probe=a/'normalized-retained-admission-completed.rs'
probe_sha256=hashlib.sha256(probe.read_bytes()).hexdigest()
manifest=[]
for repo in sorted(r.iterdir()):
 if repo.is_dir():
  for path in sorted(repo.rglob('*')):
   if path.is_file() and 'target' not in path.parts:
    manifest.append(dict(file=str(path.relative_to(r)),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
for suffix,value in [('working-sources',working),('isolated-sources',manifest)]:
 (a/(prefix+'-'+suffix+'.json')).write_text(json.dumps(value,indent=2)+'\n')
(a/(prefix+'-source.patch')).write_bytes(subprocess.check_output(['git','diff','--',*files],cwd=w/'hypercurve'))
settings=json.loads((a/'opposed-endpoint-contact-full1-build-settings.json').read_text())
env=dict(os.environ,**settings)
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
archive=a/(prefix+'-libraries'); archive.mkdir()
builds=[]; tests=[]
def verify():
 for base,rows in [(w,working),(r,manifest)]:
  for row in rows: assert hashlib.sha256((base/row['file']).read_bytes()).hexdigest()==row['sha256'],row['file']
 assert hashlib.sha256(probe.read_bytes()).hexdigest()==probe_sha256

def build(kind,repo,args):
 cmd=[cargo,*args,'--locked','--offline']
 stem=prefix+'-'+kind; start=time.monotonic()
 with (a/(stem+'.log')).open('w') as err,(a/(stem+'.jsonl')).open('w') as out:
  result=subprocess.run(cmd,cwd=r/repo,env=env,stdout=out if '--message-format=json' in args else err,stderr=err,timeout=900)
 row=dict(kind=kind,repo=repo,command=cmd,returncode=result.returncode,elapsed_seconds=time.monotonic()-start)
 builds.append(row); (a/(prefix+'-builds.json')).write_text(json.dumps(builds,indent=2)+'\n')
 print(kind,result.returncode,round(row['elapsed_seconds'],2),flush=True)
 verify()
 if result.returncode:
  print((a/(stem+'.log')).read_text()[-8000:],flush=True); raise SystemExit(1)
 return a/(stem+'.jsonl')

def run(target,binary,filters,expected):
 cmd=[str(binary),*filters,'--test-threads=2','--color','never']
 stem=prefix+'-'+target; start=time.monotonic()
 with (a/(stem+'.log')).open('w') as log:
  try: code=subprocess.run(cmd,cwd=r/'hypercurve',stdout=log,stderr=subprocess.STDOUT,timeout=300).returncode
  except subprocess.TimeoutExpired: code='timeout'
 output=(a/(stem+'.log')).read_text()
 row=dict(target=target,command=cmd,returncode=code,passed=code==0 and expected in output,expected=expected,elapsed_seconds=time.monotonic()-start,sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),log=stem+'.log')
 tests.append(row); (a/(prefix+'-tests.json')).write_text(json.dumps(tests,indent=2)+'\n')
 print(target,code,round(row['elapsed_seconds'],2),flush=True)
 print(output[-3500:],flush=True)
 return row
try:
 build('hypercurve-check','hypercurve',['check','--all-targets','--no-default-features'])
 output=build('test-build','hypercurve',['test','--release','--all-features','--lib','--test','hypercurve_bezier_region','--no-run','--message-format=json'])
 binaries={}; library=None
 for line in output.read_text().splitlines():
  item=json.loads(line)
  if item.get('reason')!='compiler-artifact' or item['target']['name'] not in ['hypercurve','hypercurve_bezier_region']: continue
  if item.get('executable'):
   assert not item['fresh'],'stale release test artifact'
   binary=archive/Path(item['executable']).name; shutil.copyfile(item['executable'],binary); binary.chmod(0o755)
   binaries[item['target']['name']]=binary
  elif item['target']['name']=='hypercurve':
   for file in item['filenames']:
    if file.endswith('.rlib'):
     library=archive/Path(file).name; shutil.copyfile(file,library)
 assert library and set(binaries)=={'hypercurve','hypercurve_bezier_region'}
 listing=subprocess.check_output([str(binaries['hypercurve_bezier_region']),'--list'],text=True)
 assert 'retained_algebraic_line_images_normalize_crossing_loops_under_both_policies: test' in listing
 (a/(prefix+'-test-list.txt')).write_text(listing)
 count=sum(line.endswith(': test') for line in listing.splitlines())
 run('bezier-region',binaries['hypercurve_bezier_region'],[],str(count)+' passed;')
 for name in ['retained_region_constructor_rejects_reused_arrangement_sources_across_loops','native_boundary_loops_convert_into_unified_region_validation','retained_region_constructor_rejects_duplicate_boundary_loops','selected_fiber_line_images_reuse_exact_real_endpoints_in_both_directions','selected_fiber_line_image_fit_rejects_nonmonotone_subrange_excursions']:
  run(name,binaries['hypercurve'],['--exact','bezier_region::tests::'+name],'1 passed;')
 binary=archive/'admission-public'
 cmd=[settings['RUSTC'],'--edition=2024','-O',str(probe),'--extern','hypercurve='+str(library),'-L','dependency='+str(Path(settings['CARGO_TARGET_DIR'])/'release/deps'),'-o',str(binary)]
 start=time.monotonic()
 result=subprocess.run(cmd,cwd=a,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=300)
 (a/(prefix+'-probe-build.log')).write_text(result.stdout)
 assert result.returncode==0,result.stdout
 output=subprocess.run([str(binary)],cwd=a,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=60)
 (a/(prefix+'-probe.log')).write_text(output.stdout)
 (a/(prefix+'-probe.json')).write_text(json.dumps(dict(command=cmd,compile_returncode=result.returncode,run_returncode=output.returncode,source_sha256=probe_sha256,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),elapsed_seconds=time.monotonic()-start),indent=2)+'\n')
 print(output.stdout,flush=True)
 assert output.returncode==0
 for repo in ['hyperbrep','hyperdrc','hypercircuit','csgrs']:
  build(repo+'-check',repo,['check','--all-targets','--no-default-features'])
 build('fuzz-check','hypercurve',['check','--manifest-path','fuzz/Cargo.toml','--bin','bezier_region'])
 if any(not row['passed'] for row in tests): raise SystemExit(1)
finally:
 verify(); print('Bound isolated, working, and probe sources unchanged.',flush=True)
