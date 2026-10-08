from pathlib import Path
import hashlib,json,os,shutil,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent
prefix='fillet-reoffset-diagnostic-20260926-v237';prior=json.loads((A/'public-fillet-families-full-20260926-v236-terminal.json').read_text())
guard=json.loads((A/'public-fillet-families-full-20260926-v236-sources.json').read_text());manifest=json.loads((A/prior['source_manifest']).read_text());archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925';assert not archive.exists()
for name,sha in manifest.items():
 src=Path(prior['source_directory'])/name;assert hashlib.sha256(src.read_bytes()).hexdigest()==sha
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
 target=build/name
 if target.read_bytes()!=src.read_bytes():shutil.copy2(src,target);os.utime(target,None)
 assert dst.stat().st_ino!=target.stat().st_ino
file='hypercurve/src/bezier_offset.rs'
p=archive/file;t=p.read_text();start=t.index('    fn selected_parallel_normal_parallel_intersections(');end=t.index('    fn parallel_contact_at_certified_parameter(',start);part=t[start:end]
original='return Ok(Classification::Uncertain(reason))'
for i in range(part.count(original)):
 part=part.replace(original, f'{{ eprintln!("selected-normal unresolved-stage={i} reason={{reason:?}}"); RETURN_UNCERTAIN(reason) }}',1)
part=part.replace('RETURN_UNCERTAIN(reason)',original)
part=part.replace('        let frame_parameter = self.selected_frame_parameter()', '        eprintln!("selected-normal enter");\n        let frame_parameter = self.selected_frame_parameter()',1)
part=part.replace('        let diagonal_location = if other.source() == frame.center_support.source() {', '        eprintln!("selected-normal source same={} reversed={}", other.source() == frame.center_support.source(), other.source().is_reversal_of(frame.center_support.source()));\n        let diagonal_location = if other.source() == frame.center_support.source() {',1)
t=t[:start]+part+t[end:]
start=t.index('    pub(crate) fn parallel_intersections(\n        &self,\n        other: &BezierParallel2,\n        range: &CurveParameterRange2,');insert=t.index('        let frame_parallel = self.source_parallel();',start)
t=t[:insert]+'        eprintln!("circle-parallel entry selected-normal={} chord-normal={} recursive={}", self.uses_selected_parallel_normal_frame(), self.uses_selected_chord_normal_frame(), self.uses_retained_circle_parallel_system());\n'+t[insert:]
p.write_text(t);shutil.copy2(p,build/file);manifest[file]=hashlib.sha256(p.read_bytes()).hexdigest()
file='hypercurve/src/curve_region_boolean.rs';p=archive/file;t=p.read_text();needle='            if let Some(blocker) = result.blockers.first() {';pos=t.index(needle,t.index('    fn build_split_topology('))+len(needle)
t=t[:pos]+'\n                eprintln!("pair-blocker indices={}:{} loops={}:{} fragments={}:{}", pair.first_carrier_index, pair.second_carrier_index, self.data.carriers[pair.first_carrier_index].loop_index, self.data.carriers[pair.second_carrier_index].loop_index, self.data.carriers[pair.first_carrier_index].fragment_index, self.data.carriers[pair.second_carrier_index].fragment_index);'+t[pos:]
p.write_text(t);shutil.copy2(p,build/file);manifest[file]=hashlib.sha256(p.read_bytes()).hexdigest()
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
report=dict(source_manifest=f'{prefix}-sources.json',source_directory=str(archive),workspace_guard='public-fillet-families-full-20260926-v236-sources.json',baseline_commit=prior['parents']['hypercurve'],cases=[],all_processes_reaped=False)
def verify():
 for name,sha in guard.items():assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
 for name,sha in manifest.items():
  for root in [archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
verify();cmd=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo','test','--test','hypercurve_curve_region_promotion','--release','--all-features','--no-run','--message-format=json','--locked','--offline'];report['command']=cmd
start=time.monotonic()
with(A/f'{prefix}-build.log').open('w')as log:code=subprocess.run(cmd,cwd=build/'hypercurve',env=env,stdout=log,stderr=subprocess.STDOUT,timeout=900).returncode
report['build_returncode']=code;report['build_elapsed_seconds']=time.monotonic()-start;verify();assert code==0
rows=[]
for line in(A/f'{prefix}-build.log').read_text().splitlines():
 try:rows.append(json.loads(line))
 except ValueError:pass
row=next(r for r in rows if r.get('reason')=='compiler-artifact'and r['target']['name']=='hypercurve_curve_region_promotion'and r.get('executable'))
binary=A/f'{prefix}-hypercurve_curve_region_promotion';shutil.copy2(row['executable'],binary);report['binary']=dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
for name in ['approximate_512_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions']:
 log=A/f'{prefix}-{name}.log';start=time.monotonic()
 with log.open('w')as out:
  try:code=subprocess.run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],cwd=archive/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=180).returncode
  except subprocess.TimeoutExpired:code=124
 report['cases'].append(dict(name=name,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start));print(name,code,log.read_text()[-2500:],flush=True)
verify();report['all_sources_unchanged']=True;report['all_processes_reaped']=True;(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print('Diagnostic audit complete; case failures are reported individually.',flush=True)
