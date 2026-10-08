from pathlib import Path
import hashlib,json,os,shutil,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent
prefix='fillet-compact-source-axes-20260926-v278';prior=json.loads((A/'compact-image-coefficients-20260926-v275-terminal.json').read_text())
guard=json.loads((A/'compact-image-coefficients-20260926-v275-sources.json').read_text());manifest=json.loads((A/prior['source_manifest']).read_text());guard={name:sha for name,sha in guard.items()if not name.startswith('fiber-probe/')};manifest=dict(guard);archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925';assert not archive.exists()
for name,sha in manifest.items():
 src=Path(prior['source_directory'])/name;assert hashlib.sha256(src.read_bytes()).hexdigest()==sha
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
 target=build/name
 if target.read_bytes()!=src.read_bytes():shutil.copy2(src,target);os.utime(target,None)
 assert dst.stat().st_ino!=target.stat().st_ino

def patch_source(file, transform):
 p=archive/file;t=transform(p.read_text());p.write_text(t);shutil.copy2(p,build/file);manifest[file]=hashlib.sha256(p.read_bytes()).hexdigest()
patch_source('hypersolve/src/tensor_resultant.rs',lambda t:t.replace('pub(crate) fn substitute_axis_value(', 'pub fn substitute_axis_value(',1))
def substitute_exact(t):
 start=t.index('fn dense_substitute_affinely_related_sources(');end=t.index('fn dense_reduce_selected_tuple_relations(',start);part=t[start:end]
 old='    let mut sources = sources.to_vec();'
 new=old+r"""
    if sources.iter().any(|source| source.exact_point_witness().is_none()) {
        let mut axis = 0;
        while axis < sources.len() {
            if let Some(value) = sources[axis].exact_point_witness() {
                polynomial = polynomial.substitute_axis_value(axis, value)?;
                eprintln!("dense-exact-source-axis before={} rational={}",sources.len(),value.exact_rational_ref().is_some());
                sources.remove(axis);
            } else {
                axis += 1;
            }
        }
    }
"""
 assert old in part;part=part.replace(old,new,1);t=t[:start]+part+t[end:]
 start=t.index('fn dense_polynomial_tuple_sign_owned(');end=t.index('fn dense_two_positive_square_root_interval_with_coefficient_precision(',start);part=t[start:end]
 part=part.replace('    if let [source] = sources.as_slice() {','    if let [source] = sources.as_slice() {\n        eprintln!("dense-tuple univariate begin coefficients={}",polynomial.coefficients().len());',1)
 part=part.replace('        if result.is_decided() || policy.has_bounded_exact_predicate_budget() {','        eprintln!("dense-tuple univariate end decided={}",result.is_decided());\n        if result.is_decided() || policy.has_bounded_exact_predicate_budget() {',1)
 return t[:start]+part+t[end:]
patch_source('hypercurve/src/bezier_offset.rs',substitute_exact)

def compact_dense(t):
 start=t.index('fn dense_canonicalize_proven_rational_coefficients(');end=t.index('/// Collapses selected tensor axes',start);part=t[start:end]
 old='            coefficient\n                .exact_rational_normal_form()\n                .map(Real::new)\n                .unwrap_or(coefficient)'
 new='            if coefficient.exact_rational_ref().is_some() {\n                return coefficient;\n            }\n            coefficient.compact_quadratic_tower()\n                .or_else(|| coefficient.exact_rational_normal_form().map(Real::new))\n                .unwrap_or(coefficient)'
 assert old in part;part=part.replace(old,new,1);t=t[:start]+part+t[end:]
 start=t.index('fn dense_polynomial_tuple_sign_owned(');end=t.index('fn dense_two_positive_square_root_interval_with_coefficient_precision(',start);part=t[start:end]
 needle='                    signed_coefficients_at_parameter(polynomial.coefficients(), &parameter, policy)?'
 assert needle in part;part=part.replace(needle,'                    eprintln!("dense-tuple source imported");\n'+needle,1)
 return t[:start]+part+t[end:]
patch_source('hypercurve/src/bezier_offset.rs',compact_dense)
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
report=dict(source_manifest=f'{prefix}-sources.json',source_directory=str(archive),workspace_guard='compact-image-coefficients-20260926-v275-sources.json',parent_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/'hypercurve',text=True).strip(),hypersolve_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/'hypersolve',text=True).strip(),cases=[],all_processes_reaped=False)
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
  try:code=subprocess.run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],cwd=archive/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=90).returncode
  except subprocess.TimeoutExpired:code=124
 report['cases'].append(dict(name=name,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start));print(name,code,log.read_text()[-2500:],flush=True)
verify();report['all_sources_unchanged']=True;report['all_processes_reaped']=True;(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print('Diagnostic audit complete; case failures are reported individually.',flush=True)
