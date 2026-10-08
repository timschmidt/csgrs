from pathlib import Path
import hashlib,json,os,shutil,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent
prefix='fillet-compact-coefficients-20260926-v274';prior=json.loads((A/'shared-source-refinement-20260926-v270-terminal.json').read_text())
guard=json.loads((A/'shared-source-refinement-20260926-v270-sources.json').read_text());manifest=json.loads((A/prior['source_manifest']).read_text());guard={name:sha for name,sha in guard.items()if not name.startswith('fiber-probe/')};manifest=dict(guard);archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925';assert not archive.exists()
for name,sha in manifest.items():
 src=Path(prior['source_directory'])/name;assert hashlib.sha256(src.read_bytes()).hexdigest()==sha
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
 target=build/name
 if target.read_bytes()!=src.read_bytes():shutil.copy2(src,target);os.utime(target,None)
 assert dst.stat().st_ino!=target.stat().st_ino

def patch_source(file, transform):
 p=archive/file;t=transform(p.read_text());p.write_text(t);shutil.copy2(p,build/file);manifest[file]=hashlib.sha256(p.read_bytes()).hexdigest()
patch_source('hyperreal/src/computable/node/quadratic_tower.rs', lambda t: t + r"""
impl Computable {
    pub(crate) fn __probe_tower(&self) -> Option<[[Rational; 3]; 3]> {
        let tower = tower_from_computable(self)?;
        let quad = |q: Quad| [q.rational, q.scale, q.disc.unwrap_or_else(Rational::zero)];
        Some([quad(tower.even), quad(tower.odd), quad(tower.radicand.unwrap_or_else(Quad::zero))])
    }
}
""")
patch_source('hyperreal/src/real/arithmetic/quadratic_tower_sign.rs', lambda t: t + r"""
impl Real {
    pub fn __probe_compact(&self) -> Option<Self> {
        let parts = self.tower_computable().__probe_tower()?;
        let quad = |[a,b,d]: [Rational; 3]| {
            let a = Self::new(a);
            if b.is_zero() { a } else { a + Self::new(b) * Self::new(d).sqrt().unwrap() }
        };
        let [even, odd, radicand] = parts;
        let even = quad(even);
        let odd = quad(odd);
        Some(if odd.definitely_zero() { even } else { even + odd * quad(radicand).sqrt().unwrap() })
    }
}
""")


def compact_coefficients(t):
 old='        if let Some(rational) = coefficient.exact_rational_normal_form() {\n            *coefficient = Real::new(rational);\n        }'
 new='        if let Some(compact) = coefficient.__probe_compact() {\n            *coefficient = compact;\n        } else if let Some(rational) = coefficient.exact_rational_normal_form() {\n            *coefficient = Real::new(rational);\n        }'
 assert old in t;t=t.replace(old,new,1)
 t=t.replace('    let (_, relation_coefficients) = relation.into_parts();','    let (_, relation_coefficients) = relation.into_parts();\n    eprintln!("compacted-image-square-free begin coefficients={} rationals={}",relation_coefficients.len(),relation_coefficients.iter().filter(|c| c.exact_rational_ref().is_some()).count());',1)
 t=t.replace('    // Resultant interpolation can retain a large arithmetic DAG even when a','    eprintln!("compacted-image-square-free end coefficients={}",polynomial_coefficients.len());\n    // Resultant interpolation can retain a large arithmetic DAG even when a',1)
 return t
patch_source('hypersolve/src/algebraic_tensor_image.rs',compact_coefficients)
def trace_gcd(t):
 start=t.index('pub(crate) fn polynomial_gcd(');end=t.index('pub(crate) fn polynomials_share_one_root_in_interval(',start);part=t[start:end]
 part=part.replace('        let (_, remainder) = polynomial_div_rem_trimmed(left, &right, policy)?;', '        eprintln!("gcd divide left={} right={}",left.len(),right.len());\n        let (_, remainder) = polynomial_div_rem_trimmed(left, &right, policy)?;\n        eprintln!("gcd remainder coefficients={}",remainder.len());',1)
 return t[:start]+part+t[end:]
patch_source('hypersolve/src/root_isolation.rs',trace_gcd)
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
env['HYPERCURVE_IMAGE_EXPORT']=str(A/f'{prefix}-image.jsonl')
report=dict(source_manifest=f'{prefix}-sources.json',source_directory=str(archive),workspace_guard='shared-source-refinement-20260926-v270-sources.json',parent_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/'hypercurve',text=True).strip(),hypersolve_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/'hypersolve',text=True).strip(),cases=[],all_processes_reaped=False)
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
