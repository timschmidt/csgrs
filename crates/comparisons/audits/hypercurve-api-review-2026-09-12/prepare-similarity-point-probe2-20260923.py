from pathlib import Path
import hashlib, json, shutil, subprocess

A=Path(__file__).resolve().parent
source=Path('/tmp/hypercurve-similarity-point-image-v1-2026-09-23')
root=Path('/tmp/hypercurve-similarity-point-probe2-2026-09-23')
assert json.loads((A/'similarity-point-image-20260923-v1-terminal.json').read_text())['all_processes_reaped']
assert not root.exists()
bindings=json.loads((A/'similarity-point-image-20260923-v1-sources.json').read_text())
for name,sha in bindings.items():
    assert hashlib.sha256((source/name).read_bytes()).hexdigest()==sha,name
    (root/name).parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source/name,root/name)
p=root/'hypercurve/src/bezier_offset.rs';s=p.read_text()
def instrument(start,end,none=False):
    global s
    a=s.index(start);b=s.index(end,a);part=s[a:b]
    tag=start.strip().split('(')[0].replace('fn ','')
    part=part.replace('return Ok(Classification::Uncertain(reason));',f'point_inverse_probe("{tag}", line!(), &format!("{{reason:?}}")); return Ok(Classification::Uncertain(reason));')
    if none:part=part.replace('return Ok(None);',f'point_inverse_probe("{tag}", line!(), "no image"); return Ok(None);')
    part=part.replace('return Ok(Classification::Decided(None));',f'point_inverse_probe("{tag}", line!(), "no point"); return Ok(Classification::Decided(None));')
    s=s[:a]+part+s[b:]
instrument('    pub(crate) fn other_parameter_for_cusp(', '\nfn rational_overlap_parameter_for_exact_cusp(')
instrument('    fn retained_point_parameter_candidates_on_target(', '    fn retained_point_parameter_candidates_on_rational_target(')
instrument('fn one_field_point_parameter_candidates(', 'fn parametric_point_parameter_candidates(')
instrument('fn parametric_point_parameter_candidates(', 'impl BezierAlgebraicCuspSemicircleMappedParameterData2 {')
instrument('    fn exact_one_field_point_image(', '    /// Compares a concentric image',True)
a=s.index('    fn retained_one_field_point_image(');b=s.index('    fn one_field_circle_tangent_parameter_candidates_on_target(',a)
part=s[a:b]
part=part.replace('        Ok(match self.retained_point_evidence() {','        let image = match self.retained_point_evidence() {')
assert part.endswith('        })\n    }\n\n')
part=part[:-len('        })\n    }\n\n')]+'''        };
        point_inverse_probe("retained_one_field_point_image",line!(),if image.is_some() { "available" } else { "unavailable" });
        Ok(image)
    }

'''
s=s[:a]+part+s[b:]
marker='\nfn rational_overlap_parameter_for_exact_cusp('
helper='''
fn point_inverse_probe(stage: &str, line: u32, detail: &str) {
    static COUNT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
    if COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed) < 128 {
        eprintln!("point inverse probe: {stage}:{line}: {detail}");
    }
}
'''
assert s.count(marker)==1;s=s.replace(marker,helper+marker)
p.write_text(s)
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true',str(p)],check=True)
bindings['hypercurve/src/bezier_offset.rs']=hashlib.sha256(p.read_bytes()).hexdigest()
(A/'similarity-point-20260923-probe2-sources.json').write_text(json.dumps(bindings,indent=2)+'\n')
runner=(A/'run-similarity-point-image-v1-20260923.py').read_text().replace(str(source),str(root)).replace('similarity-point-image-20260923-v1','similarity-point-20260923-probe2')
a=runner.index('selected=[');b=runner.index('\n]\n',a)+3
runner=runner[:a]+"selected=[('selected_fiber_transverse_mapped_cut_inverts_by_point',120)]\n"+runner[b:]
(A/'run-similarity-point-probe2-20260923.py').write_text(runner)
print('Prepared bounded stage diagnostic:',bindings['hypercurve/src/bezier_offset.rs'])
