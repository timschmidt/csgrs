from pathlib import Path
import hashlib, json, shutil, subprocess

A=Path(__file__).resolve().parent
source=Path('/tmp/hypercurve-similarity-point-probe2-2026-09-23')
dependencies=Path('/tmp/hypercurve-biquadratic-basis-v5-2026-09-23')
root=Path('/tmp/hypercurve-similarity-point-probe3-2026-09-23')
assert json.loads((A/'similarity-point-20260923-probe2-terminal.json').read_text())['all_processes_reaped']
assert not root.exists();root.mkdir()
bindings=json.loads((A/'similarity-point-20260923-probe2-sources.json').read_text())
for name,sha in bindings.items():
    assert hashlib.sha256((source/name).read_bytes()).hexdigest()==sha,name
    crate=name.split('/')[0]
    if crate=='hypercurve':
        (root/name).parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source/name,root/name)
    else:
        assert hashlib.sha256((dependencies/name).read_bytes()).hexdigest()==sha,name
        if not (root/crate).exists(): (root/crate).symlink_to(dependencies/crate,target_is_directory=True)
p=root/'hypercurve/src/bezier_offset.rs';s=p.read_text()
def instrument(start,end):
    global s
    a=s.index(start);b=s.index(end,a);part=s[a:b]
    tag=start.strip().split('(')[0].replace('fn ','')
    part=part.replace('return Ok(Classification::Uncertain(reason));',f'point_inverse_probe("{tag}",line!(),&format!("{{reason:?}}")); return Ok(Classification::Uncertain(reason));')
    part=part.replace('Ok(Classification::Uncertain(last_reason))',f'{{ point_inverse_probe("{tag}",line!(),&format!("{{last_reason:?}}")); Ok(Classification::Uncertain(last_reason)) }}')
    s=s[:a]+part+s[b:]
instrument('fn common_polynomial_roots(', 'fn first_incident_ray_polynomial_root(')
instrument('fn one_field_common_zero_parameter_candidates(', 'fn one_field_point_parameter_candidates(')
instrument('    fn point_incidence_in_domain(', '    /// Classifies whether `point` lies on this parallel')
needle='''        if let Classification::Decided(Some(point)) = self.exact_point(policy)? {
            let inverse ='''
assert s.count(needle)==1
s=s.replace(needle,'''        let exact_point = self.exact_point(policy)?;
        point_inverse_probe("source point inverse",line!(),if matches!(&exact_point,Classification::Decided(Some(_))) { "represented point" } else { "no represented point" });
        if let Classification::Decided(Some(point)) = exact_point {
            let inverse =''')
a=s.index('fn parametric_point_parameter_candidates(');b=s.index('impl BezierAlgebraicCuspSemicircleMappedParameterData2 {',a)
part=s[a:b];needle='    if let Some(value) = represented {';assert part.count(needle)==1
part=part.replace(needle,'''    point_inverse_probe("parametric point inverse",line!(),if represented.is_some() { "represented parameter" } else { "selected parameter" });
'''+needle)
s=s[:a]+part+s[b:]
a=s.index('fn mapped_circle_tangent_parameter_candidates(');b=s.index('/// Analytic replay preserves',a)
part=s[a:b];needle='    Ok(candidates.map(curve_region_parameters_from_bezier))';assert part.count(needle)==1
part=part.replace(needle,'''    if let Classification::Uncertain(reason)=&candidates { point_inverse_probe("circle tangent projection",line!(),&format!("{reason:?}")); }
'''+needle)
s=s[:a]+part+s[b:]
p.write_text(s)
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true',str(p)],check=True)
bindings['hypercurve/src/bezier_offset.rs']=hashlib.sha256(p.read_bytes()).hexdigest()
(A/'similarity-point-20260923-probe3-sources.json').write_text(json.dumps(bindings,indent=2)+'\n')
runner=(A/'run-similarity-point-probe2-20260923.py').read_text().replace(str(source),str(root)).replace('similarity-point-20260923-probe2','similarity-point-20260923-probe3')
# A diagnostic needs the test compile only. Dependencies are intentionally
# shared from the qualified physical v5 snapshot and checked in Cargo output.
a=runner.index('checks=[]');b=runner.index("command=[cargo,'test'",a)
runner=runner[:a]+'checks=[]\n'+runner[b:]
needle="assert code == 0\nbinary=";assert runner.count(needle)==1
runner=runner.replace(needle,"""assert code == 0
dependency_artifacts=[json.loads(line) for line in (audit/f'{prefix}-build.jsonl').read_text().splitlines()]
for crate in ['hyperreal','hyperlattice','hyperlimit','hypersolve']:
    matches=[row for row in dependency_artifacts if row.get('reason')=='compiler-artifact' and row['target']['name']==crate]
    assert matches and all('/tmp/hypercurve-biquadratic-basis-v5-2026-09-23/'+crate in row['package_id'] for row in matches),crate
binary=""")
(A/'run-similarity-point-probe3-20260923.py').write_text(runner)
print('Prepared bounded solver diagnostic with explicitly pinned v5 dependency paths.')
