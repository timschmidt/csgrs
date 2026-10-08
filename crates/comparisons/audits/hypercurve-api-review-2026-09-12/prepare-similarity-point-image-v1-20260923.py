from pathlib import Path
import hashlib, json, shutil, subprocess

A = Path(__file__).resolve().parent
source = Path('/tmp/hypercurve-biquadratic-basis-v5-2026-09-23')
root = Path('/tmp/hypercurve-similarity-point-image-v1-2026-09-23')
assert json.loads((A/'biquadratic-basis-20260923-qualification.json').read_text())['all_owned_processes_reaped']
assert not root.exists()
bindings = json.loads((A/'biquadratic-basis-20260923-v5-sources.json').read_text())
for name, sha in bindings.items():
    assert hashlib.sha256((source/name).read_bytes()).hexdigest() == sha, name
    (root/name).parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source/name, root/name)

p = root/'hypercurve/src/bezier_offset.rs'
s = p.read_text()
a = s.index('    fn retained_one_field_point_image(')
b = s.index('    fn one_field_circle_tangent_parameter_candidates_on_target(', a)
part = s[a:b]
old = '''            Some(
                CurvePoint2(CurvePointData2::Exact(_))'''
new = '''            Some(CurvePoint2(CurvePointData2::Similarity(point))) => {
                match point.predicate_point_evidence(policy)? {
                    Classification::Decided(Some(CurvePoint2(CurvePointData2::Algebraic(point)))) => Some(point),
                    Classification::Decided(_) | Classification::Uncertain(_) => None,
                }
            }
            Some(
                CurvePoint2(CurvePointData2::Exact(_))'''
assert part.count(old) == 1
part = part.replace(old, new).replace('CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)', 'CurvePointData2::Endpoint(_)')
s = s[:a]+part+s[b:]
a = s.index('impl BezierSimilarityPoint2 {')
b = s.index('    pub(crate) fn conservative_bounds_refined(', a)
part = s[a:b]
old = '''            CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))'''
new = '''            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(point)) => {
                let Some(point) = point.exact_one_field_point_image(policy)? else {
                    return Ok(Classification::Decided(None));
                };
                CurvePoint2::from(point)
            }
            CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))'''
assert part.count(old) == 1
part = part.replace(old, new)
s = s[:a]+part+s[b:]
p.write_text(s)
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true',str(p)],check=True)
bindings['hypercurve/src/bezier_offset.rs'] = hashlib.sha256(p.read_bytes()).hexdigest()
(A/'similarity-point-image-20260923-v1-sources.json').write_text(json.dumps(bindings,indent=2)+'\n')
runner = (A/'run-biquadratic-basis-v5-hypercurve-20260923.py').read_text().replace(str(source),str(root)).replace('biquadratic-basis-20260923-v5-hypercurve','similarity-point-image-20260923-v1').replace('biquadratic-basis-20260923-v5-sources','similarity-point-image-20260923-v1-sources')
a = runner.index('selected=['); b = runner.index('\n]\n',a)+3
runner = runner[:a]+'''selected=[
    ('selected_fiber_transverse_mapped_cut_inverts_by_point',180),
    ('similarity_transported_mapped_cusp_cut_inverts_on_analytic_overlap',120),
    ('similarity_transported_transverse_mapped_cut_inverts_by_point',120),
    ('recursive_projective_kernel_imports_similarity_of_algebraic_endpoints',120),
    ('mapped_compact_point_inverse_preserves_analytic_normal_sheet',120),
    ('arithmetic_adapter_replays_close_nonrational_bounds_strictly',120),
]
'''+runner[b:]
(A/'run-similarity-point-image-v1-20260923.py').write_text(runner)
print('Prepared two existing point projection paths:',bindings['hypercurve/src/bezier_offset.rs'])
