from pathlib import Path
import hashlib,json,shutil,subprocess
A=Path(__file__).resolve().parent
assert json.loads((A/'selected-chamfer-inverse-20260923-probe1-terminal.json').read_text())['all_processes_reaped']
repo=Path('/home/tim/Documents/GitHub/workspace/hypercurve')
p=repo/'src/curve.rs'
s=p.read_text()
old="""        let evaluator = if evaluator.retained_circular_conic().is_some()
            || matches!(
                evaluator.control_weight_sign(),
                Classification::Decided(RealSign::Positive | RealSign::Negative)
            ) {
"""
new="""        // Circular recognition is a reusable support certificate even when
        // the chart already has same-sign weights. Retain it before lowering
        // degree or publishing a trimmed chart for subsequent intersections.
        let evaluator = if evaluator.retained_circular_conic().is_some() {
"""
assert s.count(old)==1
p.write_text(s.replace(old,new))
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true',str(p)],check=True)
subprocess.run(['git','diff','--check'],cwd=repo,check=True)
source=Path('/tmp/hypercurve-selected-overlap-cache-v3-2026-09-23')
root=Path('/tmp/hypercurve-recognized-circle-evidence-v1-2026-09-23')
assert not root.exists()
root.mkdir()
shutil.copytree(source/'hypercurve',root/'hypercurve')
for dep in source.iterdir():
    if dep.name!='hypercurve' and dep.is_dir(): (root/dep.name).symlink_to(dep.resolve(),target_is_directory=True)
shutil.copy2(p,root/'hypercurve/src/curve.rs')
bind=json.loads((A/'selected-overlap-cache-20260923-v3-sources.json').read_text())
bind['hypercurve/src/curve.rs']=hashlib.sha256(p.read_bytes()).hexdigest()
for name,sha in bind.items(): assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
(A/'recognized-circle-evidence-20260923-v1-sources.json').write_text(json.dumps(bind,indent=2)+'\n')
runner=(A/'run-selected-overlap-cache-v3-20260923.py').read_text().replace(str(source),str(root)).replace('selected-overlap-cache-20260923-v3','recognized-circle-evidence-20260923-v1')
a=runner.index('selected=[')
b=runner.index('\n]\n',a)+3
runner=runner[:a]+"""selected=[
    ('nonrepresented_chord_and_retained_rational_arc_share_the_fillet_kernel',180),
    ('general_nonrepresented_chord_and_retained_rational_arc_complete_the_fillet_kernel',180),
    ('major_retained_rational_arc_and_general_chord_share_the_fillet_kernel',180),
    ('selected_circle_and_retained_rational_arc_fillet_exactly',120),
    ('selected_circle_and_major_retained_rational_arc_fillet_exactly',120),
    ('selected_circle_pair_with_rationalizable_support_fillet_exactly',120),
]
"""+runner[b:]
(A/'run-recognized-circle-evidence-v1-20260923.py').write_text(runner)
print('Frozen recognized circle evidence candidate',bind['hypercurve/src/curve.rs'])
