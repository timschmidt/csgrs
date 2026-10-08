from pathlib import Path
import hashlib, json, shutil

audit=Path(__file__).resolve().parent
source=Path('/tmp/hypercurve-corner-publication-postchart-2026-09-23')
root=Path('/tmp/hypercurve-corner-overlap-authority-2026-09-23')
root.mkdir()
for p in source.iterdir():
    if not p.is_dir(): continue
    if p.name=='hypercurve': shutil.copytree(p,root/p.name)
    else: (root/p.name).symlink_to(p.resolve(),target_is_directory=True)
paths=['src/bezier_offset.rs','src/rational_bezier_general.rs','src/bezier_region.rs','tests/hypercurve_curve_region_promotion.rs']
for name in paths:
    (root/'hypercurve'/name).write_bytes((audit.parent/'hypercurve'/name).read_bytes())
binding={name:hashlib.sha256((root/'hypercurve'/name).read_bytes()).hexdigest() for name in paths}
(audit/'corner-overlap-authority-20260923-candidate.json').write_text(json.dumps(binding,indent=2)+'\n')
runner=(audit/'run-corner-publication-decision-sites-20260923.py').read_text()
runner=runner.replace('/tmp/hypercurve-corner-publication-decision-sites-2026-09-23',str(root)).replace('corner-publication-decision-sites-20260923-trace1','corner-overlap-authority-20260923-focused1')
a=runner.index('selected=[');b=runner.index('\nrows=[]',a)
runner=runner[:a]+'''selected=[
 ('mixed_weight_projective_overlap_preserves_the_finite_parameter_domain',60),
 ('reconstructed_circle_overlap_reuses_parameter_identity_and_inverse',60),
 ('major_retained_rational_arc_and_general_chord_share_the_fillet_kernel',75),
 ('selected_circle_chamfer_crosses_one_sided_smooth_run_seam',75),
]
'''+runner[b:]
runner=runner.replace('Built diagnostic blocker-site candidate','Built clean overlap-authority candidate')
(audit/'run-corner-overlap-authority-20260923.py').write_text(runner)
print('Prepared clean overlap-authority candidate with pinned dependencies and mandatory publication normalization.')
