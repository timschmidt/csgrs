from pathlib import Path
import hashlib, json, re, shutil, subprocess

audit=Path(__file__).resolve().parent
main=audit.parent/'hypercurve'
source=Path('/tmp/hypercurve-corner-overlap-authority-2026-09-23')
root=Path('/tmp/hypercurve-circle-map-direction-2026-09-23')
root.mkdir()
for p in source.iterdir():
    if not p.is_dir():continue
    if p.name=='hypercurve':shutil.copytree(p,root/p.name)
    else:(root/p.name).symlink_to(p.resolve(),target_is_directory=True)
for name in ('src/bezier_offset.rs','src/rational_bezier_general.rs','src/bezier_region.rs'):
    (root/'hypercurve'/name).write_bytes((main/name).read_bytes())
binding={name:hashlib.sha256((root/'hypercurve'/name).read_bytes()).hexdigest() for name in ('src/bezier_offset.rs','src/rational_bezier_general.rs','src/bezier_region.rs')}
(audit/'circle-map-direction-20260923-candidate.json').write_text(json.dumps(binding,indent=2)+'\n')
runner=(audit/'run-corner-overlap-authority-20260923.py').read_text().replace(str(source),str(root)).replace('corner-overlap-authority-20260923-focused1','circle-map-direction-20260923-focused1')
a=runner.index('selected=[');b=runner.index('\nrows=[]',a)
runner=runner[:a]+'''selected=[
 ('reconstructed_circle_overlap_reuses_parameter_identity_and_inverse',60),
 ('selected_circle_chamfer_crosses_one_sided_smooth_run_seam',75),
 ('algebraic_cusp_semicircle_pair_maps_full_partial_and_endpoint_only_overlap',75),
]
'''+runner[b:]
(audit/'run-circle-map-direction-20260923.py').write_text(runner)

root=Path('/tmp/hypercurve-corner-publication-decision-sites-2026-09-23')
assert json.loads((audit/'corner-publication-decision-sites-20260923-trace3-terminal.json').read_text())['all_processes_reaped']
(root/'hypercurve/src/bezier_offset.rs').write_bytes((main/'src/bezier_offset.rs').read_bytes())
p=root/'hypercurve/src/curve_region_boolean.rs';s=p.read_text()
a=s.index('    fn regularized_fragment_decision_by_boundary_probe(');b=s.index('\n    fn algebraic_fragment_side_classification(',a)
body=s[a:b];base=s[:a].count('\n')+1;branches={}
def annotate(match):
    line=base+body[:match.start()].count('\n')
    label=f'BOUNDARY_PROBE:{line}'
    branches[label]={'file':'src/curve_region_boolean.rs','line':line,'statement':match.group()}
    return match.group()+ '\n'+match.group(1)+'eprintln!("DECISION_SITE '+label+' carrier={carrier_index} candidate={candidate_index} reason={last_reason:?}");'
body=re.sub(r'(?m)^([ \t]*)last_reason = [\s\S]*?;',annotate,body)
needle='        let outer_bounds = match retained_probe_outer_bounds'
assert body.count(needle)==1
body=body.replace(needle,'''        eprintln!("BOUNDARY_PROBE_START carrier={carrier_index} family={:?} geometry={:?} representative={:?}", self.data.carriers[carrier_index].family, std::mem::discriminant(&self.data.carriers[carrier_index].geometry), std::mem::discriminant(&representative.0));
'''+needle)
s=s[:a]+body+s[b:];p.write_text(s)
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true',str(p)],check=True)
(audit/'corner-publication-decision-sites4-20260923-branches.json').write_text(json.dumps(branches,indent=2)+'\n')
runner=(audit/'run-corner-publication-decision-sites-20260923.py').read_text().replace('corner-publication-decision-sites-20260923-trace1','corner-publication-decision-sites-20260923-trace4')
a=runner.index('selected=[');b=runner.index('\nrows=[]',a)
runner=runner[:a]+'''selected=[('major_retained_rational_arc_and_general_chord_share_the_fillet_kernel',60)]
'''+runner[b:]
(audit/'run-corner-publication-decision-sites4-20260923.py').write_text(runner)
print('Prepared clean direction-aware map candidate and narrow boundary-probe diagnostics:',len(branches),'sites; neither runner launched.')
