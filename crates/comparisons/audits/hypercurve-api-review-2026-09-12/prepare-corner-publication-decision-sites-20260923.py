from pathlib import Path
import json,shutil,subprocess

audit=Path(__file__).resolve().parent
source=Path('/tmp/hypercurve-corner-publication-blocker-sites-2026-09-23')
root=Path('/tmp/hypercurve-corner-publication-decision-sites-2026-09-23')
root.mkdir()
for p in source.iterdir():
    if not p.is_dir():continue
    if p.name=='hypercurve':shutil.copytree(p,root/p.name)
    else:(root/p.name).symlink_to(p.resolve(),target_is_directory=True)
(root/'hypercurve/tests/hypercurve_curve_region_promotion.rs').write_bytes((audit.parent/'hypercurve/tests/hypercurve_curve_region_promotion.rs').read_bytes())
branches={}
def instrument(path,start_marker,end_marker,tag):
    s=path.read_text();a=s.index(start_marker);b=s.index(end_marker,a+len(start_marker));body=s[a:b];base=s[:a].count('\n')+1
    lines=body.splitlines(True)
    for i,line in enumerate(lines):
        label=f'{tag}:{base+i}'
        message=f'eprintln!("DECISION_SITE {label} reason={{reason:?}}");'
        if line.strip()=='return Ok(Classification::Uncertain(reason));':
            indent=line[:len(line)-len(line.lstrip())]
            lines[i]=indent+message+'\n'+line
        elif 'Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),' in line:
            lines[i]=line.replace('Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),',f'Classification::Uncertain(reason) => {{ {message} return Ok(Classification::Uncertain(reason)); }},')
        elif 'Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),' in line:
            lines[i]=line.replace('Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),',f'Classification::Uncertain(reason) => {{ {message} Ok(Classification::Uncertain(reason)) }},')
        elif 'Classification::Uncertain(reason) => Classification::Uncertain(reason),' in line:
            lines[i]=line.replace('Classification::Uncertain(reason) => Classification::Uncertain(reason),',f'Classification::Uncertain(reason) => {{ {message} Classification::Uncertain(reason) }},')
        else:continue
        branches[label]=dict(file=str(path.relative_to(root/'hypercurve')),original_line=base+i,context=''.join(body.splitlines(True)[max(0,i-3):i+4]))
    path.write_text(s[:a]+''.join(lines)+s[b:])

instrument(root/'hypercurve/src/rational_bezier_general.rs','    fn intersection_context_classified(', '\n    fn retained_linear_image_contacts(', 'RATIONAL_CONTEXT')
instrument(root/'hypercurve/src/curve_intersection.rs','    pub(crate) fn restrict_raw(', '\n    /// Publishes certified paired subranges', 'OVERLAP_RESTRICT')
instrument(root/'hypercurve/src/bezier_split.rs','fn forward_corresponding_parameter_ranges(', '\n/// A native Bezier subcurve', 'PARAMETER_CLIP')
p=root/'hypercurve/src/curve_intersection.rs';s=p.read_text();a=s.index('    pub(crate) fn restrict_raw(');b=s.index('        let clip = |active, limit| {',a)
s=s[:b]+'''        eprintln!("OVERLAP_RESTRICT correspondence={:?}", std::mem::discriminant(&self.parameter_correspondence));
'''+s[b:];p.write_text(s)
p=root/'hypercurve/src/curve_region_boolean.rs';s=p.read_text();needle='    fn blocked(&self, carrier_index: usize, reason: UncertaintyReason) -> ExactCurveError {'
assert s.count(needle)==1
s=s.replace(needle,'    #[track_caller]\n'+needle)
needle='            if let Some(blocker) = result.blockers.first() {'
assert s.count(needle)==1
s=s.replace(needle,needle+'''
                eprintln!("PAIR_BLOCKER first={} second={} families=({:?},{:?}) context={:?}", pair.first_carrier_index, pair.second_carrier_index, self.data.carriers[pair.first_carrier_index].family, self.data.carriers[pair.second_carrier_index].family, std::mem::discriminant(&pair.context));''')
p.write_text(s)
paths=['src/error.rs','src/rational_bezier_general.rs','src/curve_intersection.rs','src/bezier_split.rs','src/curve_region_boolean.rs']
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true',*[str(root/'hypercurve'/p) for p in paths]],check=True)
(audit/'corner-publication-decision-sites-20260923-branches.json').write_text(json.dumps(branches,indent=2)+'\n')
runner=(audit/'run-corner-publication-blocker-sites2-20260923.py').read_text().replace(str(source),str(root)).replace('corner-publication-blocker-sites-20260923-trace2','corner-publication-decision-sites-20260923-trace1')
(audit/'run-corner-publication-decision-sites-20260923.py').write_text(runner)
print('Prepared bounded decision-site diagnostics:',len(branches),'uncertainty branches. No full geometry Debug output is enabled.')
