from pathlib import Path
import hashlib, json, re, shutil, subprocess

audit=Path(__file__).resolve().parent
main=audit.parent/'hypercurve'
source=Path('/tmp/hypercurve-corner-overlap-authority-final-2026-09-23')
root=Path('/tmp/hypercurve-circle-map-direction-final-2026-09-23')
root.mkdir()
for p in source.iterdir():
    if not p.is_dir():continue
    if p.name=='hypercurve':shutil.copytree(p,root/p.name)
    else:(root/p.name).symlink_to(p.resolve(),target_is_directory=True)
(root/'hypercurve/src/bezier_offset.rs').write_bytes((main/'src/bezier_offset.rs').read_bytes())
for name in ('src/rational_bezier_general.rs','src/bezier_region.rs'):
    assert (root/'hypercurve'/name).read_bytes()==subprocess.check_output(['git','show','905e2bc74e2ba01a4dcf0b3e7fbee8d854383949:'+name],cwd=main)
binding={name:hashlib.sha256((root/'hypercurve'/name).read_bytes()).hexdigest() for name in ('src/bezier_offset.rs','src/rational_bezier_general.rs','src/bezier_region.rs')}
(audit/'circle-map-direction-20260923-final-candidate.json').write_text(json.dumps(binding,indent=2)+'\n')

root=Path('/tmp/hypercurve-corner-publication-decision-sites-2026-09-23')
assert json.loads((audit/'corner-publication-decision-sites-20260923-trace4-terminal.json').read_text())['all_processes_reaped']
p=root/'hypercurve/src/curve_region_boolean.rs';s=p.read_text();branches={}
needle='vec![RegionPairBlocker::Uncertain(reason)]'
def annotate(match):
    line=s[:match.start()].count('\n')+1
    label=f'PAIR_UNCERTAIN:{line}'
    branches[label]={'file':'src/curve_region_boolean.rs','line':line,'context':s[max(0,match.start()-180):match.end()+90]}
    return '{ if reason == UncertaintyReason::Ordering { eprintln!("DECISION_SITE '+label+' reason={reason:?}"); } '+match.group()+' }'
s=re.sub(re.escape(needle),annotate,s)
a=s.index('    pub(crate) fn build_intersection_evidence(');b=s.index('            for blocker in result.blockers {',a)+len('            for blocker in result.blockers {')
s=s[:b]+'''
                eprintln!("EVIDENCE_PAIR_BLOCKER carriers=({},{}) families=({:?},{:?}) context={:?} blocker={:?}", pair.first_carrier_index, pair.second_carrier_index, self.data.carriers[pair.first_carrier_index].family, self.data.carriers[pair.second_carrier_index].family, std::mem::discriminant(&pair.context), std::mem::discriminant(&blocker));
'''+s[b:]
p.write_text(s)
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true',str(p)],check=True)
(audit/'corner-publication-decision-sites5-20260923-branches.json').write_text(json.dumps(branches,indent=2)+'\n')
runner=(audit/'run-corner-publication-decision-sites4-20260923.py').read_text().replace('corner-publication-decision-sites-20260923-trace4','corner-publication-decision-sites-20260923-trace5')
(audit/'run-corner-publication-decision-sites5-20260923.py').write_text(runner)
print('Prepared final production qualification and narrow pair-blocker diagnostics:',len(branches),'sites; neither runner launched.')
