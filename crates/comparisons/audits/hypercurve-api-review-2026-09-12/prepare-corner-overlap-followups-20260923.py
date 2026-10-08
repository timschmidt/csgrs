from pathlib import Path
import hashlib, json, re, shutil, subprocess

audit=Path(__file__).resolve().parent
main=audit.parent/'hypercurve'
source=Path('/tmp/hypercurve-corner-overlap-authority-2026-09-23')
root=Path('/tmp/hypercurve-corner-overlap-authority-final-2026-09-23')
root.mkdir()
for p in source.iterdir():
    if not p.is_dir():continue
    if p.name=='hypercurve':shutil.copytree(p,root/p.name)
    else:(root/p.name).symlink_to(p.resolve(),target_is_directory=True)
for name in ('src/bezier_offset.rs','src/rational_bezier_general.rs'):
    (root/'hypercurve'/name).write_bytes((main/name).read_bytes())
(root/'hypercurve/src/bezier_region.rs').write_bytes(subprocess.check_output(['git','show','fc0197e60fbe79b37a8d144a338828f76d531b03:src/bezier_region.rs'],cwd=main))
binding={name:hashlib.sha256((root/'hypercurve'/name).read_bytes()).hexdigest() for name in ('src/bezier_offset.rs','src/rational_bezier_general.rs','src/bezier_region.rs')}
(audit/'corner-overlap-authority-20260923-final-candidate.json').write_text(json.dumps(binding,indent=2)+'\n')

root=Path('/tmp/hypercurve-corner-publication-decision-sites-2026-09-23')
assert json.loads((audit/'corner-publication-decision-sites-20260923-trace2-terminal.json').read_text())['all_processes_reaped']
for name in ('src/bezier_offset.rs','src/rational_bezier_general.rs'):
    (root/'hypercurve'/name).write_bytes((main/name).read_bytes())
# Reapply the already recorded narrow diagnostics to the new two-file candidate.
preparation=(audit/'prepare-corner-publication-decision-sites2-20260923.py').read_text()
a=preparation.index('branches = {}');b=preparation.index("p=root/'hypercurve/src/bezier_split.rs';s=p.read_text()",a)
exec(preparation[a:b])
p=root/'hypercurve/src/bezier_offset.rs';s=p.read_text()
needle='        eprintln!("CUSP_CMP_EXHAUSTED first={} second={}", parameter_shape(self, 5), parameter_shape(other, 5));'
assert s.count(needle)==1
s=s.replace(needle,needle+'''
        if let (Self::Mapped(first), Self::Mapped(second)) = (self, other)
            && let (
                BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlapMap { overlap: a, source: x, source_first: af },
                BezierAlgebraicCuspSemicircleMappedParameterData2::PairOverlapMap { overlap: b, source: y, source_first: bf },
            ) = (first.as_ref(), second.as_ref())
        {
            eprintln!("CUSP_MAP_ID sides=({af},{bf}) source_equal={} ordered_charts=({},{}) swapped_charts=({},{}) orientation=({:?},{:?}) policy_equal={}",
                x.shares_exact_evidence(y), a.semicircle(true)==b.semicircle(true), a.semicircle(false)==b.semicircle(false), a.semicircle(true)==b.semicircle(false), a.semicircle(false)==b.semicircle(true), a.orientation(), b.orientation(), a.data.policy==b.data.policy);
        }
''')
p.write_text(s)
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true',str(root/'hypercurve/src/bezier_offset.rs'),str(root/'hypercurve/src/rational_bezier_general.rs')],check=True)
(audit/'corner-publication-decision-sites3-20260923-branches.json').write_text(json.dumps(branches,indent=2)+'\n')
runner=(audit/'run-corner-publication-decision-sites-20260923.py').read_text().replace('corner-publication-decision-sites-20260923-trace1','corner-publication-decision-sites-20260923-trace3')
(audit/'run-corner-publication-decision-sites3-20260923.py').write_text(runner)
print('Prepared clean production qualification and normalized diagnostic continuation; neither runner has launched.')
