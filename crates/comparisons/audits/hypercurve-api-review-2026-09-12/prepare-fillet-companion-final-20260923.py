from pathlib import Path
import json, shutil, subprocess

audit=Path(__file__).resolve().parent
source=Path('/tmp/hypercurve-fillet-companion-clean-2026-09-23')
root=Path('/tmp/hypercurve-fillet-companion-final-2026-09-23')
assert json.loads((audit/'fillet-companion-chart-20260923-broad1-terminal.json').read_text())['all_processes_reaped']
root.mkdir()
for p in source.iterdir():
    if not p.is_dir(): continue
    if p.name=='hypercurve': shutil.copytree(p,root/p.name)
    else: (root/p.name).symlink_to(p.resolve(),target_is_directory=True)
p=root/'hypercurve/src/curve_corner_chain.rs'
s=p.read_text()
a=s.index('            // Canonicalization transports the cut parameter into its replacement')
b=s.index('            let canonical_anchor_tangent =',a)
block=s[a:b]
s=s[:a]+s[b:]
block=block.replace('            // Canonicalization transports the cut parameter into its replacement\n', '            // Chord contacts above use the geometric support and exact point.\n            // Parameterized contacts below must also share the cut\'s chart.\n            // Canonicalization transports the cut parameter into its replacement\n')
needle='            if let BezierSplitFragment2::SelectedFiber(other_fragment) = other_fragment\n'
assert s.count(needle)==1
s=s.replace(needle,block+needle)
s=s.replace('unreachable!("the retained fillet companion family was checked")','unreachable!("the retained fillet companion families are exhausted")')
p.write_text(s)
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true',str(p)],check=True)
public=Path('/tmp/hypercurve-fillet-companion-public2-2026-09-23')
public.mkdir()
for child in root.iterdir():
    if not child.is_dir(): continue
    if child.name=='hypercurve': shutil.copytree(child,public/child.name)
    else: (public/child.name).symlink_to(child.resolve(),target_is_directory=True)
name='fillet_companion_public_20260923.rs'
(public/'hypercurve/examples'/name).write_bytes((Path('/tmp/hypercurve-fillet-companion-public-2026-09-23/hypercurve/examples')/name).read_bytes())
runner=(audit/'run-fillet-companion-public-20260923.py').read_text().replace('/tmp/hypercurve-fillet-companion-public-2026-09-23',str(public)).replace('fillet-companion-chart-20260923-public1','fillet-companion-chart-20260923-public2')
(audit/'run-fillet-companion-public2-20260923.py').write_text(runner)
print('Prepared final chart candidate, retaining the point-based chord route before chart transport.')
