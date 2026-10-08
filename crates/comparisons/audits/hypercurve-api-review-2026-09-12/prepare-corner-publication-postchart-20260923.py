from pathlib import Path
import hashlib, json, shutil, subprocess

audit = Path(__file__).resolve().parent
workspace = audit.parent
source = Path('/tmp/hypercurve-fillet-companion-final-2026-09-23')
root = Path('/tmp/hypercurve-corner-publication-postchart-2026-09-23')
qualification = json.loads((audit/'fillet-companion-chart-20260923-broad2-terminal.json').read_text())
assert not qualification['new_failures']
assert qualification['all_processes_reaped']
assert (workspace/'hypercurve/src/curve_corner_chain.rs').read_bytes() == (source/'hypercurve/src/curve_corner_chain.rs').read_bytes()
root.mkdir()
for child in source.iterdir():
    if not child.is_dir():
        continue
    if child.name == 'hypercurve':
        shutil.copytree(child, root/child.name)
    else:
        (root/child.name).symlink_to(child.resolve(), target_is_directory=True)
region = workspace/'hypercurve/src/bezier_region.rs'
(root/'hypercurve/src/bezier_region.rs').write_bytes(region.read_bytes())
assert 'if self.data.boundary_loops.len() == 1 && selected_circle_chain' not in region.read_text()
assert 'selected_circle_corner_candidates_publish_normalized_single_loops' in region.read_text()
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true',str(root/'hypercurve/src/bezier_region.rs')],check=True)
(audit/'corner-publication-postchart-20260923-input.json').write_text(json.dumps(dict(source=str(source),root=str(root),main_region_sha256=hashlib.sha256(region.read_bytes()).hexdigest(), candidate_region_sha256=hashlib.sha256((root/'hypercurve/src/bezier_region.rs').read_bytes()).hexdigest()),indent=2)+'\n')
print('Prepared isolated normalized corner-publication candidate.')
