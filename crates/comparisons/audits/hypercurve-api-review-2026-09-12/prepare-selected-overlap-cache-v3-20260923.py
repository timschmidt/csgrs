from pathlib import Path
import hashlib,json,shutil,subprocess
A=Path(__file__).resolve().parent
report=json.loads((A/'selected-overlap-cache-20260923-regression-terminal.json').read_text())
assert report['all_processes_reaped'] and report['all_sources_unchanged']
assert len(report['failed'])==1 and report['failed'][0]['name'].endswith('::selected_fiber_mapped_cut_inverts_on_analytic_overlap')
repo=Path('/home/tim/Documents/GitHub/workspace/hypercurve')
p=repo/'src/bezier_offset.rs'
s=p.read_text()
a=s.index('    fn selected_fiber_mapped_cut_inverts_on_analytic_overlap()')
b=s.index('\n    #[test]',a)
body=s[a:b]
needle="""            let Classification::Decided(selected_cusp_cut) = selected_overlap
"""
assert body.count(needle)==1
body=body.replace(needle,"""            let Classification::Decided(retained) = selected_overlap
                .cusp_parameter_for_other(&selected_parameter, &policy)
                .unwrap()
            else {
                panic!("the inverse correspondence must reuse the live source cut");
            };
            assert!(retained.shares_exact_evidence(selected_image.end_parameter()));
            let BezierAlgebraicCuspSemicircleParameter2::Mapped(source) =
                selected_image.end_parameter()
            else {
                panic!("the transported source cut must retain its exact map");
            };
            let weak_source = Arc::downgrade(source);
            drop(retained);
            drop(selected_image);
            assert!(weak_source.upgrade().is_none());
            // Once the source has expired, reconstruction must still exercise
            // a selected-fiber map through both subsequent carrier switches.
"""+needle)
s=s[:a]+body+s[b:]
p.write_text(s)
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true',str(p)],check=True)
subprocess.run(['git','diff','--check'],cwd=repo,check=True)
source=Path('/tmp/hypercurve-selected-overlap-cache-v2-2026-09-23')
root=Path('/tmp/hypercurve-selected-overlap-cache-v3-2026-09-23')
assert not root.exists()
root.mkdir()
shutil.copytree(source/'hypercurve',root/'hypercurve')
for dep in source.iterdir():
    if dep.name!='hypercurve' and dep.is_dir(): (root/dep.name).symlink_to(dep.resolve(),target_is_directory=True)
shutil.copy2(p,root/'hypercurve/src/bezier_offset.rs')
bind=json.loads((A/'selected-overlap-cache-20260923-v2-sources.json').read_text())
bind['hypercurve/src/bezier_offset.rs']=hashlib.sha256(p.read_bytes()).hexdigest()
for name,sha in bind.items(): assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
(A/'selected-overlap-cache-20260923-v3-sources.json').write_text(json.dumps(bind,indent=2)+'\n')
runner=(A/'run-selected-overlap-cache-v2-20260923.py').read_text().replace(str(source),str(root)).replace('selected-overlap-cache-20260923-v2','selected-overlap-cache-20260923-v3')
a=runner.index('selected=[')
b=runner.index('\n]\n',a)+3
runner=runner[:a]+"""selected=[
    ('selected_fiber_mapped_cut_inverts_on_analytic_overlap',120),
]
"""+runner[b:]
(A/'run-selected-overlap-cache-v3-20260923.py').write_text(runner)
print('Frozen test-only caller migration',bind['hypercurve/src/bezier_offset.rs'])
