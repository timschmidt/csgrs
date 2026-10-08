from pathlib import Path
import io,json,shutil,subprocess,tarfile

audit=Path(__file__).resolve().parent
workspace=audit.parent
normal=Path('/tmp/hypercurve-corner-publication-postchart-2026-09-23')
assert json.loads((audit/'corner-publication-postchart-20260923-broad1-terminal.json').read_text())['all_processes_reaped']
trace=Path('/tmp/hypercurve-corner-publication-blocker-sites-2026-09-23')
trace.mkdir()
for p in normal.iterdir():
    if not p.is_dir():continue
    if p.name=='hypercurve':shutil.copytree(p,trace/p.name)
    else:(trace/p.name).symlink_to(p.resolve(),target_is_directory=True)
p=trace/'hypercurve/src/error.rs'
s=p.read_text().replace('    pub(crate) const fn blocked(\n','    #[track_caller]\n    pub(crate) fn blocked(\n')
needle='        Self::Blocked(ExactCurveBlocker::new(operation, family, reason))'
assert s.count(needle)==1
s=s.replace(needle,'''        if matches!(reason, UncertaintyReason::RealSign | UncertaintyReason::Ordering) {
            eprintln!("BLOCKER_SITE operation={operation:?} family={family:?} reason={reason:?} caller={}", std::panic::Location::caller());
            eprintln!("{}", std::backtrace::Backtrace::force_capture());
        }
'''+needle)
p.write_text(s)
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true',str(p)],check=True)
runner=(audit/'qualify-corner-publication-postchart-20260923.py').read_text().replace(str(normal),str(trace)).replace('corner-publication-postchart-20260923-focused1','corner-publication-blocker-sites-20260923-trace1')
a=runner.index('selected=[');b=runner.index('\nrows=[]',a)
runner=runner[:a]+'''selected=[
 ('major_retained_rational_arc_and_general_chord_share_the_fillet_kernel',45),
 ('selected_circle_chamfer_crosses_one_sided_smooth_run_seam',45),
]'''+runner[b:]
runner=runner.replace('Built normalized corner candidate','Built diagnostic blocker-site candidate')
(audit/'run-corner-publication-blocker-sites-20260923.py').write_text(runner)

callers=Path('/tmp/hypercurve-fillet-composition-callers-2026-09-23')
callers.mkdir()
base=Path('/tmp/hypercurve-fillet-companion-final-2026-09-23')
for p in base.iterdir():
    if p.is_dir() and p.name!='hypercurve':(callers/p.name).symlink_to(p.resolve(),target_is_directory=True)
head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=workspace/'hypercurve',text=True).strip()
assert head=='deeed31d92f07cc6d7ffc8b93bc89061630c094d'
archive=subprocess.check_output(['git','archive',head],cwd=workspace/'hypercurve')
with tarfile.open(fileobj=io.BytesIO(archive)) as stream:stream.extractall(callers/'hypercurve',filter='data')
name='tests/hypercurve_curve_region_promotion.rs'
p=workspace/'hypercurve'/name
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true',str(p)],check=True)
(callers/'hypercurve'/name).write_bytes(p.read_bytes())
runner=(audit/'qualify-corner-publication-postchart-20260923.py').read_text().replace(str(normal),str(callers)).replace('corner-publication-postchart-20260923-focused1','fillet-composition-callers-20260923-final1')
runner=runner.replace("'test','--lib','--release'","'test','--test','hypercurve_curve_region_promotion','--release'")
runner=runner.replace("row['target']['name']=='hypercurve'","row['target']['name']=='hypercurve_curve_region_promotion'")
a=runner.index('selected=[');b=runner.index('\nrows=[]',a)
runner=runner[:a]+'''selected=[
 ('non_ph_bezier_pair_projective_fillet_retains_algebraic_extensions',400),
]'''+runner[b:]
runner=runner.replace('Built normalized corner candidate','Built migrated public composition test')
(audit/'qualify-fillet-composition-callers-20260923.py').write_text(runner)
print('Prepared separate diagnostic and caller-migration snapshots; the caller snapshot uses committed deeed31 production.')
