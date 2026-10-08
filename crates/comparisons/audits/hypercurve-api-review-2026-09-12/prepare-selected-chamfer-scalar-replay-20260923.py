from pathlib import Path
import hashlib, json, shutil, subprocess

A=Path(__file__).resolve().parent
source=Path('/tmp/hypercurve-circle-closure-probe5-2026-09-23')
root=Path('/tmp/hypercurve-selected-chamfer-scalar-replay-2026-09-23')
assert json.loads((A/'fiber-ring-arithmetic-20260923-probe1-terminal.json').read_text())['all_processes_reaped']
assert not root.exists()
bindings=json.loads((A/'circle-closure-20260923-probe5-sources.json').read_text())
for name,sha in bindings.items():
    assert hashlib.sha256((source/name).read_bytes()).hexdigest()==sha,name
    (root/name).parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(source/name,root/name)
p=root/'hypersolve/src/algebraic_fiber.rs'
s=p.read_text()
needle='            static PRINTED: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);'
assert s.count(needle)==1
insert='''            static CAPTURED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
            if let [constant] = polynomial
                && !CAPTURED.swap(true, std::sync::atomic::Ordering::Relaxed)
            {
                let replay = constant.to_json();
                assert!(replay.len() <= 1024 * 1024, "scalar replay exceeds its explicit size bound");
                std::fs::write("SCALAR_REPLAY_PATH", &replay).unwrap();
                eprintln!("SCALAR-REPLAY bytes={} immediate={:?} zero={:?} tower={:?}", replay.len(), constant.immediate_sign(), constant.zero_status(), constant.quadratic_tower_sign());
            }
'''.replace('SCALAR_REPLAY_PATH',str(A/'selected-chamfer-scalar-20260923-replay.json'))
s=s.replace(needle,insert+needle)
p.write_text(s)
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true',str(p)],check=True)
bindings['hypersolve/src/algebraic_fiber.rs']=hashlib.sha256(p.read_bytes()).hexdigest()
for name,sha in bindings.items(): assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
(A/'selected-chamfer-scalar-20260923-probe1-sources.json').write_text(json.dumps(bindings,indent=2)+'\n')
runner=(A/'run-circle-closure-probe5-20260923.py').read_text().replace(str(source),str(root)).replace('circle-closure-20260923-probe5','selected-chamfer-scalar-20260923-probe1')
a=runner.index('selected=[');b=runner.index('\n]\n',a)+3
runner=runner[:a]+"selected=[('selected_fiber_transverse_mapped_cut_inverts_by_point',90)]\n"+runner[b:]
(A/'run-selected-chamfer-scalar-replay-20260923.py').write_text(runner)
(A/'fiber-ring-arithmetic-20260923-probe1-rejected.json').write_text(json.dumps(dict(reason='Constructed determinant still has an undecided scalar zero when consumed by topology; the caller hit 120 seconds rather than completing.',production_changes=False,all_processes_reaped=True),indent=2)+'\n')
print('Prepared one bounded scalar replay extraction; original Hypersolve arithmetic retained.')
