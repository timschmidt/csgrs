"""Prepare a paired scalar probe; never mutate a closed Cargo target."""
from pathlib import Path
import hashlib, json, shutil, subprocess
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
TARGET = Path('/tmp/csgrs-strict-drilled-workflows/cost-v1')
COPIES = Path('/tmp/csgrs-strict-drilled-workflows/cost-v1-source')
CORE = Path('/tmp/csgrs-strict-drilled-workflows/candidate-v1/release/deps')
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
assert not TARGET.exists() and not COPIES.exists()
num, = CORE.glob('libnum-*.rlib')
for revision in ['baseline','candidate']:
    shutil.copytree(ROOT/'crates/hyperreal/src', COPIES/revision/'src')
for name in ['real/arithmetic/facts.rs','real/arithmetic/linear_algebra.rs']:
    path = COPIES/'baseline/src'/name
    path.write_bytes(subprocess.check_output(['git','show',f'f1092edb1:crates/hyperreal/src/{name}'],cwd=ROOT))
keep = {}
for p in [*COPIES.rglob('*.rs'), *CORE.glob('*.rlib'), *CORE.glob('*.so'), E/'scalar_cost_v1.rs', Path(__file__)]:keep[str(p)] = sha(p)
commands = [['prepare-target',['mkdir',str(TARGET)]]]
for revision in ['baseline','candidate']:
    commands.append([revision+'-library',['rustc','--crate-name','hyperreal','--crate-type','rlib','--edition=2024','-C','opt-level=3','-C','embed-bitcode=no','-D','warnings','--check-cfg','cfg(verus_keep_ghost)','--check-cfg','cfg(feature, values("default", "simple", "cached-f32-approx", "cached-f64-approx", "dispatch-trace", "serde"))','--cfg','feature="default"','--extern',f'num={num}','-L',f'dependency={CORE}',str(COPIES/revision/'src/lib.rs'),'-o',str(TARGET/f'libhyperreal_{revision}.rlib')]])
    commands.append([revision+'-probe',['rustc','--edition=2024','-C','opt-level=3','-D','warnings','--extern',f'hyperreal={TARGET}/libhyperreal_{revision}.rlib','-L',f'dependency={CORE}',str(E/'scalar_cost_v1.rs'),'-o',str(TARGET/revision)]])
for sample in range(3):
    for revision in (['baseline','candidate'] if sample%2==0 else ['candidate','baseline']):
        commands.append([f'{sample}-{revision}',[str(TARGET/revision),revision,'128']])
for suffix,value in [('.json',commands),('-seed.json',None),('-preserve.json',keep)]:
    p=E/('cost-v1'+suffix);assert not p.exists();p.write_text(json.dumps(value,indent=2)+'\n')
record={
 'baseline':'f1092edb1 Hyperreal production source',
 'candidate':'Current source; only facts.rs and linear_algebra.rs differ in the copies',
 'candidate_source_sha256':{str(p.relative_to(COPIES/'candidate/src')):sha(p) for p in (COPIES/'candidate/src').rglob('*.rs')},
 'kind':'Paired scalar completion/resource probe, not a complete-workflow speed claim',
 'flags':'Both direct rustc builds use identical command arguments except source/output paths. Default scalar feature, same frozen num and transitive dependencies. Separate real Cargo qualification covers default/all-features and downstream behavior.',
 'samples':'Three fresh processes per revision, alternating order. Each process runs four cases by three APIs by two construction boundaries, 32 warmups and 128 measured operations per row. Prepared mode constructs operands once outside timing; rebuilt mode includes operand construction/drop. Scalar constant caches are not globally cold.',
 'oracle':'Independent product cancellation and integer result; sign bounds for positive shifted sine factors and a strictly positive 1e-12 displacement. Certification is counted, never inferred from exit zero. All controls require certified correct signs. Baseline commutator Unknown is explicitly counted; its timing is not a faster/slower successful cancellation.',
 'boundaries':'Timing includes operation, exact rational checks where applicable, certificate query, result materialization as Real and drop. Printing metric rows is outside timing. No scalar serialization or mesh output is timed. This is a scalar-construction/proof contract only.',
 'resources':'System allocator counters for successful allocation calls, requested bytes, peak and retained live bytes above post-warmup baseline. Not total RSS or allocations outside System. External 8 GiB address space and 1800-second wall containment. No internal total-work/byte/cancellation bound claimed.',
 'artifact_migration':'Opposite identical symbolic products now construct exact rational zero, so newly generated expression serialization may differ. Existing encoded expressions remain valid; no wire schema or public signature changes. This is a completeness extension, not a serialization-preserving optimization.',
 'hardware':subprocess.check_output(['lscpu'],text=True),
 'rustc':subprocess.check_output(['rustc','-vV'],text=True),
 'cargo_lock_sha256':sha(ROOT/'Cargo.lock'),
 'limitations':'Shared host without CPU pinning; instrumented, small focused samples. No downstream speedup or unchanged mesh artifact claim.'}
(E/'cost-contract-v1.json').write_text(json.dumps(record,indent=2)+'\n')
print({'protected_files':len(keep),'commands':len(commands)})
