"""Freeze a matched scalar-cost comparison before executing either revision."""
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
BASE = subprocess.check_output(['git', 'rev-parse', '7d69ef51d'], cwd=ROOT, text=True).strip()
SOURCE = Path('/tmp/csgrs-factored-product-proofs/cost-v1-source')
TARGET = Path('/tmp/csgrs-factored-product-proofs/cost-build-v1')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dump(path, value):
    assert not path.exists()
    path.write_text(json.dumps(value, indent=2) + '\n')


SOURCE.mkdir(exist_ok=False)
maps = {}
for label in ['baseline', 'candidate']:
    maps[label] = {}
    if label == 'baseline':
        names = subprocess.check_output([
            'git', 'ls-tree', '-r', '--name-only', BASE, '--', 'crates/hyperreal/src',
        ], cwd=ROOT, text=True).splitlines()
    else:
        names = subprocess.check_output([
            'git', 'ls-files', '--cached', '--others', '--exclude-standard', '--',
            'crates/hyperreal/src',
        ], cwd=ROOT, text=True).splitlines()
    for name in sorted(set(names)):
        relative = Path(name).relative_to('crates/hyperreal')
        copied = SOURCE / label / relative
        copied.parent.mkdir(parents=True, exist_ok=True)
        copied.write_bytes(subprocess.check_output(['git', 'show', BASE + ':' + name], cwd=ROOT)
                           if label == 'baseline' else (ROOT / name).read_bytes())
        maps[label][str(relative)] = sha(copied)

with tarfile.open(E / 'cost-source-v1.tar.gz', 'x:gz') as archive:
    for label in maps:
        archive.add(SOURCE / label, arcname=label)

commands = [['prepare-target', ['mkdir', str(TARGET)]]]
template = dict(json.loads((E / 'candidate-v2.json').read_text()))['hyperreal-build']
old_target = '/tmp/csgrs-factored-product-proofs/candidate-v2'
external = '/run/user/1000/csgrs-quotient-factor-proofs/diagnostic-v3/external'
controls = ROOT / 'docs/evidence/2026-10-15-strict-drilled-workflows/scalar_cost_v1.rs'
for label in maps:
    library = TARGET / ('libhyperreal_' + label + '.rlib')
    command = [arg.replace(old_target, str(TARGET)) for arg in template]
    command[command.index(str(ROOT / 'crates/hyperreal/src/lib.rs'))] = str(SOURCE / label / 'src/lib.rs')
    command[command.index('-o') + 1] = str(library)
    commands.append([label + '-library', command])
    for kind, source in [('controls', controls), ('forms', E / 'normal_form_cost_v1.rs')]:
        binary = TARGET / (label + '-' + kind)
        commands.append([label + '-' + kind + '-build', [
            'rustc', '--edition=2024', '-C', 'opt-level=3', '-C', 'debuginfo=1',
            '-C', 'force-frame-pointers=yes', '-D', 'warnings',
            '--extern', 'hyperreal=' + str(library), '-L', 'dependency=' + external,
            str(source), '-o', str(binary),
        ]])
        # The unchanged E6 control probe's "candidate" argument means its
        # already-admitted opposite-product identity completes. Both revisions
        # here include that earlier capability, so both require full completion.
        argument = 'candidate' if kind == 'controls' else label
        commands.append([label + '-' + kind + '-preflight', [str(binary), argument, '1']])

preserve = json.loads((E / 'candidate-v2-preserve.json').read_text())
for path in SOURCE.rglob('*'):
    if path.is_file():
        preserve[str(path)] = sha(path)
for path in [controls, E / 'normal_form_cost_v1.rs', E / 'counted_allocator_v1.rs']:
    preserve[str(path)] = sha(path)
for suffix, value in [('.json', commands), ('-seed.json', None),
                      ('-target.json', str(TARGET)), ('-preserve.json', preserve)]:
    dump(E / ('cost-build-v1' + suffix), value)

contract = {
    'baseline': BASE,
    'baseline_scope': 'Last broad-qualified E6 scalar, before E7-E12 polynomial/quotient/shared-factor additions.',
    'candidate_parent': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
    'candidate_scope': 'Frozen E12 Hyperreal source; exact copied hashes below. Hypersolve edits affect tests only.',
    'sources_sha256': maps,
    'source_archive_sha256': sha(E / 'cost-source-v1.tar.gz'),
    'input_sources_sha256': {str(p.relative_to(ROOT)): sha(p) for p in [controls, E / 'normal_form_cost_v1.rs', E / 'counted_allocator_v1.rs']},
    'build_commands': commands,
    'cargo_lock_sha256': sha(ROOT / 'Cargo.lock'),
    'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
    'hardware': subprocess.check_output(['lscpu'], text=True),
    'measurement': {
        'pairs': 5, 'order': 'baseline/candidate, candidate/baseline, baseline/candidate, candidate/baseline, baseline/candidate',
        'control_iterations_per_row': 4096, 'form_iterations_per_row': 256,
        'warmup_per_row': 32,
        'controls': 'Unchanged four E6 cases, three public product-sum APIs, rebuilt and retained operands; both revisions must certify every query.',
        'forms': 'Affine zero, equivalent quotients, quotient factor eighth powers, nested shared-factor zero, +/-1e-12 displacements and unresolved trigonometric control; rebuilt and retained expressions.',
        'timing': 'Construction included for rebuilt mode; retained mode preconstructs and warms the same operands/expression. Query, exact result/sign checks, materialization and temporary destruction are included. Process startup and output formatting are excluded.',
        'cache': 'Single-threaded process per sample; expression-local proof/approximation state retained only in prepared mode; global allocator is counted System. Rebuilt is fresh expression, not a claim of a cold operating-system cache.',
        'correctness': 'Every definite form result must equal the independently derived rational. Candidate must complete each admitted form; unresolved trigonometric control remains Unknown. Controls require exact sign/explicit rational identities. Unknown counts are reported separately, never treated as successful speedup.',
        'costs': 'Wall nanoseconds, allocation/reallocation calls, requested bytes, additional peak live requested bytes, retained byte delta. These are allocator-request quantities, not actual heap usage or RSS; per-process time -v is retained separately.',
        'limits': 'External 1800-second wall and 8 GiB per-process address-space containment. No internal full-query work/heap bound is claimed.',
        'interpretation': 'Shared host, no exclusive CPU ownership, instrumented focused comparison. All fixed samples, unfavorable rows and incomplete outcomes are retained; no cross-library or downstream speedup claim.',
    },
    'artifact_contract': 'Scalar proof queries preserve authoritative expression serialization as exercised by the all-feature regressions. This is a completeness extension; newly successful workflows are not compared as faster versions of earlier failed operations. Binary/archive sizes are reported with identical build flags.',
}
dump(E / 'cost-contract-v1.json', contract)
print('Prepared cost-build-v1 with', len(commands), 'commands and', len(preserve), 'pinned inputs')
