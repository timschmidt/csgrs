"""Compile matched large-coefficient diagnostics from verified frozen inputs."""
import shutil
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True
from coefficient_runner_v1 import ROOT, RUNTIME, W, dump, read, resources, run, sha, violation

old = W.parent / 'circle-support-index/cost-build-v4'
protocol = read(old / 'protocol.json')
assert read(old / 'result.json')['source_and_read_only_dependencies_unchanged']
assert read('/dev/shm/csgrs-circle-support-consumers-v1/consumer-validation-v1/result.json')['cargo_target_closed']
assert subprocess.check_output(['rustc', '-vV'], text=True) == protocol['rustc']
RUNTIME.mkdir(mode=0o700, exist_ok=False)
output = RUNTIME / 'build-v1'
output.mkdir()
assert violation(resources()) is None
dependencies = protocol['read_only_dependencies_sha256']
assert all(sha(path) == digest for path, digest in dependencies.items())
pins = {str(path): sha(path) for path in [
    Path(__file__), W / 'coefficient_runner_v1.py', W / 'coefficient_probe_v1.rs',
    W / 'coefficient-contract-v1.md', old / 'protocol.json', old / 'result.json',
]}
sources = {}
current = Path('/dev/shm/csgrs-circle-support-consumers-v1/candidate-source-v1')
hook = '\n#[cfg(test)]\n#[path = "circle_support_cost_probe.rs"]\nmod circle_support_cost_probe;\n'
for label, description in protocol['sources'].items():
    original = Path(description['path'])
    source = output / (label + '-source')
    source.mkdir()
    for relative, digest in description['sha256'].items():
        assert sha(original / relative) == digest
        target = source / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(original / relative, target)
    if label == 'candidate':
        intersection = source / 'crates/hypercurve/src/curve_intersection.rs'
        assert intersection.read_text().endswith(hook)
        assert intersection.read_text()[:-len(hook)] == (current / 'crates/hypercurve/src/curve_intersection.rs').read_text()
        for relative in description['sha256']:
            if relative not in ['crates/hypercurve/README.md', 'crates/hypercurve/src/curve_intersection.rs',
                                'crates/hypercurve/src/circle_support_cost_probe.rs']:
                assert sha(source / relative) == sha(current / relative)
    probe = source / 'crates/hypercurve/src/circle_support_cost_probe.rs'
    shutil.copy2(W / 'coefficient_probe_v1.rs', probe)
    source_pins = {str(path.relative_to(source)): sha(path) for path in source.rglob('*') if path.is_file()}
    sources[label] = {'path': str(source), 'sha256': source_pins}
dump(output / 'protocol.json', {
    'input_sha256': pins, 'sources': sources, 'read_only_dependencies_sha256': dependencies,
    'externs': protocol['externs'], 'rustc': protocol['rustc'],
    'hardware': subprocess.check_output(['lscpu'], text=True), 'features': [],
    'only_diagnostic_changed_from_frozen_cost_sources': True,
    'candidate_production_source_equals_current_isolated_candidate': True,
    'same_dependency_libraries_for_both_variants': True, 'completed_cargo_targets_reopened': False,
    'external_timeout_seconds': 1800, 'address_space_bytes': 8 * 1024**3,
    'initial_resources': resources(), 'minimum_runtime_free_bytes': 256 * 1024**2,
    'minimum_repository_free_bytes': 768 * 1024**2, 'minimum_host_available_memory_bytes': 2 * 1024**3,
    'resource_probe_interval_seconds': 5, 'internal_bounds_claimed': False,
})
rows, binaries = [], {}
for label, description in sources.items():
    source = Path(description['path'])
    binary = output / (label + '-tests')
    command = ['rustc', '--crate-name', 'hypercurve', '--edition=2024', '--test',
               str(source / 'crates/hypercurve/src/lib.rs'), '-C', 'opt-level=3', '-C', 'codegen-units=16',
               '-C', 'embed-bitcode=no', '-D', 'warnings', '--remap-path-prefix',
               str(source) + '=/csgrs-circle-support-coefficients-v1',
               '-L', 'dependency=' + str(Path(protocol['externs']['hyperreal']).parent), '-o', str(binary)]
    for name, path in protocol['externs'].items():
        command += ['--extern', name + '=' + path]
    row = run(output, label, command, source)
    rows.append(row)
    dump(output / 'commands.json', rows)
    if row['exit_code'] == 0:
        binaries[label] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
    if row['external_stop_reason']:
        break
assert all(sha(path) == digest for path, digest in (dependencies | pins).items())
assert all(sha(Path(description['path']) / path) == digest
           for description in sources.values() for path, digest in description['sha256'].items())
result = {'complete': True, 'output_closed': True, 'binaries': binaries,
          'command_failures': [row['name'] for row in rows if row['exit_code'] != 0],
          'source_and_read_only_dependencies_unchanged': True, 'cache_admitted': False}
dump(output / 'result.json', result)
raise SystemExit(0 if len(binaries) == 2 else 101)
