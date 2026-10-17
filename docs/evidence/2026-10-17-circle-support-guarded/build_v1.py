"""Reproduce the rejected cost, then build a guarded isolated revision."""
import gzip
import json
import shutil
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True
from runner_v1 import ROOT, RUNTIME, W, dump, read, resources, run, sha, violation

base = 'd1e1f737447bc099c008fc7404e86db141b0611d'
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == base
subprocess.run(['git', 'diff', '--quiet'], cwd=ROOT, check=True)
subprocess.run(['git', 'diff', '--cached', '--quiet'], cwd=ROOT, check=True)
old = W.parent / 'circle-support-index/cost-build-v4'
prior = read(old / 'protocol.json')
e41 = Path('/run/user/1000/csgrs-circle-support-coefficients-v1')
previous = read(e41 / 'build-v1/result.json')
assert previous['complete'] and previous['output_closed'] and not previous['command_failures']
assert subprocess.check_output(['rustc', '-vV'], text=True) == prior['rustc']
dependencies = prior['read_only_dependencies_sha256']
assert all(sha(path) == digest for path, digest in dependencies.items())
with gzip.open('/dev/shm/csgrs-circle-support-consumers-v1/preparation-v1/main-input-hashes.json.gz', 'rt') as stream:
    old_main = json.load(stream)
main = {path: sha(ROOT / path) for path in old_main}
assert {path for path in main if main[path] != old_main[path]} == {'docs/properties-status.json'}
cache_hook = '\n#[cfg(test)]\n#[path = "circle_support_cache_tests.rs"]\nmod circle_support_cache_tests;\n'
cost_hook = '\n#[cfg(test)]\n#[path = "circle_support_cost_probe.rs"]\nmod circle_support_cost_probe;\n'
intersection = 'crates/hypercurve/src/curve_intersection.rs'
baseline_old = Path(prior['sources']['baseline']['path'])
assert (baseline_old / intersection).read_text().replace(cache_hook, '').replace(cost_hook, '') == (ROOT / intersection).read_text()
for path, digest in prior['sources']['baseline']['sha256'].items():
    assert sha(baseline_old / path) == digest
    if path not in [intersection, 'crates/hypercurve/src/circle_support_cache_tests.rs',
                    'crates/hypercurve/src/circle_support_cost_probe.rs']:
        assert sha(ROOT / path) == digest
RUNTIME.mkdir(mode=0o700, exist_ok=False)
assert violation(resources()) is None
repro = RUNTIME / 'reproduction-v1'
repro.mkdir()
dump(repro / 'protocol.json', {
    'base_commit': base, 'current_main_production_equivalent_to_frozen_baseline': True,
    'input_sha256': {str(path): sha(path) for path in [Path(__file__), W / 'contract-v1.md',
                     W / 'runner_v1.py', e41 / 'build-v1/result.json', e41 / 'build-v1/protocol.json']},
    'binaries': previous['binaries'], 'new_prototype_not_yet_modified': True,
})
reproduced = []
for label, binary in previous['binaries'].items():
    assert sha(binary['path']) == binary['sha256']
    row = run(repro, label, [binary['path'],
              'curve_intersection::circle_support_cost_probe::shared_coefficient_lookup_cost',
              '--exact', '--ignored', '--nocapture', '--test-threads=1'], repro,
              {'CIRCLE_SHARED_BITS': '1048576', 'CIRCLE_SHARED_REPEATS': '20000'})
    reproduced.append(row)
    assert row['exit_code'] == 0
    text = (repro / (label + '.stdout')).read_text()
    assert 'CIRCLE_SHARED_CERTAINTY Certified' in text and 'test result: ok. 1 passed;' in text
dump(repro / 'commands.json', reproduced)
dump(repro / 'result.json', {'complete': True, 'output_closed': True,
                           'command_failures': [], 'current_main_production_equivalent': True})

output = RUNTIME / 'build-v1'
output.mkdir()
pins = {str(path): sha(path) for path in [
    Path(__file__), W / 'runner_v1.py', W / 'contract-v1.md', W / 'transition_tests_v1.rs',
    old / 'protocol.json', old / 'result.json', repro / 'result.json',
    W.parent / 'circle-support-consumers/coefficient_probe_v1.rs',
]}
sources, commands = {}, []
for label, description in prior['sources'].items():
    original = Path(description['path'])
    source = output / (label + '-source')
    source.mkdir()
    for path, digest in description['sha256'].items():
        assert sha(original / path) == digest
        dest = source / path
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(original / path, dest)
    tests = source / 'crates/hypercurve/src/circle_support_cache_tests.rs'
    tests.write_text(tests.read_text() + (W / 'transition_tests_v1.rs').read_text())
    probe = source / 'crates/hypercurve/src/circle_support_coefficient_probe.rs'
    shutil.copy2(W.parent / 'circle-support-consumers/coefficient_probe_v1.rs', probe)
    path = source / intersection
    text = path.read_text()
    text += '\n#[cfg(test)]\n#[path = "circle_support_coefficient_probe.rs"]\nmod circle_support_coefficient_probe;\n'
    if label == 'candidate':
        old_support = '''            Some([
                circle.center.x().exact_rational_ref()?.clone(),
                circle.center.y().exact_rational_ref()?.clone(),
                circle.radius_squared.exact_rational_ref()?.clone(),
            ])'''
        new_support = '''            let values = [
                circle.center.x().exact_rational_ref()?,
                circle.center.y().exact_rational_ref()?,
                circle.radius_squared.exact_rational_ref()?,
            ];
            // This cost hint reads stored bit lengths without reducing values.
            // Very large coefficients keep the original equality path.
            if values.iter().any(|value| {
                value.storage_class() == hyperreal::RationalStorageClass::VeryLarge
            }) {
                return None;
            }
            Some(values.map(Clone::clone))'''
        assert text.count(old_support) == 1
        text = text.replace(old_support, new_support)
        key = '''        let key = ExactCircularSupportPairKey::new(first, second);
        let indexed = key
            .as_ref()
            .and_then(|key| self.exact_circular_support_index.get(key).copied());
'''
        assert text.count(key) == 1
        text = text.replace(key, '')
        anchor = '        let retained = if key.is_some() {'
        assert text.count(anchor) == 1
        text = text.replace(anchor, '''        // Small batches preserve the cheap original scan, including shared
        // Point2 identity hits. Populate the index only when inserting a miss.
        let small_batch = self.circular_support_relations.len() <= 8;
        if small_batch {
            if let Some(index) = (0..self.circular_support_relations.len()).find(|&index| matches(index)) {
                return Ok(Some(self.circular_support_relations[index].relation.clone()));
            }
        }
''' + key + '''        let retained = if small_batch {
            None
        } else if key.is_some() {''')
        text = text.replace('''            // Unequal rational keys cannot match. Earlier symbolic entries
            // still get their original equality proof and priority; scalar''', '''            // Unequal indexed keys cannot match. Earlier unindexed entries
            // still get their original equality proof and priority; scalar''')
        readme = source / 'crates/hypercurve/README.md'
        prior_readme = Path('/dev/shm/csgrs-circle-support-consumers-v1/candidate-source-v1/crates/hypercurve/README.md')
        expected = read('/dev/shm/csgrs-circle-support-consumers-v1/preparation-v1/result.json')['proposal_sha256']['crates/hypercurve/README.md']
        assert sha(prior_readme) == expected
        docs = prior_readme.read_text()
        old_docs = '''  synchronous operation under one predicate policy. Exact rational centers and
  squared radii use an ordered value key; symbolic supports retain the existing
  equality proofs and first-match order. Hits reuse the same relation and ordered
  witnesses.'''
        new_docs = '''  synchronous operation under one predicate policy. Batches of up to eight
  retained entries use ordered equality lookup. Larger batches index exact
  rational centers and squared radii when their scalar storage class is not
  `VeryLarge`; symbolic and very large supports retain ordered equality lookup.
  These cost choices do not restrict supported inputs. Earlier unindexed entries
  keep their original matching priority. Hits reuse the same relation and ordered
  witnesses.'''
        assert docs.count(old_docs) == 1
        readme.write_text(docs.replace(old_docs, new_docs))
    path.write_text(text)
    row = run(output, label + '-format', ['rustfmt', '--edition', '2024', '--config', 'skip_children=true',
              str(path), str(tests)], source)
    commands.append(row)
    assert row['exit_code'] == 0
    source_pins = {str(path.relative_to(source)): sha(path) for path in source.rglob('*') if path.is_file()}
    sources[label] = {'path': str(source), 'sha256': source_pins}
assert (Path(sources['baseline']['path']) / 'crates/hypercurve/src/circle_support_cache_tests.rs').read_bytes() == (Path(sources['candidate']['path']) / 'crates/hypercurve/src/circle_support_cache_tests.rs').read_bytes()
with gzip.open(output / 'main-input-hashes.json.gz', 'wt') as stream:
    json.dump(main, stream, sort_keys=True)
for relative in [intersection, 'crates/hypercurve/src/circle_support_cache_tests.rs', 'crates/hypercurve/README.md']:
    compared = subprocess.run(['git', 'diff', '--no-index', '--binary', '--',
                               str(Path(sources['baseline']['path']) / relative),
                               str(Path(sources['candidate']['path']) / relative)], stdout=subprocess.PIPE)
    assert compared.returncode in [0, 1]
    (output / (Path(relative).name + '.patch')).write_bytes(compared.stdout)
dump(output / 'protocol.json', {
    'base_commit': base, 'input_sha256': pins, 'sources': sources,
    'read_only_dependencies_sha256': dependencies, 'externs': prior['externs'], 'rustc': prior['rustc'],
    'hardware': subprocess.check_output(['lscpu'], text=True), 'features': [],
    'main_input_hashes_sha256': sha(output / 'main-input-hashes.json.gz'),
    'same_diagnostics_and_dependency_libraries_for_both_variants': True,
    'completed_cargo_targets_reopened': False, 'new_production_change_isolated': True,
    'external_timeout_seconds': 1800, 'address_space_bytes': 8 * 1024**3,
    'initial_resources': resources(), 'minimum_runtime_free_bytes': 256 * 1024**2,
    'minimum_repository_free_bytes': 768 * 1024**2, 'minimum_host_available_memory_bytes': 2 * 1024**3,
    'resource_probe_interval_seconds': 5, 'internal_bounds_claimed': False,
})
binaries = {}
for label, description in sources.items():
    source = Path(description['path'])
    binary = output / (label + '-tests')
    command = ['rustc', '--crate-name', 'hypercurve', '--edition=2024', '--test',
               str(source / 'crates/hypercurve/src/lib.rs'), '-C', 'opt-level=3', '-C', 'codegen-units=16',
               '-C', 'embed-bitcode=no', '-D', 'warnings', '--remap-path-prefix',
               str(source) + '=/csgrs-circle-support-guarded-v1',
               '-L', 'dependency=' + str(Path(prior['externs']['hyperreal']).parent), '-o', str(binary)]
    for name, path in prior['externs'].items():
        command += ['--extern', name + '=' + path]
    row = run(output, label, command, source)
    commands.append(row)
    dump(output / 'commands.json', commands)
    if row['exit_code'] == 0:
        binaries[label] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
    if row['external_stop_reason']:
        break
assert all(sha(path) == digest for path, digest in (dependencies | pins).items())
assert all(sha(ROOT / path) == digest for path, digest in main.items())
assert all(sha(Path(description['path']) / path) == digest
           for description in sources.values() for path, digest in description['sha256'].items())
dump(output / 'result.json', {'complete': True, 'output_closed': True, 'binaries': binaries,
     'command_failures': [row['name'] for row in commands if row['exit_code'] != 0],
     'sources_and_dependencies_unchanged': True, 'main_sources_unchanged': True, 'cache_admitted': False})
raise SystemExit(0 if len(binaries) == 2 else 101)
