"""Preserve the exact three-file proposal without applying it to main."""
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True
from runner_v1 import ROOT, RUNTIME, W, dump, read, sha

build = RUNTIME / 'build-v3'
protocol = read(build / 'protocol.json')
assert not read(build / 'result.json')['command_failures']
assert read(RUNTIME / 'corpus-v2/result.json')['exit_code'] == 0
assert not read(RUNTIME / 'all-feature-library-v1/result.json')['command_failures']
assert read(RUNTIME / 'cost-matched-v1/result.json')['all_exact_rows_equal_and_certified']
assert read(RUNTIME / 'heap-v1/result.json')['all_instrumented_witnesses_identical']
source = protocol['sources']['candidate']
assert all(sha(Path(source['path']) / path) == digest for path, digest in source['sha256'].items())
output = RUNTIME / 'proposal-v1'
output.mkdir(exist_ok=False)
paths = ['crates/hypercurve/src/curve_intersection.rs', 'crates/hypercurve/src/circle_support_cache_tests.rs',
         'crates/hypercurve/README.md']
proposal = {}
for relative in paths:
    original = Path(source['path']) / relative
    text = original.read_text()
    if relative.endswith('curve_intersection.rs'):
        for module in ['circle_support_cost_probe', 'circle_support_coefficient_probe']:
            hook = f'\n#[cfg(test)]\n#[path = "{module}.rs"]\nmod {module};\n'
            assert text.count(hook) == 1
            text = text.replace(hook, '')
    dest = output / 'source' / relative
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(text)
    proposal[relative] = sha(dest)
    left = str(ROOT / relative) if (ROOT / relative).is_file() else '/dev/null'
    result = subprocess.run(['git', 'diff', '--no-index', '--binary', '--', left, str(dest)], stdout=subprocess.PIPE)
    assert result.returncode == 1
    (output / (Path(relative).name + '.patch')).write_bytes(result.stdout)
with (output / 'format.stdout').open('wb') as out, (output / 'format.stderr').open('wb') as err:
    formatted = subprocess.run(['rustfmt', '--check', '--edition', '2024', '--config', 'skip_children=true',
        str(output / 'source/crates/hypercurve/src/curve_intersection.rs'),
        str(output / 'source/crates/hypercurve/src/circle_support_cache_tests.rs')], stdout=out, stderr=err)
assert formatted.returncode == 0
dump(output / 'result.json', {
    'complete': True, 'output_closed': True, 'main_parent_commit': protocol['base_commit'],
    'source': str(output / 'source'), 'proposal_sha256': proposal,
    'input_sha256': {str(path): sha(path) for path in [Path(__file__), build / 'protocol.json', build / 'result.json']},
    'only_ignored_diagnostic_module_hooks_removed_from_qualified_source': True,
    'semantic_tests_and_production_source_unchanged': True, 'source_changes_applied': False,
    'format_exit_code': formatted.returncode,
    'cache_admitted': False, 'full_plan_complete': False,
})
