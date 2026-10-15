"""Preserve the failed build and prepare the monotone-publication integration."""
import importlib.util
import json
from pathlib import Path
import re
import shutil
import sys

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location('prepare_v1', E / 'prepare_v1.py')
p = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(p)
SOURCE = p.ROOT / 'target/scalar-main-integration-v3/source'
DEST = p.ROOT / 'target/scalar-main-integration-v4/source'


def main():
    before = p.read_map(E / 'freeze-v3/source-hashes.json.gz')
    assert p.inventory(SOURCE) == before and not DEST.exists()
    result = json.loads((E / 'qualification-v3/result.json').read_text())
    assert result['commands_executed'] == 4 and result['last_exit_code'] == 101 and not result['complete']
    log = (E / 'qualification-v3/default-tests.stderr').read_text()
    assert 'no method named `replace_exact_sign`' in log
    p.dump(E / 'closure-v3.json', {
        'terminal_exit_code_observed': 101, 'complete': False,
        'passed': ['workspace formatting', 'included-source formatting', '51 acceptance controls'],
        'failed': 'Default test compilation: the current-main tower reconstruction still called the old replacement setter, which is test-only in the candidate.',
        'numerical_tests_executed': False,
        'artifacts_sha256': p.inventory(E / 'qualification-v3'),
        'source_record_sha256': p.sha(E / 'freeze-v3/source-hashes.json.gz'),
        'runtime_admitted': False,
    })
    shutil.copytree(SOURCE, DEST)
    tower = DEST / 'crates/hyperreal/src/computable/node/quadratic_tower.rs'
    text = tower.read_text()
    old = '.replace_exact_sign(ExactSignCache::Valid(sign));'
    assert text.count(old) == 1
    tower.write_text(text.replace(old, '.publish_exact_sign(ExactSignCache::Valid(sign));'))
    # Register the same thread-exit cleanup already qualified on main. Keep
    # native MPFR values alive for the complete test and release TLS caches
    # only at thread exit, after local values have been dropped.
    cleanup = []
    changed = json.loads((E / 'freeze-v3/result.json').read_text())['changes_from_main_sha256']
    for name in sorted(changed):
        if not name.startswith('crates/hyperreal/src/') or not name.endswith('.rs'):
            continue
        path = DEST / name
        text = path.read_text()
        if 'rug::' not in text:
            continue
        functions = list(re.finditer(r'(?m)^([ ]*)fn ([a-zA-Z0-9_]+)\(', text))
        for index in range(len(functions) - 1, -1, -1):
            match = functions[index]
            end = functions[index + 1].start() if index + 1 < len(functions) else len(text)
            body = text[match.start():end]
            if 'Float::' not in body or 'cleanup_on_thread_exit()' in body:
                continue
            opening = text.index('{', match.end(), end) + 1
            assert text[opening] == '\n'
            indent = match.group(1) + '    '
            text = text[:opening] + '\n' + indent + 'crate::mpfr_test::cleanup_on_thread_exit();' + text[opening:]
            cleanup.append({'path': name, 'function': match.group(2)})
        path.write_text(text)
    after = p.inventory(DEST)
    assert p.inventory(SOURCE) == before
    p.dump(E / 'resolution-v4.json', {
        'source_directory': str(DEST), 'source_parent': 'freeze-v3/source-hashes.json.gz',
        'source_parent_sha256': p.sha(E / 'freeze-v3/source-hashes.json.gz'),
        'runner_sha256': p.sha(Path(__file__)),
        'changes_from_failed_source_sha256': {name: digest for name, digest in after.items() if before.get(name) != digest},
        'mpfr_thread_cleanup_registrations': cleanup,
        'scope': 'Use monotone sign publication in the integrated tower-source path. Extend existing test-only MPFR cleanup to newly introduced oracle functions. No oracle inputs or assertions change.',
        'runtime_admitted': False,
    })
    print(json.dumps({'changed_from_failed_source': sum(before.get(n) != h for n, h in after.items()),
                      'mpfr_cleanup_registrations': len(cleanup)}), flush=True)


if __name__ == '__main__':
    main()
