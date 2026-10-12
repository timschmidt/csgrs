"""Freeze the import-order correction and targeted negative-proof commands."""
import ast
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / 'target/scalar-evaluation-candidate/source'
OUT = E / 'dependency-integration-correction-v2489'
PREFIX = 'crates/hyperreal/verification/'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    before_record = E / 'dependency-integration-v2480/working-source-hashes.json.gz'
    before = json.loads(gzip.decompress(before_record.read_bytes()))
    assert all(sha(SOURCE / n) == h for n, h in before.items())
    assert not OUT.exists()
    guard = read(E / 'typed-api-prototype-v2483-integrated-rejections/proof-checks/tooling.json')
    assert guard['exit_code'] == 0 and guard['rejected_invocations'] == 318 and guard['baseline_successes'] == 8
    assert read(E / 'typed-api-prototype-v2485-integrated-included-fmt/result.json')['exit_code'] == 1
    texts = {}

    def edit(name, old, new):
        key = PREFIX + name
        text = texts.get(key, (SOURCE / key).read_text())
        assert text.count(old) == 1, (name, old)
        texts[key] = text.replace(old, new)

    edit('computable_bounds.rs', 'use num::bigint::Sign;\nuse crate::structural::{MagnitudeBits, RealSign};',
         'use crate::structural::{MagnitudeBits, RealSign};\nuse num::bigint::Sign;')
    # Keep each complete baseline command; change only commands for negative controls.
    name = 'test_rejections.py'
    edit(name, '            result = subprocess.run(command, capture_output=True, text=True)',
         '            module = Path(name).stem if name in MODEL_FILES else "verified::" + Path(name).stem\n'
         '            selected = command[:-1] + ["--verify-only-module", module, command[-1]]\n'
         '            result = subprocess.run(selected, capture_output=True, text=True)')
    edit(name, '        result = subprocess.run(command, capture_output=True, text=True)',
         '        selected = command[:-1] + ["--verify-only-module", "verified::word", command[-1]]\n'
         '        result = subprocess.run(selected, capture_output=True, text=True)')
    for name, module, anchor in [
        ('test_bound_rejections.py', 'computable_bounds', '        original = bound.read_text()\n'),
        ('test_cache_rejections.py', 'computable_cache', "        production = root / 'src/computable/node/representation.rs'\n"),
        ('test_bit_metadata_rejections.py', 'bit_length_model', '        for name, changed in changes:\n'),
        ('test_digit_scan_rejections.py', 'digit_scan_model', '        for name, changed in changes:\n'),
        ('test_word_scan_rejections.py', 'word_scan_model', '        for name, changed in changes:\n'),
    ]:
        edit(name, anchor, "        # The complete baseline above verifies every unchanged supporting module.\n"
             + "        command = command[:-1] + ['--verify-only-module', '" + module + "', command[-1]]\n" + anchor)
    doc = PREFIX + 'README.md'
    texts[doc] = (SOURCE / doc).read_text() + '''

Each mutation suite verifies its complete unmodified baseline before testing
faults. Negative controls select the module whose source they change, retaining
the same mutation templates, supporting inputs, `--no-cheating` flag and proof
failure criteria. This avoids repeatedly verifying unchanged modules. The full
scalar proof remains a separate required check; module selection does not
establish whole-crate completion or caller invariants.
'''
    # No mutation, failure criterion, temporary-source recipe or baseline is edited.
    mutation_ast = {}
    for name, changed in texts.items():
        if not name.endswith('.py'):
            continue
        def declarations(text):
            tree = ast.parse(text)
            return [ast.dump(node, include_attributes=False) for node in tree.body
                    if isinstance(node, (ast.Assign, ast.AugAssign))]
        assert declarations(changed) == declarations((SOURCE / name).read_text())
        mutation_ast[name] = hashlib.sha256(json.dumps(declarations(changed), sort_keys=True).encode()).hexdigest()
    OUT.mkdir()
    for name, changed in texts.items():
        for prefix, data in [('original', (SOURCE / name).read_bytes()), ('candidate', changed.encode())]:
            destination = OUT / prefix / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(data)
    overlay = {n: sha(OUT / 'candidate' / n) for n in texts}
    protocol = {'runner_sha256': sha(Path(__file__)), 'previous_source_record_sha256': sha(before_record),
        'overlay_files_sha256': overlay, 'prior_files_sha256': {n: before[n] for n in texts},
        'unchanged_mutation_declaration_ast_sha256': mutation_ast,
        'full_sweep_result_sha256': sha(E / 'typed-api-prototype-v2483-integrated-rejections/result.json'),
        'format_failure_result_sha256': sha(E / 'typed-api-prototype-v2485-integrated-included-fmt/result.json'),
        'scope': 'One import permutation fixes formatting. Six guard scripts select only the mutated module for negative runs; complete baselines, mutations, source construction and rejection criteria stay unchanged. Fresh full positive proof and the entire 318-control suite must pass before retention. No runtime or proof contract/model body changes.'}
    (OUT / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    for name in texts:
        shutil.copy2(OUT / 'candidate' / name, SOURCE / name)
    after = before | overlay
    assert all(sha(SOURCE / n) == h for n, h in after.items())
    record = OUT / 'working-source-hashes.json.gz'
    record.write_bytes(gzip.compress((json.dumps(after, sort_keys=True) + '\n').encode(), mtime=0))
    result = {'protocol_sha256': sha(OUT / 'protocol.json'), 'source_record_sha256': sha(record),
              'source_files': len(after), 'changed_files': len(overlay), 'runtime_admitted': False}
    (OUT / 'applied.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
