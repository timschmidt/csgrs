"""Append the feature-gating correction; never rewrite closed validation records."""
import copy
import gzip
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import sys

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('validation', E / 'validate_v1.py')
v = importlib.util.module_from_spec(spec)
spec.loader.exec_module(v)
ROOT = v.ROOT


def main():
    output = E / 'qualification-correction-v1.json'
    assert not output.exists()
    original_path = E / 'closure-v1.json'
    original = json.loads(original_path.read_text())
    original_digest = v.sha(original_path)
    assert v.sha(E / 'close_v1.py') == original['closure_runner_sha256']
    phase = E / 'downstream-v1'
    record = json.loads((phase / 'result.json').read_text())
    assert [row['exit_code'] for row in record['commands']] == [0, 0]
    binaries = dict(original['preserved_executables_sha256'])
    for row in record['commands']:
        assert row['sources_unchanged']
        for stream in ['stdout', 'stderr']:
            assert v.sha(phase / (row['name'] + '.' + stream)) == row[stream + '_sha256']
        assert not re.search(r'^warning(?:\[|:)', (phase / (row['name'] + '.stderr')).read_text(), re.M)
        binaries.update(row['binaries_sha256'])
    with gzip.open(phase / 'source-hashes.json.gz', 'rt') as f:
        assert v.sources() == json.load(f)
    assert all(v.sha(ROOT / path) == digest for path, digest in binaries.items())
    assert all(v.sha(ROOT / path) == digest for path, digest in original['pending_native_sha256'].items())
    text = (phase / 'downstream-release-model.stdout').read_text()
    match = re.search(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;', text)
    assert match and int(match[1]) == 1 and int(match[2]) == 0
    assert 'external_obj_vrml_and_gltf_package_models_resolve_with_digest_and_scene_identity ... ok' in text
    totals = dict(zip(['passed', 'failed', 'ignored', 'measured', 'filtered'], map(int, match.groups())))
    entry = copy.deepcopy(original['qualification'])
    entry['test_totals']['downstream-release-model'] = totals
    entry['evidence'] = str(output.relative_to(ROOT))
    entry['qualification_correction'] = {
        'finding': 'The original downstream test command returned success with zero tests because the target requires both drc and interchange. The initial commentary credited it too early. Those two original downstream invocations establish no runtime coverage.',
        'corrected_commands': str((E / 'downstream-v1.json').relative_to(ROOT)),
        'result': 'The selected actual package-model test passed with both required features enabled; strict Clippy also passed for that target and feature combination.',
        'retained_original': str(original_path.relative_to(ROOT)),
        'retained_original_sha256': original_digest,
    }
    ledger_path = ROOT / 'docs/properties-status.json'
    raw = ledger_path.read_text()
    ledger_before = v.sha(ledger_path)
    ledger = json.loads(raw)
    baseline = json.loads(subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT))
    outer = lambda data: {k: value for k, value in data.items() if k != 'current_work'}
    assert outer(ledger) == outer(baseline)
    assert ledger['current_work']['primitive_failure_visibility'] == original['qualification']
    ledger['current_work']['primitive_failure_visibility'] = entry
    match = re.search(r'(?m)^([ \t]*)"current_work": *', raw)
    start = match.end()
    _, end = json.JSONDecoder().raw_decode(raw[start:])
    replacement = json.dumps(ledger['current_work'], indent=2).replace('\n', '\n' + match.group(1))
    updated = raw[:start] + replacement + raw[start + end:]
    assert json.loads(updated) == ledger and outer(json.loads(updated)) == outer(baseline)
    ledger_path.write_text(updated)
    v.dump(output, {
        'qualification': entry, 'correction_runner_sha256': v.sha(Path(__file__)),
        'prior_closure_sha256': original_digest, 'downstream_record': record,
        'preserved_executables_sha256': binaries,
        'ledger_before_sha256': ledger_before, 'ledger_after_sha256': v.sha(ledger_path),
        'only_current_work_changed_in_ledger': True, 'full_plan_complete': False,
    })
    assert v.sha(original_path) == original_digest
    print(json.dumps({'qualification_corrected': True, 'downstream_tests': totals, 'full_plan_complete': False}), flush=True)


if __name__ == '__main__':
    main()
