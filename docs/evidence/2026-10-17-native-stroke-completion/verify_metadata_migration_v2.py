"""Independently verify that artifact deltas are only the committed version migration."""
import copy, hashlib, io, json, re, zipfile
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]
O = W / 'metadata-migration-v2'
D = W / 'artifact-mismatches-v1'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def digest(data):
    return 'sha256:' + hashlib.sha256(data).hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, value):
    Path(p).write_text(json.dumps(value, indent=2) + '\n')

assert read(D / 'result.json')['complete']
producer = 'crates/hyperdrc/src/readiness.rs'
producer_bytes = (ROOT / producer).read_bytes()
assert b'pub const CHECK_IMPLEMENTATION_VERSION: u32 = 21;' in producer_bytes
assert b'check_version: CHECK_IMPLEMENTATION_VERSION,' in producer_bytes
for snapshot in ['baseline-source-v1', 'candidate-source-v4', 'candidate-source-v5']:
    assert (W / snapshot / producer).read_bytes() == producer_bytes
O.mkdir(exist_ok=False)
rows = []
for slug in ['esp32', 'esp32s3', 'nano', 'rp2040', 'stm32f103', 'uno']:
    folder = D / (slug + '_runs_end_to_end_through_hyperdrc')
    expected, actual = [(folder / (label + '.json')).read_bytes() for label in ['expected', 'actual']]
    fixture = 'crates/hypercircuit/tests/fixtures/easyduino/fast-drc/' + slug + '.json'
    assert (ROOT / fixture).read_bytes() == expected
    before, after = json.loads(expected), json.loads(actual)
    assert len(before['coverage']['checks']) == len(after['coverage']['checks']) == 160
    assert all(c['check_version'] == 20 for c in before['coverage']['checks'])
    assert all(c['check_version'] == 21 for c in after['coverage']['checks'])
    old, new = b'"check_version": 20', b'"check_version": 21'
    assert expected.count(old) == 160 and expected.replace(old, new) == actual
    rows.append({'fixture': fixture, 'expected_sha256': sha(folder / 'expected.json'),
        'actual_sha256': sha(folder / 'actual.json'), 'replacement_count': 160,
        'only_changed_bytes': 'check_version: 20 -> 21', 'independently_derived_bytes_match': True})

name = 'default_manufacturing_release_is_unsigned_deterministic_and_self_verifying'
folder = D / name
expected, actual = [(folder / (label + '.zip')).read_bytes() for label in ['expected', 'actual']]
fixture = 'crates/hypercircuit/tests/fixtures/manufacturing-release/release-board-v1.zip'
assert (ROOT / fixture).read_bytes() == expected
old_zip, new_zip = [zipfile.ZipFile(io.BytesIO(data)) for data in [expected, actual]]
assert old_zip.namelist() == new_zip.namelist() and len(old_zip.namelist()) == 22
changed = [p for p in old_zip.namelist() if old_zip.read(p) != new_zip.read(p)]
assert changed == ['evidence/hyperdrc.json', 'manufacturing-release.json']
for old_info, new_info in zip(old_zip.infolist(), new_zip.infolist()):
    for key in ['filename', 'date_time', 'compress_type', 'flag_bits', 'create_system',
                'create_version', 'extract_version', 'comment', 'extra', 'external_attr', 'internal_attr']:
        assert getattr(old_info, key) == getattr(new_info, key), (old_info.filename, key)
old_report, new_report = [z.read('evidence/hyperdrc.json') for z in [old_zip, new_zip]]
assert old_report.count(b'"check_version":1,') == 160
assert old_report.replace(b'"check_version":1,', b'"check_version":21,') == new_report
old_manifest, new_manifest = [json.loads(z.read('manufacturing-release.json')) for z in [old_zip, new_zip]]
core_source = ROOT / 'crates/hypercircuit/src/manufacturing_release.rs'
assert 'serde_json::to_vec(self).expect("release core is infallibly serializable")' in core_source.read_text()

def core_digest(core):
    # The stored struct field order is retained by json.loads; all numbers in
    # this core are integers. Compact UTF-8 JSON matches the declared encoding.
    def check(value):
        assert not isinstance(value, float)
        if isinstance(value, dict):
            for child in value.values():
                check(child)
        if isinstance(value, list):
            for child in value:
                check(child)
    check(core)
    return digest(json.dumps(core, ensure_ascii=False, separators=(',', ':')).encode())

assert core_digest(old_manifest['core']) == old_manifest['core_digest']
assert core_digest(new_manifest['core']) == new_manifest['core_digest']
derived = copy.deepcopy(old_manifest)
descriptors = derived['core']['artifacts']['artifacts']
matches = [d for d in descriptors if d['path'] == 'evidence/hyperdrc.json']
assert len(matches) == 1
descriptor = matches[0]
assert descriptor['byte_length'] == len(old_report) and descriptor['digest'] == digest(old_report)
assert derived['core']['drc_report_digest'] == digest(old_report)
descriptor['byte_length'] = len(new_report)
descriptor['digest'] = digest(new_report)
derived['core']['drc_report_digest'] = digest(new_report)
derived['core_digest'] = core_digest(derived['core'])
assert derived == new_manifest
# Rebuild the archive independently with unchanged entry metadata and the two
# independently validated replacement payloads.
rebuilt = io.BytesIO()
with zipfile.ZipFile(rebuilt, 'w') as z:
    for info in old_zip.infolist():
        z.writestr(copy.copy(info), new_zip.read(info.filename))
assert rebuilt.getvalue() == actual
(O / 'independently-rebuilt-release.zip').write_bytes(rebuilt.getvalue())
rows.append({'fixture': fixture, 'expected_sha256': sha(folder / 'expected.zip'),
    'actual_sha256': sha(folder / 'actual.zip'), 'changed_entries': changed,
    'version_substitutions': 160, 'old_version': 1, 'new_version': 21,
    'drc_report_old_bytes': len(old_report), 'drc_report_new_bytes': len(new_report),
    'independently_recomputed_drc_digest': digest(new_report),
    'independently_recomputed_core_digest': derived['core_digest'],
    'entry_metadata_unchanged': True, 'independently_rebuilt_archive_matches': True})
dump(O / 'result.json', {'complete': True, 'runner_sha256': sha(__file__),
    'contract_sha256': sha(W / 'metadata-contract-v1.md'),
    'decoded_differences_sha256': sha(D / 'result.json'),
    'producer_sha256': sha(ROOT / producer), 'core_serialization_source_sha256': sha(core_source),
    'fixtures': rows, 'geometry_findings_and_other_artifacts_unchanged': True,
    'preserved_failed_attempt': 'metadata-migration-v1 expected spaces in compact DRC JSON; no fixture was changed',
    'fixture_bytes_replaced': False, 'runtime_tests_reexecuted': False})
print(json.dumps({'complete': True, 'independently_verified_fixture_migrations': len(rows),
    'independently_rebuilt_archive_matches': True}), flush=True)
