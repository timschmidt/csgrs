"""Compare complete native reports from the already frozen CLI executables."""
import gzip
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import resource
import signal
import subprocess
import sys
import time
import zipfile

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parents[1]
ROOT = E.parents[2]
WORK = Path('/dev/shm/csgrs-native-metadata-isolation/report-replay-v1')
OUT = E / 'report-replay-v1'
BASE = E.parent / '2026-10-15-factored-product-proofs/validate_v8.py'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def compressed(path, value):
    with path.open('xb') as raw:
        with gzip.GzipFile(fileobj=raw, mode='wb', mtime=0, filename='') as stream:
            stream.write(value)


assert sha(BASE) == 'f3ee76657ed7c1e8792452ded7c2380c1ad03a41abb312d80f9adf68b4dc6f05'
spec = importlib.util.spec_from_file_location('main_guard', BASE)
guard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)
preparation = json.loads((E / 'preparation-v1.json').read_text())
assert sha(E / 'source-hashes-v1.json.gz') == preparation['source_hashes_sha256']
with gzip.open(E / 'source-hashes-v1.json.gz', 'rt') as stream:
    maps = json.load(stream)
runner_hash = sha(Path(__file__))
binaries = {}
for variant in ['baseline', 'candidate']:
    phase = variant + '-v1'
    result = json.loads((E / phase / 'result.json').read_text())
    commands = json.loads((E / (phase + '.json')).read_text())
    assert [r['name'] for r in result['commands']] == [r[0] for r in commands]
    assert all(r['sources_unchanged'] for r in result['commands'])
    assert next(r for r in result['commands'] if r['name'] == 'native-cli')['exit_code'] == 0
    events = [json.loads(line) for line in (E / phase / 'native-cli.stdout').read_text().splitlines()]
    artifacts = [event for event in events if event.get('reason') == 'compiler-artifact'
                 and event.get('executable') and event['target']['name'] == 'hyperdrc'
                 and 'bin' in event['target']['kind']]
    assert len(artifacts) == 1
    artifact = artifacts[0]
    binary = Path(artifact['executable'])
    target = ROOT / json.loads((E / (phase + '-target.json')).read_text())
    source = Path(preparation['sources'][variant])
    assert binary.is_relative_to(target)
    assert Path(artifact['manifest_path']) == source / 'crates/hyperdrc/Cargo.toml'
    binaries[variant] = {'path': str(binary), 'sha256': sha(binary),
                         'bytes': binary.stat().st_size, 'features': artifact['features'],
                         'manifest_path': artifact['manifest_path'],
                         'validation_sha256': sha(E / phase / 'result.json'),
                         'build_stdout_sha256': sha(E / phase / 'native-cli.stdout')}
assert binaries['baseline']['features'] == binaries['candidate']['features']
WORK.mkdir(exist_ok=False)
OUT.mkdir(exist_ok=False)
(WORK / 'tmp').mkdir()
fixtures = [
    {'label': 'cparti-fpga', 'archive': 'CPArti FPGA dev board.zip',
     'entry': 'CPArti FPGA dev board.kicad_pcb', 'expected_findings': 12,
     'order': ['baseline', 'candidate']},
    {'label': 'hvp109a', 'archive': 'HVP109A.zip', 'entry': 'HVP109A.kicad_pcb',
     'expected_findings': 197, 'order': ['candidate', 'baseline']},
]
inputs = {}
for fixture in fixtures:
    relative = 'crates/hyperdrc/docs/' + fixture['archive']
    archive = Path(preparation['sources']['baseline']) / relative
    assert sha(archive) == maps['original'][relative]
    destination = WORK / 'inputs' / fixture['label']
    destination.mkdir(parents=True)
    with zipfile.ZipFile(archive) as zipped:
        assert zipped.testzip() is None
        for entry in zipped.infolist():
            assert (destination / entry.filename).resolve().is_relative_to(destination.resolve())
        zipped.extractall(destination)
    board = destination / fixture['entry']
    assert board.is_file()
    fixture.update({'board': str(board), 'board_sha256': sha(board),
                    'archive_sha256': sha(archive)})
    inputs.update({str(p): sha(p) for p in destination.rglob('*') if p.is_file()})
compressed(OUT / 'input-hashes.json.gz', (json.dumps(inputs, sort_keys=True) + '\n').encode())
environment = {'TMPDIR': str(WORK / 'tmp'), 'CARGO_TERM_COLOR': 'never'}
protocol = {'scope': __doc__, 'runner_sha256': runner_hash,
            'guard_sha256': sha(BASE), 'preparation_sha256': sha(E / 'preparation-v1.json'),
            'source_hashes_sha256': sha(E / 'source-hashes-v1.json.gz'),
            'binaries': binaries, 'fixtures': fixtures, 'environment': environment,
            'input_hashes_sha256': sha(OUT / 'input-hashes.json.gz'),
            'expected_cli_exit': 1, 'exit_contract': 'A complete report with active findings exits 1; validate its JSON separately.',
            'normalization': None, 'wall_limit_seconds': 1800,
            'address_space_limit_bytes': 8 * 1024**3,
            'limitations': 'Four fresh CLI replays establish complete report comparison only. No application speedup, independent geometry proof, internal resource bound or release claim.',
            'runtime_admitted': False}
dump(OUT / 'protocol.json', protocol)


def verify():
    assert guard.sources() == maps['main_guard']
    assert sha(Path(__file__)) == runner_hash
    for variant, source in preparation['sources'].items():
        assert all(sha(Path(source) / name) == digest for name, digest in maps[variant].items())
    assert all(sha(path) == digest for path, digest in inputs.items())
    assert all(sha(row['path']) == row['sha256'] for row in binaries.values())


commands, comparisons = [], []
for fixture in fixtures:
    reports = {}
    for variant in fixture['order']:
        verify()
        label = fixture['label'] + '-' + variant
        raw = WORK / (label + '.json')
        command = [binaries[variant]['path'], '--kicad-pcb', fixture['board'],
                   '--check', 'layer-sanity', '--check', 'board-outline-sanity',
                   '--check', 'min-copper-neck', '--check', 'drill-spacing',
                   '--min-width', '0.0762', '--format', 'json']
        print(json.dumps({'started': label}), flush=True)
        started = time.monotonic()
        with raw.open('wb') as stdout, (OUT / (label + '.stderr')).open('wb') as stderr:
            child = subprocess.Popen(['/usr/bin/time', '-v', '-o', str(OUT / (label + '.resources')), *command],
                                     cwd=ROOT, env=os.environ | environment, stdout=stdout, stderr=stderr,
                                     start_new_session=True,
                                     preexec_fn=lambda: resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3)))
            try:
                code = child.wait(timeout=1800)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                try:
                    child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
                code = 124
        duration = time.monotonic() - started
        verify()
        data = raw.read_bytes()
        compressed(OUT / (label + '.json.gz'), data)
        row = {'name': label, 'command': command, 'exit_code': code, 'seconds': duration,
               'stdout_path': str(raw), 'stdout_sha256': sha(raw), 'stdout_bytes': len(data),
               'stderr_sha256': sha(OUT / (label + '.stderr')),
               'resources_sha256': sha(OUT / (label + '.resources')), 'sources_unchanged': True}
        try:
            report = json.loads(data)
            row['complete_expected_report'] = (code == 1 and bool(report['inputs'])
                and report['violation_count'] == fixture['expected_findings']
                and len(report['violations']) == fixture['expected_findings'])
        except (ValueError, KeyError, TypeError) as error:
            report = None
            row.update({'complete_expected_report': False, 'report_error': str(error)})
        commands.append(row)
        dump(OUT / 'commands.json', commands)
        reports[variant] = (data, report, row['complete_expected_report'])
        print(json.dumps(row), flush=True)
    left, right = reports['baseline'], reports['candidate']
    comparisons.append({'fixture': fixture['label'], 'both_complete': left[2] and right[2],
                        'complete_bytes_identical': left[0] == right[0],
                        'complete_json_values_identical': left[1] == right[1],
                        'baseline_bytes': len(left[0]), 'candidate_bytes': len(right[0]),
                        'baseline_sha256': hashlib.sha256(left[0]).hexdigest(),
                        'candidate_sha256': hashlib.sha256(right[0]).hexdigest()})
    dump(OUT / 'result.json', {'complete': False, 'comparisons': comparisons})
verify()
result = {'complete': True, 'comparisons': comparisons,
          'all_reports_identical': all(r['both_complete'] and r['complete_bytes_identical']
                                      and r['complete_json_values_identical'] for r in comparisons),
          'source_and_input_identity_verified': True, 'runtime_admitted': False,
          'application_speedup_claimed': False}
dump(OUT / 'result.json', result)
print(json.dumps(result), flush=True)
if not result['all_reports_identical']:
    raise SystemExit(101)
