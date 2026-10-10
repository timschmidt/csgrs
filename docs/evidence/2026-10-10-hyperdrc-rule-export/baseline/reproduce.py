"""Preserve admitted rule-export failures and test real KiCad rule consumption."""
from fractions import Fraction
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import resource
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
PREVIOUS = EVIDENCE.parent / '2026-10-10-hyperreal-formatting-frontier'
OUT = EVIDENCE / 'baseline'
WORK = ROOT / 'target/hyperdrc-rule-export-baseline'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
info = lambda path: {'bytes': path.stat().st_size, 'sha256': digest(path)}
record = read(PREVIOUS / 'checked-scientific/sources.json')
main_record = read(PREVIOUS / 'integrated-main/sources.json')
maps = []
for source in [record, main_record]:
    path = Path(source['source_hashes_file'])
    assert digest(path) == source['source_hashes_file_sha256']
    maps.append((Path(source['source_directory']), json.loads(gzip.decompress(path.read_bytes()))))
validation = read(PREVIOUS / 'downstream-checked-scientific.json')
libraries = [(Path(name), value) for name, value in validation['libraries'].items() if name.endswith('/libhyperdrc.rlib')]
assert len(libraries) == 1
library, library_info = libraries[0]
assert info(library) == library_info
OUT.mkdir()
WORK.mkdir()
shutil.copyfile(__file__, OUT / 'reproduce.py')
shutil.copyfile(EVIDENCE / 'export_probe.rs', OUT / 'probe.rs')
shutil.copyfile(EVIDENCE / 'consumer-board.kicad_pcb', OUT / 'consumer-board.kicad_pcb')
linked = WORK / 'libhyperdrc_admitted.rlib'
shutil.copyfile(library, linked)
(OUT / 'library.json').write_text(json.dumps({'path': str(library), 'linked_path': str(linked), **library_info}, indent=2) + '\n')
probe_hash = digest(OUT / 'probe.rs')
commands, exports, consumers = [], [], []


def verify():
    assert all(digest(root / name) == expected for root, values in maps for name, expected in values.items())
    assert info(linked) == library_info
    assert digest(OUT / 'probe.rs') == probe_hash


def limit_probe():
    resource.setrlimit(resource.RLIMIT_AS, (512 * 1024 * 1024, 512 * 1024 * 1024))


def run(name, command, seconds=60, probe=False, cwd=ROOT, env=None):
    verify()
    print('start', name, flush=True)
    start = time.monotonic()
    with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
        result = subprocess.run(['timeout', '--signal=TERM', '--kill-after=2s', str(seconds), *command], cwd=cwd, env=env, stdout=stdout, stderr=stderr, preexec_fn=limit_probe if probe else None, check=False)
    row = {'name': name, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start, 'timeout_seconds': seconds, 'probe_address_space_cap_mib': 512 if probe else None}
    commands.append(row)
    (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
    verify()
    print('finish', name, result.returncode, flush=True)
    return row


assert run('format', ['rustfmt', '--edition', '2024', '--check', str(OUT / 'probe.rs')])['exit_code'] == 0
binary = WORK / 'probe'
assert run('build', ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(OUT / 'probe.rs'), '-L', 'dependency=' + str(Path(record['target_directory']) / 'release/deps'), '--extern', 'hyperdrc=' + str(linked), '-o', str(binary)])['exit_code'] == 0
(OUT / 'binary.json').write_text(json.dumps({'path': str(binary), **info(binary)}, indent=2) + '\n')
families = ['default', '0.15', '0.1234567', '1e-7', '1e-40', '1e-400', '1e20', '1e400', '0', '-0.1', '2147.483647', '2147.483648', 'pi', 'unresolved_zero', 'unresolved_positive', 'unresolved_tiny', 'aborted_pi']
for index, family in enumerate(families):
    for preparation in ['fresh', 'prepared']:
        name = f'export-{index:02}-{preparation}'
        destination = OUT / (name + '.kicad_dru')
        result = run(name, [str(binary), family, preparation, 'render', str(destination)], seconds=3, probe=True)
        output = destination.read_text() if destination.exists() else None
        token = re.search(r'\(constraint track_width \(min ([^\s)]+)mm\)\)', output) if output is not None else None
        value = token.group(1) if token else None
        original = None
        try:
            original = Fraction(family)
        except ValueError:
            pass
        exports.append({'family': family, 'preparation': preparation, 'exit_code': result['exit_code'], 'timed_out': result['exit_code'] == 124, 'output': str(destination.relative_to(EVIDENCE)) if output is not None else None, 'track_width_token': value, 'exact_token_value': str(Fraction(value)) if value is not None else None, 'exact_source_value': str(original) if original is not None else None, 'last_stage': (OUT / (name + '.stderr')).read_text().splitlines()[-1:]})
        (OUT / 'exports.json').write_text(json.dumps(exports, indent=2) + '\n')
merge_inputs = {
    'empty': '',
    'same-line-custom': '(version 1)\n(rule "hyperdrc-track-width" (constraint track_width (min 0.01mm))) (rule "KEEP-SAME-LINE" (constraint clearance (min 0.2mm)))\n',
    'comment-parenthesis': '(version 1)\n(rule "hyperdrc-track-width"\n # An unmatched comment parenthesis (\n (constraint track_width (min 0.01mm)))\n(rule "KEEP-AFTER-COMMENT" (constraint clearance (min 0.2mm)))\n',
    'unterminated-owned-block': '(version 1)\n; BEGIN HYPERDRC GENERATED RULES\n(rule "KEEP-AFTER-UNMATCHED-MARKER" (constraint clearance (min 0.2mm)))\n',
}
merges = []
for name, text in merge_inputs.items():
    original = OUT / ('merge-input-' + name + '.kicad_dru')
    original.write_text(text)
    destination = OUT / ('merge-' + name + '.kicad_dru')
    result = run('merge-' + name, [str(binary), 'default', 'fresh', str(original), str(destination)], seconds=3, probe=True)
    output = destination.read_text() if destination.exists() else ''
    markers = re.findall(r'KEEP-[A-Z-]+', text)
    merges.append({'case': name, 'exit_code': result['exit_code'], 'original': str(original.relative_to(EVIDENCE)), 'output': str(destination.relative_to(EVIDENCE)), 'version_headers': output.count('(version 1)'), 'custom_markers_preserved': {marker: marker in output for marker in markers}})
(OUT / 'merges.json').write_text(json.dumps(merges, indent=2) + '\n')
assert run('kicad-version', ['kicad-cli', '--version'])['exit_code'] == 0
env = os.environ | {'KICAD_CONFIG_HOME': str(WORK / 'config'), 'LC_ALL': 'C.UTF-8'}
default = (OUT / 'export-00-fresh.kicad_dru').read_text()
correct_comments = lambda text: re.sub(r'(?m)^(\s*);', r'\1#', text)
decks = {
    'generated-default': default,
    'comments-only-corrected': correct_comments(default),
    'manual-control': '(version 1)\n# independent consumer control\n(rule "manual-width-control" (condition "A.Type == \'track\'") (constraint track_width (min 0.15mm)))\n',
    'scientific-control': '(version 1)\n(rule "scientific-control" (condition "A.Type == \'track\'") (constraint track_width (min 1.5e-1mm)))\n',
    'empty-merge-comments-corrected': correct_comments((OUT / 'merge-empty.kicad_dru').read_text()),
}
for name, deck in decks.items():
    directory = OUT / ('consumer-' + name)
    directory.mkdir()
    board = directory / 'probe.kicad_pcb'
    shutil.copyfile(EVIDENCE / 'consumer-board.kicad_pcb', board)
    (directory / 'probe.kicad_pro').write_text('{}\n')
    (directory / 'probe.kicad_dru').write_text(deck)
    output = directory / 'report.json'
    result = run('consumer-' + name, ['kicad-cli', 'pcb', 'drc', '--format', 'json', '--output', str(output), str(board)], seconds=30, env=env)
    consumers.append({'case': name, 'exit_code': result['exit_code'], 'directory': str(directory.relative_to(EVIDENCE)), 'report_exists': output.exists(), 'report': read(output) if output.exists() else None})
    (OUT / 'consumers.json').write_text(json.dumps(consumers, indent=2) + '\n')
verify()
summary = {'scope': 'Unchanged admitted exporter baseline and real KiCad CLI consumers. External process cutoffs are observations, not typed library limits. The comments-only controls modify fixture copies, not production source.', 'source_record': str(PREVIOUS / 'checked-scientific/sources.json'), 'source_record_sha256': digest(PREVIOUS / 'checked-scientific/sources.json'), 'main_source_record': str(PREVIOUS / 'integrated-main/sources.json'), 'main_source_record_sha256': digest(PREVIOUS / 'integrated-main/sources.json'), 'all_source_library_and_probe_hashes_unchanged': True, 'export_cases': len(exports), 'merge_cases': len(merges), 'consumer_cases': len(consumers), 'exports': exports, 'merges': merges, 'consumer_outcomes': [{key: value for key, value in row.items() if key != 'report'} for row in consumers], 'kicad_version': (OUT / 'kicad-version.stdout').read_text().strip(), 'commands': commands}
summary['artifacts'] = {str(path.relative_to(EVIDENCE)): info(path) for path in sorted(OUT.rglob('*')) if path.is_file()}
summary['scripts'] = {name: info(EVIDENCE / name) for name in ['export_probe.rs', 'consumer-board.kicad_pcb', 'reproduce_baseline.py']}
(EVIDENCE / 'baseline.json').write_text(json.dumps(summary, indent=2) + '\n')
print('Rule export baseline complete:', len(exports), 'export cases,', len(merges), 'merge cases,', len(consumers), 'real KiCad consumers.', flush=True)
