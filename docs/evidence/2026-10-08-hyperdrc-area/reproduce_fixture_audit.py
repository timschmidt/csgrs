"""Audit the extra CPArti duplicate-layer finding on the full archived board."""
import hashlib, json, pathlib, subprocess, sys, zipfile

record = pathlib.Path(__file__).resolve().parent
out = pathlib.Path(sys.argv[1]).resolve()
out.mkdir(parents=True, exist_ok=False)
expected = json.loads((record / 'validation/sources.json').read_text())['source_hashes']
current = lambda: {p: hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest() for p in expected}
assert current() == expected
archive = pathlib.Path('crates/hyperdrc/docs/CPArti FPGA dev board.zip')
with zipfile.ZipFile(archive) as package:
    board = out / 'CPArti FPGA dev board.kicad_pcb'
    board.write_bytes(package.read(board.name))
library_command = ['cargo', 'build', '--locked', '--release', '-p', 'hyperdrc', '--lib', '--message-format=json']
with (out / 'library-build.jsonl').open('w') as stdout, (out / 'library-build.log').open('w') as stderr:
    subprocess.run(library_command, stdout=stdout, stderr=stderr, check=True)
artifacts = [json.loads(line) for line in (out / 'library-build.jsonl').read_text().splitlines()]
commands = []
for name, source, libraries in [('candidate', 'fixture_audit.rs', ['hyperdrc', 'clap', 'serde_json']), ('area-witness', 'fixture_area_witness.rs', ['hyperdrc', 'hyperlimit', 'serde_json'])]:
    binary = out / name
    command = ['rustc', '--edition=2024', '-O', str(record / source), '-L', 'dependency=target/release/deps']
    for library in libraries:
        rlib = next(f for a in artifacts if a.get('reason') == 'compiler-artifact' and a['target']['name'] == library for f in a['filenames'] if f.endswith('.rlib'))
        command += ['--extern', library + '=' + rlib]
    command += ['-o', str(binary)]
    with (out / (name + '-build.log')).open('w') as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    run = [str(binary), str(board)]
    with (out / (name + '.out')).open('w') as stdout, (out / (name + '.log')).open('w') as stderr:
        subprocess.run(run, stdout=stdout, stderr=stderr, check=True)
    output = (out / (name + '.out')).read_text()
    if name == 'candidate':
        start = output.rfind('\n[\n')
        assert start >= 0
        value = json.loads(output[start + 1:])
        assert len(value) == 5
    else:
        value = json.loads(output)
        assert value['native_symmetric_difference_empty'] and value['native_symmetric_difference_area_zero']
        assert not value['old_authored_area_gate'] and value['duplicate_findings'] == 1
    (out / (name + '.json')).write_text(json.dumps(value, indent=2) + '\n')
    commands.append({'name': name, 'build_command': command, 'run_command': run, 'source_sha256': hashlib.sha256((record / source).read_bytes()).hexdigest()})
manifest = {'source_hashes': expected, 'source_unchanged': current() == expected, 'archive_sha256': hashlib.sha256(archive.read_bytes()).hexdigest(), 'board_sha256': hashlib.sha256(board.read_bytes()).hexdigest(), 'library_build_command': library_command, 'commands': commands, 'scope': 'Full archived CPArti board; only the layer-sanity check is selected for diagnostic replay. The additional finding compares the parsed In1.Cu and B.Cu native sets. The independent witness asserts both native areas are exactly 10000, their symmetric difference is empty with area zero, and the old authored-area sum gate rejects the pair at its unchanged 0.000001 tolerance. This algebraic gate replay is not an execution of the entire ancestor implementation, and does not certify all upstream source parsing.'}
(out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
assert manifest['source_unchanged']
print('Full-board finding audit passed', flush=True)
