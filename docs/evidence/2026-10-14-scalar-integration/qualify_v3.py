"""Run serial scalar integration checks on frozen inputs, stopping on failure."""
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location('prepare_v1', E / 'prepare_v1.py')
p = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(p)
WORK = p.ROOT / 'target/scalar-main-integration-v3'
SOURCE = WORK / 'source'
OUT = E / 'qualification-v3'
ENV = {
    'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
    'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never', 'RUSTUP_TOOLCHAIN': 'stable',
    'CARGO_TARGET_DIR': str(WORK / 'qualification-target'), 'RUSTFLAGS': '',
    'PYTHONDONTWRITEBYTECODE': '1', 'HYPERREAL_SKIP_BENCHMARK_REPORTS': '1',
}


def main():
    assert not OUT.exists() and not Path(ENV['CARGO_TARGET_DIR']).exists()
    frozen = json.loads((E / 'freeze-v3/result.json').read_text())
    main_sources = p.read_map(p.MAIN_RECORD)
    source = p.read_map(E / 'freeze-v3/source-hashes.json.gz')
    assert frozen['complete'] and frozen['source_record_sha256'] == p.sha(E / 'freeze-v3/source-hashes.json.gz')
    commands = [
        ('workspace-fmt', ['cargo', 'fmt', '--all', '--', '--check']),
        ('included-fmt', ['rustfmt', '--edition', '2024', '--config-path', 'crates/.rustfmt.toml',
                          '--config', 'skip_children=true', '--check', *frozen['included_format_inputs']]),
        ('acceptance-controls', ['python3', 'crates/hyperreal/verification/test_acceptance.py']),
        ('default-tests', ['cargo', 'test', '--locked', '-p', 'hyperreal', '--lib', '--tests']),
        ('all-feature-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal',
                               '--all-features', '--lib', '--tests']),
        ('all-feature-doctests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal',
                                  '--all-features', '--doc']),
        ('default-clippy', ['cargo', 'clippy', '--locked', '-p', 'hyperreal', '--all-targets', '--', '-D', 'warnings']),
        ('all-feature-consumer-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal',
            '-p', 'hyperlimit', '-p', 'hyperlattice', '-p', 'hypercurve', '-p', 'hyperdrc',
            '--all-targets', '--all-features', '--', '-D', 'warnings']),
    ]
    inputs = {name: p.sha(E / name) for name in ['prepare_v1.py', 'qualify_v3.py',
               'resolution-v3.json', 'freeze-v3/result.json', 'freeze-v3/source-hashes.json.gz', 'freeze-v3/overlay.tar.gz']}

    def guard():
        assert p.inventory(SOURCE) == source
        assert all(p.sha(p.ROOT / name) == digest for name, digest in main_sources.items())
        assert all(p.sha(E / name) == digest for name, digest in inputs.items())

    guard()
    OUT.mkdir()
    p.dump(OUT / 'protocol.json', {
        'commands': commands, 'environment': ENV, 'source_directory': str(SOURCE),
        'inputs_sha256': inputs, 'main_source_record': str(p.MAIN_RECORD.relative_to(p.ROOT)),
        'main_source_record_sha256': p.sha(p.MAIN_RECORD),
        'compiler': subprocess.check_output(['rustc', '-Vv'], text=True),
        'scope': 'Fresh integration correctness and compilation only. No numerical replay, cost, memory, size, formal requalification, native geometry runtime, original-cap or whole-plan admission is established.',
        'stop_on_first_failure': True, 'resume_retry_filter_or_pool_closed_trials': False,
    })
    rows = []
    for name, command in commands:
        guard()
        start = time.monotonic()
        print(json.dumps({'started': name}), flush=True)
        with (OUT / f'{name}.stdout').open('wb') as stdout, (OUT / f'{name}.stderr').open('wb') as stderr:
            result = subprocess.run(command, cwd=SOURCE, env=os.environ | ENV, stdout=stdout, stderr=stderr)
        guard()
        batches = [dict(zip(['passed', 'failed', 'ignored', 'measured', 'filtered'], map(int, match)))
                   for match in re.findall(r'test result: \w+\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out',
                                           (OUT / f'{name}.stdout').read_text())]
        row = {'name': name, 'exit_code': result.returncode, 'seconds': time.monotonic() - start,
               'stdout_sha256': p.sha(OUT / f'{name}.stdout'), 'stderr_sha256': p.sha(OUT / f'{name}.stderr'),
               'test_batches': batches, 'sources_unchanged': True}
        rows.append(row)
        p.dump(OUT / 'commands.json', rows)
        p.dump(OUT / 'result.json', {'complete': len(rows) == len(commands) and all(r['exit_code'] == 0 for r in rows),
                                   'commands_executed': len(rows), 'commands_expected': len(commands),
                                   'last_exit_code': result.returncode, 'protocol_sha256': p.sha(OUT / 'protocol.json'),
                                   'runtime_admitted': False})
        print(json.dumps(row), flush=True)
        if result.returncode:
            raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()
