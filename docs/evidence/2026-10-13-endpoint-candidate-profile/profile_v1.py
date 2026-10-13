"""Profile the frozen endpoint candidate on the original complete board fixtures.

This separate build enables frame pointers and line tables. Sampled userspace
cycle periods diagnose this build; neither its runtime nor its percentages are
matched release-speedup evidence. The allocation-instrumented copies are not used.
All commands run sequentially. Raw logs are preserved losslessly as gzip files.
"""
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PARENT = ROOT / 'docs/evidence/2026-10-12-endpoint-coordinate-reuse'
ATTRIBUTION = ROOT / 'docs/evidence/2026-10-12-endpoint-cost-attribution/interpretation-v1.json'
SOURCE = ROOT / 'target/endpoint-coordinate-reuse/source'
WORK = ROOT / 'target/endpoint-candidate-profile-v1'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    out = E / 'profile-v1'
    out.mkdir(exist_ok=False)
    WORK.mkdir(exist_ok=False)
    raw = WORK / 'raw'
    raw.mkdir()
    original = json.loads(gzip.decompress((PARENT / 'baseline-source-hashes.json.gz').read_bytes()))
    delta = json.loads((PARENT / 'preparation.json').read_text())['source_changes_sha256']
    current = original | delta
    guard = json.loads((PARENT / 'preparation.json').read_text())['main_source_sha256']
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never',
           'CARGO_TARGET_DIR': str(WORK / 'target'), 'RUSTFLAGS': '-Cforce-frame-pointers=yes',
           'CARGO_PROFILE_RELEASE_DEBUG': '1', 'DEBUGINFOD_URLS': ''}
    protocol = {
        'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
        'previous_goal_turn': 'progress',
        'parent_decision_sha256': sha(PARENT / 'decision-v1.json'),
        'allocation_attribution_sha256': sha(ATTRIBUTION),
        'command_timeout_seconds': 1800,
        'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'parent_preparation_sha256': sha(PARENT / 'preparation.json'),
        'parent_validation_sha256': sha(PARENT / 'validation-v1/protocol.json'),
        'source': str(SOURCE), 'source_count': len(current), 'environment': env,
        'inherited_rustflags': os.environ.get('RUSTFLAGS'),
        'compiler': subprocess.check_output(['rustc', '-vV'], text=True),
        'perf_version': subprocess.check_output(['perf', 'version'], text=True),
        'perf_event_paranoid': Path('/proc/sys/kernel/perf_event_paranoid').read_text().strip(),
        'sampling': '49 Hz cycles:u, frame-pointer call chains; no inline expansion during decoding.',
        'original_worker_limit_seconds': 120, 'runtime_admitted': False,
    }
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    (out / 'source-hashes.json.gz').write_bytes(gzip.compress(json.dumps(current, sort_keys=True).encode(), mtime=0))
    commands = []
    binary = None
    binary_sha = None

    def verify():
        assert sources() == guard
        assert sha(Path(__file__)) == protocol['runner_sha256']
        assert sha(ATTRIBUTION) == protocol['allocation_attribution_sha256']
        assert all(sha(SOURCE / name) == expected for name, expected in current.items())
        if binary is not None:
            assert sha(binary) == binary_sha

    def run(name, command):
        verify()
        print(json.dumps({"start": name}), flush=True)
        command = ["timeout", "--signal=TERM", "--kill-after=10s", "1800", *command]
        start = time.monotonic()
        with (raw / (name + '.stdout')).open('wb') as stdout, (raw / (name + '.stderr')).open('wb') as stderr:
            code = subprocess.run(command, cwd=SOURCE, env=os.environ | env,
                                  stdout=stdout, stderr=stderr).returncode
        verify()
        record = {'name': name, 'command': command, 'exit_code': code,
                  'seconds': time.monotonic() - start, 'sources_unchanged': True}
        data = {}
        for stream in ['stdout', 'stderr']:
            data[stream] = (raw / (name + '.' + stream)).read_bytes()
            compressed = out / (name + '.' + stream + '.gz')
            compressed.write_bytes(gzip.compress(data[stream], mtime=0))
            record[stream + '_sha256'] = hashlib.sha256(data[stream]).hexdigest()
            record[stream + '_bytes'] = len(data[stream])
            record[stream + '_gzip_sha256'] = sha(compressed)
        commands.append(record)
        (out / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
        print(json.dumps({'name': name, 'exit_code': code, 'seconds': record['seconds']}), flush=True)
        return code, data

    code, logs = run('build', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc',
                             '--all-features', '--lib', '--no-run', '--message-format=json'])
    assert code == 0
    events = [json.loads(line) for line in logs['stdout'].splitlines() if line.startswith(b'{')]
    artifacts = [row for row in events if row.get('reason') == 'compiler-artifact'
                 and row.get('executable') and row['target']['name'] == 'hyperdrc']
    assert len(artifacts) == 1
    artifact = artifacts[0]
    assert Path(artifact['manifest_path']) == SOURCE / 'crates/hyperdrc/Cargo.toml'
    binary = Path(artifact['executable'])
    assert binary.is_relative_to(WORK / 'target')
    binary_sha = sha(binary)
    (out / 'binary.json').write_text(json.dumps({
        'path': str(binary), 'sha256': binary_sha, 'bytes': binary.stat().st_size,
        'manifest_path': artifact['manifest_path'], 'features': artifact['features'],
        'package_id': artifact['package_id'],
    }, indent=2) + '\n')
    profile = out / 'perf.data'
    code, logs = run('record', ['perf', 'record', '-F', '49', '-e', 'cycles:u', '--call-graph', 'fp',
                               '--compression-level=5', '--no-buildid-cache', '-o', str(profile), '--',
                               str(binary), 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite',
                               '--exact', '--test-threads=1', '--nocapture'])
    assert code in [0, 101] and profile.is_file() and profile.stat().st_size > 0
    combined = (logs['stdout'] + logs['stderr']).decode()
    assert 'skipping complex project fixture test' not in combined
    assert combined.count('hyperdrc: finished emit report') == 2
    assert '1 passed' in combined or '1 failed' in combined
    profiled_worker = re.search(r'took ([0-9.]+)s of worker time', combined)
    record_code = code
    for name, args in [
        ('self-cost', ['report', '--stdio', '--stdio-color', 'never', '--no-inline', '--no-children',
                       '--call-graph', 'none', '--sort', 'symbol,dso', '--percent-limit', '0.05']),
        ('inclusive-cost', ['report', '--stdio', '--stdio-color', 'never', '--no-inline', '--children',
                            '--call-graph', 'flat,1,caller', '--sort', 'symbol,dso', '--percent-limit', '0.5']),
        ('header', ['report', '--header-only', '--stdio']),
        ('stacks', ['script', '--show-lost-events', '--no-inline', '-F', 'comm,pid,tid,time,period,event,ip,sym,dso']),
    ]:
        code, _ = run(name, ['perf'] + args + ['-i', str(profile)])
        assert code == 0
    verify()
    manifest = {'complete': True, 'scope': __doc__, 'binary': str(binary), 'binary_sha256': binary_sha,
                'record_exit_code': record_code,
                'profiled_worker_cpu_seconds': float(profiled_worker[1]) if profiled_worker else None,
                'source_unchanged': True, 'main_runtime_unchanged': True, 'runtime_admitted': False,
                'artifacts_sha256': {p.name: sha(p) for p in sorted(out.iterdir()) if p.is_file()}}
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps({'complete': True, 'record_exit_code': record_code, 'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
