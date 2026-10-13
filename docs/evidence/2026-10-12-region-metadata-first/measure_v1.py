"""Fixed paired constructor costs on separate copies with identical probes.

Six source-metadata controls, three query phases, five alternating fresh-process
pairs per mode. Time uses 1024 inputs per case; allocation mode uses 128. Fixture
construction and output-buffer reservation are excluded. The allocator wrapper
is present in both variants; TLS counting is active only in allocation mode.
Retained bytes include returned region owners and any scalar cache changes.
This is a local instrumented operation comparison, not whole-board timing.
"""
import difflib
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
WORK = ROOT / 'target/region-metadata-first/cost-v1'
CANDIDATE = ROOT / 'target/region-metadata-first/source'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    out = E / 'cost-v1'
    out.mkdir(exist_ok=False)
    WORK.mkdir(exist_ok=False)
    preparation = json.loads((E / 'preparation.json').read_text())
    original = json.loads(gzip.decompress((E / 'baseline-source-hashes.json.gz').read_bytes()))
    candidate = original | preparation['source_changes_sha256']
    assert json.loads((E / 'validation-v1/result.json').read_text())['complete']
    assert all(sha(CANDIDATE / name) == digest for name, digest in candidate.items())
    region = 'crates/hypercurve/src/bezier_region.rs'
    tests = 'crates/hypercurve/src/bezier_region/metadata_order_tests.rs'
    maps, dirs, targets = {}, {}, {}
    for variant in ['baseline', 'candidate']:
        location = WORK / variant
        location.mkdir()
        dirs[variant], targets[variant] = location / 'source', location / 'target'
        with (out / ('copy-' + variant + '.log')).open('w') as log:
            subprocess.run(['cp', '-a', '--reflink=auto', str(CANDIDATE), str(dirs[variant])],
                           stdout=log, stderr=subprocess.STDOUT, check=True)
        if variant == 'baseline':
            path = dirs[variant] / region
            text = path.read_text()
            before = '            boundary_loop.arrangement_sources == candidate.arrangement_sources\n                && boundary_loop.fragments == candidate.fragments'
            after = '            boundary_loop.fragments == candidate.fragments\n                && boundary_loop.arrangement_sources == candidate.arrangement_sources'
            assert text.count(before) == 1
            path.write_text(text.replace(before, after))
        path = dirs[variant] / tests
        path.write_text(path.read_text() + '\n' + (E / 'cost_probe.rs').read_text())
        with (out / ('format-' + variant + '.log')).open('w') as log:
            subprocess.run(['rustfmt', '--edition', '2024', str(path)],
                           stdout=log, stderr=subprocess.STDOUT, check=True)
        current = {name: sha(dirs[variant] / name) for name in candidate}
        changed = [name for name, digest in current.items() if digest != candidate[name]]
        assert set(changed) == ({region, tests} if variant == 'baseline' else {tests})
        maps[variant] = current
        with tarfile.open(out / (variant + '-delta.tar.gz'), 'w:gz') as archive:
            for name in changed:
                archive.add(dirs[variant] / name, arcname=name)
        patch = ''.join(''.join(difflib.unified_diff(
            (CANDIDATE / name).read_text().splitlines(True),
            (dirs[variant] / name).read_text().splitlines(True), 'a/' + name, 'b/' + name)) for name in changed)
        (out / (variant + '.patch')).write_text(patch)
    assert maps['baseline'][tests] == maps['candidate'][tests]
    assert {name for name in candidate if maps['baseline'][name] != maps['candidate'][name]} == {region}
    with (out / 'source-hashes.json.gz').open('xb') as raw:
        with gzip.GzipFile(fileobj=raw, mode='wb', mtime=0, filename='') as stream:
            stream.write((json.dumps(maps, indent=2) + '\n').encode())
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never'}
    schedule = [{'mode': mode, 'iterations': count, 'pair': pair,
                 'order': ['baseline', 'candidate'] if pair % 2 == 0 else ['candidate', 'baseline']}
                for mode, count in [('time', 1024), ('memory', 128)] for pair in range(5)]
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
                'probe_sha256': sha(E / 'cost_probe.rs'), 'preparation_sha256': sha(E / 'preparation.json'),
                'source_hashes_sha256': sha(out / 'source-hashes.json.gz'),
                'schedule': schedule, 'environment': env,
                'sources': {name: str(path) for name, path in dirs.items()},
                'targets': {name: str(path) for name, path in targets.items()},
                'compiler': subprocess.check_output(['rustc', '-vV'], text=True),
                'inherited_rustflags': os.environ.get('RUSTFLAGS'), 'runtime_admitted': False}
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    commands = []

    def verify():
        assert sources() == preparation['main_source_sha256']
        assert sha(Path(__file__)) == protocol['runner_sha256']
        assert sha(E / 'cost_probe.rs') == protocol['probe_sha256']
        for variant in dirs:
            for name in [region, tests]:
                assert sha(dirs[variant] / name) == maps[variant][name]

    def run(name, command, variant, extra=None):
        verify()
        started = time.monotonic()
        with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(command, cwd=dirs[variant],
                                  env=os.environ | env | {'CARGO_TARGET_DIR': str(targets[variant])} | (extra or {}),
                                  stdout=stdout, stderr=stderr).returncode
        verify()
        row = {'name': name, 'command': command, 'variant': variant, 'extra_environment': extra or {},
               'exit_code': code, 'seconds': time.monotonic() - started,
               'stdout_sha256': sha(out / (name + '.stdout')), 'stderr_sha256': sha(out / (name + '.stderr'))}
        commands.append(row)
        (out / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
        print(json.dumps(row), flush=True)
        assert code == 0
        return (out / (name + '.stdout')).read_text()

    binaries = {}
    for variant in ['baseline', 'candidate']:
        run('clippy-' + variant, ['cargo', 'clippy', '--locked', '--release', '-p', 'hypercurve', '--all-features', '--lib', '--tests', '--', '-D', 'warnings'], variant)
        log = run('build-' + variant, ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--all-features', '--lib', '--no-run', '--message-format=json'], variant)
        events = [json.loads(line) for line in log.splitlines() if line.startswith('{')]
        artifacts = [event for event in events if event.get('reason') == 'compiler-artifact'
                     and event.get('executable') and event['target']['name'] == 'hypercurve']
        assert len(artifacts) == 1
        artifact = artifacts[0]
        assert Path(artifact['manifest_path']) == dirs[variant] / 'crates/hypercurve/Cargo.toml'
        binary = Path(artifact['executable'])
        assert binary.is_relative_to(targets[variant])
        binaries[variant] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size,
                             'features': artifact['features'], 'manifest_path': artifact['manifest_path']}
        run('regressions-' + variant, [str(binary), 'metadata_order_tests', '--test-threads=1'], variant)
    assert binaries['baseline']['sha256'] != binaries['candidate']['sha256']
    assert binaries['baseline']['features'] == binaries['candidate']['features']
    (out / 'binaries.json').write_text(json.dumps(binaries, indent=2) + '\n')
    observations = []
    for sample in schedule:
        for variant in sample['order']:
            binary = binaries[variant]
            assert sha(Path(binary['path'])) == binary['sha256']
            label = f"{sample['mode']}-{sample['pair']}-{variant}"
            log = run(label, [binary['path'], 'bezier_region::metadata_order_tests::cost::probe', '--exact', '--ignored', '--test-threads=1', '--nocapture'], variant,
                      {'REGION_PROBE_MODE': sample['mode'], 'REGION_PROBE_ITERATIONS': str(sample['iterations'])})
            rows = []
            for line in log.splitlines():
                if 'REGION_COST ' in line:
                    kind, phase, count, nanos, calls, allocated, live, peak = line.split('REGION_COST ', 1)[1].split()
                    rows.append({'case': kind, 'phase': phase, 'iterations': int(count), 'nanos': int(nanos),
                                 'allocation_calls': int(calls), 'allocated_bytes': int(allocated),
                                 'retained_bytes': int(live), 'peak_extra_bytes': int(peak)})
            assert len(rows) == 18 and all(row['iterations'] == sample['iterations'] for row in rows)
            observations.append({'mode': sample['mode'], 'pair': sample['pair'], 'variant': variant, 'rows': rows})
            (out / 'observations.json').write_text(json.dumps(observations, indent=2) + '\n')
    verify()
    for variant in dirs:
        assert all(sha(dirs[variant] / name) == digest for name, digest in maps[variant].items())
    assert all(sha(CANDIDATE / name) == digest for name, digest in candidate.items())
    (out / 'result.json').write_text(json.dumps({'complete': True, 'fresh_processes': len(observations),
                                               'observations': sum(len(row['rows']) for row in observations),
                                               'all_expected_outcomes_passed': True,
                                               'sources_unchanged': True, 'runtime_admitted': False}, indent=2) + '\n')


if __name__ == '__main__':
    main()
