"""Retain integrated proof controls and every fresh dependency compiler receipt."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import runpy
import shutil
import subprocess
import sys

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
CRATE = ROOT / 'target/scalar-evaluation-candidate/source/crates/hyperreal'
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    phase, entrypoint = sys.argv[1:]
    assert entrypoint in {'test_acceptance.py', 'test_rejections.py', 'test_approximation_bound_rejections.py'}
    out = E / phase / 'proof-checks'
    out.mkdir()
    pin = json.loads((CRATE / 'verification/toolchain.json').read_text())
    tool = ROOT / 'crates/hyperreal/target/verus' / pin['version'] / 'verus-x86-linux'
    tool_hashes = {str(p.relative_to(tool)): sha(p) for p in sorted(tool.rglob('*')) if p.is_file()}
    os.environ['VERUS'] = str(tool / 'verus')
    os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
    sys.path[:0] = [str(CRATE / 'scripts'), str(CRATE / 'verification')]
    import verify_verus
    import verus_dependencies
    reports = CRATE / 'target/proof-checks' / phase
    assert not reports.exists()
    reports.mkdir(parents=True)
    verify_verus.REPORTS = reports
    actual_run = subprocess.run
    actual_prepare = verus_dependencies.prepare_dependencies
    calls, builds = [], []

    def retain_prepare(*args, **kwargs):
        arguments, evidence = actual_prepare(*args, **kwargs)
        directory = out / ('dependency-build-' + str(len(builds)))
        directory.mkdir()
        config_path = Path(evidence['verified_metadata']['config'])
        for name in ['compiler-config.json', 'compiler-invocation.json', 'compiler-result.json',
                     'compiler.stdout.json', 'compiler.stderr.log']:
            shutil.copyfile(config_path.parent / name, directory / name)
        for name in ['dependencies-build.stdout', 'dependencies-build.stderr', 'dependencies.json']:
            shutil.copyfile(args[2] / name, directory / name)
        builds.append({'arguments': arguments, 'directory': directory.name,
            'files_sha256': {p.name: sha(p) for p in sorted(directory.iterdir())},
            'verified_metadata': evidence['verified_metadata']})
        return arguments, evidence

    def retain_run(command, *args, **kwargs):
        proof = str(command[0]) == str(tool / 'verus') and any(str(a).endswith('lib.rs') for a in command)
        record = None
        if proof:
            library = next(Path(a) for a in command if str(a).endswith('lib.rs'))
            inputs = library.parent
            if inputs.name == 'verification' and (inputs.parent / 'src').is_dir():
                inputs = inputs.parent
            record = {'command': [str(a) for a in command], 'input_root': str(inputs),
                'rust_inputs_sha256': {str(p.relative_to(inputs)): sha(p)
                                      for p in sorted(inputs.rglob('*.rs'))}}
        result = actual_run(command, *args, **kwargs)
        if record is not None:
            record.update(exit_code=result.returncode, stdout=result.stdout, stderr=result.stderr)
            assert record['rust_inputs_sha256'] == {str(p.relative_to(inputs)): sha(p)
                                                   for p in sorted(inputs.rglob('*.rs'))}
            calls.append(record)
        return result

    subprocess.run = retain_run
    verus_dependencies.prepare_dependencies = retain_prepare
    verify_verus.prepare_dependencies = retain_prepare
    script = CRATE / 'verification' / entrypoint
    sys.argv = [str(script)]
    code = 1
    try:
        runpy.run_path(str(script), run_name='__main__')
        code = 0
    except SystemExit as error:
        code = error.code if isinstance(error.code, int) else (1 if error.code else 0)
        if code:
            raise
    finally:
        subprocess.run = actual_run
        verus_dependencies.prepare_dependencies = actual_prepare
        verify_verus.prepare_dependencies = actual_prepare
        log = out / 'verus-invocations.jsonl.gz'
        log.write_bytes(gzip.compress(''.join(json.dumps(call, sort_keys=True) + '\n'
                                            for call in calls).encode(), mtime=0))
        unchanged = tool_hashes == {str(p.relative_to(tool)): sha(p)
                                   for p in sorted(tool.rglob('*')) if p.is_file()}
        data = {'entrypoint': str(script), 'entrypoint_sha256': sha(script), 'runner_sha256': sha(__file__),
            'exit_code': code, 'pinned_toolchain': pin, 'tool_directory': str(tool), 'tool_sha256': tool_hashes,
            'tool_unchanged': unchanged, 'verus_invocations': len(calls),
            'baseline_successes': sum(call['exit_code'] == 0 for call in calls),
            'rejected_invocations': sum(call['exit_code'] != 0 for call in calls),
            'invocation_log_sha256': sha(log), 'dependency_builds': builds,
            'scope': 'Execute the candidate entry point unchanged, capture actual temporary Rust hashes and verifier results, and archive each dependency compiler receipt. Negative proof checks do not discharge caller premises or admit runtime changes.'}
        (out / 'tooling.json').write_text(json.dumps(data, indent=2) + '\n')
        assert unchanged
    sys.exit(code)


if __name__ == '__main__':
    main()
