#!/usr/bin/python3
"""Verify and compile the selected num-bigint package with Cargo's exact arguments."""
import json
import os
from pathlib import Path
import re
import subprocess
import sys

sys.dont_write_bytecode = True
from verus_bigint import check_inputs, require, sha256, valid_report


def main():
    rustc, *arguments = sys.argv[1:]
    if '--crate-name' not in arguments or arguments[arguments.index('--crate-name') + 1] != 'num_bigint':
        os.execv(rustc, [rustc, *arguments])
    config_path = Path(os.environ['HYPERREAL_VERUS_DEPENDENCY_CONFIG'])
    config_sha256 = sha256(config_path)
    config = json.loads(config_path.read_text())
    check_inputs(config)
    require(not Path(config['result']).exists() and not Path(config['invocation']).exists(),
            'Verified dependency compilation generation was already used')
    require(Path(os.environ['CARGO_MANIFEST_DIR']).resolve() == Path(config['vendor']),
            'Cargo selected another num-bigint package')
    require(str(Path(config['vendor']) / 'src/lib.rs') in arguments, 'Unexpected dependency compilation root')
    require(arguments[arguments.index('--crate-type') + 1] == 'lib', 'Unexpected dependency compilation target')
    command = [config['tool'], '--no-cheating', '--output-json', '--compile', '--export', config['export'], *arguments]
    invocation = {'command': command, 'cargo_arguments': arguments, 'rustc': rustc,
                  'manifest_directory': os.environ['CARGO_MANIFEST_DIR'], 'config_sha256': config_sha256}
    Path(config['invocation']).write_text(json.dumps(invocation, indent=2) + '\n')
    descriptors = sorted({int(n) for pair in re.findall(
        r'--jobserver-(?:auth|fds)=(\d+),(\d+)', os.environ.get('CARGO_MAKEFLAGS', '')) for n in pair})
    for descriptor in descriptors:
        os.fstat(descriptor)
    run = subprocess.run(command, capture_output=True, text=True, pass_fds=tuple(descriptors))
    Path(config['stdout']).write_text(run.stdout)
    Path(config['stderr']).write_text(run.stderr)
    report = json.loads(run.stdout) if run.stdout.strip() else {}
    verified = report.get('verification-results', {})
    diagnostics = [json.loads(line) for line in run.stderr.splitlines()]
    warnings = sum(item.get('level') == 'warning' for item in diagnostics)
    library_paths = {item['artifact'] for item in diagnostics if item.get('$message_type') == 'artifact'
                     and item.get('emit') in {'metadata', 'link'}}
    libraries = {name: sha256(name) for name in library_paths if name.endswith(('.rlib', '.rmeta'))}
    passed = (run.returncode == 0 and valid_report(report) and warnings == 0
              and bool(libraries) and Path(config['export']).is_file())
    check_inputs(config)
    require(sha256(config_path) == config_sha256, 'Dependency compiler configuration changed while running')
    result = {'gate_passed': passed, 'exit_code': run.returncode, 'verified': verified.get('verified'),
              'verification_results': verified, 'warnings': warnings, 'config_sha256': config_sha256,
              'library_artifacts': libraries, 'inherited_jobserver_descriptors': descriptors,
              'export_sha256': sha256(config['export']) if Path(config['export']).is_file() else None,
              'stdout_sha256': sha256(config['stdout']), 'stderr_sha256': sha256(config['stderr']),
              'invocation_sha256': sha256(config['invocation'])}
    Path(config['result']).write_text(json.dumps(result, indent=2) + '\n')
    sys.stderr.write(run.stderr)
    sys.exit(0 if passed else (run.returncode or 1))


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        sys.exit('verified dependency compiler: ' + str(error))
