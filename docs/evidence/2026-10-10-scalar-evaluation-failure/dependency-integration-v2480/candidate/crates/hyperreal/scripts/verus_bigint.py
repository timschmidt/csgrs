"""Bind verified numeric metadata to its source, Cargo library and fresh export."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile

EXPECTED_UNITS = 453
REQUIRED_FUNCTIONS = {
    'num_bigint::bigint::BigInt::' + name for name in
    ['sign', 'magnitude', 'metadata_sign', 'metadata_magnitude',
     'metadata_storage_bits', 'metadata_normalized', 'metadata_value']
} | {
    'num_bigint::biguint::BigUint::' + name for name in
    ['bits', 'trailing_zeros', 'trailing_ones', 'metadata_value',
     'metadata_normalized', 'metadata_storage_bits']
} | {
    'num_bigint::biguint::metadata::' + name for name in
    ['first_different', 'last_nonzero', 'trim_zero_suffix', 'zero_count', 'one_count', 'bit_count']
}


def sha256(path):
    with Path(path).open('rb') as source:
        return hashlib.file_digest(source, 'sha256').hexdigest()


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def catalog(root):
    root = Path(root)
    result = {}
    for directory, children, names in os.walk(root):
        children[:] = sorted(n for n in children if n not in {'target', '.git', '__pycache__'})
        for name in [*children, *names]:
            require(not (Path(directory) / name).is_symlink(), 'Symbolic-link verification input: ' + name)
        for name in sorted(names):
            path = Path(directory) / name
            if path.is_file():
                result[str(path.relative_to(root))] = sha256(path)
    return result


def check_files(files, message):
    require(all(Path(name).is_file() and sha256(name) == value for name, value in files.items()), message)


def check_inputs(config):
    require(catalog(config['vendor']) == config['vendor_files'], 'Verified dependency source catalog changed')
    require(catalog(config['tool_directory']) == config['tool_files'], 'Verified dependency tool catalog changed')
    check_files(config['model_sources'], 'Copied dependency proof models changed')
    check_files(config['helpers'], 'Verified dependency build helpers changed')


def valid_report(report):
    verified = report.get('verification-results', {})
    return (verified.get('verified') == EXPECTED_UNITS and verified.get('errors') == 0
            and verified.get('success') is True and verified.get('is-verifying-entire-crate') is True
            and not verified.get('encountered-error', True) and not verified.get('encountered-vir-error', True)
            and REQUIRED_FUNCTIONS <= report.get('func-details', {}).keys())


def prepare_export(root, pin, packages, target_base, verus_path=None):
    root = Path(root).resolve()
    selected = [p for p in packages if p['name'] == 'num-bigint']
    require(len(selected) == 1, 'Expected one production-bound num-bigint package')
    package = selected[0]
    require(package.get('source') is None and package['version'] == '0.4.6',
            'Verified metadata requires the local num-bigint 0.4.6 package')
    vendor = Path(package['manifest_path']).resolve().parent
    provenance_path = vendor / 'verification/provenance.json'
    require(provenance_path.is_file(), 'Verified dependency model provenance is missing')
    provenance = json.loads(provenance_path.read_text())
    records = provenance.get('proof_models', {})
    require(len(records) == 17, 'Unexpected dependency model inventory')
    model_sources = {}
    for name, record in records.items():
        relative = record['source'].removeprefix('crates/hyperreal/')
        require(relative != record['source'], 'Unexpected scalar model source prefix')
        original = (root / relative).resolve()
        copied = (vendor / name).resolve()
        require(original.is_relative_to(root) and copied.is_relative_to(vendor), 'Model source escapes its package')
        require(original.is_file() and copied.is_file(), 'Dependency model source is missing')
        require(sha256(original) == sha256(copied) == record['sha256'], 'Dependency model copy is stale')
        model_sources[str(original)] = record['sha256']
    default = root / 'target/verus' / pin['version'] / 'verus-x86-linux/verus'
    tool = Path(verus_path or os.environ.get('VERUS', default)).expanduser().resolve()
    require(tool.is_file(), 'Pinned dependency verifier is missing')
    version_run = subprocess.run([str(tool), '--version', '--output-json'], capture_output=True, text=True, check=True)
    version = json.loads(version_run.stdout)['verus']
    require(version['version'] == pin['version'] and version['commit'] == pin['commit'],
            'Dependency verifier does not match the pinned version')
    target_base = Path(target_base)
    target_base.mkdir(parents=True, exist_ok=True)
    generation = Path(tempfile.mkdtemp(prefix='verified-', dir=target_base)).resolve()
    scripts = Path(__file__).resolve().parent
    wrapper = scripts / 'verus_rustc_wrapper.py'
    helpers = {str(path): sha256(path) for path in [Path(__file__).resolve(), wrapper, scripts / 'verus_dependencies.py']}
    config = {
        'vendor': str(vendor), 'package_id': package['id'], 'vendor_files': catalog(vendor),
        'model_sources': model_sources, 'tool': str(tool), 'tool_directory': str(tool.parent),
        'tool_files': catalog(tool.parent), 'pin': pin, 'helpers': helpers,
        'generation': str(generation), 'export': str(generation / 'num_bigint.vir'),
        'result': str(generation / 'compiler-result.json'),
        'invocation': str(generation / 'compiler-invocation.json'),
        'stdout': str(generation / 'compiler.stdout.json'), 'stderr': str(generation / 'compiler.stderr.log'),
    }
    config_path = generation / 'compiler-config.json'
    config_path.write_text(json.dumps(config, indent=2) + '\n')
    context = {'config': str(config_path), 'config_sha256': sha256(config_path), 'package_id': package['id']}
    overrides = {
        'CARGO_TARGET_DIR': str(generation / 'cargo'), 'CARGO_INCREMENTAL': '0', 'CARGO_PROFILE_DEV_DEBUG': '0',
        'RUSTC_WRAPPER': str(wrapper), 'RUSTC_WORKSPACE_WRAPPER': '',
        'HYPERREAL_VERUS_DEPENDENCY_CONFIG': str(config_path),
        'CARGO_ENCODED_RUSTFLAGS': '\x1f'.join(['-D', 'warnings', '-L', 'dependency=' + str(tool.parent)]),
    }
    check_inputs(config)
    return context, overrides


def read_config(context):
    require(sha256(context['config']) == context['config_sha256'], 'Verified dependency configuration changed')
    config = json.loads(Path(context['config']).read_text())
    require(config['package_id'] == context['package_id'], 'Verified dependency package binding changed')
    check_inputs(config)
    return config


def bind_export(context, cargo_artifacts):
    config = read_config(context)
    result_path = Path(config['result'])
    require(result_path.is_file(), 'Cargo did not run the verified dependency compiler')
    result = json.loads(result_path.read_text())
    require(result['gate_passed'], 'Dependency proof or compilation failed')
    matches = [a for a in cargo_artifacts if a['package_id'] == context['package_id']
               and a['target']['name'] == 'num_bigint' and a['target']['kind'] == ['lib']]
    require(len(matches) == 1, 'Expected one verified num-bigint Cargo artifact')
    libraries = {name: sha256(name) for name in matches[0]['filenames'] if name.endswith(('.rlib', '.rmeta'))}
    require(any(name.endswith('.rlib') for name in libraries), 'Verified Cargo dependency library is missing')
    require(libraries == result['library_artifacts'], 'Cargo dependency library does not match the proved compilation')
    evidence = context | {
        'result_sha256': sha256(result_path), 'export': config['export'], 'export_sha256': result['export_sha256'],
        'library_artifacts': libraries,
    }
    check_export(evidence)
    return ['--import', 'num_bigint=' + config['export']], evidence


def check_export(evidence):
    config = read_config(evidence)
    require(sha256(config['result']) == evidence['result_sha256'], 'Verified compiler result changed')
    result = json.loads(Path(config['result']).read_text())
    require(result['gate_passed'] and result['exit_code'] == 0 and result['warnings'] == 0,
            'Invalid dependency compiler result')
    require(result['config_sha256'] == evidence['config_sha256'], 'Compiler used another dependency configuration')
    require(result['verified'] == EXPECTED_UNITS, 'Dependency proof coverage changed')
    require(result['export_sha256'] == evidence['export_sha256'], 'Dependency export result binding changed')
    require(evidence['export'] == config['export'], 'Dependency export path changed')
    check_files({config['export']: evidence['export_sha256']}, 'Dependency proof export changed')
    check_files(evidence['library_artifacts'], 'Verified dependency library changed')
    require(result['library_artifacts'] == evidence['library_artifacts'], 'Verified dependency library binding changed')
    check_files({config['stdout']: result['stdout_sha256'], config['stderr']: result['stderr_sha256'],
                 config['invocation']: result['invocation_sha256']}, 'Verified compiler receipt changed')
    invocation = json.loads(Path(config['invocation']).read_text())
    require(invocation['config_sha256'] == evidence['config_sha256'], 'Compiler invocation configuration changed')
    require(Path(invocation['manifest_directory']).resolve() == Path(config['vendor']), 'Compiler package path changed')
    require(str(Path(config['vendor']) / 'src/lib.rs') in invocation['cargo_arguments'], 'Compiler source root changed')
    require(invocation['command'] == [config['tool'], '--no-cheating', '--output-json', '--compile', '--export',
                                    config['export'], *invocation['cargo_arguments']], 'Compiler arguments changed')
    report = json.loads(Path(config['stdout']).read_text())
    require(valid_report(report), 'Dependency proof report is incomplete')
    require(report['verus']['version'] == config['pin']['version']
            and report['verus']['commit'] == config['pin']['commit'], 'Dependency proof report uses another verifier')
    diagnostics = [json.loads(line) for line in Path(config['stderr']).read_text().splitlines()]
    require(not any(item.get('level') == 'warning' for item in diagnostics), 'Dependency compiler reported warnings')
