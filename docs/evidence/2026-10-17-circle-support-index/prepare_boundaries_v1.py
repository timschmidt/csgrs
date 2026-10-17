"""Create a frozen boundary runner using the same source-verifying containment."""
from pathlib import Path
W = Path(__file__).resolve().parent
prefix = (W / 'qualify_v2.py').read_text().split("run('format', ['cargo', 'fmt', '--all', '--', '--check'])")[0]
prefix = prefix.replace("O = W / 'candidate-qualification-v2'", "O = W / 'boundary-validation-v1'")
prefix = prefix.replace("Path('/tmp/csgrs-circle-support-qualification-v2')", "Path('/tmp/csgrs-circle-support-boundaries-v1')")
prefix = prefix.replace('def run(name, command, docs=False):', 'def run(name, command, docs=False, extra=None, wasm=False):')
prefix = prefix.replace("command_env = env | ({'RUSTDOCFLAGS': '-D warnings'} if docs else {})", "command_env = env | ({'RUSTDOCFLAGS': '-D warnings'} if docs else {}) | (extra or {})")
prefix = prefix.replace('preexec_fn=limits, start_new_session=True', 'preexec_fn=wasm_limits if wasm else limits, start_new_session=True')
prefix = prefix.replace("'rustdocflags': command_env.get('RUSTDOCFLAGS'),", "'rustdocflags': command_env.get('RUSTDOCFLAGS'), 'environment_overrides': extra or {}, 'address_space_bytes': (64 if wasm else 8) * 1024**3,")
suffix = '''
def wasm_limits():
    resource.setrlimit(resource.RLIMIT_AS, (64 * 1024**3, 64 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

old = W.parent / 'export-projection-contracts'
tool = read(old / 'wasm-tools-v1/result.json')
assert tool['exit_code'] == 0 and tool['version'] == 'wasm-bindgen 0.2.126'
assert sha(tool['binary']) == tool['binary_sha256']
typescript = old / 'wasm-source-v1/tests/wasm/node_modules/typescript'
assert read(typescript / 'package.json')['version'] == '5.7.3'
tools = {str(p): sha(p) for p in typescript.rglob('*') if p.is_file()}
tools[tool['binary']] = tool['binary_sha256']
dump(O / 'tools.json', {'read_only_sha256': tools, 'wasm_limits': '64 GiB address space for V8 reservations; not a heap bound',
                       'node': subprocess.check_output(['node', '--version'], text=True),
                       'tsc': str(typescript / 'bin/tsc'), 'wasm_bindgen': tool['binary']})

run('format', ['cargo', 'fmt', '--all', '--', '--check'])
run('ffi-build', cargo('build', 'csgrs-ffi', '--message-format=json'))
run('ffi-tests', cargo('test', 'csgrs-ffi', '--no-fail-fast', '--', '--test-threads=1'))
run('ffi-clippy', cargo('clippy', 'csgrs-ffi', '--all-targets', '--', '-D', 'warnings'))
library = T / 'release/libcsgrs_ffi.so'
assert library.is_file()
shared = {'CSGRS_LIBRARY': str(library), 'PYTHONDONTWRITEBYTECODE': '1'}
for name in ['empty_composition', 'curve_constructor_failures', 'planar_operation_failures', 'exact_linear_modeling']:
    for language, cc, standard in [('c', 'cc', '-std=c11'), ('cpp', 'c++', '-std=c++17')]:
        source = S / 'tests/ffi' / (name + '.' + language)
        if not source.exists():
            continue
        binary = O / (name + '-' + language)
        run(name + '-' + language + '-build', [cc, standard, '-Wall', '-Wextra', '-Werror', '-I', str(S / 'crates/csgrs-ffi/include'),
             '-I', str(S / 'bindings/cpp'), str(source), '-L', str(T / 'release'), '-lcsgrs_ffi',
             '-Wl,-rpath,' + str(T / 'release'), '-o', str(binary)])
        run(name + '-' + language, [str(binary)])
    source = S / 'tests/ffi' / (name + '.py')
    if source.exists():
        run(name + '-python', ['python3', str(source)], extra=shared)
run('wasm-shell-syntax', ['bash', '-n', str(S / 'scripts/test-wasm-bindings.sh')])
run('wasm-consumers', ['bash', str(S / 'scripts/test-wasm-bindings.sh')],
    extra={'WASM_BINDGEN': tool['binary'], 'TSC': str(typescript / 'bin/tsc')}, wasm=True)
run('wasm-clippy', ['cargo', 'clippy', '--offline', '--locked', '--profile', 'wasm', '--target', 'wasm32-unknown-unknown',
    '--no-default-features', '--features', 'wasm', '--lib', '--', '-D', 'warnings'], wasm=True)
assert all(sha(p) == h for p, h in tools.items())
artifacts = [library, T / 'wasm32-unknown-unknown/wasm/csgrs.wasm', *sorted((T / 'wasm-bindings-node').glob('*'))]
dump(O / 'result.json', {'complete': True, 'command_failures': [r['name'] for r in rows if r['exit_code']],
    'sources_unchanged': True, 'cargo_target_closed': True, 'native_runtime_admitted': False, 'full_plan_complete': False,
    'main_sources_modified': False, 'full_pre_push_matrix_executed': False, 'published_package_pipeline_executed': False,
    'read_only_tools_unchanged': True, 'artifacts': {str(p): {'sha256': sha(p), 'bytes': p.stat().st_size} for p in artifacts if p.is_file()}})
print(json.dumps({'complete': True, 'failures': [r['name'] for r in rows if r['exit_code']]}), flush=True)
sys.exit(101 if any(r['exit_code'] for r in rows) else 0)
'''
path = W / 'boundaries_v1.py'
with path.open('x') as f:
    f.write(prefix + suffix)
print(path)
