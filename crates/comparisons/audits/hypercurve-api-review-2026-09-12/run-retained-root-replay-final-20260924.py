from pathlib import Path
import hashlib, json, os, shutil, subprocess, sys, time
A = Path(__file__).resolve().parent
W = A.parent
version = sys.argv[1]
assert version.startswith('v') and version[1:].isdigit()
root = Path(f'/tmp/hypercurve-retained-coordinate-identity-{version}-20260924')
prefix = f'retained-coordinate-identity-20260924-{version}'
manifest = json.loads((A / f'{prefix}-sources.json').read_text())
def verify():
    for name, sha in manifest.items():
        assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
env = dict(os.environ, **json.loads((A / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
toolchain = Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin')
cargo = str(toolchain / 'cargo')
repo = root / 'hypercurve'
report = dict(checks=[], cases=[], all_processes_reaped=False)
def save():
    verify()
    report['all_sources_unchanged'] = True
    (A / f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2) + '\n')
def run(label, command, limit):
    log = A / f'{prefix}-{label}.log'
    start = time.monotonic()
    with log.open('w') as out:
        try:
            code = subprocess.run(command, cwd=repo, env=env, stdout=out, stderr=subprocess.STDOUT, timeout=limit).returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    verify()
    row = dict(label=label, command=command, returncode=code, elapsed_seconds=time.monotonic()-start, log=log.name)
    print(label, code, log.read_text()[-1800:] if code else '', flush=True)
    return row
verify()
changed_files = ['src/bezier_offset.rs', 'src/bezier_region.rs']
row = run('fmt', [str(toolchain / 'rustfmt'), '--edition', '2024', '--check', *changed_files], 60)
report['checks'].append(row)
assert row['returncode'] == 0
diagnostic = len(sys.argv) > 2 and sys.argv[2] == 'diagnostic'
for feature in ([] if diagnostic else ['--all-features', '--no-default-features']):
    row = run('clippy-' + str(len(report['checks'])), [cargo, 'clippy', '--all-targets', feature, '--locked', '--offline', '--', '-D', 'warnings'], 1200)
    report['checks'].append(row)
    if row['returncode']:
        report['all_processes_reaped'] = True
        save()
        raise SystemExit(1)
os.utime(repo / 'src/lib.rs', None)
command = [cargo, 'test', '--lib', '--release', '--all-features', '--no-run', '--message-format=json', '--locked', '--offline']
with (A / f'{prefix}-build.jsonl').open('w') as out, (A / f'{prefix}-build.log').open('w') as err:
    code = subprocess.run(command, cwd=repo, env=env, stdout=out, stderr=err, timeout=1200).returncode
verify()
assert code == 0
artifacts = [json.loads(line) for line in (A / f'{prefix}-build.jsonl').read_text().splitlines()]
artifact = next(row for row in artifacts if row.get('reason') == 'compiler-artifact' and row['target']['name'] == 'hypercurve' and row.get('executable'))
assert not artifact['fresh']
binary = A / f'{prefix}-libtest'
shutil.copy2(artifact['executable'], binary)
report['binary_sha256'] = hashlib.sha256(binary.read_bytes()).hexdigest()
names = [line[:-6] for line in subprocess.check_output([str(binary), '--list'], text=True).splitlines() if line.endswith(': test')]
suffixes = ['one_fragment_materialized_loop_chamfers_to_one_middle_interval', 'one_fragment_selected_loop_chamfers_from_one_interval', 'analytic_axis_order_replays_a_local_root_in_the_other_points_field', 'retained_chord_coordinate_and_linear_queries_observe_requested_policy', 'algebraic_chord_source_incidence_deflates_only_the_retained_contact', 'recursive_projective_kernel_imports_similarity_of_algebraic_endpoints', 'algebraic_chord_retains_independent_endpoint_fields_under_both_policies', 'independent_field_corner_edits_preserve_normalized_sets', 'one_fragment_materialized_loop_extends_algebraic_chamfer_cuts_once', 'one_fragment_selected_loop_extends_chamfer_cuts_on_its_analytic_carrier']
for suffix in (suffixes[:1] if diagnostic else suffixes):
    matches = [name for name in names if name.rsplit('::', 1)[-1] == suffix]
    assert len(matches) == 1, suffix
    row = run(suffix, [str(binary), '--exact', matches[0], '--nocapture', '--test-threads=1', '--color', 'never'], 75)
    report['cases'].append(row)
    if row['returncode']:
        report['all_processes_reaped'] = True
        save()
        raise SystemExit(1)
report['all_processes_reaped'] = True
save()
print('Terminal; all retained-coordinate qualification processes reaped.', flush=True)
