from pathlib import Path
import hashlib, json, os, shutil, signal, subprocess, time

audit = Path(__file__).resolve().parent
root = Path('/tmp/hypercurve-chord-normal-witness-2026-09-23')
prefix = 'chord-normal-witness-20260923-focused1'
env = dict(os.environ, **json.loads((audit/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
bindings = json.loads((audit/'chord-normal-witness-20260923-candidate-sources.json').read_text())

def verify():
    for name, sha in bindings.items():
        assert hashlib.sha256((root/name).read_bytes()).hexdigest() == sha, name

def run(command, repo, out, err, bound):
    process = subprocess.Popen(command, cwd=repo, env=env, stdout=out, stderr=err, start_new_session=True)
    try:
        return process.wait(timeout=bound)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()
        return 'timeout'

verify()
(audit/f'{prefix}-sources.json').write_text(json.dumps(bindings, indent=2)+'\n')
reports = {}
for crate in ['hypersolve', 'hypercurve']:
    repo = root/crate
    stem = f'{prefix}-{crate}'
    command = [cargo, 'test', '--lib', '--release', '--all-features', '--no-run', '--message-format=json', '--locked', '--offline']
    with (audit/f'{stem}-build.jsonl').open('w') as out, (audit/f'{stem}-build.log').open('w') as err:
        code = run(command, repo, out, err, 1200)
    verify()
    assert code == 0, (crate, code)
    artifacts = []
    for line in (audit/f'{stem}-build.jsonl').read_text().splitlines():
        row = json.loads(line)
        if row.get('reason') == 'compiler-artifact' and row.get('executable') and row['target']['name'] == crate:
            artifacts.append(row)
    assert len(artifacts) == 1
    binary = audit/f'{stem}-libtest'
    shutil.copy2(artifacts[0]['executable'], binary)
    sha = hashlib.sha256(binary.read_bytes()).hexdigest()
    names = [line[:-6] for line in subprocess.check_output([str(binary), '--list'], text=True).splitlines() if line.endswith(': test')]
    if crate == 'hypersolve':
        selected = [(name, 75) for name in names if name.startswith('algebraic_tensor_image::tests::')]
        assert any('selected_low_degree_witness_does_not_depend_on_interval_precision' in name for name, _ in selected)
        assert any('selected_low_degree_witness_accepts_exact_nonrational_coefficients' in name for name, _ in selected)
    else:
        suffixes = ['chord_normal_recursive_frame_retains_center_and_oriented_unit_normal', 'parallel_normal_recursive_frame_preserves_source_parameter_and_positive_speed', 'rank_independent_chord_normal_circle_publishes_rational_overlap', 'dense_chord_normal_independent_anchor_uses_rank_independent_fallback', 'nonrepresented_chord_and_selected_circle_complete_the_fillet_kernel', 'nonrepresented_chord_and_retained_rational_arc_share_the_fillet_kernel', 'selected_circle_and_promoted_line_extend_through_the_chord_support_cell']
        selected = []
        for suffix in suffixes:
            matches = [name for name in names if name.rsplit('::', 1)[-1] == suffix]
            assert len(matches) == 1, suffix
            selected.append((matches[0], 75))
    rows = []
    print('Built', crate, sha, flush=True)
    for index, (name, bound) in enumerate(selected):
        log = audit/f'{stem}-case-{index}.log'
        start = time.monotonic()
        with log.open('w') as out:
            code = run([str(binary), '--exact', name, '--nocapture', '--test-threads=1', '--color', 'never'], repo, out, subprocess.STDOUT, bound)
        verify()
        output = log.read_text()
        assert 'running 1 test' in output
        row = dict(name=name, returncode=code, passed=code == 0 and '1 passed;' in output, limit_seconds=bound, elapsed_seconds=time.monotonic()-start, log=log.name)
        rows.append(row)
        (audit/f'{stem}-cases.json').write_text(json.dumps(rows, indent=2)+'\n')
        print(row, output[-1200:] if not row['passed'] else '', flush=True)
    reports[crate] = dict(cases=rows, binary_sha256=sha)
    if crate == 'hypersolve':
        assert all(row['passed'] for row in rows), 'Hypersolve focused regressions must pass before Hypercurve build'
verify()
reports.update(all_sources_unchanged=True, all_processes_reaped=True)
(audit/f'{prefix}-terminal.json').write_text(json.dumps(reports, indent=2)+'\n')
print('Terminal; both focused builds and all cases reaped.', flush=True)
