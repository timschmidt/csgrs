import hashlib, json, subprocess, sys, tarfile, time
from pathlib import Path
out = Path(sys.argv[1]).resolve()
out.mkdir(parents=True, exist_ok=False)
paths = sorted([p for name in ['hyperreal', 'hyperlimit', 'hyperlattice', 'hypersolve', 'hypercurve', 'hyperdrc'] for p in Path('crates', name).rglob('*') if p.is_file() and 'target' not in p.parts and (p.suffix == '.rs' or p.name in ['Cargo.toml', 'Cargo.lock'])] + list(Path('src').rglob('*.rs')) + [Path(p) for p in ['Cargo.toml', 'Cargo.lock', '.cargo/config.toml', '.rustfmt.toml', 'crates/.rustfmt.toml', 'crates/hyperdrc/docs/CPArti FPGA dev board.zip', 'crates/hyperdrc/docs/HVP109A.zip']])
def hashes():
    return {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
initial = hashes()
with tarfile.open(out / 'sources.tar.gz', 'w:gz') as archive:
    for p in paths:
        archive.add(p, arcname=str(p))
record = dict(base_revision=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(), source_hashes=initial, unchanged=True)
(out / 'sources.json').write_text(json.dumps(record, indent=2) + '\n')
(out / 'reproduce.py').write_text(Path(__file__).read_text())
commands = [('format', ['cargo', 'fmt', '--all', '--', '--check']), ('focused-circle-contact-oracle', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'retained_circle_line_contacts', '--', '--test-threads=1']), ('focused-negative-weight-contact', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'negative_common_weights_preserve_geometric_line_crossing_direction', '--', '--test-threads=1']), ('focused-finite-line-roots', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'finite_line_roots_preserve_tangency_crossing_and_denominator_sign', '--', '--test-threads=1']), ('focused-homogeneous-line-contact', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'homogeneous_representation_preserves_exact_degree_elevation', '--', '--test-threads=1']), ('focused-boolean-kernel', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'curve_region_boolean::', '--', '--test-threads=2']), ('focused-bbox-unit', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'bbox::tests::', '--', '--test-threads=1']), ('focused-bbox-integration', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--test', 'hypercurve_bbox', '--', '--test-threads=1']), ('focused-policy-budgets', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'policy::budget_tests::', '--', '--test-threads=1']), ('focused-cached-operand-order', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'cached_interval_tests::', '--', '--test-threads=1']), ('focused-native-contour-winding', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'contour::tests::', '--', '--test-threads=1']), ('focused-contour-integration', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--test', 'hypercurve_contour', '--', '--test-threads=1']), ('focused-native-winding-reuse', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'native_winding_reuse', '--', '--test-threads=1']), ('focused-materialized-winding-jump', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'materialized_single_boundary_windings_match_two_rays', '--', '--test-threads=1']), ('retained-circle-support-regressions', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'retained_circle_support', '--', '--test-threads=1']), ('focused-circular-arc-recovery', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'arc_bezier::', '--', '--test-threads=1']), ('focused-injectivity-certificates', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'injectivity_certificates_own_their_domain_and_circle_chart', '--', '--test-threads=1']), ('focused-tangent-order', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'retained_and_represented_tangents_share_filled_face_order', '--', '--test-threads=1']), ('focused-boundary-tangents', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'boundary_tangent', '--', '--test-threads=1']), ('focused-boundary-side-rays', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'boundary_side_rays', '--', '--test-threads=1']), ('focused-affine-chart', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'retained_affine_line_chart', '--', '--test-threads=1']), ('focused-region-boolean', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--test', 'hypercurve_curve_region_boolean', '--', '--test-threads=1']), ('focused-line-support', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'retained_delta_tests', '--', '--test-threads=2']), ('focused-line-intersections', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'intersect::', '--', '--test-threads=2']), ('focused-fit-stroke', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--test', 'hypercurve_bezier_fit_offset', '--test', 'native_linear_stroke', '--test', 'hypercurve_curve_region_stroke', '--', '--test-threads=2']), ('focused-drc', ['cargo', 'test', '--locked', '-p', 'hyperdrc', '--release', '--test', 'native_linear_strokes', '--', '--test-threads=2']), ('curve-clippy', ['cargo', 'clippy', '--locked', '-p', 'hypercurve', '--release', '--all-targets', '--', '-D', 'warnings']), ('build-board-smoke', ['cargo', 'test', '--locked', '-p', 'hyperdrc', '--release', '--lib', '--no-run', '--message-format=json'])]
status = []
def run(name, command):
    assert hashes() == initial
    started = time.monotonic()
    with (out / (name + '.log')).open('w') as log:
        code = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT).returncode
    unchanged = hashes() == initial
    status.append(dict(name=name, command=command, exit_code=code, elapsed_seconds=time.monotonic()-started))
    (out / 'status.json').write_text(json.dumps(status, indent=2) + '\n')
    record['unchanged'] = unchanged
    (out / 'sources.json').write_text(json.dumps(record, indent=2) + '\n')
    assert unchanged
    print(name, code, flush=True)
    if code:
        raise SystemExit(code)
for name, command in commands:
    run(name, command)
executables = []
for line in (out / 'build-board-smoke.log').read_text().splitlines():
    try:
        item = json.loads(line)
    except ValueError:
        continue
    if item.get('reason') == 'compiler-artifact' and item.get('target', {}).get('name') == 'hyperdrc' and item.get('executable'):
        executables.append(item['executable'])
assert len(executables) == 1, executables
run('full-board-smoke', ['/usr/bin/time', '-v', '-o', str(out / 'full-board-smoke-time.log'), executables[0], 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite', '--exact', '--test-threads=1', '--nocapture'])
log = (out / 'full-board-smoke.log').read_text()
assert 'skipping complex project fixture test' not in log
assert log.count('hyperdrc: finished emit report') == 2
