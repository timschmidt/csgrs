from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib, json, re, subprocess, sys, time

root = Path('/home/tim/Documents/GitHub/workspace')
audit = root / 'hypercurve-api-review-2026-09-12'
mode = sys.argv[1]
assert mode in {'focused', 'full'}

def artifacts(repo, stem):
    assert (audit / (stem + '.exit')).read_text().strip() == '0'
    binaries = {}
    for line in (audit / (stem + '.jsonl')).read_text().splitlines():
        item = json.loads(line)
        if item.get('reason') == 'compiler-artifact' and item.get('executable') and item['profile']['test']:
            name = 'lib' if item['target']['kind'] == ['lib'] else item['target']['name']
            binaries[name] = item['executable']
    return [{'repo': repo, 'target': name, 'binary': path} for name, path in binaries.items()]

jobs = artifacts('hypercurve', 'shared-parameter-component-overlap-test-build')
assert len(jobs) == 48, len(jobs)
held_out = {
    'lib': [
        'selected_parallel_normal_circle_intersects_genuinely_analytic_parallel_in_one_fiber',
        'independent_oblique_chord_pair_fillets_extend_on_infinite_supports',
        'selected_circle_and_analytic_parallel_extend_on_full_supports',
        'pair_native_boolean_algebraic_chord_corner_publishes_a_third_generation_fillet',
    ],
    'hypercurve_analytic_parallel_region': [
        'radical_parallel_cusp_offsets_exactly_under_both_policies',
        'retained_rational_arc_and_analytic_parallel_fillet_exactly',
    ],
    'hypercurve_pcb_process_image': [
        'easyduino_scale_process_image_containment_corpus',
        'easyduino_uno_scale_process_image_with_holes_corpus',
    ],
}
# Find the PCB target by its listed tests rather than presuming its target name.
for job in jobs:
    if 'pcb' in job['target']:
        held_out[job['target']] = held_out['hypercurve_pcb_process_image']

if mode == 'focused':
    lib = next(job for job in jobs if job['target'] == 'lib')
    jobs = [{**lib, 'filter': name} for name in [
        'analytic_point_equality_replays_algebraic_source_and_normal_sheet',
        'analytic_axis_order_reuses_polynomial_speed_across_reduced_tangent_fields',
        'curve_support::tests',
        'algebraic_chord_parallel_boolean_keeps_exterior_and_selected_ranges',
        'algebraic_chord_analytic_parallel_pair_replays_contacts_and_overlap',
        'curve_intersection::curve_support_intersection::circle_dispatch_tests',
        'algebraic_cusp_semicircle_replays_a_selected_circle_component',
        'curve_region_trim::tests',
        'curve_region_boolean::certified_successor_tests::curve_trim',
        'native_span_publication_preserves_geometry_and_parameter_lineage',
        'curve_region_retains_and_classifies_an_algebraic_cusp_semicircle',
    ]]
else:
    jobs += artifacts('hyperbrep', 'shared-parameter-component-overlap-hyperbrep-test-build')
    jobs.sort(key=lambda job: (0 if job['target'] == 'lib' and job['repo'] == 'hypercurve' else 1 if job['target'] == 'hypercurve_bezier_fit_offset' else 2, job['repo'], job['target']))

baseline = json.loads((audit / 'shared-parameter-component-overlap-baseline-failures.json').read_text())

def run(job):
    command = [job['binary'], '--test-threads=4', '--color', 'never']
    if 'filter' in job:
        command.append(job['filter'])
    elif job['repo'] == 'hypercurve':
        for name in held_out.get(job['target'], []):
            command += ['--skip', name]
    suffix = job.get('filter', job['repo'] + '-' + job['target'])
    stem = 'shared-parameter-component-overlap-' + mode + '-' + suffix
    log = audit / (stem + '.log')
    start = time.monotonic()
    timed_out = False
    with log.open('w') as output:
        try:
            result = subprocess.run(command, cwd=root / job['repo'], stdout=output, stderr=subprocess.STDOUT, timeout=300)
            code = result.returncode
        except subprocess.TimeoutExpired:
            code = 124
            timed_out = True
    elapsed = time.monotonic() - start
    (audit / (stem + '.exit')).write_text(str(code) + '\n')
    text = log.read_text()
    statuses = re.findall(r'^test ([^\n]+?) \.\.\. (ok|FAILED|ignored[^\n]*)$', text, re.M)
    passed = [name for name, status in statuses if status == 'ok']
    failed = [name for name, status in statuses if status == 'FAILED']
    ignored = [name for name, status in statuses if status.startswith('ignored')]
    summaries = re.findall(r'^test result: .*$', text, re.M)
    expected = baseline.get(job['target'], []) if job['repo'] == 'hypercurve' else []
    record = {**job, 'command': command, 'sha256': hashlib.sha256(Path(job['binary']).read_bytes()).hexdigest(),
              'returncode': code, 'timed_out': timed_out, 'elapsed_seconds': elapsed, 'log': log.name,
              'passed': passed, 'failed': failed, 'ignored': ignored, 'summaries': summaries,
              'new_failures': sorted(set(failed) - set(expected)),
              'known_failures': sorted(set(failed) & set(expected))}
    (audit / (stem + '.json')).write_text(json.dumps(record, indent=2) + '\n')
    print(suffix, 'exit', code, 'passed', len(passed), 'failed', len(failed), 'new', record['new_failures'], 'seconds', round(elapsed, 2), flush=True)
    return record

results = []
with ThreadPoolExecutor(max_workers=1 if mode == 'focused' else 2) as executor:
    futures = [executor.submit(run, job) for job in jobs]
    for future in as_completed(futures):
        results.append(future.result())
(audit / ('shared-parameter-component-overlap-' + mode + '-results.json')).write_text(json.dumps(results, indent=2) + '\n')
print('TOTAL', {key: sum(len(row[key]) for row in results) for key in ['passed', 'failed', 'ignored', 'new_failures']}, flush=True)
