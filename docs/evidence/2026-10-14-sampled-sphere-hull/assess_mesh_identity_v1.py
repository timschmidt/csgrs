"""Reparse every fixed sample and report all costs, including regressions."""
import hashlib
import json
from pathlib import Path
from statistics import median

E = Path(__file__).resolve().parent
OUT = E / 'mesh-identity-costs-v1'
FIELDS = ['allocation_calls', 'allocated_bytes', 'retained_bytes_delta', 'peak_bytes_delta']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    destination = E / 'mesh-identity-cost-assessment-v1.json'
    assert not destination.exists()
    result = json.loads((OUT / 'result.json').read_text())
    assert result['complete'] and result['commands_passed'] == 35
    assert all(sha(OUT / p) == h for p, h in result['artifacts_sha256'].items())
    commands = json.loads((OUT / 'commands.json').read_text())
    assert len(commands) == 35 and all(c['exit_code'] == 0 for c in commands)
    protocol = json.loads((OUT / 'protocol.json').read_text())
    trials = json.loads((OUT / 'trials.json').read_text())
    witnesses = {}
    for trial in trials:
        raw = [json.loads(line) for line in (OUT / (trial['name'] + '.stdout')).read_text().splitlines()]
        assert [r for r in raw if 'name' in r] == trial['rows']
        assert [r for r in raw if 'witness' in r] == trial['witnesses']
        assert trial['witnesses'] == witnesses.setdefault(trial['count'], trial['witnesses'])
        assert trial['compatibility']['positions_get_mut'] == (trial['group'] == 'baseline')
        assert trial['compatibility']['triangles_get_mut'] == (trial['group'] == 'baseline')
        assert trial['compatibility']['mesh_bytes'] == 40
    samples = [t for t in trials if not t['preflight']]
    assert len(samples) == 24 and len(trials) == 28
    rows = []
    for phase in protocol['phases']:
        time_groups = {}
        memory_groups = {}
        for group in ['baseline', 'candidate']:
            for mode, container in [('time', time_groups), ('memory', memory_groups)]:
                selected = sorted([t for t in samples if t['mode'] == mode and t['group'] == group], key=lambda t: t['round'])
                assert [t['round'] for t in selected] == list(range(1, 7))
                container[group] = [next(r for r in t['rows'] if r['name'] == phase) for t in selected]
        ratios = [c['nanos'] / b['nanos'] for b, c in zip(time_groups['baseline'], time_groups['candidate'])]
        memories = {g: {f: sorted({r[f] for r in data}) for f in FIELDS} for g, data in memory_groups.items()}
        repeatable = all(len(v) == 1 for g in memories.values() for v in g.values())
        delta = {f: memories['candidate'][f][0] - memories['baseline'][f][0] for f in FIELDS} if repeatable else None
        rows.append({'phase': phase, 'iterations_time': time_groups['baseline'][0]['iterations'],
            'baseline_nanos': [r['nanos'] for r in time_groups['baseline']],
            'candidate_nanos': [r['nanos'] for r in time_groups['candidate']],
            'paired_candidate_over_baseline': ratios, 'median_paired_ratio': median(ratios),
            'candidate_slower_pairs': sum(v > 1 for v in ratios),
            'iterations_memory': memory_groups['baseline'][0]['iterations'],
            'memory_observed_values': memories, 'all_memory_fields_repeat': repeatable,
            'candidate_minus_baseline_memory': delta})
    sizes = {}
    binaries = json.loads((OUT / 'binaries.json').read_text())
    for group in ['baseline', 'candidate']:
        sections = {}
        for line in (OUT / ('size-' + group + '.stdout')).read_text().splitlines():
            fields = line.split()
            if len(fields) == 3 and fields[0].startswith('.'):
                sections[fields[0]] = int(fields[1])
        assert '.text' in sections
        sizes[group] = {'binary_bytes': binaries[group + '-time']['bytes'], 'sections': sections}
    assessment = {'cost_result_sha256': sha(OUT / 'result.json'), 'assessor_sha256': sha(__file__),
        'complete': True, 'commands_passed': 35, 'fixed_samples': 24, 'preflights': 4,
        'all_raw_rows_reparsed': True, 'all_exact_final_witnesses_match': True,
        'all_phases': rows, 'native_uninstrumented_sizes': sizes,
        'raw_binary_delta_bytes': sizes['candidate']['binary_bytes'] - sizes['baseline']['binary_bytes'],
        'text_delta_bytes': sizes['candidate']['sections']['.text'] - sizes['baseline']['sections']['.text'],
        'compatibility': 'All baseline fresh cube buffers allow Arc::get_mut; all candidate fresh buffers return None. Public docs prescribe Arc::make_mut.',
        'interpretation_limits': ['Six paired observations are descriptive, not confidence intervals or significance tests.',
            'Requested allocator bytes are not RSS, usable sizes, or whole-process peaks.',
            'The final exact output per repeated fixed-geometry phase is witnessed; this does not claim every intermediate is serialized.',
            'Safe edit phases measure allocation and release without reusing the known-broken baseline cached queries.',
            'No original sphere hull or native DRC performance gate is improved by these controls.'],
        'runtime_admitted': False, 'plan_complete': False}
    destination.write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({'phases': [{k: r[k] for k in ['phase', 'median_paired_ratio', 'candidate_slower_pairs',
        'candidate_minus_baseline_memory']} for r in rows], 'binary_delta': assessment['raw_binary_delta_bytes'],
        'text_delta': assessment['text_delta_bytes'], 'assessment_sha256': sha(destination)}))


if __name__ == '__main__':
    main()
