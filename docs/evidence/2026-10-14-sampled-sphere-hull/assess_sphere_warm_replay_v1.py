"""Close the fixed default-allocator replay diagnostic without cost claims."""
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
TRIAL = E / 'sphere-warm-replay-v1'
OUT = E / 'sphere-warm-replay-assessment-v1.json'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    assert not OUT.exists()
    result = json.loads((TRIAL / 'result.json').read_text())
    assert result['complete'] and result['commands_all_zero'] == 13
    assert result['all_phases_match_prior_exact_support_proofs']
    assert all(sha(TRIAL / name) == digest for name, digest in result['artifacts_sha256'].items())
    protocol = json.loads((TRIAL / 'protocol.json').read_text())
    assert all(sha(E / name) == digest for name, digest in protocol['inputs_sha256'].items())
    assert all(sha(row['path']) == row['sha256'] for libraries in protocol['libraries'].values() for row in libraries.values())
    binaries = json.loads((TRIAL / 'binaries.json').read_text())
    assert all(sha(row['path']) == row['sha256'] for row in binaries.values())
    root_mapping = json.loads(gzip.decompress((TRIAL / 'root-source-hashes.json.gz').read_bytes()))
    assert len(root_mapping) == 10129
    assert all(sha(ROOT / name) == digest for name, digest in root_mapping.items())
    for group, preparation in [('baseline', 'mesh-identity-preparation-v1'),
                               ('late', 'sphere-cyclotomic-v2'), ('image', 'sphere-cyclotomic-image-v1')]:
        mapping = json.loads(gzip.decompress((E / preparation / 'source-hashes.json.gz').read_bytes()))
        assert all(sha(Path(protocol['sources'][group]) / name) == digest for name, digest in mapping.items())
    commands = json.loads((TRIAL / 'commands.json').read_text())
    assert len(commands) == 13 and all(row['exit_code'] == 0 for row in commands)
    assert all(sha(TRIAL / (row['name'] + '.' + stream)) == row[stream + '_sha256']
               for row in commands for stream in ['stdout', 'stderr'])
    spec = importlib.util.spec_from_file_location('replay_parser', E / 'probe_sphere_warm_replay_v1.py')
    parser = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(parser)
    observations = json.loads((TRIAL / 'observations.json').read_text())
    expected = [(group, case) for group, cases in parser.GROUPS.items() for case in cases]
    assert [(row['group'], row['case']) for row in observations] == expected
    for row in observations:
        group, case = row['group'], row['case']
        changed = case == 'original_medium'
        assert row['cold_equals_warm'] == (not changed) and row['warm_equals_replay']
        assert row['oriented_faces_removed_from_cold'] == row['oriented_faces_added_to_warm'] == (118 if changed else 0)
        for phase in ['cold', 'warm', 'replay']:
            path = TRIAL / (group + '-' + case + '-' + phase + '.witness')
            assert sha(path) == row['witnesses_sha256'][phase]
            parser.parse(path.read_text(), case)
            reference = (E / 'sphere-warm-row-counterexample-v1/warm.stdout' if changed and phase != 'cold'
                         else E / f'sphere-cyclotomic-v2/candidate-{case}.stdout')
            assert path.read_bytes() == reference.read_bytes()
    assessment = {'complete': True, 'driver_terminal_exit_code': 0, 'commands_all_zero': 13,
        'native_processes': 9, 'hull_queries': 27,
        'result_sha256': sha(TRIAL / 'result.json'), 'assessor_sha256': sha(__file__),
        'all_frozen_sources_inputs_binaries_and_libraries_rechecked': True,
        'all_phase_outputs_match_prior_independent_exact_support_proofs': True,
        'changed_cases': [{'group': group, 'case': 'original_medium', 'changed_triangles': 118,
            'changed_supporting_planes': 59} for group in ['late', 'image']],
        'other_seven_processes_have_identical_cold_warm_replay_outputs': True,
        'interpretation': 'The same 32x16 triangulation instability occurs with the default allocator in both certificate candidates. Allocation instrumentation and the finite-field filter are not necessary for this counterexample. The exact geometric set is preserved; its serialized rows differ.',
        'not_a_cost_trial': True, 'resource_trial_resumed': False,
        'runtime_admitted': False, 'plan_complete': False}
    OUT.write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({'complete': True, 'assessment_sha256': sha(OUT)}))


if __name__ == '__main__':
    main()
