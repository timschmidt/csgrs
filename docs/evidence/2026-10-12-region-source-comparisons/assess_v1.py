"""Verify the finished diagnostic and decide whether to run a separate trial."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'diagnostic-v1'
PARENT = ROOT / 'docs/evidence/2026-10-12-small-integer-certificates'
SOURCE = ROOT / 'target/region-source-comparisons/diagnostic-v1/source'
BASE = ROOT / 'target/small-integer-certificates/source'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    result = json.loads((OUT / 'result.json').read_text())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    identity = json.loads((OUT / 'binary.json').read_text())
    rows = json.loads((OUT / 'counters.json').read_text())
    assert result['complete'] and not result['runtime_admitted']
    assert sha(E / 'diagnose_v1.py') == protocol['runner_sha256']
    assert sha(Path(identity['path'])) == identity['sha256']
    assert sha(OUT / 'instrumentation.patch') == protocol['instrumentation_patch_sha256']
    assert sha(OUT / 'instrumentation.tar.gz') == protocol['instrumentation_archive_sha256']
    original = json.loads(gzip.decompress((PARENT / 'baseline-source-hashes.json.gz').read_bytes()))
    baseline = original | json.loads((PARENT / 'validation-v1/protocol.json').read_text())['source_changes_sha256']
    assert all(sha(BASE / name) == digest for name, digest in baseline.items())
    current = baseline | protocol['only_instrumented_sources']
    assert all(sha(SOURCE / name) == digest for name, digest in current.items())
    assert sources() == json.loads((PARENT / 'preparation.json').read_text())['main_source_sha256']
    for phase in result['phases']:
        assert phase['sources_unchanged']
        assert phase['exit_code'] == (101 if phase['name'] == 'full-board' else 0)
        for stream in ['stdout', 'stderr']:
            assert sha(OUT / (phase['name'] + '.' + stream)) == phase[stream + '_sha256']
    stderr = (OUT / 'full-board.stderr').read_text()
    worker = float(re.search(r'KiCad smoke suite took ([0-9.]+)s of worker time', stderr)[1])
    counts = [int(value) for value in re.findall(r'finished run \((\d+) active finding', stderr)]
    assert counts == [12, 197]
    assert stderr.count('finished emit report') == 2
    assert worker > protocol['unchanged_worker_limit_seconds'] == 120
    observed = sum(row['calls'] for row in rows)
    nanos = sum(row['geometry_nanos'] for row in rows)
    mismatch = [row for row in rows if row['metadata_kind'] in ['left_absent', 'right_absent', 'unequal']]
    mismatch_calls = sum(row['calls'] for row in mismatch)
    mismatch_nanos = sum(row['geometry_nanos'] for row in mismatch)
    assert observed == result['observed_comparisons'] == 34056513
    assessment = {
        'decision': 'advance-isolated-metadata-first-trial',
        'goal_turn_classification': 'progress',
        'parent': 'Frozen integer-leaf candidate; rejected coefficient merge absent.',
        'observed_comparisons': observed,
        'metadata_mismatch_comparisons': mismatch_calls,
        'metadata_mismatch_call_percent': mismatch_calls * 100 / observed,
        'geometry_query_seconds': nanos / 1e9,
        'metadata_mismatch_geometry_seconds': mismatch_nanos / 1e9,
        'metadata_mismatch_geometry_percent': mismatch_nanos * 100 / nanos,
        'rows': rows,
        'diagnostic_validation': 'Two new diagnostic regressions, three existing provenance/duplicate regressions, strict release all-feature/all-target curve Clippy and workspace formatting pass. These counts overlap no full-suite execution: no full curve suite ran here.',
        'original_full_board_worker_cpu_seconds': worker,
        'original_worker_cpu_limit_seconds': 120,
        'full_board_passed': False,
        'observed_finding_counts': counts,
        'complete_report_identity_checked': False,
        'limits': [
            'Counters observe only exiting threads. Geometry timers and counters perturb execution; this is neither a matched performance comparison nor a predicted saving.',
            'Original geometry queries and conditional metadata comparisons remain. Four equal-geometry pairs had unequal source metadata; their final duplicate result stayed false.',
            'Checking metadata first avoids queries and their incidental warming. Later facts, outcomes, reports, allocation/retention and original caps require separate checks.',
            'The two original board fixtures ran to completion, but the unchanged CPU assertion failed. Finding counts do not establish complete report identity.',
        ],
        'next_step': 'Fork the integer-leaf parent into separate source/target directories; swap only the two existing conjunction operands. Cover absent/equal/different provenance, shared/independently equal geometry, error ordering, reused arrangement indices, retained cache facts and clones. Run complete curve validation, original native caps, complete report replay and measured cost/memory/size controls before any admission.',
        'artifacts_sha256': {str(p.relative_to(E)): sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
        'main_runtime_unchanged': True,
        'runtime_admitted': False,
        'plan_complete': False,
    }
    (E / 'assessment-v1.json').write_text(json.dumps(assessment, indent=2) + '\n')
    status_path = ROOT / 'docs/properties-status.json'
    status = json.loads(status_path.read_text())
    assert len(status['properties']) == 39 and len(status['targets']) == 14
    work = status['current_work']
    work['isolated_region_source_comparisons'] = {
        'status': assessment['decision'],
        'evidence': str((E / 'assessment-v1.json').relative_to(ROOT)),
        'result': f'{mismatch_calls} of {observed} observed comparisons have unequal source metadata, accounting for {mismatch_nanos / 1e9:.9f} of {nanos / 1e9:.9f} instrumented geometry seconds. Four unequal-source pairs have equal geometry. These observations support a separate trial, not a speedup or admission claim.',
        'validation': assessment['diagnostic_validation'],
        'original_full_board': f'{worker} worker CPU seconds / original 120; failed. Both boards completed with 12 and 197 findings. Complete report identity is not checked here.',
        'next_step': assessment['next_step'],
        'runtime_admitted': False,
    }
    work['pending_native_geometry']['next_step'] = assessment['next_step']
    status_path.write_text(json.dumps(status, indent=2) + '\n')
    print(json.dumps({key: assessment[key] for key in [
        'decision', 'observed_comparisons', 'metadata_mismatch_comparisons',
        'geometry_query_seconds', 'metadata_mismatch_geometry_seconds',
        'metadata_mismatch_geometry_percent', 'original_full_board_worker_cpu_seconds']}))


if __name__ == '__main__':
    main()
