#!/usr/bin/env python3
"""Retain the scoped revised optimization without discarding unfavorable trials."""
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    implementation = read(E/'integer-scale-implementation-v1367.json')
    paired = read(E/implementation['paired_cost'])
    first = read(E/'integer-scale-implementation-v1314.json')
    out = E/'integer-scale-retention-v1367.json'
    assert not out.exists()
    assert first['status'].endswith('not retained')
    assert paired['processes'] == 576 and paired['guards_unchanged']
    assert implementation['preexisting_tests_unchanged'] and implementation['all_five_corrected_baseline_tests_and_caps_unchanged']
    assert sum(r['cases'] for r in implementation['replays'].values()) == 1584
    assert all(r['receipts_identical_except_commands'] for r in implementation['replays'].values())
    binary = Path(paired['provenance']['integer']['binaries']['candidate'])
    assert sha(binary) == paired['provenance']['integer']['binary_sha256_at_paired_run']['candidate']
    review = E/'integer-scale-direct-call-review-v1367'
    assert not review.exists()
    review.mkdir()
    names = subprocess.check_output(['nm', '-S', '-C', str(binary)], text=True)
    lines = [r for r in names.splitlines() if r.endswith('hyperreal::computable::node::scale_at_precision_ref')]
    assert len(lines) == 1
    address, size = [int(s, 16) for s in lines[0].split()[:2]]
    command = ['objdump', '-d', '-C', f'--start-address={address}', f'--stop-address={address+size}', str(binary)]
    assembly = subprocess.check_output(command, text=True)
    assert '<hyperreal::computable::node::scale_at_precision>' not in assembly
    (review/'symbols.stdout').write_text(names)
    (review/'helper.stdout').write_text(assembly)
    (review/'record.json').write_text(json.dumps({'binary': str(binary), 'sha256': sha(binary),
        'command': command, 'nested_owned_scale_call_absent': True,
        'scope': 'Static confirmation that the additional nested owned-scaling call was removed; not proof that this alone caused every timing change.'}, indent=2)+'\n')
    integer = {r['arguments'][0]: round(r['median_percent_change'], 2) for r in paired['cases']['integer']}
    ordinary = [r['median_percent_change'] for k, rows in paired['cases'].items() if k != 'integer' for r in rows]
    sizes = {k: round(100*(r['candidate']/r['baseline']-1), 4) for k, r in implementation['executable_sizes'].items()}
    out.write_text(json.dumps({
        'decision': 'Retain the revised optimization in the isolated candidate; broad checkpoint pending; no main-runtime admission',
        'source_record_sha256': implementation['source_record_sha256'],
        'implementation_record': 'integer-scale-implementation-v1367.json',
        'baseline_commit': 'a4e9efe58', 'prior_unretained_attempt_commit': '47a48785e',
        'prior_unretained_attempt': 'integer-scale-implementation-v1314.json',
        'runtime_scope': 'Six private implementation/facade files share rounded right scaling. Existing fine clone/left-shift arithmetic runs directly without an extra nested scale call.',
        'correctness': 'All five corrected frozen regressions and all earlier tests are unchanged. 1054 release scalar unit tests plus one child pass, strict five-crate Clippy passes, and all 1584 focused replay receipts agree except executable commands.',
        'allocation': 'Across 64 observations, public one-unit coarse queries request 40-56 Rust bytes and raw dispatch 0-16, independent of 65/4097/65537/1000001-bit input width. Million-bit baseline queries copied at least 125008 bytes before public scope overhead. Fixtures and independent oracles are excluded.',
        'integer_paired_median_percent_changes': integer,
        'ordinary_24_fixture_paired_percent_range': [min(ordinary), max(ordinary)],
        'standalone_probe_size_percent_changes': sizes,
        'first_attempt': 'The first borrowed helper nested an owned helper call for fine/identity queries. It was not retained after paired +9.65% small_identity, +13.46% atan_wide and +14.28% pi_wide observations. Both original and alternating trials remain immutable.',
        'revised_cost': 'The revised paired small_identity observation is +0.29%, atan_wide -3.34% and pi_wide -11.15%. Wide coarse/gap/halfway medians improve 97.28%/99.54%/54.73%. Small fine is +3.91%; individual paired trials span both signs. The evidence supports this scoped allocation/coarse-cost optimization, not a uniform speedup or absence of ordinary overhead.',
        'unfavorable_trials_retained': 'Separate revised trials include +28.58% terms_wide, +24.42% pi_small and +21.08% atan_wide; their alternating medians are -1.38%, -0.01% and -3.34%. No samples were removed, and all original trials/checksums are retained.',
        'static_call_record': str((review/'record.json').relative_to(E)),
        'scope_limits': ['Local micro-workloads and Rust allocation requests, not full-board performance, native allocations, RSS or peak-live memory.',
            'Negative right shifts can still scan discarded bits. EvaluationBudget promises cooperative work units, not per-operation bytes/time.',
            'Fresh full positive/negative proof checks and broader regression/replay checkpoint remain required; changed scaling code is outside the conditional proof target.',
            'All 39 properties, 14 targets, crate future work and the original 49 worktree entries remain in scope.']
    }, indent=2)+'\n')
    print(json.dumps({'record': out.name, 'integer_paired_changes': integer, 'ordinary_range': [min(ordinary), max(ordinary)], 'size_changes': sizes}))


if __name__ == '__main__':
    main()
