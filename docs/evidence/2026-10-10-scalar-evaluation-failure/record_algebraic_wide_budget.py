#!/usr/bin/env python3
"""Record wide public-budget coverage that bypasses the earlier square-root shortcut."""
from collections import Counter
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
P = E / 'typed-api-prototype-v549-algebraic-wide-budget-replay'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
EXPECTED = 'd6527953cefc76fb9c5f914434459b74e5949b08a22c755a5b83684a1b4b7deb'


def main():
    summary = read(P / 'summary.json')
    assert summary['source_hashes_sha256'] == EXPECTED and summary['guards_unchanged']
    assert summary['cases'] == summary['zero_budget_retained_proofs'] == 144
    assert summary['numeric_observations'] == 288 and summary['incomplete_cases'] == summary['invalid_samples'] == 0
    for name, value in summary['artifacts_sha256'].items():
        assert sha(P / name) == value
    old_probe = (E / 'algebraic_budget_probe.rs').read_text()
    assert old_probe.count('-16384') == 6
    assert old_probe.replace('-16384', '-65536') == (P / 'probe.rs').read_text()
    assert sha(P / 'probe.rs') == sha(E / 'algebraic_wide_budget_probe.rs')
    assert sha(P / 'reproduce.py') == sha(E / 'reproduce_algebraic_wide_budget.py')
    rows = read(P / 'receipts.json')
    assert len(rows) == 144
    identities = []
    for row in rows:
        assert row['complete'] and row['command']['exit_code'] == 0 and len(row['records']) == 1
        r = row['records'][0]
        assert r['degree'] in [3,4] and r['factors'] == 2 and r['coefficient_bits'] in [64,256,1024]
        assert r['before_precision'] == -65536 and r['after_precision'] == -8
        assert r['consumed'] + r['remaining'] == r['allowance'] and r['zero_budget_proof_retained']
        if r['perturbation'] == 0 and r['allowance'] == 1_000_000:
            assert r['consumed'] > 4
            identities.append({k:r[k] for k in ['degree','factors','coefficient_bits','inverse','answer','consumed','remaining']})
    assert len(identities) == 12
    for degree in [3,4]:
        for bits in [64,256,1024]:
            for inverse in [False,True]:
                assert sum(r['degree']==degree and r['coefficient_bits']==bits and r['inverse']==inverse for r in identities)==1
    old_rows = read(E/'typed-api-prototype-v523-algebraic-budget-replay/receipts.json')
    earlier = [row['records'][0] for row in old_rows if row['records'][0]['degree']==2 and row['records'][0]['coefficient_bits']>0 and row['records'][0]['perturbation']==0 and row['records'][0]['allowance']==1_000_000]
    assert len(earlier)==6 and all(r['answer']=='Zero' and r['consumed']==4 for r in earlier)
    out = E / 'algebraic-wide-budget-coverage-v549.json'
    assert not out.exists()
    record = {'phase':P.name,'source_record_sha256':EXPECTED,'cases':144,'numeric_observations':288,'zero_budget_retained_proofs':144,
        'reason':'Existing wide square-root identities are resolved by earlier normal-form proof work and consume four shared units. Add cube/fourth-root inputs to exercise public-budget algebraic metadata and coefficient charges. All 288 original algebraic fixtures and all 2929 older replay fixtures remain unchanged.',
        'degrees':[3,4],'coefficient_bits':[64,256,1024],'factors':2,'reciprocal_modes':[False,True],
        'perturbations':['-2^-96','0','2^-96'],'allowances':[0,8,256,1000000],'numerical_precision':-65536,
        'precision_reason':'Wide radicand products can hit the existing 4096-bit dependency-search cap. Raising precision retains the conservative larger field-degree zero certificate instead of reducing coefficient widths.',
        'generous_zero_identity_observations':identities, 'generous_answers':dict(Counter(r['answer'] or 'Unknown' for r in identities)),
        'probe_delta_from_original':'Only the six -16384 precision constants become -65536; exact root construction, perturbation oracles, retry and retained-proof assertions are unchanged.',
        'prior_square_root_work_units':4,'summary_sha256':sha(P/'summary.json'),'probe_sha256':sha(P/'probe.rs'),'harness_sha256':sha(P/'reproduce.py'),
        'limits':'Finite public/serialized replay with numeric warmup outside the explicit allowance. Rational oracles check approximations and every definite sign; unlimited retry and zero-budget proof retention are required. It is not an allocation/time proof or a complete accounting/concurrency matrix.'}
    out.write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps({'cases':144,'numeric_observations':288,'generous_answers':record['generous_answers'],'identity_work_range':[min(r['consumed'] for r in identities),max(r['consumed'] for r in identities)]}))


if __name__ == '__main__':
    main()
