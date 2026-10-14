"""Preserve qualified sources and compare three groups on the original scalar probes."""
import json
from pathlib import Path

from eligibility_common_v1 import E, ROOT, PARENT, WORK, PREP, ENV, sha, dump, compressed, verify

COST_WORK = WORK / 'scalar-costs-v1'
COST_PREP = E / 'scalar-cost-preparation-v1'
GROUPS = ['main', 'borrowed', 'candidate']
SUITES = ['rational', 'quadratic', 'integer']
ORDERS = [
    ['main', 'borrowed', 'candidate'],
    ['borrowed', 'candidate', 'main'],
    ['candidate', 'main', 'borrowed'],
    ['main', 'candidate', 'borrowed'],
    ['candidate', 'borrowed', 'main'],
    ['borrowed', 'main', 'candidate'],
]
INPUTS = ['eligibility_cost_common_v1.py', 'prepare_scalar_costs_v1.py', 'measure_scalar_costs_v1.py',
          'assess_scalar_costs_v1.py', 'scalar_cost_rows_v1.py', 'scalar-cost-scope-v1.json']


def verify_qualification(full=False):
    prep = json.loads((PREP / 'protocol.json').read_text())
    verify(prep, full=full)
    out = E / 'qualification-v1'
    result = json.loads((out / 'result.json').read_text())
    assert result['complete'] and result['all_features_scalar_tests'] == 879
    assert all(sha(out / p) == h for p, h in result['artifact_sha256'].items())
    binaries = json.loads((out / 'binaries.json').read_text())
    library = json.loads((out / 'library.json').read_text())
    assert all(sha(row['path']) == row['sha256'] for row in [*binaries.values(), library])
    assessment = json.loads((E / 'qualification-assessment-v1.json').read_text())
    assert assessment['status'] == 'lookup-eligibility-qualified-costs-pending'
    assert assessment['result_sha256'] == sha(out / 'result.json')
    assert assessment['assessor_sha256'] == sha(E / 'assess_qualification_v1.py')
    return prep


def verify_costs(prep, full=False):
    verify_qualification(full=full)
    assert all(sha(E / p) == h for p, h in prep['inputs_sha256'].items())
    assert all(sha(ROOT / p) == h for p, h in prep['prior_inputs_sha256'].items())
    assert all(sha(ROOT / row['path']) == row['sha256'] for row in prep['probes'].values())
    assert sha(E / 'scalar_cost_rows_v1.py') == sha(PARENT / 'cost_rows_v1.py')
    assert all(sha(row['path']) == row['sha256'] for row in prep['reused_binaries'].values())
    assert all(sha(row['path']) == row['sha256'] for group in prep['reused_libraries'].values() for row in group.values())
