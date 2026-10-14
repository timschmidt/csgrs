"""Integrity and fixed ordering for the three-group borrowed-integer cost trial."""
import gzip
import json
from pathlib import Path

from common_v1 import E, ROOT, PARENT, PREP, WORK, ENV, sha, dump, compressed, verify

COST_PREP = E / 'cost-preparation-v1'
COST_WORK = WORK / 'cost-v1'
ORDERS = [
    ['main', 'full-cache', 'candidate'],
    ['full-cache', 'candidate', 'main'],
    ['candidate', 'main', 'full-cache'],
    ['main', 'candidate', 'full-cache'],
    ['candidate', 'full-cache', 'main'],
    ['full-cache', 'main', 'candidate'],
]


def verify_qualification(full=False):
    base = json.loads((PREP / 'protocol.json').read_text())
    verify(base, full=full)
    out = E / 'qualification-v1'
    result = json.loads((out / 'result.json').read_text())
    assert result['complete'] and result['scalar_tests'] == 876
    assert all(sha(out / p) == h for p, h in result['artifact_sha256'].items())
    binaries = json.loads((out / 'binaries.json').read_text())
    assert all(sha(r['path']) == r['sha256'] for r in binaries.values())
    library = json.loads((out / 'library.json').read_text())
    assert sha(library['path']) == library['sha256']
    assessment = json.loads((E / 'qualification-assessment-v1.json').read_text())
    assert assessment['result_sha256'] == sha(out / 'result.json')
    assert assessment['assessor_sha256'] == sha(E / 'assess_qualification_v1.py')
    assert assessment['comparisons_sha256'] == sha(E / 'integer-allocation-comparisons-v1.json')
    assert assessment['status'] == 'signed-word-fixture-allocation-repaired-broader-costs-pending'
    return base


def verify_costs(prep, full=False):
    base = verify_qualification(full=full)
    assert all(sha(E / p) == h for p, h in prep['inputs_sha256'].items())
    assert all(sha(ROOT / p) == h for p, h in prep['prior_inputs_sha256'].items())
    assert sha(COST_PREP / 'metadata-source.patch.gz') == prep['metadata_patch_sha256']
    assert sha(COST_PREP / 'metadata-source-delta.tar.gz') == prep['metadata_archive_sha256']
    for label, variant in prep['variants'].items():
        path = COST_PREP / (label + '-source-hashes.json.gz')
        assert sha(path) == variant['source_hashes_sha256']
        mapping = json.loads(gzip.decompress(path.read_bytes()))
        selected = mapping if full else {p: h for p, h in mapping.items()
                                         if p in base['main_source_sha256'] or p in variant['delta_sha256']}
        source = Path(variant['source'])
        assert all(sha(source / p) == h for p, h in selected.items()), label
        if full:
            assert {str(p.relative_to(source)) for p in source.rglob('*') if p.is_file()} == set(mapping)
    for libraries in prep['reused_libraries'].values():
        assert all(sha(r['path']) == r['sha256'] for r in libraries.values())
    assert all(sha(r['path']) == r['sha256'] for r in prep['reused_binaries'].values())
    assert all(sha(ROOT / r['path']) == r['sha256'] for r in prep['probes'].values())
