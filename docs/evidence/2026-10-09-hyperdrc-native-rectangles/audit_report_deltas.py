"""Approve enumerated report deltas only after direct native geometry replay.

Usage: python3 audit_report_deltas.py REPORT_OUTPUT
Every other complete byte, including order and identity, must remain unchanged.
"""
from fractions import Fraction
import gzip
import hashlib
import json
from pathlib import Path
import sys

record = Path(__file__).resolve().parent
out = Path(sys.argv[1])
load = lambda path: json.loads(path.read_bytes())
info = lambda path: dict(bytes=path.stat().st_size, sha256=hashlib.sha256(path.read_bytes()).hexdigest())
manifest = load(out / 'manifest.json')
geometry = {side: load(out / f'geometry-probe/{side}.json') for side in ['before','after']}
geometry_manifest = load(out / 'geometry-probe/manifest.json')
for side, version in [('before',16),('after',17)]:
    assert geometry[side]['version'] == version
    provenance = geometry_manifest[side]
    assert provenance['build_exit_code'] == provenance['exit_code'] == 0
    assert provenance['source'] == info(record / 'report_geometry_probe.rs')
    for path, expected in {**provenance['library'], **provenance['dependencies']}.items():
        if Path(path).exists():
            assert info(Path(path)) == expected

fraction = lambda text: sum(map(Fraction, text.split()))
old_pairs, new_pairs = [geometry[side]['mask_pairs'] for side in ['before','after']]
assert [pair['indices'] for pair in old_pairs] == [pair['indices'] for pair in new_pairs] == [[0,1],[17,64]]
assert [pair['squared_distance'] for pair in old_pairs] == ['0','0']
assert [pair['zero_order'] for pair in old_pairs] == ['Equal','Equal']
assert [pair['zero_order'] for pair in new_pairs] == ['Greater','Equal']
gap = Fraction(3, 2**56)
assert fraction(new_pairs[0]['squared_distance']) == gap * gap
assert new_pairs[1]['squared_distance'] == '0'
for pairs in [old_pairs,new_pairs]:
    for pair in pairs:
        assert pair['indices'] == [f['source_index'] for f in pair['features']]
assert old_pairs[0]['features'][0] == new_pairs[0]['features'][0]
assert old_pairs[1]['features'][1] == new_pairs[1]['features'][1]
left, right = new_pairs[0]['features']
lb, rb = [list(map(fraction, f['native_bounds'])) for f in [left,right]]
assert lb[1] - rb[3] == gap
assert max(lb[0], rb[0]) < min(lb[2], rb[2])
# Independent rational reconstruction: import binary64 center/size first,
# then halve/add exactly. These are the three affected native rectangles.
rectangle_inputs = [
    (new_pairs[0]['features'][1], [171.6497,127.1512], [0.254,0.1524]),
    (new_pairs[1]['features'][0], [160.03387,91.20129], [0.8,0.6]),
    (geometry['after']['via_overlaps'][0]['pad'], [146.09444,106.51035], [0.8,0.6]),
]
for feature, center, size in rectangle_inputs:
    c, s = [list(map(Fraction.from_float, values)) for values in [center,size]]
    expected = [c[0]-s[0]/2,c[1]-s[1]/2,c[0]+s[0]/2,c[1]+s[1]/2]
    assert list(map(fraction, feature['native_bounds'])) == expected
    assert fraction(feature['native_area']) == s[0]*s[1]

def spans(text):
    decoder = json.JSONDecoder()
    position = text.index('"violations":[') + len('"violations":[')
    result = []
    while text[position] != ']':
        value, end = decoder.raw_decode(text, position)
        result.append((value,position,end))
        position = end
        if text[position] == ',': position += 1
    return result

overlap_replacements = [
    ('"total_area":0.01713007434955216', '"total_area":0.017130074350461655'),
    ('"area":0.01713007434955216', '"area":0.017130074350461655'),
    ('146.3333248429091,106.76463275830689', '146.33332484290906,106.76463275830689'),
    ('146.4660949554169,106.68125469914561', '146.46609495541688,106.68125469914563'),
]
pair_replacements = [
    ('"id":"f1589a2cb500855a"', '"id":"15a3441dd90f7f91"'),
    ('"evidence_id":"sha256:f3fb8785c260de4a78b3cab95a56aeadc13726fb1a190850d5b4071158c93848"', '"evidence_id":"sha256:8b0ad7caf7fc1c9c4970403ff761249027b39c300ab01937c26cc12424ee7047"'),
    ('[171.5481,127.2274],[171.6497,127.1512]', '[160.03387,91.20129],[160.70632,92.39634]'),
]
reports = {}
for label, count in [('cparti-fpga',2610),('hvp109a',658)]:
    before = gzip.decompress((record / f'baseline-reports/{label}.json.gz').read_bytes())
    candidates = [gzip.decompress((out / f'trial-{trial}-{label}.json.gz').read_bytes()) for trial in [1,2]]
    assert candidates[0] == candidates[1]
    candidate = candidates[0]
    assert before.count(b'"check_version":16') == 12
    versioned = before.replace(b'"check_version":16',b'"check_version":17').decode()
    old_json, new_json = json.loads(before), json.loads(candidate)
    assert old_json['violation_count'] == new_json['violation_count'] == count
    assert old_json['summary'] == new_json['summary']
    replacements, changed = {}, []
    objects = spans(versioned)
    assert len(objects) == count
    if label == 'hvp109a':
        for index, geometry_index in [(208,0),(209,1)]:
            left, right = old_json['violations'][index], new_json['violations'][index]
            assert left['check'] == right['check'] == 'dense-pad-via-spacing-readiness'
            for side, finding in [('before',left),('after',right)]:
                overlap = geometry[side]['via_overlaps'][geometry_index]
                assert overlap['shapes'] == finding['polygons']
                assert overlap['pad']['source_index'] == 335
                assert overlap['via']['source_index'] == 3337 + geometry_index
                assert overlap['layer'] == finding['layers'][1]
            assert geometry['before']['via_overlaps'][geometry_index]['via'] == geometry['after']['via_overlaps'][geometry_index]['via']
            replacements[index] = overlap_replacements
        replacements[211] = pair_replacements
        assert old_json['violations'][211]['check'] == new_json['violations'][211]['check'] == 'dense-pad-mask-bridge-readiness'
    chunks, position = [], 0
    for index, edits in replacements.items():
        value, start, end = objects[index]
        expected = versioned[start:end]
        for before_token, after_token in edits:
            assert expected.count(before_token) == 1, (label,index,before_token)
            expected = expected.replace(before_token,after_token)
        assert json.loads(expected) == new_json['violations'][index]
        changed.append(dict(index=index,check=value['check'],exact_replacements=edits))
        chunks.extend([versioned[position:start],expected])
        position = end
    expected = ''.join(chunks) + versioned[position:]
    assert expected.encode() == candidate, f'Unreviewed report byte: {label}'
    row = manifest['reports'][label]
    assert row['bytes'] == len(candidate) and row['sha256'] == hashlib.sha256(candidate).hexdigest()
    assert row['baseline_only_version_changed'] == (label == 'cparti-fpga')
    reports[label] = dict(version_fields=12,findings=count,changed_findings=changed,
                          candidate_bytes=len(candidate),candidate_sha256=hashlib.sha256(candidate).hexdigest(),
                          all_other_bytes_unchanged=True,candidate_repetitions_identical=True)
result = dict(status='passed',
    scope='Twelve version fields per board. CPArti has no other change. HVP109A has two via-overlap report polygons with four area fields and six coordinate components changed, plus one newly selected exact-touching mask pair with four marker components and both IDs changed. Direct native replay reproduces both old/new polygons. The old mask pair has a certified gap of 3/2^56 mm after rectangle repair, while the new pair remains exactly touching; independent rational reconstruction verifies the three affected rectangles. All counts, diagnostics and every other complete report byte remain unchanged. This is specific correctness evidence, not a general tolerance or universal determinism claim.',
    reports=reports, geometry_replay='geometry-probe/manifest.json',
    independent_rectangle_input_reconstruction=True,
    mask_pair_gap=dict(numerator=3,denominator=2**56,squared_distance=str(gap*gap)),
    geometry_artifacts={f'geometry-probe/{side}.json':info(out/f'geometry-probe/{side}.json') for side in ['before','after']})
(out / 'baseline-delta-audit.json').write_text(json.dumps(result,indent=2)+'\n')
print('Complete report bytes verified: twelve version fields each, two overlap polygons and one mask pair on HVP109A only.')
