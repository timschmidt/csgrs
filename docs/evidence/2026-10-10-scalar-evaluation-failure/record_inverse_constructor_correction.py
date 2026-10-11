#!/usr/bin/env python3
"""Record the P4 correction to new opaque-offset test expectations."""
import difflib
import gzip
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    out = E/'inverse-constructor-corrected-baseline-v1232.json'
    assert not out.exists()
    record = E/'typed-api-prototype-v1221-inverse-constructor-corrected-baseline-build/source-hashes.json.gz'
    before = json.loads(gzip.decompress((E/'typed-api-prototype-v1206-inverse-constructor-baseline-build/source-hashes.json.gz').read_bytes()))
    after = json.loads(gzip.decompress(record.read_bytes()))
    test = 'crates/hyperreal/src/computable/node/inverse_constructor_preflight_tests.rs'
    assert {n for n,v in after.items() if before.get(n) != v} == {test}
    for n,v in after.items(): assert sha(ROOT/'target/scalar-evaluation-candidate/source'/n) == v,n
    old = (E/'typed-api-prototype-v1206-inverse-constructor-baseline-build/inputs'/test).read_text()
    new = (record.parent/'inputs'/test).read_text()
    marker = '    #[test]\n    fn inverse_constructor_opaque_retries_preserve_completed_proofs()'
    assert old.split(marker)[0] == new.split(marker)[0]
    caps = read(E/'inverse-constructor-baseline-caps-v1232.json')
    assert caps['source_record_sha256'] == sha(record)
    codes=[]
    for phase,row in caps['tests'].items():
        assert row['result'] == read(E/phase/'result.json')
        assert row['result']['source_unchanged'] and row['result']['main_unchanged']
        assert sha(E/phase/'source-hashes.json.gz') == sha(record)
        assert row['cap']['address_space_bytes'] == 192*1024*1024
        assert row['cap']['cpu_seconds_soft_hard'] == [2,3] and row['cap']['wall_seconds_limit'] == 4
        assert not row['cap']['wall_timeout']
        codes.append(row['result']['exit_code'])
    assert codes == [101,101,0,0,0,0,0,101,0,0,0]
    patch = E/'inverse-constructor-test-correction-v1232.patch'
    assert not patch.exists()
    patch.write_text(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='v1206/'+test,tofile='v1221/'+test)))
    out.write_text(json.dumps({
        'status':'Corrected baseline before runtime edits; three measured allocation failures and eight passing guards',
        'baseline_commit':'e30d8e06c','previous_record':'inverse-constructor-baseline-v1216.json',
        'source_record':str(record.relative_to(E)),'source_record_sha256':sha(record),
        'test_correction_patch':patch.name,'patch_sha256':sha(patch),
        'unchanged_allocation_assertions_and_first_eight_test_bodies':True,
        'baseline_caps':'inverse-constructor-baseline-caps-v1232.json','codes':codes,
        'diagnostics':{
            'v1218':'First diagnostic itself incorrectly required a first funded query and a subsequent unlimited query to have equal certainty. It failed after observing finite=false, first=None, subsequent=Positive. The failed source and receipt remain archived.',
            'v1220':'All twelve diagnostic schedules pass after checking soundness rather than equality of certainty. For all six ln(2) schedules, both cold unlimited and cold funded queries return Unknown; a second funded query certifies the correct sign. All six sqrt(2) schedules certify on the first query.',
            'correction':'The two new log-offset tests now permit a first query to warm child facts, then require the next query to certify. Every allowance through the measured two-query cost plus two checks answer soundness, work conservation, unlimited retry and zero-work retention. The sqrt(2) cases keep their original exact first-query cutpoint oracle. Numeric-cache emptiness is required only for sqrt(2), since the cold log path may legitimately evaluate coarse samples.'},
        'coverage':{'corrected_opaque_retry_schedules':12,'opaque_cutpoint_schedules':12,'domain_warming_schedules':12},
        'limits':['No production runtime source changed.','No existing pre-v1195 test changed.','All frozen phase artifacts and earlier failures are preserved.','All 39 properties, 14 targets and crate future work remain open to their previous scope.']
    },indent=2)+'\n')
    print(json.dumps({'record':out.name,'codes':codes}))


if __name__ == '__main__':
    main()
