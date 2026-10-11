#!/usr/bin/env python3
"""Retain each frozen inverse-trig DAG regression under the original caps."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda p:json.loads(p.read_text())
sha = lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    build = E/'typed-api-prototype-v928-inverse-dag-regression-build'
    r = read(build/'result.json')
    assert r['exit_code'] == 0 and r['source_unchanged'] and r['main_unchanged']
    paths = re.findall(r'Executable unittests src/lib\.rs \(([^)]+)\)',(build/'stderr.log').read_text())
    assert len(paths) == 1
    binary = Path(paths[0]); binary_hash = sha(binary)
    cases = [
        ('ordinary_atan_linear_preserves_coefficient','resource'),
        ('ordinary_atan_terms_preserves_coefficient','resource'),
        ('ordinary_acos_linear_preserves_coefficient','resource'),
        ('different_paths_preserve_coefficient','resource'),
        ('depth_limit_is_path_specific','pass'),
        ('shared_work_finishes_in_ten_thousand_units','assertion'),
        ('every_cutpoint_discards_partial_forms','pass'),
        ('unshared_reconstruction_has_the_same_form','pass'),
        ('shared_undefined_cancellation_has_no_certificate','pass'),
    ]
    rows = {}
    for version,(test,expected) in enumerate(cases,start=929):
        phase = f'typed-api-prototype-v{version}-inverse-dag-regression-cap'
        assert not (E/phase).exists()
        selected = 'computable::node::inverse_trig_dag_tests::inverse_trig_dag_'+test
        subprocess.run([sys.executable,str(E/'validate_candidate_v2.py'),phase,'--exec',
            sys.executable,str(E/'run_limited_unit_test.py'),str(binary),binary_hash,
            selected,'--exact','--nocapture','--test-threads=1'],cwd=ROOT,check=True)
        r = read(E/phase/'result.json')
        assert r['source_unchanged'] and r['main_unchanged']
        assert sha(E/phase/'source-hashes.json.gz') == sha(build/'source-hashes.json.gz')
        cap = json.loads((E/phase/'stdout.log').read_text().splitlines()[-1])
        assert cap['address_space_bytes'] == 192*1024*1024 and cap['cpu_seconds_soft_hard'] == [2,3]
        assert cap['wall_seconds_limit'] == 4 and cap['binary_sha256'] == binary_hash
        if expected == 'resource':assert r['exit_code'] in [124,152],(phase,r)
        elif expected == 'assertion':assert r['exit_code'] == 101,(phase,r)
        else:
            assert r['exit_code'] == 0,(phase,r)
            assert '1 passed; 0 failed;' in (E/phase/'stdout.log').read_text()
        rows[phase] = {'test':test,'expected':expected,'result':r,'cap':cap}
        print(phase,expected,'preserved',flush=True)
    p = E/'inverse-dag-baseline-caps-v937.json';assert not p.exists()
    p.write_text(json.dumps({'source_record':str((build/'source-hashes.json.gz').relative_to(E)),
        'source_record_sha256':sha(build/'source-hashes.json.gz'),'tests':rows},indent=2)+'\n')


if __name__ == '__main__':
    main()
