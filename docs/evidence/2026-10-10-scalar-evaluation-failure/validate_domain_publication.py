"""Run the frozen child-sign publication diagnostic against one archived build."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
E=Path(__file__).resolve().parent
ROOT=E.parents[2]
build=E/sys.argv[1]
phase=sys.argv[2]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
result=json.loads((build/'result.json').read_text())
assert result['exit_code']==0 and result['source_unchanged'] and result['main_unchanged']
paths=re.findall(r'Executable unittests src/lib\.rs \(([^)]+)\)',(build/'stderr.log').read_text())
assert len(paths)==1
binary=Path(paths[0])
assert not (E/phase).exists()
subprocess.run([sys.executable,str(E/'validate_candidate_v2.py'),phase,'--exec',
 sys.executable,str(E/'run_limited_unit_test.py'),str(binary),sha(binary),
 'computable::node::domain_sign_tests::domain_sign_computed_leaf_proof_is_retained',
 '--exact','--nocapture','--test-threads=1'],cwd=ROOT,check=True)
assert sha(E/phase/'source-hashes.json.gz')==sha(build/'source-hashes.json.gz')
