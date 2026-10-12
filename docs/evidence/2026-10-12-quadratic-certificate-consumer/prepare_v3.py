"""Fork the rejected consumer without changing any completed experimental input."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
WORK = ROOT / 'target/quadratic-certificate-consumer'
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources

sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    original = json.loads(gzip.decompress((E / 'baseline-source-hashes.json.gz').read_bytes()))
    previous_delta = json.loads((E / 'validation-v2/protocol.json').read_text())['source_changes_sha256']
    previous = original | previous_delta
    guard = json.loads((E / 'preparation.json').read_text())['main_source_sha256']
    assert all(sha(WORK / 'source' / p) == value for p, value in previous.items())
    assert sources() == guard
    target = WORK / 'v3'
    target.mkdir(exist_ok=False)
    out = E / 'preparation-v3'
    out.mkdir(exist_ok=False)
    with (out / 'copy.log').open('w') as log:
        subprocess.run(['cp', '-a', '--reflink=auto', str(WORK / 'source'), str(target / 'source')],
                       stdout=log, stderr=subprocess.STDOUT, check=True)
    assert all(sha(target / 'source' / p) == value for p, value in previous.items())
    assert sources() == guard
    record = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
              'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'rejected_cost_assessment_sha256': sha(E / 'cost-assessment-v1.json'),
              'previous_validation_protocol_sha256': sha(E / 'validation-v2/protocol.json'),
              'copied_files': len(previous), 'source': str(target / 'source'),
              'main_runtime_unchanged': True, 'runtime_admitted': False}
    (out / 'result.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(record), flush=True)


if __name__ == '__main__':
    main()
