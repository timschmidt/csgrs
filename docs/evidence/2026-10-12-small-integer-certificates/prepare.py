"""Fork the persistent-cache baseline to evaluate exact integer-leaf reuse.

Both direct quadratic traversals were rejected on fixed scalar costs. This
trial changes only exact integer-leaf conversion and preserves the original
normalizer, cache policy, fallback and native fixtures. Prior turns made
progress by recording and committing both rejected trials.
"""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PREVIOUS = ROOT / 'docs/evidence/2026-10-12-persistent-laurent-cache'
BASE = ROOT / 'target/persistent-laurent-cache/candidate/source'
WORK = ROOT / 'target/small-integer-certificates'
SOURCE = WORK / 'source'
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources

sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    original = json.loads(gzip.decompress((PREVIOUS / 'initial-source-hashes.json.gz').read_bytes()))
    delta = json.loads((PREVIOUS / 'validation-v1/protocol.json').read_text())['source_changes_sha256']
    expected = original | delta
    main_expected = json.loads((PREVIOUS / 'preparation.json').read_text())['main_source_sha256']
    assert sources() == main_expected
    assert all(sha(BASE / p) == value for p, value in expected.items())
    WORK.mkdir(exist_ok=False)
    with (E / 'copy.log').open('w') as log:
        subprocess.run(['cp', '-a', '--reflink=auto', str(BASE), str(SOURCE)],
                       stdout=log, stderr=subprocess.STDOUT, check=True)
    assert all(sha(SOURCE / p) == value for p, value in expected.items())
    assert sources() == main_expected
    with (E / 'baseline-source-hashes.json.gz').open('xb') as raw:
        with gzip.GzipFile(fileobj=raw, mode='wb', mtime=0, filename='') as stream:
            stream.write((json.dumps(expected, indent=2) + '\n').encode())
    record = {'scope': __doc__, 'previous_goal_turn': 'progress',
              'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'runner_sha256': sha(Path(__file__)), 'copied_files': len(expected),
              'baseline_source': str(BASE), 'candidate_source': str(SOURCE),
              'source_hashes_sha256': sha(E / 'baseline-source-hashes.json.gz'),
              'prior_rejection_sha256': sha(ROOT / 'docs/evidence/2026-10-12-quadratic-certificate-consumer/cost-assessment-v2.json'),
              'main_source_sha256': main_expected,
              'required_gates': ['Preserve all five persistent-cache regressions, including the eight/nine-term fallback counterexample.',
                                 'Exact integer-leaf values across signed word boundaries, cache reuse, clones and original fallback frontiers.',
                                 'Strict Clippy, formatting, scalar tests, matched time/allocation/size controls, consumer tests, original full-board limits and complete report replay.'],
              'main_runtime_unchanged': True, 'runtime_admitted': False}
    (E / 'preparation.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps({'copied_files': len(expected), 'main_runtime_unchanged': True}), flush=True)


if __name__ == '__main__':
    main()
