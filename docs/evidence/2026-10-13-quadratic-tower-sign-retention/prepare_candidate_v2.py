"""Prepare exact sign transfer across an already established tower equality."""
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
BASE = E / 'baseline-v1'
OUT = E / 'preparation-v2'
WORK = ROOT / 'target/quadratic-tower-sign-retention/candidate-v2'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    OUT.mkdir(exist_ok=False)
    WORK.mkdir(exist_ok=False)
    baseline = json.loads((BASE / 'protocol.json').read_text())
    result = json.loads((BASE / 'result.json').read_text())
    assert result['complete'] and len(result['regression_failures']) == 6 and len(result['control_passes']) == 3
    before = json.loads(gzip.decompress((BASE / 'baseline-source-hashes.json.gz').read_bytes()))
    original = json.loads(gzip.decompress((BASE / 'original-source-hashes.json.gz').read_bytes()))
    source = Path(baseline['source'])
    current = WORK / 'source'
    assert sources() == baseline['main_source_sha256']
    assert all(sha(source / name) == digest for name, digest in before.items())
    assert all(sha(ROOT / name) == digest for name, digest in original.items())
    with (OUT / 'copy.log').open('w') as log:
        subprocess.run(['cp', '-a', '--reflink=auto', str(source), str(current)], stdout=log, stderr=subprocess.STDOUT, check=True)
    facade = 'crates/hyperreal/src/real/arithmetic/quadratic_tower_sign.rs'
    node = 'crates/hyperreal/src/computable/node/quadratic_tower.rs'
    test = 'crates/hyperreal/src/real/arithmetic/quadratic_tower_sign_retention_tests.rs'
    shutil.copyfile(E / 'quadratic_tower_sign_retention_tests_v2.rs', current / test)
    text = (current / facade).read_text()
    old = '    /// are preserved; no numerical approximation is used.\n'
    new = ('    /// are preserved; no numerical approximation is used. An already retained\n'
           '    /// exact sign remains available to subsequent lower-precision queries.\n')
    assert text.count(old) == 1
    text = text.replace(old, new)
    anchor = '        compact *= Self::new(self.rational.clone());\n'
    added = '''        if matches!(compact.class, Irrational) {
            // Before applying our outer scale, the reconstructed Real equals
            // our computable payload. Its own rational scale can have either
            // sign, so transfer the proof to its payload with that correction.
            compact
                .computable_ref()
                .retain_sign_of_scaled_tower_source(self.computable_ref(), compact.rational.sign());
        }
'''
    assert text.count(anchor) == 1
    text = text.replace(anchor, added + anchor)
    (current / facade).write_text(text)
    body = (current / node).read_text()
    anchor_node = '    pub(crate) fn quadratic_tower_sign(&self) -> Option<RealSign> {\n'
    helper = '''    /// Carry a sign across tower reconstruction without evaluating either DAG.
    /// The caller has proved `source = scale * self`, with the supplied scale
    /// sign. A zero scale says nothing about this payload and is ignored.
    pub(crate) fn retain_sign_of_scaled_tower_source(&self, source: &Self, scale_sign: Sign) {
        let Some(sign) = source.immediate_sign() else {
            return;
        };
        let sign = match scale_sign {
            Sign::Plus => private_sign(sign),
            Sign::Minus => negate_sign(private_sign(sign)),
            Sign::NoSign => return,
        };
        self.internal
            .facts
            .replace_exact_sign(ExactSignCache::Valid(sign));
    }

'''
    assert body.count(anchor_node) == 1
    (current / node).write_text(body.replace(anchor_node, helper + anchor_node))
    with (OUT / 'format.log').open('w') as log:
        subprocess.run(['rustfmt', '--edition', '2024', str(current / node), str(current / test)], stdout=log, stderr=subprocess.STDOUT, check=True)
    assert (current / facade).read_text().replace(added, '').replace(new, old) == (source / facade).read_text()
    assert (current / node).read_text().replace(helper, '') == (source / node).read_text()
    assert (current / test).read_bytes() == (E / 'quadratic_tower_sign_retention_tests_v2.rs').read_bytes()
    after = {name: sha(current / name) for name in before}
    changes = {name: digest for name, digest in after.items() if before[name] != digest}
    assert set(changes) == {facade, node, test}
    with tarfile.open(OUT / 'candidate-delta.tar.gz', 'w:gz') as archive:
        for name in changes:
            archive.add(current / name, arcname=name)
    patch = ''.join(''.join(difflib.unified_diff((source / name).read_text().splitlines(True),
                                               (current / name).read_text().splitlines(True),
                                               'a/' + name, 'b/' + name)) for name in changes)
    (OUT / 'candidate.patch').write_text(patch)
    (OUT / 'candidate-source-hashes.json.gz').write_bytes(gzip.compress((json.dumps(after, sort_keys=True) + '\n').encode(), mtime=0))
    protocol = {'scope': __doc__, 'runner_sha256': sha(Path(__file__)), 'failed_preparation_sha256': sha(E / 'preparation-failure-v1.json'), 'qualification_runner_sha256': sha(E / 'qualify_v2.py'),
                'test_sha256': sha(E / 'quadratic_tower_sign_retention_tests_v2.rs'),
                'baseline_protocol_sha256': sha(BASE / 'protocol.json'), 'baseline_result_sha256': sha(BASE / 'result.json'),
                'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                'source': str(current), 'target': str(WORK / 'qualification-target'), 'source_count': len(after),
                'source_changes_sha256': changes, 'patch_sha256': sha(OUT / 'candidate.patch'),
                'delta_archive_sha256': sha(OUT / 'candidate-delta.tar.gz'),
                'baseline_runtime_recovered_after_removing_transfer_and_doc': True,
                'new_test_functions': 10, 'shared_baseline_test_functions': 9,
                'extra_test': 'A negative tower reconstructed after cancellation reverses the payload scale. This control checks both outer signs and the shared positive root.',
                'proof': 'Before multiplying by the original outer rational, source_payload = rebuilt_scale * rebuilt_payload. For nonzero rebuilt_scale, sign(rebuilt_payload) = sign(source_payload) * sign(rebuilt_scale). Only a retained exact sign is copied; Unknown and zero-scale provide no inferred payload sign. Named/rational outputs already expose their exact sign and skip the transfer.',
                'limits': 'No new cache field, owner, numerical refinement, recognition budget, public API, or rejected experimental runtime. This has not yet passed candidate qualification or matched resource controls.',
                'runtime_admitted': False}
    assert sources() == baseline['main_source_sha256']
    assert all(sha(source / name) == digest for name, digest in before.items())
    assert all(sha(ROOT / name) == digest for name, digest in original.items())
    (OUT / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    print(json.dumps({'source_files': len(after), 'changed_files': list(changes), 'main_and_baseline_unchanged': True}))


if __name__ == '__main__':
    main()
