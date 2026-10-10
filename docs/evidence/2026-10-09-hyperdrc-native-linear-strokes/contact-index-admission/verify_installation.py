"""Check selective installation of the measured repair without staging prototype work.

Run ``prepare`` after every measurement has finished, apply source-delta.patch
separately to the worktree and index, then run ``verify``. Git mutations remain
explicit caller actions. Reversing the patch in a temporary directory proves
that unrelated worktree changes survive byte for byte.
"""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
CANDIDATE = EVIDENCE / 'contact-index-directed-width'
OUT = EVIDENCE / 'contact-index-admission'
read = lambda path: json.loads(path.read_text())
sha = lambda data: hashlib.sha256(data).hexdigest()
digest = lambda path: sha(path.read_bytes())
source = read(CANDIDATE / 'sources.json')
main = read(Path(source['main_source_record']))
admission = read(EVIDENCE / 'diagnostics-58.json')
assert admission['decision'].startswith('kept as an exactness repair')
changed = source['only_modified_sources']
assert set(changed) == {
    'crates/hypercurve/src/curve_region_boolean.rs',
    'crates/hypercurve/src/curve_region_boolean/contact_index_tests.rs',
}
assert all(command['exit_code'] == 0 for command in read(EVIDENCE / 'contact-index-directed-validation/status.json'))
assert len(read(EVIDENCE / 'contact-index-directed-replay/manifest.json')['commands']) == 20
with tarfile.open(CANDIDATE / 'source-delta.tar.gz') as archive:
    assert set(archive.getnames()) == set(changed)
    tested = {name: archive.extractfile(name).read() for name in changed}
assert all(sha(data) == source['source_hashes'][name] for name, data in tested.items())


def git(*args, check=True):
    return subprocess.run(['git', *args], cwd=ROOT, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, check=check)


def write(name, value):
    (OUT / name).write_text(json.dumps(value, indent=2) + '\n')


if sys.argv[1:] == ['prepare']:
    assert not git('diff', '--cached', '--name-only').stdout
    assert all(digest(ROOT / name) == value for name, value in main['source_hashes'].items())
    OUT.mkdir()
    shutil.copyfile(__file__, OUT / 'verify_installation.py')
    shutil.copyfile(CANDIDATE / 'source-delta.patch', OUT / 'source-delta.patch')
    files = {}
    for name in changed:
        old = git('show', 'HEAD:' + name, check=False)
        path = ROOT / name
        if old.returncode:
            assert old.returncode == 128 and not path.exists()
        else:
            assert name in main['source_hashes'] and path.exists()
        files[name] = {
            'head_sha256': sha(old.stdout) if old.returncode == 0 else None,
            'tested_sha256': sha(tested[name]),
            'worktree_before_sha256': digest(path) if path.exists() else None,
        }
    patch = str(OUT / 'source-delta.patch')
    git('apply', '--check', patch)
    git('apply', '--cached', '--check', patch)
    write('preparation.json', {
        'head_before': git('rev-parse', 'HEAD').stdout.decode().strip(),
        'source_archive': '../contact-index-directed-width/source-delta.tar.gz',
        'source_record': '../contact-index-directed-width/sources.json',
        'source_patch_sha256': digest(Path(patch)),
        'files': files,
    })
    print('Prepared selective patch; runtime source and index are unchanged.')
elif sys.argv[1:] == ['verify']:
    record = read(OUT / 'preparation.json')
    assert git('rev-parse', 'HEAD').stdout.decode().strip() == record['head_before']
    assert digest(OUT / 'source-delta.patch') == record['source_patch_sha256']
    assert set(git('diff', '--cached', '--name-only').stdout.decode().splitlines()) == set(changed)
    for name in changed:
        staged = git('show', ':' + name).stdout
        assert staged == tested[name]
        record['files'][name]['staged_sha256'] = sha(staged)
        record['files'][name]['worktree_after_sha256'] = digest(ROOT / name)
    assert all(digest(ROOT / name) == value for name, value in main['source_hashes'].items() if name not in changed)
    with tempfile.TemporaryDirectory(prefix='contact-index-reverse-', dir='/tmp') as directory:
        scratch = Path(directory)
        for name in changed:
            destination = scratch / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / name, destination)
        subprocess.run(['git', 'apply', '--reverse', str(OUT / 'source-delta.patch')],
                       cwd=scratch, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        for name in changed:
            expected = record['files'][name]['worktree_before_sha256']
            assert digest(scratch / name) == expected if expected else not (scratch / name).exists()
    command = ['cargo', 'fmt', '--all', '--', '--check']
    start = time.monotonic()
    with (OUT / 'format.log').open('w') as log:
        result = subprocess.run(command, cwd=ROOT,
                                env=os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_NET_OFFLINE': 'true'},
                                stdout=log, stderr=subprocess.STDOUT)
    record['format_after_apply'] = {'command': command, 'exit_code': result.returncode,
                                    'wall_seconds': time.monotonic() - start}
    write('format-status.json', record['format_after_apply'])
    assert result.returncode == 0
    assert all(digest(ROOT / name) == value for name, value in main['source_hashes'].items() if name not in changed)
    assert all(digest(ROOT / name) == record['files'][name]['worktree_after_sha256'] for name in changed)
    record.update({
        'decision': 'kept exactness repair: only the tested contact-index widths, mathematical order keys and regression module are staged',
        'validation_record': '../diagnostics-58.json',
        'staged_source_matches_tested_archive': True,
        'all_unrelated_main_prototype_edits_preserved_byte_for_byte': True,
        'broader_native_prototype_admitted': False,
    })
    write('admission.json', record)
    print('Staged source matches the tested archive; all unrelated prototype edits are preserved; formatting passes.')
else:
    raise SystemExit('usage: verify_contact_index_installation.py prepare|verify')
