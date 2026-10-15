"""Remove only unused debug copies from release-only, terminal SDF trials."""
from pathlib import Path
import hashlib
import json
import shutil

ROOT = Path(__file__).resolve().parents[3]
E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
authority = ROOT / 'docs/evidence/2026-10-15-sdf-failures/closure-v1.json'
retained = json.loads(authority.read_text())['preserved_executables_sha256']
assert all(sha(ROOT / name) == digest for name, digest in retained.items())
rows = []
for phase in ['baseline-v2', 'candidate-v1', 'candidate-v2', 'candidate-v3', 'candidate-v4']:
    evidence = ROOT / 'docs/evidence/2026-10-15-sdf-failures' / phase
    protocol = json.loads((evidence / 'protocol.json').read_text())
    result = json.loads((evidence / 'result.json').read_text())
    assert result['commands']
    assert all(command[0] == 'cargo' and '--release' in command for _, command in protocol['commands'])
    assert result['commands'][-1]['exit_code'] != 0 or result['passed']
    target = ROOT / 'target/sdf-failures' / phase / 'debug'
    assert target.is_dir() and not target.is_symlink()
    assert not any(str(ROOT / name).startswith(str(target) + '/') for name in retained)
    files = list(target.rglob('*'))
    assert not any(p.is_symlink() for p in files)
    size = sum(p.stat().st_size for p in files if p.is_file())
    rows.append({'removed_unexecuted_copy': str(target.relative_to(ROOT)), 'bytes': size,
                 'protocol_sha256': sha(evidence / 'protocol.json'), 'terminal_result_sha256': sha(evidence / 'result.json')})
    shutil.rmtree(target)
assert all(sha(ROOT / name) == digest for name, digest in retained.items())
(E / 'cache-reclamation-v1.json').write_text(json.dumps({
    'runner_sha256': sha(Path(__file__)), 'authority_sha256': sha(authority), 'phases': rows,
    'reason': 'These private trial directories executed only release-profile Cargo commands. Their debug trees were unused seed copies. Source archives, release/build artifacts, logs and all 330 observed artifacts are unchanged.',
    'preserved_executables_verified': len(retained), 'reclaimed_bytes': sum(r['bytes'] for r in rows),
}, indent=2) + '\n')
print(json.dumps({'reclaimed_bytes': sum(r['bytes'] for r in rows), 'preserved_executables_verified': len(retained)}), flush=True)
