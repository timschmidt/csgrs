"""Correct the fixture allowlist from the retained E35 admission record."""
import hashlib
import json
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]
source = W / 'checkpoint_v1.py'
destination = W / 'checkpoint_v2.py'
assert not destination.exists()
assert not (ROOT / 'docs/evidence/2026-10-17-native-track-cost-attribution').exists()
old = """allowed = {'docs/properties-status.json'} | {
    p for p in inherited if p.startswith('crates/hypercircuit/tests/fixtures/') and p.endswith('.json')
    and p in differences}
assert set(differences) == allowed and len(differences) == 8"""
new = """admitted = read(ROOT / 'docs/evidence/2026-10-17-native-stroke-completion/checkpoint-v1.json')['implementation_sha256']
fixtures = {p for p in admitted if p.startswith('crates/hypercircuit/tests/fixtures/')}
assert len(fixtures) == 7
allowed = {'docs/properties-status.json'} | fixtures
assert set(differences) == allowed and len(differences) == 8
assert all(differences[p]['main'] == admitted[p] for p in fixtures)"""
text = source.read_text()
assert text.count(old) == 1
text = text.replace(old, new).replace('expected JSON artifacts', 'expected artifacts (six JSON files and one release ZIP)')
destination.write_text(text)
def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()
(W / 'checkpoint-preparation-v2.json').write_text(json.dumps({
    'complete': True, 'original_runner_sha256': sha(source), 'corrected_runner_sha256': sha(destination),
    'reason': 'The seventh admitted expected artifact is a release ZIP, not a JSON file. The first guard failed before any evidence directory or ledger mutation.',
    'ledger_unchanged': sha(ROOT / 'docs/properties-status.json') == json.loads((ROOT / 'docs/evidence/2026-10-17-kicad-native-strokes/checkpoint-v1.json').read_text())['ledger_after_sha256']
}, indent=2) + '\n')
