"""Version the retained containment harness for the exact track candidate."""
import ast, hashlib, json
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]
OLD = ROOT / 'target/properties/native-stroke-completion'
source = OLD / 'consumer_common_v1.py'
protocol = json.loads((OLD / 'consumer-validation-v1/protocol.json').read_text())
assert hashlib.sha256(source.read_bytes()).hexdigest() == protocol['common_sha256']
text = source.read_text().replace('target/properties/native-stroke-completion', 'target/properties/kicad-native-strokes')
for old, new in [('candidate-source-v5', 'candidate-source-v1'), ('candidate-preparation-v5', 'candidate-preparation-v1'),
    ('consumer-validation-v1', 'candidate-validation-v1'), ('consumer-target-v1', 'candidate-target-v1'),
    ('consumer-scratch-v1', 'candidate-scratch-v1'), ('consumer-executables-v1', 'candidate-executables-v1'),
    ('b137381247a03342ce5b6184710475fcc22c9d63', 'a843bf79dfe329fb51ababaaf803752bb74a0f64')]:
    assert old in text, old
    text = text.replace(old, new)
ast.parse(text)
dest = W / 'validation_common_v1.py'
assert not dest.exists()
dest.write_text(text)
record = {'complete': True, 'source': str(source), 'source_sha256': protocol['common_sha256'],
    'created': str(dest), 'created_sha256': hashlib.sha256(dest.read_bytes()).hexdigest(),
    'runner_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    'limits_unchanged': True, 'per_command_seconds': 1800, 'per_process_address_space_bytes': 8 * 1024**3}
p = W / 'validation-preparation-v1.json'
assert not p.exists()
p.write_text(json.dumps(record, indent=2) + '\n')
print(json.dumps({'complete': True, 'created': str(dest)}), flush=True)
