"""Version the corrected demo harness; leave executed runners unchanged."""
import ast, hashlib, json
from pathlib import Path

W = Path(__file__).resolve().parent

def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def replace_once(text, old, new):
    assert text.count(old) == 1, old
    return text.replace(old, new, 1)

assert read(W / 'ui-isolation-v1/result.json')['all_final_v6_inputs_byte_identical']
protocol = read(W / 'ui-validation-v1/protocol.json')
assert sha(W / 'ui_common_v3.py') == protocol['common_sha256']
assert sha(W / 'ui_v4.py') == protocol['runner_sha256']
assert read(W / 'qualification-tail-v4/commands.json')[-1]['exit_code'] == 101
common = (W / 'ui_common_v3.py').read_text()
common = replace_once(common, "SOURCE=WORK/'candidate-source-v6'", "SOURCE=Path('/tmp/csgrs-native-stroke-ui-source-v1')")
common = replace_once(common, "self.source=WORK/('baseline-source-v1' if kind=='baseline' else 'candidate-source-v6')", "self.source=SOURCE\n        assert read(WORK/'ui-isolation-v1/result.json')['all_final_v6_inputs_byte_identical']")
for name in ['validation', 'target', 'scratch', 'executables']:
    common = replace_once(common, "'ui-" + name + "-v1'", "'ui-" + name + "-v2'")
common = replace_once(common, "'source':str(self.source),'source_hashes_sha256':sha(self.manifest),", "'source':str(self.source),'source_hashes_sha256':sha(self.manifest),\n                       'source_isolation_result_sha256':sha(WORK/'ui-isolation-v1/result.json'),\n                       'preserved_failed_validation':'ui-validation-v1',")
runner = (W / 'ui_v4.py').read_text().replace('from ui_common_v3 import', 'from ui_common_v4 import')
runner = replace_once(runner, "v=Validation('candidate',__file__)", "assert read(WORK/'ui-validation-v1/result.json')['unexpected_exits']==['ui-tests','ui-clippy','ui-wasm']\nassert read(WORK/'ui-isolation-v1/result.json')['complete']\nv=Validation('candidate',__file__)")
comparison = (W / 'compare_v4.py').read_text().replace('ui-validation-v1', 'ui-validation-v2')
records = {}
for name, text in [('ui_common_v4.py', common), ('ui_v5.py', runner), ('compare_v5.py', comparison)]:
    ast.parse(text, filename=name)
    dest = W / name
    assert not dest.exists()
    dest.write_text(text)
    records[name] = sha(dest)
dest = W / 'ui-runner-preparation-v1.json'
assert not dest.exists()
dest.write_text(json.dumps({'complete': True, 'runner_sha256': sha(__file__), 'created_sha256': records,
    'production_sources_changed': False, 'executed_runners_changed': False,
    'ui_commands_and_limits_unchanged': True, 'comparison_protocol_unchanged': True}, indent=2) + '\n')
print(json.dumps({'complete': True, 'created': list(records)}), flush=True)
