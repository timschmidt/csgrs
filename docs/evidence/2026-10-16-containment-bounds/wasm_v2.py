"""Execute real generated JS and compiled TypeScript consumers on a fresh target."""
import sys
sys.dont_write_bytecode=True
from validation_common_v2 import Validation,SOURCE,sha

session=Validation('wasm',__file__)
session.run('shell-syntax',['bash','-n',str(SOURCE/'scripts/test-wasm-bindings.sh')])
session.run('real-consumers',['bash',str(SOURCE/'scripts/test-wasm-bindings.sh')])
session.run('wasm-clippy',['cargo','clippy','--locked','--profile','wasm','--target','wasm32-unknown-unknown',
                         '--no-default-features','--features','wasm','--lib','--','-D','warnings'])
text=(session.out/'real-consumers.stdout').read_text()
consumer_passed=all(value in text for value in ['# tests 12','# pass 12','# tests 13','# pass 13','# tests 10','# pass 10','# fail 0',
    'WASM certainty, query status, geometry, and invalid-input checks passed',
    'WASM export projection, failure, cache and composition contracts passed',
    'TypeScript native STL pipeline and discriminated errors passed'])
artifacts=[session.target/'wasm32-unknown-unknown/wasm/csgrs.wasm',*sorted((session.target/'wasm-bindings-node').glob('*'))]
records={str(path):{'sha256':sha(path),'bytes':path.stat().st_size} for path in artifacts if path.is_file()}
status=session.finish({'real_js_and_compiled_typescript_consumers_passed':consumer_passed,
                       'public_contract_tests':35,'mesh_contract_tests':13,'winding_contract_tests':10,'prior_io_contract_tests':12,'artifacts':records})
raise SystemExit(status if status else (0 if consumer_passed else 101))
