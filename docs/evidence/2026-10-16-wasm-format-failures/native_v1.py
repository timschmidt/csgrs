"""Final native feature, test, lint and documentation consumers."""
import sys
sys.dont_write_bytecode=True
from validation_common_v1 import Validation,SOURCE

session=Validation('native',__file__)
def cargo(action,*arguments):
    return ['cargo',action,'--manifest-path',str(SOURCE/'Cargo.toml'),'--locked','--release','-p','csgrs',*arguments]
session.run('format',['cargo','fmt','--manifest-path',str(SOURCE/'Cargo.toml'),'--all','--','--check'])
session.run('root-all',cargo('test','--all-features','--no-fail-fast','--','--test-threads=2'))
session.run('wasm-feature',cargo('test','--no-default-features','--features','wasm','--no-fail-fast','--','--test-threads=2'))
session.run('strict-clippy',cargo('clippy','--all-features','--all-targets','--','-D','warnings'))
session.run('wasm-feature-clippy',cargo('clippy','--no-default-features','--features','wasm','--all-targets','--','-D','warnings'))
session.run('docs',cargo('doc','--all-features','--no-deps'),docs=True)
raise SystemExit(session.finish())
