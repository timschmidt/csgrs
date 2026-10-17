"""Check host compilation, documentation and formatting of the qualified WASM-only change."""
import sys
sys.dont_write_bytecode=True
from wasm_validation_common_v1 import Validation
v=Validation('native',__file__)
v.run('format',['cargo','fmt','--all','--','--check'])
v.run('all-feature-all-target-clippy',['cargo','clippy','--locked','--release','-p','csgrs','--all-features','--all-targets','--','-D','warnings'])
v.run('all-feature-docs',['cargo','doc','--locked','--release','-p','csgrs','--all-features','--no-deps'],docs=True)
raise SystemExit(v.finish({'scope':'Host compilation/lint/docs of a WASM-only error-mapping change. Its runtime acceptance is the real JS and compiled TS consumer suite. No additional native geometry coverage claimed.'}))
