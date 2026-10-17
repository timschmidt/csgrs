"""Reproduce new source-validation assertions without altering runtime code."""
import sys
sys.dont_write_bytecode=True
from validation_common_v1 import Validation,WORK,read,sha
v=Validation('baseline',__file__)
v.run('containment-inputs',['cargo','test','--locked','--release','-p','csgrs','--all-features','--test','containment_inputs','--','--nocapture','--test-threads=1'],expected=101)
prior=read(WORK.parent/'wasm-mesh-failure-review/wasm-validation-v1/result.json')
assert prior['complete'] and not prior['unexpected_exits']
artifacts=prior['artifacts']
assert all(sha(p)==r['sha256'] for p,r in artifacts.items())
js=next(p for p in artifacts if p.endswith('/csgrs.js'))
v.run('wasm-containment-inputs',['node',str(v.source/'tests/wasm/mesh_errors.cjs'),js],expected=1)
assert all(sha(p)==r['sha256'] for p,r in artifacts.items())
raise SystemExit(v.finish({'read_only_wasm_artifacts':artifacts,'new_runtime_code':False}))
