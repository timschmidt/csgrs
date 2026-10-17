"""Check the direct demo consumer in a new target after native qualification."""
import sys
sys.dont_write_bytecode=True
from ui_common_v2 import Validation,read,WORK
assert read(WORK/'candidate-validation-v4/result.json')['unexpected_exits']==['curve-all','circuit-all','clippy']
assert read(WORK/'consumer-validation-v1/result.json')['unexpected_exits']==[]
v=Validation('candidate',__file__)
manifest='crates/hypercurve/examples/hypercurve_ui/Cargo.toml'
v.run('ui-tests',['cargo','test','--offline','--locked','--manifest-path',manifest,'--release','--no-fail-fast','--','--test-threads=1'])
v.run('ui-clippy',['cargo','clippy','--offline','--locked','--manifest-path',manifest,'--release','--all-targets','--','-D','warnings'])
v.run('ui-wasm',['cargo','build','--offline','--locked','--manifest-path',manifest,'--release','--target','wasm32-unknown-unknown'])
sys.exit(v.finish({'affected_demo_consumer':'Polyline::outline -> CurveRegion2::stroke_path',
    'tests_profile':'release; full pre-push debug matrix remains open','published_package_pipeline_executed':False}))
