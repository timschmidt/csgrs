"""Validate the actual committed-source alternative without changing pending main files."""
import sys
import time
sys.dont_write_bytecode=True
from validation_common_v1 import Validation,WORK,read

candidate=read(WORK/'candidate-validation-v1/result.json')
assert candidate['complete']
v=Validation('committed',__file__)
cargo=['cargo','test','--locked','--release']
v.run('format',['cargo','fmt','--all','--','--check'])
v.run('root-all',cargo+['-p','csgrs','--all-features','--no-fail-fast','--','--test-threads=2'])
v.run('root-minimal',cargo+['-p','csgrs','--no-default-features','--no-fail-fast','--','--test-threads=2'])
v.run('curve-default',cargo+['-p','hypercurve','--no-default-features','--no-fail-fast','--','--test-threads=2'])
v.run('curve-all',cargo+['-p','hypercurve','--all-features','--no-fail-fast','--','--test-threads=2'])
v.run('native-other-tests',cargo+['-p','hyperdrc','--all-features','--no-fail-fast','--','--test-threads=1',
    '--skip','zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite',
    '--skip','zz_complex_project_performance_tests::min_copper_neck_width_completes_on_copper_layers',
    '--skip','zz_complex_project_performance_tests::gerber_package_completes_smoke_check_suite'])
for name,test in [('full-board','kicad_board_completes_smoke_check_suite'),
                  ('copper-neck','min_copper_neck_width_completes_on_copper_layers'),
                  ('gerber','gerber_package_completes_smoke_check_suite')]:
    if name=='full-board':
        # Independent WASM compilation may overlap earlier correctness checks,
        # but every original timing gate waits for that build to finish.
        deadline=time.monotonic()+1800
        while not (WORK/'wasm-validation-v1/result.json').exists():
            if time.monotonic()>deadline:raise RuntimeError('WASM qualification did not complete before timing gates')
            time.sleep(1)
        assert read(WORK/'wasm-validation-v1/result.json')['complete']
    v.run(name,cargo+['-p','hyperdrc','--all-features','--lib','zz_complex_project_performance_tests::'+test,
                     '--','--exact','--test-threads=1','--nocapture'])
v.run('strict-clippy',['cargo','clippy','--locked','--release','-p','csgrs','-p','hypercurve','-p','hyperdrc',
                       '--all-features','--all-targets','--','-D','warnings'])
v.run('minimal-clippy',['cargo','clippy','--locked','--release','-p','csgrs','-p','hypercurve',
                        '--no-default-features','--all-targets','--','-D','warnings'])
v.run('docs',['cargo','doc','--locked','--release','-p','csgrs','-p','hypercurve','-p','hyperdrc',
              '--all-features','--no-deps'],docs=True)
v.run('native-cli',['cargo','build','--locked','--release','-p','hyperdrc','--all-features','--message-format=json'])
sys.exit(v.finish())
