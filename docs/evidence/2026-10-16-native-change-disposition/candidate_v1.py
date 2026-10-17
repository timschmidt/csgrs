"""Fresh candidate regressions and unchanged original runtime gates."""
import sys
sys.dont_write_bytecode=True
from validation_common_v1 import Validation

v=Validation('candidate',__file__)
cargo=['cargo','test','--locked','--release']
v.run('format',['cargo','fmt','--all','--','--check'])
v.run('native-curve-regressions',cargo+['-p','hypercurve','--all-features','--test','native_linear_stroke','--','--test-threads=1'])
v.run('native-drc-regressions',cargo+['-p','hyperdrc','--all-features','--test','native_linear_strokes','--','--test-threads=1'])
v.run('gate-build',cargo+['-p','hyperdrc','--all-features','--lib','--no-run'])
for name,test in [('full-board','kicad_board_completes_smoke_check_suite'),
                  ('copper-neck','min_copper_neck_width_completes_on_copper_layers'),
                  ('gerber','gerber_package_completes_smoke_check_suite')]:
    v.run(name,cargo+['-p','hyperdrc','--all-features','--lib','zz_complex_project_performance_tests::'+test,
                     '--','--exact','--test-threads=1','--nocapture'])
sys.exit(v.finish())
