"""Qualify the current DRC baseline and its complete original workload gates."""
import sys
sys.dont_write_bytecode=True
from validation_common_v1 import Validation
v=Validation('baseline',__file__)
cargo=['cargo','test','--locked','--release','-p','hyperdrc','--all-features']
v.run('native-tests',cargo+['--no-fail-fast','--','--test-threads=1',
    '--skip','zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite',
    '--skip','zz_complex_project_performance_tests::min_copper_neck_width_completes_on_copper_layers',
    '--skip','zz_complex_project_performance_tests::gerber_package_completes_smoke_check_suite'])
for name,test in [('full-board','kicad_board_completes_smoke_check_suite'),
    ('copper-neck','min_copper_neck_width_completes_on_copper_layers'),
    ('gerber','gerber_package_completes_smoke_check_suite')]:
    v.run(name,cargo+['--lib','zz_complex_project_performance_tests::'+test,'--','--exact','--test-threads=1','--nocapture'])
v.run('native-cli',['cargo','build','--locked','--release','-p','hyperdrc','--all-features','--message-format=json'])
raise SystemExit(v.finish())
