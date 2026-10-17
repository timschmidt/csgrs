"""Validate the complete candidate without changing original workload gates."""
import sys
sys.dont_write_bytecode=True
from validation_common_v2 import Validation
v=Validation('candidate',__file__)
cargo=['cargo','test','--locked','--release','-p','hyperdrc','--all-features']
v.run('format',['cargo','fmt','--all','--','--check'])
v.run('track-failures',cargo+['--test','kicad_track_failures','--','--test-threads=1'])
v.run('native-tests',cargo+['--no-fail-fast','--','--test-threads=1',
    '--skip','zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite',
    '--skip','zz_complex_project_performance_tests::min_copper_neck_width_completes_on_copper_layers',
    '--skip','zz_complex_project_performance_tests::gerber_package_completes_smoke_check_suite'])
for name,test in [('full-board','kicad_board_completes_smoke_check_suite'),
    ('copper-neck','min_copper_neck_width_completes_on_copper_layers'),
    ('gerber','gerber_package_completes_smoke_check_suite')]:
    v.run(name,cargo+['--lib','zz_complex_project_performance_tests::'+test,'--','--exact','--test-threads=1','--nocapture'])
v.run('clippy',['cargo','clippy','--locked','--release','-p','hyperdrc','--all-features','--all-targets','--','-D','warnings'])
v.run('docs',['cargo','doc','--locked','--release','-p','hyperdrc','--all-features','--no-deps'],docs=True)
v.run('native-cli',['cargo','build','--locked','--release','-p','hyperdrc','--all-features','--message-format=json'])
raise SystemExit(v.finish())
