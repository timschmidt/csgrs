"""Qualify the final kernel sources through affected native consumers."""
import sys
sys.dont_write_bytecode=True
from validation_common_v4 import Validation
v=Validation('candidate',__file__)
v.run('format',['cargo','fmt','--all','--','--check'])
v.run('stroke-regressions',['cargo','test','--offline','--locked','-p','hypercurve','--release','--test','native_linear_stroke','--test','native_linear_stroke_contracts','--','--nocapture','--test-threads=1'])
v.run('curve-default',['cargo','test','--no-fail-fast','--offline','--locked','-p','hypercurve','--release','--','--test-threads=1'])
v.run('curve-all',['cargo','test','--no-fail-fast','--offline','--locked','-p','hypercurve','--release','--all-features','--','--test-threads=1'])
v.run('root-all',['cargo','test','--no-fail-fast','--offline','--locked','-p','csgrs','--release','--all-features','--','--test-threads=1'])
v.run('root-minimal',['cargo','test','--no-fail-fast','--offline','--locked','-p','csgrs','--release','--no-default-features','--','--test-threads=1'])
v.run('native-tests',['cargo','test','--no-fail-fast','--offline','--locked','-p','hyperdrc','--release','--all-features','--','--test-threads=1',
    '--skip','zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite',
    '--skip','zz_complex_project_performance_tests::min_copper_neck_width_completes_on_copper_layers',
    '--skip','zz_complex_project_performance_tests::gerber_package_completes_smoke_check_suite'])
v.run('full-board',['cargo','test','--offline','--locked','-p','hyperdrc','--release','--all-features','--lib','zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite','--','--exact','--nocapture','--test-threads=1'])
v.run('copper-neck',['cargo','test','--offline','--locked','-p','hyperdrc','--release','--all-features','--lib','zz_complex_project_performance_tests::min_copper_neck_width_completes_on_copper_layers','--','--exact','--nocapture','--test-threads=1'])
v.run('gerber',['cargo','test','--offline','--locked','-p','hyperdrc','--release','--all-features','--lib','zz_complex_project_performance_tests::gerber_package_completes_smoke_check_suite','--','--exact','--nocapture','--test-threads=1'])
v.run('circuit-all',['cargo','test','--no-fail-fast','--offline','--locked','-p','hypercircuit','--release','--all-features','--','--test-threads=1'])
v.run('clippy',['cargo','clippy','--offline','--locked','-p','hypercurve','-p','csgrs','-p','hyperdrc','-p','hypercircuit','--release','--all-features','--all-targets','--','-D','warnings'])
v.run('docs',['cargo','doc','--offline','--locked','-p','hypercurve','--release','--all-features','--no-deps'],docs=True)
v.run('native-cli',['cargo','build','--offline','--locked','-p','hyperdrc','--release','--all-features','--message-format=json'])
sys.exit(v.finish({'affected_native_consumers':['hypercurve','csgrs','hyperdrc','hypercircuit'],
    'original_workload_gates_unchanged':True,'full_pre_push_matrix_executed':False,'published_package_pipeline_executed':False}))
