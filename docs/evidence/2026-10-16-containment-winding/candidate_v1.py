"""Qualify shared validation and all affected host consumers in one fresh target."""
import sys
sys.dont_write_bytecode=True
from validation_common_v1 import Validation
v=Validation('candidate',__file__)
v.run('format',['cargo','fmt','--all','--','--check'])
v.run('containment-regressions',['cargo','test','--locked','--release','-p','csgrs','--all-features','--test','containment_inputs','--test','containment_winding','--','--nocapture','--test-threads=1'])
v.run('mesh-and-modeling-all-features',['cargo','test','--locked','--release','-p','hypermesh','-p','csgrs','--all-features'])
v.run('modeling-minimal',['cargo','test','--locked','--release','-p','csgrs','--no-default-features'])
v.run('all-feature-all-target-clippy',['cargo','clippy','--locked','--release','-p','hypermesh','-p','csgrs','--all-features','--all-targets','--','-D','warnings'])
v.run('minimal-all-target-clippy',['cargo','clippy','--locked','--release','-p','csgrs','--no-default-features','--all-targets','--','-D','warnings'])
v.run('all-feature-docs',['cargo','doc','--locked','--release','-p','hypermesh','-p','csgrs','--all-features','--no-deps'],docs=True)
v.run('cost-libraries',['cargo','build','--locked','--release','-p','csgrs','--all-features','--message-format=json'])
raise SystemExit(v.finish())
