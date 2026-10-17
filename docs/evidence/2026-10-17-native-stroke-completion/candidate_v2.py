"""Evaluate native stroke construction with all original curve regressions."""
import sys
sys.dont_write_bytecode=True
from validation_common_v2 import Validation
v=Validation('candidate',__file__)
v.run('format',['cargo','fmt','--all','--','--check'])
v.run('original-curve-regressions',['cargo','test','--offline','--locked','-p','hypercurve','--release','--test','native_linear_stroke','--','--nocapture','--test-threads=1'])
v.run('native-library',['cargo','build','--offline','--locked','-p','hypercurve','--release','--message-format=json'])
sys.exit(v.finish({'experiment_only':True,'full_qualification_claimed':False}))
