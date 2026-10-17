"""Qualify native downstream bindings without reopening any completed target."""
import json,subprocess,sys
sys.dont_write_bytecode=True
from validation_common_v3 import Validation,WORK,sha,dump
v=Validation('ffi',__file__); probes=WORK/'ffi-probes-v1'
pins={str(p):sha(p) for p in probes.iterdir() if p.is_file()}
v.protocol.update({'probe_sha256':pins,'cc':subprocess.check_output(['cc','--version'],text=True),'cxx':subprocess.check_output(['c++','--version'],text=True),'python':sys.version,'library_environment':'CSGRS_LIBRARY='+str(v.target/'release/libcsgrs_ffi.so')})
dump(v.out/'protocol.json',v.protocol)
v.run('ffi-tests',['cargo','test','--locked','--release','-p','csgrs-ffi'])
v.run('ffi-clippy',['cargo','clippy','--locked','--release','-p','csgrs-ffi','--all-targets','--','-D','warnings'])
v.run('ffi-docs',['cargo','doc','--locked','--release','-p','csgrs-ffi','--no-deps'],docs=True)
v.run('ffi-library',['cargo','build','--locked','--release','-p','csgrs-ffi'])
lib=v.target/'release/libcsgrs_ffi.so';v.env['CSGRS_LIBRARY']=str(lib)
out=v.retained/'consumers';out.mkdir(exist_ok=False)
for name in ['empty_composition','boolean_failures','polyhedral_construction','cancelled_flat']:
 for language,compiler,standard in [('c','cc','c11'),('cpp','c++','c++17')]:
  source=(probes if name=='cancelled_flat' else v.source/'tests/ffi')/(name+'.'+language)
  binary=out/(name+'-'+language)
  v.run(name+'-'+language+'-compile',[compiler,'-std='+standard,'-O2','-Wall','-Wextra','-Werror','-I'+str(v.source/'crates/csgrs-ffi/include'),'-I'+str(v.source/'bindings/cpp'),str(source),'-L'+str(lib.parent),'-Wl,-rpath,'+str(lib.parent),'-lcsgrs_ffi','-o',str(binary)])
  v.run(name+'-'+language,[str(binary)])
 source=(probes if name=='cancelled_flat' else v.source/'tests/ffi')/(name+'.py')
 command=['python3','-B',str(source)]+([str(v.source/'bindings/python')] if name=='cancelled_flat' else [])
 v.run(name+'-python',command)
assert all(sha(p)==h for p,h in pins.items())
artifacts={str(p):{'sha256':sha(p),'bytes':p.stat().st_size} for p in [lib,*out.iterdir()] if p.is_file()}
raise SystemExit(v.finish({'real_c_cpp_python_consumers_executed':True,'new_cancelled_sheet_consumers':['C: 72 operation/order/geometry cases plus downstream composition','C++: 4 operations plus downstream composition','Python: 72 operation/order/geometry cases plus downstream composition'],'end_to_end_certainty_interface_claimed':False,'artifacts':artifacts}))
