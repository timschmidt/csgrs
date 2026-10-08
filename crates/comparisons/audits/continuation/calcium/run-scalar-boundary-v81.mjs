import assert from 'node:assert/strict';
import {sha,json,workspace,retainedSources} from './zero-factor-retained-sources-v75.mjs';
import {captured} from './point-qualified-capture.mjs';
const o=json('scalar-boundary-origin-v81.json'),lib=workspace+'/exact-real-references/flint';
const sources=()=>{assert.deepEqual(retainedSources(),o.current);for(const[p,h]of Object.entries({...o.files,...o.libraries,...o.configurationFiles}))assert.equal(sha(p),h,p);};
sources();
await captured('scalar-boundary-compile-v81','.','gcc',['-O2','-g0','-ffunction-sections','-fdata-sections','-Wall','-Wextra','-Werror',
 '-isystem',lib+'/src','flint-scalar-boundary-v81.c','-L',lib,'-Wl,-rpath,'+lib,'-Wl,--gc-sections','-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',o.binary]);sources();
await captured('scalar-boundary-linkage-v81','.','ldd',[o.binary]);sources();
await captured('scalar-boundary-native-v81','.','timeout',['120s','prlimit','--as=1073741824','--cpu=90','--',o.binary]);sources();
await captured('scalar-boundary-memcheck-v81','.','timeout',['180s','valgrind','--tool=memcheck','--leak-check=full','--show-leak-kinds=all',
 '--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=97',o.binary]);sources();
console.log(JSON.stringify({checkpoint:81,status:'native-collected',expectedRows:19924,productionChanges:0}));
