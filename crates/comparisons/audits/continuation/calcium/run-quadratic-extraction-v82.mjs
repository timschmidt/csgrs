import assert from 'node:assert/strict';
import {sha,json,workspace,retainedSources} from './zero-factor-retained-sources-v75.mjs';
import {captured} from './point-qualified-capture.mjs';
const o=json('quadratic-extraction-origin-v82.json'),lib=workspace+'/exact-real-references/flint',input='quadratic-extraction-input-v82.tsv';
const sources=()=>{assert.deepEqual(retainedSources(),o.current);for(const[p,h]of Object.entries({...o.files,...o.libraries,...o.configurationFiles}))assert.equal(sha(p),h,p);};
sources();
await captured('quadratic-extraction-compile-v82','.','gcc',['-O2','-g0','-Wall','-Wextra','-Werror','-isystem',lib+'/src',
 'flint-quadratic-extraction-v82.c','-L',lib,'-Wl,-rpath,'+lib,'-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',o.binary]);sources();
await captured('quadratic-extraction-linkage-v82','.','ldd',[o.binary]);sources();
await captured('quadratic-extraction-native-v82','.','timeout',['120s','prlimit','--as=1073741824','--cpu=90','--',o.binary,input]);sources();
await captured('quadratic-extraction-memcheck-v82','.','timeout',['180s','valgrind','--tool=memcheck','--leak-check=full','--show-leak-kinds=all',
 '--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=97',o.binary,input]);sources();
console.log(JSON.stringify({checkpoint:82,status:'native-collected',expectedRows:4651,productionChanges:0}));
