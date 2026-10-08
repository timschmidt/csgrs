import {sha,json,retainedSources,workspace} from './zero-factor-retained-sources-v75.mjs';
import {captured} from './point-qualified-capture.mjs';
import assert from 'node:assert/strict';
const o=json('qqbar-inverse-origin-v80.json');
const sources=()=>{assert.deepEqual(retainedSources(),o.current);for(const[p,h]of Object.entries({...o.files,...o.libraries,...o.configurationFiles}))assert.equal(sha(p),h,p);};
sources();const lib=workspace+'/exact-real-references/flint';
for(const [name,source,binary]of [['corpus','flint-qqbar-inverse-v80.c',o.binary],['counterexample','flint-qqbar-inverse-counterexample-v80.c',o.counterexampleBinary]]){
 await captured('qqbar-inverse-'+name+'-compile-v80','.','gcc',['-O2','-g0','-ffunction-sections','-fdata-sections','-Wall','-Wextra','-Werror',
  '-isystem',lib+'/src',source,'-L',lib,'-Wl,-rpath,'+lib,'-Wl,--gc-sections','-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',binary]);sources();
 await captured('qqbar-inverse-'+name+'-linkage-v80','.','ldd',[binary]);
}
await captured('qqbar-inverse-native-v80','.',o.binary,[]);sources();
await captured('qqbar-inverse-memcheck-v80','.','valgrind',['--tool=memcheck','--leak-check=full','--show-leak-kinds=all',
 '--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=97',o.binary]);sources();
await captured('qqbar-inverse-counterexample-native-v80','.','timeout',['45s','prlimit','--as=1073741824','--cpu=30','--',o.counterexampleBinary]);sources();
console.log(JSON.stringify({checkpoint:80,status:'native-collection-complete',expectedRows:2163,executables:2,retained:false,
 note:'Collection only; independent mathematical checking and the public counterexample proof must pass separately.'}));
