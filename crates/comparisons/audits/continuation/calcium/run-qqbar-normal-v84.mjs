import {writeFileSync}from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json,workspace,retainedSources}from './zero-factor-retained-sources-v75.mjs';
import {captured}from './point-qualified-capture.mjs';
const o=json('qqbar-remainder-origin-v84.json'),lib=workspace+'/exact-real-references/flint',binary=o.dir+'/normal';
const files=Object.fromEntries(['flint-qqbar-normal-v84.c','run-qqbar-normal-v84.mjs'].map(p=>[p,sha(p)]));
writeFileSync('qqbar-normal-origin-v84.json',JSON.stringify({recorded:new Date().toISOString(),files,binary,current:retainedSources(),
 hyperRereadRanges:{'hypersolve/src/algebraic_binary.rs':[[125,245]],'hypersolve/src/resultant.rs':[[240,355]],'hypersolve/src/polynomial.rs':[[95,155],[210,260]]},
 hyperRereadHashes:Object.fromEntries(['hypersolve/src/algebraic_binary.rs','hypersolve/src/resultant.rs','hypersolve/src/polynomial.rs'].map(p=>[p,sha(workspace+'/'+p)])),
 donorDocReread:{path:'flint:doc/source/qqbar.rst',range:[500,610],sha256:sha(lib+'/doc/source/qqbar.rst')}},null,2)+'\n',{flag:'wx'});
const sources=()=>{assert.deepEqual(retainedSources(),o.current);for(const[p,h]of Object.entries({...o.files,...files,...o.libraries,...o.configurationFiles}))assert.equal(sha(p),h,p);};
sources();
await captured('qqbar-normal-compile-v84','.','gcc',['-O2','-g1','-ffunction-sections','-fdata-sections','-Wall','-Wextra','-Werror','-isystem',lib+'/src','flint-qqbar-normal-v84.c','-L',lib,'-Wl,-rpath,'+lib,'-Wl,--gc-sections','-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',binary]);sources();
await captured('qqbar-normal-native-v84','.','timeout',['120s','prlimit','--core=0','--as=1073741824','--cpu=90','--',binary]);sources();
await captured('qqbar-normal-mem-v84','.','timeout',['180s','prlimit','--core=0','--','valgrind','--tool=memcheck','--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=97',binary]);sources();
console.log(JSON.stringify({checkpoint:84,status:'normal-results-collected-not-independently-qualified',productionChanges:0}));
