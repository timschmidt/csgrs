import {writeFileSync}from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json,workspace,retainedSources}from './zero-factor-retained-sources-v75.mjs';
import {captured}from './point-qualified-capture.mjs';
const o=json('symbolic-boundary-origin-v83.json'),lib=workspace+'/exact-real-references/flint',binary=o.dir+'/components';
const files=Object.fromEntries(['run-complex-roundtrip-v83.mjs','flint-complex-roundtrip-v83.c','flint-symbolic-boundary-fixed-v83.c'].map(p=>[p,sha(p)]));
writeFileSync('complex-roundtrip-origin-v83.json',JSON.stringify({recorded:new Date().toISOString(),files,binary,current:retainedSources()},null,2)+'\n',{flag:'wx'});
const sources=()=>{assert.deepEqual(retainedSources(),o.current);for(const[p,h]of Object.entries({...o.files,...files,...o.libraries,...o.configurationFiles}))assert.equal(sha(p),h,p);};
sources();
await captured('complex-roundtrip-compile-v83','.','gcc',['-O2','-g0','-ffunction-sections','-fdata-sections','-Wall','-Wextra','-Werror','-isystem',lib+'/src',
 'flint-complex-roundtrip-v83.c','-L',lib,'-Wl,-rpath,'+lib,'-Wl,--gc-sections','-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',binary]);sources();
await captured('complex-roundtrip-native-v83','.','timeout',['120s','prlimit','--as=1073741824','--cpu=90','--',binary]);sources();
await captured('complex-roundtrip-memcheck-v83','.','timeout',['180s','valgrind','--tool=memcheck','--leak-check=full','--show-leak-kinds=all',
 '--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=97',binary]);sources();
console.log(JSON.stringify({checkpoint:83,status:'complex-roundtrip-collected',expectedRows:2497,productionChanges:0}));
