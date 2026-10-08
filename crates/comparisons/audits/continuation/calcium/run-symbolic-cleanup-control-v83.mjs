import {readFileSync,writeFileSync}from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json,workspace,retainedSources}from './zero-factor-retained-sources-v75.mjs';
import {captured}from './point-qualified-capture.mjs';
const o=json('symbolic-boundary-origin-v83.json'),lib=workspace+'/exact-real-references/flint',binary=o.dir+'/cleanup-control',source='flint-set-fexpr-cleanup-control-v83.c';
const donor=readFileSync(lib+'/src/qqbar/set_fexpr.c','utf8'),fixed=readFileSync(source,'utf8');
const added='                        fmpz_clear(p);\n                        fmpz_clear(q);\n';
assert.equal(fixed.split(added).length,2);assert.equal(fixed.replace(added,''),donor);
const files=Object.fromEntries(['run-symbolic-cleanup-control-v83.mjs',source,'flint-symbolic-boundary-fixed-v83.c'].map(p=>[p,sha(p)]));
writeFileSync('symbolic-cleanup-control-origin-v83.json',JSON.stringify({recorded:new Date().toISOString(),files,binary,current:retainedSources(),donorSha256:sha(lib+'/src/qqbar/set_fexpr.c'),
 note:'Single audit-only translation-unit copy, two added clears, public symbol renamed for linkage. No donor/live source change or proposed Hyper retention.'},null,2)+'\n',{flag:'wx'});
const sources=()=>{assert.deepEqual(retainedSources(),o.current);for(const[p,h]of Object.entries({...o.files,...files,...o.libraries,...o.configurationFiles}))assert.equal(sha(p),h,p);};
sources();
await captured('symbolic-cleanup-control-compile-v83','.','gcc',['-O2','-g0','-Wall','-Wextra','-Werror','-Wno-unused-parameter',
 '-Dqqbar_set_fexpr=qqbar_set_fexpr_cleanup_control','-isystem',lib+'/src','flint-symbolic-boundary-fixed-v83.c',source,
 '-L',lib,'-Wl,-rpath,'+lib,'-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',binary]);sources();
for(const which of [0,1,2]){
 await captured('symbolic-cleanup-control-'+which+'-v83','.','timeout',['60s','prlimit','--as=1073741824','--cpu=30','--',binary,'powers',String(which),'256']);sources();
 await captured('symbolic-cleanup-control-mem-'+which+'-v83','.','timeout',['120s','valgrind','--tool=memcheck','--leak-check=full','--show-leak-kinds=all',
  '--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=97',binary,'powers',String(which),'256']);sources();
}
console.log(JSON.stringify({checkpoint:83,status:'cleanup-causal-control-collected',cases:3,rowsPerCase:257,productionChanges:0}));
