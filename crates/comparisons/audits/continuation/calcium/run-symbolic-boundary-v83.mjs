import assert from 'node:assert/strict';
import {writeFileSync}from 'node:fs';
import {sha,json,workspace,retainedSources}from './zero-factor-retained-sources-v75.mjs';
import {captured}from './point-qualified-capture.mjs';
const o=json('symbolic-boundary-origin-v83.json'),lib=workspace+'/exact-real-references/flint';
const files={'run-symbolic-boundary-v83.mjs':sha('run-symbolic-boundary-v83.mjs')};
writeFileSync('symbolic-boundary-run-origin-v83.json',JSON.stringify({recorded:new Date().toISOString(),files},null,2)+'\n',{flag:'wx'});
const sources=()=>{assert.deepEqual(retainedSources(),o.current);for(const[p,h]of Object.entries({...o.files,...files,...o.libraries,...o.configurationFiles}))assert.equal(sha(p),h,p);};
sources();
await captured('symbolic-boundary-compile-v83','.','gcc',['-O2','-g0','-Wall','-Wextra','-Werror','-isystem',lib+'/src',
 'flint-symbolic-boundary-v83.c','-L',lib,'-Wl,-rpath,'+lib,'-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',o.binary]);sources();
await captured('symbolic-boundary-linkage-v83','.','ldd',[o.binary]);sources();
await captured('symbolic-boundary-native-v83','.','timeout',['60s','prlimit','--as=1073741824','--cpu=30','--',o.binary,'angles']);sources();
const mem=['120s','valgrind','--tool=memcheck','--leak-check=full','--show-leak-kinds=all',
 '--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=97',o.binary];
await captured('symbolic-boundary-memcheck-v83','.','timeout',[...mem,'angles']);sources();
for(const which of [0,1,2])for(const count of [1,32,256]){
 const tag='symbolic-power-'+which+'-'+count+'-v83';
 await captured(tag,'.','timeout',['60s','prlimit','--as=1073741824','--cpu=30','--',o.binary,'powers',String(which),String(count)]);sources();
 const mt='symbolic-power-mem-'+which+'-'+count+'-v83';
 try{await captured(mt,'.','timeout',[...mem,'powers',String(which),String(count)]);}
 catch(error){const g=json('results/'+mt+'.json');assert.equal(g.code,97);assert.equal(g.signal,null);console.log(JSON.stringify({diagnosticGate:mt,code:97,note:'Memcheck reported errors/loss; preserved as diagnostic failure, not clean success.'}));}
 sources();
}
console.log(JSON.stringify({checkpoint:83,status:'symbolic-boundary-collected',angleRows:309,powerCases:9,productionChanges:0}));
