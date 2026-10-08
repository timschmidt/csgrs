import {writeFileSync}from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json,workspace,retainedSources}from './zero-factor-retained-sources-v75.mjs';
import {captured}from './point-qualified-capture.mjs';
const o=json('qqbar-remainder-origin-v84.json'),lib=workspace+'/exact-real-references/flint',binary=o.dir+'/cleanup-cubic';
const files=Object.fromEntries(['flint-qqbar-cleanup-cubic-v84.c','run-qqbar-cleanup-cubic-v84.mjs'].map(p=>[p,sha(p)]));
writeFileSync('qqbar-cleanup-cubic-origin-v84.json',JSON.stringify({recorded:new Date().toISOString(),files,binary,current:retainedSources(),reason:'Quadratic cases at limits 1 and 2 reject before root allocation; cubic input x^3-sqrt(2) tests post-factorization failure at limit 4.'},null,2)+'\n',{flag:'wx'});
const sources=()=>{assert.deepEqual(retainedSources(),o.current);for(const[p,h]of Object.entries({...o.files,...files,...o.libraries,...o.configurationFiles}))assert.equal(sha(p),h,p);};
sources();
await captured('qqbar-cleanup-cubic-compile-v84','.','gcc',['-O2','-g1','-ffunction-sections','-fdata-sections','-Wall','-Wextra','-Werror','-isystem',lib+'/src','flint-qqbar-cleanup-cubic-v84.c','-L',lib,'-Wl,-rpath,'+lib,'-Wl,--gc-sections','-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',binary]);
const outcomes=[];
for(const limit of [4,8])for(const count of [1,32,128])for(const mode of ['native','mem']){
 const tag='qqbar-cleanup-cubic-'+mode+'-'+limit+'-'+count+'-v84',rest=['cleanup',String(limit),String(count)];
 const args=mode==='native'?['20s','prlimit','--core=0','--as=1073741824','--cpu=10','--',binary,...rest]:['120s','prlimit','--core=0','--','valgrind','--tool=memcheck','--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=97',binary,...rest];
 try{await captured(tag,'.','timeout',args);}catch(error){console.log(JSON.stringify({tag,diagnosticFailure:String(error)}));}
 const g=json('results/'+tag+'.json');outcomes.push({tag,code:g.code,signal:g.signal});sources();
}
console.log(JSON.stringify({checkpoint:84,status:'cubic-diagnostics-collected-not-numerically-qualified',outcomes,productionChanges:0}));
