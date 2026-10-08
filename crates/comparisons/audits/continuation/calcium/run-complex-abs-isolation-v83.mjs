import {writeFileSync}from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json,workspace,retainedSources}from './zero-factor-retained-sources-v75.mjs';
import {captured}from './point-qualified-capture.mjs';
const o=json('symbolic-boundary-origin-v83.json'),lib=workspace+'/exact-real-references/flint',binary=o.dir+'/isolated-abs';
const files=Object.fromEntries(['run-complex-abs-isolation-v83.mjs','flint-complex-abs-isolation-v83.c','flint-symbolic-boundary-fixed-v83.c'].map(p=>[p,sha(p)]));
writeFileSync('complex-abs-isolation-origin-v83.json',JSON.stringify({recorded:new Date().toISOString(),files,binary,current:retainedSources()},null,2)+'\n',{flag:'wx'});
const sources=()=>{assert.deepEqual(retainedSources(),o.current);for(const[p,h]of Object.entries({...o.files,...files,...o.libraries,...o.configurationFiles}))assert.equal(sha(p),h,p);};
sources();
await captured('complex-abs-isolation-compile-v83','.','gcc',['-O2','-g0','-ffunction-sections','-fdata-sections','-Wall','-Wextra','-Werror','-isystem',lib+'/src',
 'flint-complex-abs-isolation-v83.c','-L',lib,'-Wl,-rpath,'+lib,'-Wl,--gc-sections','-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',binary]);sources();
const outcomes=[];
for(const mode of ['native','none','memcheck']){
 const tag='complex-abs-isolation-'+mode+'-v83',args=['60s','prlimit','--core=0','--cpu=30','--',...(mode==='native'?[]:['valgrind','--tool='+mode,...(mode==='memcheck'?['--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=97']:[])]),binary];
 try{await captured(tag,'.','timeout',args);}catch(error){outcomes.push({tag,diagnosticFailure:String(error)});}
 const g=json('results/'+tag+'.json');outcomes.push({tag,code:g.code,signal:g.signal});sources();
}
console.log(JSON.stringify({checkpoint:83,status:'isolated-abs-diagnostics-collected',outcomes,productionChanges:0}));
