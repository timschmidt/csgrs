import {writeFileSync}from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json,workspace,retainedSources}from './zero-factor-retained-sources-v75.mjs';
import {captured}from './point-qualified-capture.mjs';
const o=json('qqbar-remainder-origin-v84.json'),lib=workspace+'/exact-real-references/flint',files={'flint-qqbar-remainder-fixed-v84.c':sha('flint-qqbar-remainder-fixed-v84.c'),'run-qqbar-remainder-qualified-v84.mjs':sha('run-qqbar-remainder-qualified-v84.mjs')};
writeFileSync('qqbar-remainder-run-origin-qualified-v84.json',JSON.stringify({recorded:new Date().toISOString(),files,current:retainedSources()},null,2)+'\n',{flag:'wx'});
const sources=()=>{assert.deepEqual(retainedSources(),o.current);for(const[p,h]of Object.entries({...o.files,...files,...o.libraries,...o.configurationFiles}))assert.equal(sha(p),h,p);
 assert.equal(sha(lib+'/src/qqbar/roots_poly_squarefree.c'),o.donorSources['flint:src/qqbar/roots_poly_squarefree.c']);};
sources();
const cc=['-O2','-g1','-ffunction-sections','-fdata-sections','-Wall','-Wextra','-Werror','-isystem',lib+'/src'],
 link=['-L',lib,'-Wl,-rpath,'+lib,'-Wl,--gc-sections','-lflint','-lmpfr','-lgmp','-lm','-lpthread'];
await captured('qqbar-remainder-compile-fixed-v84','.','gcc',[...cc,'flint-qqbar-remainder-fixed-v84.c',...link,'-o',o.binary]);sources();
await captured('qqbar-remainder-sanitized-compile-qualified-v84','.','gcc',[...cc,'-Wno-error=sign-compare','-fsanitize=undefined','-fno-sanitize-recover=undefined',
 '-D_qqbar_roots_poly_squarefree=qqbar_roots_squarefree_sanitized_v84','flint-qqbar-remainder-fixed-v84.c',lib+'/src/qqbar/roots_poly_squarefree.c',...link,'-o',o.sanitized]);sources();
await captured('qqbar-remainder-linkage-v84','.','ldd',[o.binary]);sources();
const outcomes=[];
async function diagnostic(tag,args){
 try{await captured(tag,'.','timeout',args);}catch(error){console.log(JSON.stringify({tag,diagnosticFailure:String(error)}));}
 const g=json('results/'+tag+'.json');outcomes.push({tag,code:g.code,signal:g.signal});sources();
}
const native=(binary,rest)=>['20s','prlimit','--core=0','--as=1073741824','--cpu=10','--',binary,...rest];
for(const generic of [0,1])for(const exponent of [0,30,63,64,80,256])for(const terms of [1,2]){
 await diagnostic('qqbar-monomial-'+generic+'-'+exponent+'-'+terms+'-v84',native(o.binary,['monomial',String(generic),String(exponent),String(terms)]));
}
for(const degree of [57,58,59,60,61,62,63]){
 await diagnostic('qqbar-root-limit-sanitized-'+degree+'-v84',native(o.sanitized,['limit',String(degree),'0']));
}
for(const binary of ['native','sanitized'])await diagnostic('qqbar-root-limit-bounded-'+binary+'-v84',native(binary==='native'?o.binary:o.sanitized,['limit','61','1']));
for(const limit of [1,2,4])for(const count of [1,32,128]){
 const rest=['cleanup',String(limit),String(count)];
 await diagnostic('qqbar-root-cleanup-'+limit+'-'+count+'-v84',native(o.binary,rest));
 await diagnostic('qqbar-root-cleanup-mem-'+limit+'-'+count+'-v84',['120s','prlimit','--core=0','--','valgrind','--tool=memcheck','--leak-check=full','--show-leak-kinds=all',
  '--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=97',o.binary,...rest]);
}
console.log(JSON.stringify({checkpoint:84,status:'diagnostics-collected-not-declared-numerically-passing',outcomes,productionChanges:0}));
