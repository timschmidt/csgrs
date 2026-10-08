import {spawn} from 'node:child_process';
import assert from 'node:assert/strict';
import {sha,json} from './point-image-sources.mjs';
const b=json('point-image-guard-binaries.json');
for(const p of Object.values(b.binaries))assert.equal(sha(p.path),p.sha256);
const gates=[['public',b.binaries.cpu.path,'check'],
 ['public-check','node','check-point-image.mjs','results/power-sums-public-baseline.stdout','results/point-image-guard-public.stdout'],
 ['memcheck','valgrind','--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=99',b.binaries.cpu.path,'check']];
for(const[tag,...args]of gates){
 const c=spawn(process.execPath,['capture.mjs','point-image-guard-'+tag,'.',...args],{stdio:'inherit'});
 const r=await new Promise((ok,fail)=>{c.on('error',fail);c.on('close',(code,signal)=>ok({code,signal}));});
 assert.equal(r.code,0,tag);assert.equal(r.signal,null,tag);
}
console.log(JSON.stringify({status:'pass',gates:gates.map(g=>'point-image-guard-'+g[0])}));
