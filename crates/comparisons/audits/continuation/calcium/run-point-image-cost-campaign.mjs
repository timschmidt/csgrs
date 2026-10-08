import {spawn} from 'node:child_process';
import assert from 'node:assert/strict';
for(const mode of ['cpu','allocation']){
 const c=spawn(process.execPath,['capture.mjs','point-image-cost-'+mode,'.','node','run-point-image-costs.mjs',mode],{stdio:'inherit'});
 const r=await new Promise((ok,fail)=>{c.on('error',fail);c.on('close',(code,signal)=>ok({code,signal}));});
 assert.equal(r.code,0,mode);assert.equal(r.signal,null,mode);
}
console.log(JSON.stringify({status:'pass',gates:['point-image-cost-cpu','point-image-cost-allocation']}));
