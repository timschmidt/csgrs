import {writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
export function cases(){
 const rows=[],add=(family,k,op,truth='equal',extra={})=>rows.push({id:rows.length,family,k,op,truth,...extra});
 for(const family of ['twelfth','perturbed'])for(const k of [1,5,7,11,13,17,19,23])for(const op of ['sin','cos','tan','cot'])add(family,k,op,family==='perturbed'?'not-equal':'equal');
 for(const k of [2,3,4])for(const op of ['sin','cos','tan','cot'])add('known-angle',k,op);
 for(const k of [1,13])for(const op of ['sin','cos','tan','cot'])add('periodic',k,op);
 for(const family of ['seventh-identity','rational-trig-identity','surd-trig-identity','exp-identity','rational-equal','rational-unequal','quadratic-identity','nested-surd-identity'])add(family,0,'control',family==='rational-unequal'?'not-equal':'equal');
 for(const count of [8,64])add('deep-seventh-identity',0,'control','equal',{count});
 for(const bits of [1000,1030])add('scaled-twelfth',1,'sin','equal',{bits});
 assert.equal(rows.length,96);return rows;
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 writeFileSync('twelfth-cost-input-v78.json',JSON.stringify(cases(),null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({checkpoint:78,cases:96,groups:576}));
}
