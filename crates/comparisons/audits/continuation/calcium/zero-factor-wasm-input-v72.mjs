import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
const read=p=>readFileSync(p,'utf8').trimEnd().split('\n').map(JSON.parse);
export function fullCases(){
 const cases=[];
 for(const [corpus,tag,count]of [['public','power-rebased-baseline-public-v68',6440],['wide','power-wide-baseline-run-v69',1840],['degree','zero-factor-degree-baseline-v70',66]]){
  const rows=read('results/'+tag+'.stdout').filter(r=>r.report&&(corpus==='public'||r.policy===0));assert.equal(rows.length,count);
  for(const [index,row]of rows.entries())cases.push({id:cases.length,label:corpus+'-'+index,source:{corpus,index},
   left:row.left,right:row.right,operation:row.report.operation});
 }assert.equal(cases.length,8346);return cases;
}
const cache=new Map();
export function fullReference(c,policy,variant){
 const {corpus,index}=c.source;let tag;
 if(corpus==='public')tag=variant==='baseline'?'power-rebased-baseline-'+(policy?'approx':'public')+'-v68':'zero-factor-'+(policy?'approx':'public')+'-run-v70';
 else if(corpus==='wide')tag=variant==='baseline'?'power-wide-baseline-run-v69':'zero-factor-wide-run-v70';
 else tag=variant==='baseline'?'zero-factor-degree-baseline-v70':'zero-factor-degree-run-v70';
 const key=tag+':'+policy;
 if(!cache.has(key))cache.set(key,read('results/'+tag+'.stdout').filter(r=>r.report&&(corpus==='public'||r.policy===policy)));
 const row=cache.get(key)[index];assert(row);assert.deepEqual(row.left,c.left);assert.deepEqual(row.right,c.right);assert.equal(row.report.operation,c.operation);
 return row.report;
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const cases=fullCases();writeFileSync('zero-factor-wasm-input-v72.json',JSON.stringify(cases)+'\n',{flag:'wx'});
 console.log(JSON.stringify({cases:cases.length,queriesPerVariant:cases.length*2}));
}
