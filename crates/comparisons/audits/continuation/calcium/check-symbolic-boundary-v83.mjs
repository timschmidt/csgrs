import {readFileSync}from 'node:fs';
import {resolve}from 'node:path';
import {fileURLToPath}from 'node:url';
import assert from 'node:assert/strict';
import {cases,checkRow,selfTest,checkValue,piValue}from './symbolic-boundary-oracle-v83.mjs';
function checkRecords(rows){
 const inputs=cases();assert.equal(rows.length,309);assert.deepEqual(rows.at(-1),{terminal:true,rows:308});let checks=0;const counts={},defects=[];
 for(let i=0;i<inputs.length;i++){const out=checkRow(rows[i],inputs[i]);checks+=out.checks;counts[out.outcome]=(counts[out.outcome]??0)+1;
  if(out.outcome.startsWith('false-'))defects.push({...inputs[i],kind:out.outcome});}
 return{records:309,checks,counts,defects};
}
export function symbolicEvidence(){
 const oracle=selfTest(),raw=readFileSync('results/symbolic-boundary-native-v83.stdout');
 assert(raw.equals(readFileSync('results/symbolic-boundary-memcheck-v83.stdout')));assert(raw.length>0&&raw.at(-1)===10);
 const rows=raw.toString('utf8').trimEnd().split('\n').map(JSON.parse),result=checkRecords(rows),inputs=cases();
 let corruptions=0;const index=inputs.findIndex(c=>c.family==='large-angle'&&c.exponent===62),base=rows[index],c=inputs[index];
 for(const mutate of [r=>r.poly[0]='0',r=>r.real=['0','0','0'],r=>r.imag=['1','1','0'],r=>r.ok=0,r=>r.initial=0,r=>r.exponent=61]){
  const bad=structuredClone(base);mutate(bad);assert.throws(()=>checkRow(bad,c));corruptions++;
 }
 assert.throws(()=>checkRecords(rows.slice(0,-1)));corruptions++;
 const duplicate=rows.slice();duplicate[1]=rows[0];assert.throws(()=>checkRecords(duplicate));corruptions++;
 const powers=[];
 for(const which of [0,1,2])for(const count of [1,32,256]){
  const tag='symbolic-power-'+which+'-'+count+'-v83',mt='symbolic-power-mem-'+which+'-'+count+'-v83',a=readFileSync('results/'+tag+'.stdout');
  assert(a.equals(readFileSync('results/'+mt+'.stdout')));const rs=a.toString().trimEnd().split('\n').map(JSON.parse);
  assert.equal(rs.length,count+1);assert.deepEqual(rs.at(-1),{terminal:true,rows:count});
  for(let i=0;i<count;i++){
   const r=rs[i];assert.equal(r.family,'power');assert.equal(r.which,which);assert.equal(r.iteration,i);assert.equal(r.ok,which?0:1);
   if(which)assert.deepEqual(Object.keys(r).sort(),['family','which','iteration','ok'].sort());else checkValue(r,piValue(1,0n,1n));
  }
  const mem=readFileSync('results/'+mt+'.stderr','utf8'),g=JSON.parse(readFileSync('results/'+mt+'.json','utf8'));
  assert.equal(g.code,which?97:0);assert.equal(g.signal,null);
  const numbers=s=>Number(s.replaceAll(',',''));
  const heap=mem.match(/in use at exit: ([\d,]+) bytes in ([\d,]+) blocks/),lost=mem.match(/definitely lost: ([\d,]+) bytes in ([\d,]+) blocks/),
   indirect=mem.match(/indirectly lost: ([\d,]+) bytes in ([\d,]+) blocks/),total=mem.match(/([\d,]+) allocs, ([\d,]+) frees, ([\d,]+) bytes allocated/),
   errors=mem.match(/ERROR SUMMARY: (\d+) errors from (\d+) contexts/);assert(heap&&total&&errors);
  if(which){assert(lost&&indirect);assert.equal(numbers(lost[1]),69632);assert.equal(numbers(lost[2]),1);
   assert.equal(numbers(indirect[1]),16*count);assert.equal(numbers(indirect[2]),count);assert.equal(numbers(heap[1]),69632+16*count);assert.equal(numbers(heap[2]),count+1);
   assert.equal(errors[1],'2');assert(mem.includes('possibly lost: 0 bytes')&&mem.includes('still reachable: 0 bytes'));}
  else{assert.equal(numbers(heap[1]),0);assert.equal(numbers(heap[2]),0);assert.equal(errors[1],'0');}
  powers.push({which,count,code:g.code,liveBytes:numbers(heap[1]),liveBlocks:numbers(heap[2]),
   directLostBytes:lost?numbers(lost[1]):0,indirectLostBytes:indirect?numbers(indirect[1]):0,
   allocations:numbers(total[1]),frees:numbers(total[2]),requestedBytes:numbers(total[3]),errors:Number(errors[1])});
 }
 return{checkpoint:83,status:'independently-classified-expression-defects',oracle,...result,bytes:raw.length,corruptions,powers,
  limits:'Pinned current library behavior: false-success outcomes are confirmed defects, not passing numerical answers. Expected Memcheck failures remain failures. Archive tests not executed; malformed trusted serialization is not tested.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(symbolicEvidence()));
