import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {cases,checkRow,selfTest,expected} from './quadratic-extraction-oracle-v82.mjs';
export function checkRecords(rows){
 const inputs=cases();assert.equal(rows.length,4651);assert.deepEqual(rows.at(-1),{terminal:true,rows:4650});
 let checks=0,index=0;const counts={},changedMode0=[],changedMode2=[];
 for(const c of inputs){
  const outputs=[];
  for(let mode=0;mode<3;mode++)for(let phase=0;phase<2;phase++){
   const r=rows[index++];checks+=checkRow(r,c,mode,phase);counts[c.family]=(counts[c.family]??0)+1;
   if(phase){assert.deepEqual(['a','b','c','q'].map(k=>r[k]),['a','b','c','q'].map(k=>rows[index-2][k]));checks++;}
   else outputs.push(r);
  }
  if(outputs[0].c!==outputs[1].c)changedMode0.push(c.id);
  if(outputs[2].c!==outputs[1].c)changedMode2.push(c.id);
 }
 return{inputs:775,records:4651,checks,families:counts,mode0NonminimalInputs:changedMode0,mode2NonminimalInputs:changedMode2};
}
export function quadraticEvidence(){
 selfTest();const raw=readFileSync('results/quadratic-extraction-native-v82.stdout');
 assert(raw.equals(readFileSync('results/quadratic-extraction-memcheck-v82.stdout')));
 assert(raw.length>0&&raw.at(-1)===10);const rows=raw.toString('utf8').trimEnd().split('\n').map(JSON.parse),result=checkRecords(rows);
 const inputs=cases(),index=rows.findIndex(r=>r.mode===1&&r.phase===0&&r.b!=='0'&&BigInt(r.b)%3n===0n&&BigInt(r.c)>1n),base=rows[index],c=inputs[base.id];
 assert(index>=0);let corruptions=0;
 const reject=f=>{const r=structuredClone(base);f(r);assert.throws(()=>checkRow(r,c,1,0));corruptions++;};
 reject(r=>r.a=String(BigInt(r.a)+1n));reject(r=>r.b=String(-BigInt(r.b)));reject(r=>r.c=String(-BigInt(r.c)));
 reject(r=>r.q=String(-BigInt(r.q)));reject(r=>{for(const k of ['a','b','q'])r[k]=String(2n*BigInt(r[k]));});
 reject(r=>{r.b=String(BigInt(r.b)/3n);r.c=String(BigInt(r.c)*9n);});
 reject(r=>r.poly[0]=String(BigInt(r.poly[0])+1n));reject(r=>r.reconstructedPoly[0]=String(BigInt(r.reconstructedPoly[0])+1n));
 reject(r=>r.real=['0','0','0']);reject(r=>r.reconstructedReal=['0','0','0']);
 reject(r=>r.real=[r.real[1],r.real[0],r.real[2]]);reject(r=>r.real=['-1','1','0']);
 reject(r=>r.id++);reject(r=>r.mode=0);reject(r=>r.phase=1);
 const imag=rows.find(r=>r.mode===1&&r.phase===0&&expected(inputs[r.id]).imag.b[0]!==0n),bad=structuredClone(imag);
 bad.imag=['0','0','0'];assert.throws(()=>checkRow(bad,inputs[bad.id],1,0));corruptions++;
 for(const mutate of [r=>r.pop(),r=>r.splice(0,1),r=>r[0]=r[1],r=>r[r.length-1].rows--]){
  const r=rows.slice();r[r.length-1]={...r.at(-1)};mutate(r);assert.throws(()=>checkRecords(r));corruptions++;
 }
 assert.equal(corruptions,20);
 return{checkpoint:82,status:'independently-checked-quadratic-extraction',...result,bytes:raw.length,corruptions,
  limits:'Known degree-1/2 corpus, not all quadratics. Complete/smooth factoring cost is not benchmarked. Both selected components checked independently; archive/upstream tests not newly executed.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(quadraticEvidence()));
