import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import {checkValue,piValue,selfTest,q} from './symbolic-boundary-oracle-v83.mjs';
import {scalar,fzero,fmul,integer} from './point-extended-field.mjs';
const scales=[q(-7,3),q(0),q(1),q(5,2)];
function expected(scale,k,op){
 const unit=piValue(6,BigInt(k),12n),s=scales[scale],real=fmul(scalar(s),unit.re),imag=fmul(scalar(s),unit.im);
 if(op===0)return{re:real,im:fzero()};
 if(op===1)return{re:imag,im:fzero()};
 if(op===2)return{re:scalar(q(s[0]<0n?-s[0]:s[0],s[1])),im:fzero()};
 if(op===3)return{re:scalar(q(s[0]*s[0],s[1]*s[1])),im:fzero()};
 if(op===4){const sign=integer(s[0]<0n?-1:s[0]>0n?1:0);return{re:fmul(sign,unit.re),im:fmul(sign,unit.im)};}
 return{re:real,im:imag};
}
function rowCheck(r,c){
 for(const[k,v]of Object.entries(c))assert.equal(r[k],v,k);
 const round=c.family==='roundtrip';
 if(round){
  assert([0,1].includes(r.generated));
  if(c.method<3)assert.equal(r.generated,1);
  if(c.method===3)assert.equal(r.generated,c.scale===1||c.k%12===0?1:0);
  if(r.generated)assert.equal(r.parsed,1,'generated formula must parse successfully');
 }
 const hasValue=!round||r.generated;
 assert.deepEqual(Object.keys(r).sort(),[...Object.keys(c),...(round?['generated',...(r.generated?['parsed']:[])]:[]),...(hasValue?['poly','real','imag']:[])].sort());
 return hasValue?checkValue(r,expected(c.scale,c.k,c.family==='component'?c.op:c.family==='pair'?c.part:5)):0;
}
export function checkComplexRecords(rows){
 assert.equal(rows.length,2497);assert.deepEqual(rows.at(-1),{terminal:true,rows:2496});
 let index=0,checks=0;const generated=Array(10).fill(0),declined=Array(10).fill(0);
 const consume=c=>{const r=rows[index++];checks+=rowCheck(r,c);if(c.family==='roundtrip')(r.generated?generated:declined)[c.method]++;};
 for(let scale=0;scale<4;scale++)for(let k=0;k<24;k++){
  for(let op=0;op<5;op++)for(let alias=0;alias<2;alias++)consume({family:'component',scale,k,op,alias});
  for(let alias=0;alias<3;alias++)for(let part=0;part<2;part++)consume({family:'pair',scale,k,alias,part});
  for(let method=0;method<10;method++)consume({family:'roundtrip',scale,k,method});
 }
 assert.equal(index,2496);return{inputs:96,records:2497,checks,componentRows:960,pairRows:576,roundtripRows:960,generated,declined};
}
export function complexEvidence(){
 selfTest();const raw=readFileSync('results/complex-roundtrip-native-v83.stdout');assert(raw.length>0&&raw.at(-1)===10);
 const rows=raw.toString().trimEnd().split('\n').map(JSON.parse),result=checkComplexRecords(rows);
 let corruptions=0;const c={family:'component',scale:0,k:1,op:0,alias:0},base=rows[26];
 for(const mutate of [r=>r.poly[0]='0',r=>r.real=['0','0','0'],r=>r.imag=['1','1','0'],r=>r.k++,r=>r.alias++,r=>r.extra=1]){
  const bad=structuredClone(base);mutate(bad);assert.throws(()=>rowCheck(bad,c));corruptions++;
 }
 for(const mutate of [r=>r.pop(),r=>r[1]=r[0],r=>r[r.length-1]={terminal:true,rows:2495}]){
  const bad=rows.slice();mutate(bad);assert.throws(()=>checkComplexRecords(bad));corruptions++;
 }
 const trip=rows.find(r=>r.family==='roundtrip'&&r.method===0),bad=structuredClone(trip);bad.parsed=0;
 assert.throws(()=>rowCheck(bad,{family:'roundtrip',scale:trip.scale,k:trip.k,method:0}));corruptions++;
 const gate=JSON.parse(readFileSync('results/complex-roundtrip-memcheck-v83.json')),
  mem=readFileSync('results/complex-roundtrip-memcheck-v83.stderr','utf8');
 assert.notEqual(gate.code,0);assert(mem.includes('fmpz_lll_is_reduced_d')&&mem.includes('SIGABRT'));
 return{checkpoint:83,status:'native-complex-and-roundtrip-correct-memory-qualification-failed',...result,bytes:raw.length,corruptions,
  memcheck:{code:gate.code,signal:gate.signal,status:'aborted in donor LLL assertion; incomplete stream, no clean-memory claim'},
  limits:'Native exact bounded field corpus only. Formula failures are optional capability outcomes. Memcheck abort cause not yet isolated; abort-time live allocations are not classified as normal-exit leaks.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(complexEvidence()));
