import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {correctedConversionSource} from './prepare-arf-conversion-v2.mjs';
const read=p=>readFileSync(p,'utf8');
const log=(tag,ext)=>read('results/'+tag+'.'+ext);
const hex=s=>s.startsWith('-')?-BigInt('0x'+s.slice(1)):BigInt('0x'+s);
const abs=n=>n<0n?-n:n,sgn=n=>Number(n>0n)-Number(n<0n);
const q=(m,e)=>e<0?[m,1n<<BigInt(-e)]:[m<<BigInt(e),1n];
const cmp=([a,b],[c,d])=>sgn(a*d-c*b);
const canon=(m,e)=>{if(!m)return['0',0];while(m%2n===0n){m/=2n;e++;}return[m.toString(16),e];};
const INF=0x7ff0000000000000n,MAX=INF-1n,SIGN=1n<<63n;
// Monotone bit encoding -> exact value. Infinity's rounding neighbor is the
// finite mathematical value 2^1024, used only for the overflow midpoint.
const bitsValue=bits=>{if(bits===INF)return[1n,1024];const be=Number(bits>>52n);
 return[(bits&((1n<<52n)-1n))+(be?1n<<52n:0n),be?be-1075:-1074];};
function bracket(target) {
 let lo=0n,hi=INF;
 while(hi-lo>1n) {const mid=(lo+hi)>>1n;
  if(cmp(q(...bitsValue(mid)),target)<=0)lo=mid;else hi=mid;}
 const lower=q(...bitsValue(lo)),upper=q(...bitsValue(hi));
 const exact=cmp(lower,target)===0;
 const midpoint=[lower[0]*upper[1]+upper[0]*lower[1],2n*lower[1]*upper[1]];
 return{lo,hi,exact,midcmp:cmp(target,midpoint)};
}
function binaryResult(b,negative,mode,zero) {
 if(zero)return 0n;
 const away=mode===1||(mode===2&&negative)||(mode===3&&!negative);
 let r=b.lo;
 if(!b.exact&&(mode===4?(b.midcmp>0||(b.midcmp===0&&(b.lo&1n)!==0n)):away))r=b.hi;
 return r|(negative?SIGN:0n);
}
function integerCheck(n,target,mode) {
 const[a,b]=target,negative=a<0n,mag=abs(a),v=abs(n),delta=v*b-mag;
 assert(n===0n||(n<0n)===negative);
 const exact=delta===0n;
 if(mode===4) {assert(abs(2n*delta)<=b);if(abs(2n*delta)===b)assert.equal(v%2n,0n);}
 else {const away=mode===1||(mode===2&&negative)||(mode===3&&!negative);
  if(away)assert(delta>=0n&&delta<b);else assert(delta<=0n&&delta>-b);}
 return{exact,tie:mode===4&&abs(2n*delta)===b};
}
function fixtures() {
 const out=[],widths=[1,2,3,31,32,33,52,53,54,63,64,65,127,128,129,255,256,257];
 const peaks=[-1100,-1076,-1075,-1074,-1023,-1022,-1021,-1,0,1,52,53,54,970,971,972,1022,1023,1024,1100];
 const add=(m,e)=>out.push({m,e});
 for(const w of widths)for(let v=0;v<3;v++) {
  const base=v===0?1n<<BigInt(w-1):v===1?(1n<<BigInt(w))-1n:
   (1n<<BigInt(w-1))|(1n<<BigInt(Math.floor(w/2)))|1n;
  for(const peak of peaks)for(const sign of [1n,-1n])add(sign*base,peak+1-base.toString(2).length);
 }
 for(const grid of [-1074,-1022,-53,0,971])for(const base of
  [0n,1n,2n,(1n<<52n)-1n,1n<<52n,(1n<<52n)+1n,(1n<<53n)-2n,(1n<<53n)-1n])
  for(const delta of [1n,2n,3n])for(const sign of [1n,-1n])add(sign*(4n*base+delta),grid-2);
 add(0n,0);assert.equal(out.length,2401);return out;
}
export function checkArfConversion() {
 assert.equal(read('flint-arf-conversion-v2-controls.c'),correctedConversionSource());
 const native=log('arf-conversion-v2-native','stdout');assert.equal(native,log('arf-conversion-v2-memcheck','stdout'));
 const rows=native.trimEnd().split('\n').map(JSON.parse),summary=rows.pop(),fs=fixtures();
 assert.equal(rows.length,fs.length);
 let previous=[0n,1n],checks=0,imports=0,siChecks=0,ties=0,subnormal=0,signedZeros=0,infinities=0,binaryTies=0,integerTies=0;
 for(let i=0;i<fs.length;i++) {
  const r=rows[i],{m,e}=fs[i],target=q(m,e),at=[abs(target[0]),target[1]],canonical=canon(m,e);
  assert.deepEqual([r.id,r.m,r.e],[i,m.toString(16),e]);
  assert.deepEqual(r.input,canonical);assert.deepEqual(r.preserved,canonical);checks+=2;
  const b=bracket(at),negative=m<0n,top=m?abs(m).toString(2).length+e:0;
  assert.equal(r.double.length,5);
  for(let mode=0;mode<5;mode++) {
   const bits=binaryResult(b,negative,mode,m===0n),entry=r.double[mode];
   assert.equal(entry[0],bits.toString(16).padStart(16,'0'));checks++;
   const magnitude=bits&~SIGN;
   subnormal+=Number(magnitude>0n&&magnitude<(1n<<52n));
   signedZeros+=Number(bits===SIGN);infinities+=Number(magnitude===INF);
   if(magnitude===INF)assert.equal(entry[1],null);
   else {let[bm,be]=bitsValue(magnitude);if(negative)bm=-bm;
    assert.deepEqual(entry[1],canon(bm,be));checks++;imports++;}
  }
  if(m&&top-1<=1023&&!b.exact&&b.midcmp===0){ties++;binaryTies++;}
  assert.equal(r.integer.length,5);
  for(let mode=0;mode<5;mode++) {
   const[n,ret,si]=r.integer[mode],integer=hex(n),result=integerCheck(integer,target,mode);
   assert.equal(ret,Number(!result.exact));checks++;
   const fits=integer>=-(1n<<63n)&&integer<(1n<<63n);
   if(fits){assert.equal(si,integer.toString());checks++;siChecks++;}else assert.equal(si,null);
   if(result.tie){ties+=2;integerTies++;}
  }
  assert.equal(r.integralOps.length,6);
  for(let op=0;op<3;op++)for(let alias=0;alias<2;alias++) {
   const expected=canon(hex(r.integer[op+2][0]),0);
   assert.deepEqual(r.integralOps[2*op+alias],expected);checks++;
  }
  assert.equal(r.frexp.length,2);
  for(const result of r.frexp){assert.deepEqual(result,[canon(m,e-top),top]);checks+=2;}
  const bottom=canonical[1],cm=hex(canonical[0]);
  assert.equal(r.isInt,Number(m===0n||bottom>=0));checks++;
  if(m) {assert.deepEqual(r.bounds,[top-Number(abs(cm)===1n),top,String(top)]);checks+=3;}
  else {assert.deepEqual(r.bounds,[null,null,(-(1n<<63n)+1n).toString()]);checks++;}
  const pe=[-1074,-1,0,bottom-1,bottom,bottom+1,top];assert.equal(r.powers.length,7);
  for(let j=0;j<7;j++){const p=q(1n,pe[j]);assert.deepEqual(r.powers[j],
   [pe[j],Number(m===0n||bottom>=pe[j]),cmp(target,p),cmp(at,p)]);checks+=2;}
  assert.deepEqual(r.previousCmp,[cmp(target,previous),cmp(at,[abs(previous[0]),previous[1]])]);checks++;
  previous=target;
 }
 assert.deepEqual(summary,{summary:true,fixtures:2401,checks:117705,failures:0,ties:300,aliases:9604,
  finiteImports:11281,signedConversions:7985});
 assert.equal(checks,summary.checks);assert.equal(imports,summary.finiteImports);assert.equal(siChecks,summary.signedConversions);assert.equal(ties,summary.ties);
 // The failed v1 differs only in bounds serialization/zero expectations.
 const oldRows=log('arf-conversion-native','stdout').trimEnd().split('\n').map(JSON.parse),oldSummary=oldRows.pop();
 assert.deepEqual(oldSummary,{...summary,checks:117707,failures:1});
 assert.equal(oldRows.length,rows.length);
 for(let i=0;i<rows.length;i++){
  const old=oldRows[i],current=structuredClone(rows[i]);
  if(i===2400)assert.deepEqual(old.bounds,[0,0,Number(-9223372036854775807n)]);
  else assert.deepEqual(old.bounds,[...current.bounds.slice(0,2),Number(current.bounds[2])]);
  delete old.bounds;delete current.bounds;assert.deepEqual(old,current);
 }
 assert.equal(log('arf-conversion-native','stderr'),'FAIL fixture 2400 check 117691: bounded signed exponent\n');
 const gateTags=['arf-conversion-compile','arf-conversion-native','arf-conversion-linked-libraries',
  'arf-conversion-v2-compile','arf-conversion-v2-native','arf-conversion-v2-linked-libraries','arf-conversion-v2-memcheck'];
 for(const tag of gateTags){const g=JSON.parse(log(tag,'json'));assert.equal(g.code,tag==='arf-conversion-native'?1:0);
  assert.equal(g.signal,null);assert(Date.parse(g.finished)>=Date.parse(g.started));
  if(!tag.endsWith('memcheck')&&tag!=='arf-conversion-native')assert.equal(log(tag,'stderr'),'');}
 const mem=log('arf-conversion-v2-memcheck','stderr');
 assert.match(mem,/ERROR SUMMARY: 0 errors from 0 contexts \(suppressed: 0 from 0\)/);
 assert.match(mem,/in use at exit: 0 bytes in 0 blocks/);assert.match(mem,/All heap blocks were freed/);
 const usage=mem.match(/total heap usage: ([\d,]+) allocs, ([\d,]+) frees, ([\d,]+) bytes allocated/).slice(1).map(s=>Number(s.replaceAll(',','')));
 assert.equal(usage[0],usage[1]);
 return{summary,independentAssertions:checks,binary64BitResults:rows.length*5,nearestEvenBinary64:rows.length,
  binaryMidpoints:binaryTies,integerMidpoints:integerTies,subnormalResults:subnormal,negativeZeroResults:signedZeros,
  overflowInfinityResults:infinities,integerValues:rows.length*5,integralOperationResults:rows.length*6,
  publicAliases:summary.aliases,identicalV2NumericalBytes:Buffer.byteLength(native),preservedV1Bytes:Buffer.byteLength(log('arf-conversion-native','stdout')),
  memory:{errors:0,suppressed:0,liveBytes:0,liveBlocks:0,allocations:usage[0],frees:usage[1],cumulativeBytesIncludingOracle:usage[2]},
  limits:'Finite dyadics through257 mantissa bits and peak exponents -1100..1100, native64 ADX/FE_TONEAREST only. Exact GMP grid oracle and independent BigInt monotone binary64 search/full integer certificates; complete values, not residues. Zero fmpz magnitude bounds skipped because unspecified; signed sentinel serialized exactly. No NaN/infinity input, excessive exponent, invalid memory, parser/crash reproduction or full upstream suite. No donor-only speed/allocation/peak measurement or new Hyper performance claim. Original one-failure harness and binary preserved; no donor defect inferred.'};
}
if(process.argv.includes('--arf-conversion-summary'))console.log(JSON.stringify(checkArfConversion()));
