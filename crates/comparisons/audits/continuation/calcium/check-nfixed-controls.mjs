import {readFileSync} from 'node:fs';
import {dirname,resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url));
const read=p=>readFileSync(resolve(here,p),'utf8');
const hex=s=>{const m=/^(\-?)0x([0-9a-f]+)(?:\.([0-9a-f]+))?p([+-]?\d+)$/.exec(s);assert(m,s);
 return (m[1]?-1:1)*parseInt(m[2]+(m[3]??''),16)*2**(+m[4]-4*(m[3]?.length??0));};
function checkRun(native) {
 const lines=native.trimEnd().split('\n'),summary=JSON.parse(lines.pop()),rows=lines.map(s=>s.split(','));
 const expected=[];let comparisons=0,index=0,maxErrorRatio=0;
 const matrixGroups=Array.from({length:5},(_,algorithm)=>({algorithm,cases:0,comparisons:0,maxErrorRatio:0}));
 for(let n=2;n<=8;n++)for(const len of [1,2,3,8,17])for(let pattern=0;pattern<4;pattern++)for(let layout=0;layout<4;layout++) {
  const r=rows[index++];assert.equal(r.length,9);assert.deepEqual(r.slice(0,5),['dot',n,len,pattern,layout].map(String));
  const error=hex(r[5]),observed=hex(r[6]);assert.equal(error,(2*n-1)*len);assert(observed>=0&&observed<=error);
  assert.deepEqual(r.slice(7),['1','1']);comparisons++;
  maxErrorRatio=Math.max(maxErrorRatio,observed/error);expected.push(r);
 }
 const precisions=[2,3,4,8,12,22,47,66],shapes=[[1,1,1],[2,3,4],[3,4,2],[3,3,3],[8,8,8],[11,26,13],
  [26,26,26],[27,27,27],[36,36,36],[37,37,37],[50,50,50],[51,51,51],[56,56,56],[57,57,57],[58,57,59]];
 for(const[pi,limbs]of precisions.entries())for(const[m,n,p]of shapes.slice(0,pi<5?15:4))for(let pattern=0;pattern<4;pattern++)
  for(let algorithm=0;algorithm<5;algorithm++) {
   const r=rows[index++];assert.equal(r.length,12);assert.deepEqual(r.slice(0,7),['matrix',limbs,m,n,p,pattern,algorithm].map(String));
   const bound=hex(r[7]),error=hex(r[8]),observed=hex(r[9]);assert(bound>=0&&bound<1/64);assert(error>0&&observed>=0&&observed<=error);
   assert.deepEqual(r.slice(10),['0','1']);comparisons+=m*p;
   const g=matrixGroups[algorithm];g.cases++;g.comparisons+=m*p;g.maxErrorRatio=Math.max(g.maxErrorRatio,observed/error);expected.push(r);
  }
 const witness=[];
 for(const limbs of [2,3,4,8,12,66])for(let mode=0;mode<4;mode++) {
  const r=rows[index++];assert.equal(r.length,7);assert.deepEqual(r.slice(0,3),['bound',limbs,mode].map(String));
  const bound=hex(r[3]),actual=hex(r[4]);assert.equal(actual,2**-18);assert(bound>=3*2**-20&&bound<actual);
  assert.deepEqual(r.slice(5),['0','1']);witness.push({limbs,mode,bound,actual});expected.push(r);
 }
 const cutoffDiscrepancies=[];
 for(const limbs of [2,3,4,8,12,66])for(let n=24;n<=60;n++) {
  const r=rows[index++],cutoff=limbs<=3?(n%2?57:50):(n%2?37:26);
  assert.equal(r.length,9);assert.deepEqual(r.slice(0,4),['cutoff',limbs,n,cutoff].map(String));
  const [automatic,explicit,autoError,explicitError]=r.slice(4,8).map(hex);
  const equal=automatic===explicit&&autoError===explicitError;assert.equal(r[8],String(+equal));
  if(!equal){assert.equal(n%2,0);assert(automatic<explicit);assert(autoError<explicitError);
   cutoffDiscrepancies.push({limbs,n,cutoff,automatic,explicit,autoError,explicitError});}
  expected.push(r);
 }
 assert.equal(index,2546);assert.deepEqual(rows,expected);assert.equal(comparisons,1922900);assert.equal(cutoffDiscrepancies.length,52);
 assert.deepEqual(summary,{suite:'nfixed-controls',rows:2546,comparisons:1922900,failed_cases:24,bound_failures:24,cutoff_differences:52});
 const mem=read('results/nfixed-controls-memcheck.stderr');assert.match(mem,/in use at exit: 0 bytes in 0 blocks/);
 assert.match(mem,/17,379,886 allocs, 17,379,886 frees, 1,052,400,360 bytes allocated/);
 assert.match(mem,/ERROR SUMMARY: 0 errors from 0 contexts \(suppressed: 0 from 0\)/);
 for(const profile of ['debug','release']) {
  const out=read('results/nfixed-hyper-filter-'+profile+'.stdout');
  assert.deepEqual([...out.matchAll(/^test (.+) \.\.\. ok$/gm)].map(m=>m[1]),[
   'resolve::tests::signed_term_filter_decides_same_sign_terms_without_magnitude',
   'resolve::tests::signed_term_filter_leaves_mixed_signs_to_exact_pipeline']);
  assert.match(out,/2 passed; 0 failed; 0 ignored; 0 measured; 240 filtered out/);
 }
 return {summary,dotCases:560,dotMaxErrorRatio:maxErrorRatio,matrixGroups,witness,cutoffDiscrepancies,
  memory:{errors:0,liveBytes:0,allocations:17379886,requestedBytes:1052400360},hyperExistingTestsPerProfile:2};
}
export function checkFixedControls() {
 const native=read('results/nfixed-controls-native.stdout'),memory=read('results/nfixed-controls-memcheck.stdout');
 const result=checkRun(native),memoryResult=checkRun(memory);
 assert.deepEqual(result.summary,memoryResult.summary);assert.equal(result.dotMaxErrorRatio,memoryResult.dotMaxErrorRatio);
 const a=native.trimEnd().split('\n'),b=memory.trimEnd().split('\n');assert.equal(a.length,b.length);
 const fields={};let differingRows=0,differingFields=0;
 for(let i=0;i<a.length;i++)if(a[i]!==b[i]) {
  differingRows++;const x=a[i].split(','),y=b[i].split(',');assert.equal(x.length,y.length);
  for(let j=0;j<x.length;j++)if(x[j]!==y[j]) {
   const key=x[0]+':'+j;assert(['matrix:7','matrix:8','bound:3'].includes(key),key);
   fields[key]=(fields[key]??0)+1;differingFields++;
  }
 }
 assert.equal(differingRows,908);assert.equal(differingFields,1225);
 assert.deepEqual(fields,{'matrix:8':769,'matrix:7':450,'bound:3':6});
 const controls=['native','memcheck'].map(v=>{
  const lines=read('results/nfixed-fenv-'+v+'.stdout').trimEnd().split('\n'),summary=JSON.parse(lines.pop());
  assert.deepEqual(summary,{suite:'fenv-product',rows:12,failures:v==='native'?0:4});
  return {rows:lines.map(s=>s.split(',')),summary};
 });
 for(let mode=0;mode<4;mode++)for(let i=0;i<3;i++) {
  const n=controls[0].rows[3*mode+i],v=controls[1].rows[3*mode+i];assert.equal(n.length,6);assert.equal(v.length,6);
  for(const r of[n,v]){assert.deepEqual(r.slice(0,2),[mode,i].map(String));assert.equal(r[5],'1');assert.equal(r[4],String(+(r[2]===r[3])));}
  assert.equal(n[2],n[3]);assert.equal(v[3],n[3]);assert.equal(v[2],controls[0].rows[i][2]);
 }
 const mem=read('results/nfixed-fenv-memcheck.stderr');assert.match(mem,/in use at exit: 0 bytes in 0 blocks/);
 assert.match(mem,/ERROR SUMMARY: 0 errors from 0 contexts/);
 return {...result,memoryRun:{summary:memoryResult.summary,matrixGroups:memoryResult.matrixGroups,witness:memoryResult.witness},
  instrumentation:{differingRows,differingFields,fields,fenv:controls}};
}
if(process.argv.includes('--nfixed-controls-summary'))console.log(JSON.stringify(checkFixedControls()));
