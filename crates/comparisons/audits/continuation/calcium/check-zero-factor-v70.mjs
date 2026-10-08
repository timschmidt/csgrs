import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {expectedBinary,determinantCount,q,qc,parse} from './zero-factor-polynomial-v70.mjs';
import {root,rootCount,image,evaluate} from './power-rational-oracle-v69.mjs';
import {degreeCases} from './zero-factor-degree-cases-v70.mjs';
import {makeWideCases} from './power-wide-cases-v69.mjs';
const read=p=>readFileSync(p,'utf8').trimEnd().split('\n').map(JSON.parse);
const operations=['Add','Subtract','Multiply','Divide'];
// A closed singleton contains one distinct polynomial root exactly when direct
// rational evaluation vanishes; no GCD/Sturm sequence is needed for that case.
function exactCount(p,lo,hi){return qc(lo,hi)===0?Number(evaluate(p,lo)[0]===0n):rootCount(p,lo,hi);}
function imageCount(p,lo,hi){const count=exactCount(p,lo,hi);return count-Number(qc(lo,hi)!==0&&evaluate(p,lo)[0]===0n);}
export function checkZeroFactor(){
 assert.deepEqual(JSON.parse(readFileSync('zero-factor-degree-input-v70.json','utf8')),degreeCases());
 assert.deepEqual(JSON.parse(readFileSync('power-wide-input-v69.json','utf8')),makeWideCases());
 const corpora=[
  ['public','power-rebased-baseline-public-v68','zero-factor-public-run-v70'],
  ['approx','power-rebased-baseline-approx-v68','zero-factor-approx-run-v70'],
  ['wide','power-wide-baseline-run-v69','zero-factor-wide-run-v70'],
  ['degree','zero-factor-degree-baseline-v70','zero-factor-degree-run-v70']
 ];
 const counts={},failures=[],summaries={};
 const check=(name,ok,detail)=>{counts[name]=(counts[name]??0)+1;if(!ok)failures.push({name,...detail});};
 for(const [corpus,bt,ct]of corpora){
  const before=read('results/'+bt+'.stdout'),after=read('results/'+ct+'.stdout');
  assert.equal(before.length,after.length);assert.deepEqual(before.at(-1),after.at(-1));
  const summary={queries:before.length-1,unchanged:0,changed:0,transitions:{},baseline:{},candidate:{}};
  for(let i=0;i<before.length-1;i++){
   const a=before[i],b=after[i],detail={corpus,row:i};
   assert.deepEqual({...a,report:undefined},{...b,report:undefined});
   const operation=operations.indexOf(b.report.operation);assert(operation>=0);
   const changed=JSON.stringify(a)!==JSON.stringify(b);summary[changed?'changed':'unchanged']++;
   for(const [variant,row]of [['baseline',a],['candidate',b]])summary[variant][row.report.status]=(summary[variant][row.report.status]??0)+1;
   if(changed){
    const transition=a.report.status+' -> '+b.report.status;summary.transitions[transition]=(summary.transitions[transition]??0)+1;
    check('change-only-certified-unused-divisor-zero',operation===3&&parse(b.right.polynomial[0])[0]===0n
     &&(qc(parse(b.right.lower),q(0))>0||qc(parse(b.right.upper),q(0))<0),detail);
   }
   // Old baseline corpora were independently qualified in 68/69. This pass
   // checks every candidate full polynomial and separately both new-degree variants.
   for(const [variant,row]of corpus==='degree'?[['baseline',a],['candidate',b]]:[['candidate',b]]){
    const d={...detail,variant},expected=expectedBinary(row.left,row.right,operation,variant==='candidate'),report=row.report;
    if(expected.status){
     check('exact-guard-or-zero-resultant',report.status===expected.status&&report.root===null,{...d,expected:expected.status,actual:report.status});continue;
    }
    const p=expected.polynomial.map(n=>q(n)),left=root(row.left),right=root(row.right),bounds=image([left.lo,left.hi],[right.lo,right.hi],operation);
    // Authored rational source carriers have one closed-interval root; their
    // nonpoint lower endpoints are not roots, so this agrees with ownership.
    for(const [side,s]of [['left',left],['right',right]]){
     check('source-selected-root',exactCount(s.p,s.lo,s.hi)===1,{...d,side});
     check('source-half-open-owner',s.exact!==null||evaluate(s.p,s.lo)[0]!==0n,{...d,side});
    }
    if(report.status==='NonIsolatingImageInterval'){
     check('nonisolating-image',report.root===null&&imageCount(p,...bounds)!==1,d);continue;
    }
    check('certified-result',report.status==='Transformed'&&report.root!==null&&report.message===null,{...d,status:report.status});
    if(!report.root)continue;const result=report.root,r=root(result);
    check('complete-signed-polynomial',JSON.stringify(result.polynomial)===JSON.stringify(expected.polynomial.map(n=>n+'/1')),{...d,actual:result.polynomial,expected:expected.polynomial});
    check('result-metadata',result.constraint===row.left.constraint&&result.symbol===row.left.symbol&&result.intervalIndex===row.left.intervalIndex
     &&result.count===1&&result.validation==='Valid'&&result.validationMessage===null,d);
    check('contained-unique-root',qc(r.lo,bounds[0])>=0&&qc(r.hi,bounds[1])<=0&&qc(r.lo,r.hi)<=0
     &&(r.exact===null?imageCount(r.p,r.lo,r.hi):exactCount(r.p,r.lo,r.hi))===1,d);
    if(r.exact!==null)check('exact-witness-replay',qc(r.exact,r.lo)>=0&&qc(r.exact,r.hi)<=0&&evaluate(r.p,r.exact)[0]===0n,d);
    if(qc(left.lo,left.hi)===0&&qc(right.lo,right.hi)===0){
     check('selected-point',qc(r.lo,bounds[0])===0&&qc(r.hi,bounds[1])===0&&r.exact!==null&&qc(r.exact,bounds[0])===0,d);
    }
   }
  }
  summaries[corpus]=summary;
 }
 return{checkpoint:70,status:failures.length?'fail':'pass',summaries,counts,totalChecks:Object.values(counts).reduce((a,b)=>a+b,0),
  independentDeterminants:determinantCount(),failures,
  limits:'Rational point and interval corpora, signed full-polynomial comparisons, preserved guard/ownership/proof checks. New answers and degree admissions are not equal-work speedups. No exhaustive completeness or performance/consumer/size/retention claim.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const r=checkZeroFactor();console.log(JSON.stringify(r));if(r.status!=='pass')process.exitCode=1;
}
