import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {q,scalar,integer,root2,root3,fzero,fadd,fsub,fmul,fdiv,fneg,feq,fkey,fcompare,
 realValue,evaluate,productRoots,monic,fieldSelfTest} from './point-extended-field.mjs';
const read=p=>readFileSync(p,'utf8').trim().split('\n').map(JSON.parse);
const op=[fadd,fsub,fmul,fdiv],opNames=['Add','Subtract','Multiply','Divide'];
const unique=xs=>[...new Map(xs.map(x=>[fkey(x),x])).values()];
function inputs(family){
 const a=root2(),b=root3(),z=fzero(),one=integer(1),half3=scalar(q(3,2));
 const pair=(value,roots)=>({value,roots});
 const r2=()=>pair(a,[a,fneg(a)]),r3=()=>pair(b,[b,fneg(b)]),r0=()=>pair(z,[z]),r1=()=>pair(one,[one]),rhalf=()=>pair(half3,[half3]);
 return [[r2,r2],[r2,r3],[r2,rhalf],[r1,r2],[r2,r2],[r2,rhalf],[r0,r0],[r0,r2],[r1,r1],[r0,r0],[r0,r0],[r1,r1]][family].map(f=>f());
}
const samePolynomial=(a,b)=>a.length===b.length&&a.every((c,i)=>feq(c,b[i]));
function validate(row){
 const failures=[],checks={};
 const check=(name,ok,detail)=>{checks[name]=(checks[name]??0)+1;if(!ok)failures.push({name,case:row.case,policy:row.policy,history:row.history,detail});};
 const operation=row.case%4,source=inputs(Math.floor(row.case/4));
 function root(r,expected,roots,label){
  const polynomial=r.polynomial.map(realValue),lower=realValue(r.lower),upper=realValue(r.upper),exact=r.exact===null?null:realValue(r.exact);
  check(label+'-metadata',r.constraint===7&&r.symbol===11&&r.intervalIndex===3&&r.count===1&&r.validation==='Valid'&&r.validationMessage===null);
  check(label+'-ordered',fcompare(lower,upper)<=0);
  check(label+'-selected-value-contained',fcompare(lower,expected)<=0&&fcompare(expected,upper)<=0);
  check(label+'-full-polynomial',samePolynomial(monic(polynomial),productRoots(roots)),{actual:monic(polynomial).map(fkey),expected:productRoots(roots).map(fkey)});
  check(label+'-polynomial-vanishing',feq(evaluate(polynomial,expected),fzero()));
  if(exact!==null){check(label+'-exact-witness',feq(exact,expected));check(label+'-witness-vanishing',feq(evaluate(polynomial,exact),fzero()));}
  const owned=unique(roots).filter(x=>fcompare(x,upper)<=0&&(exact!==null?fcompare(lower,x)<=0:fcompare(lower,x)<0));
  check(label+'-distinct-root-count',owned.length===1,{owned:owned.map(fkey)});
  return{polynomial,lower,upper,exact};
 }
 const left=root(row.left,source[0].value,source[0].roots,'left'),right=root(row.right,source[1].value,source[1].roots,'right');
 check('operation',row.report.operation===opNames[operation]);
 const denominatorZero=operation===3&&feq(source[1].value,fzero());
 const denominatorIntervalZero=operation===3&&fcompare(right.lower,fzero())<=0&&fcompare(fzero(),right.upper)<=0;
 if(row.report.status==='Transformed'){
  check('successful-report',row.report.message===null&&row.report.root!==null);
  check('nonzero-divisor',!denominatorIntervalZero);
  assert(!denominatorZero);
  const expected=op[operation](source[0].value,source[1].value);
  const roots=source[0].roots.flatMap(a=>source[1].roots.map(b=>op[operation](a,b)));
  const out=root(row.report.root,expected,roots,'output');
  let image;
  if(operation===0)image=[fadd(left.lower,right.lower),fadd(left.upper,right.upper)];
  else if(operation===1)image=[fsub(left.lower,right.upper),fsub(left.upper,right.lower)];
  else image=[left.lower,left.upper].flatMap(a=>[right.lower,right.upper].map(b=>op[operation](a,b))).sort(fcompare).filter((_,i)=>i===0||i===3);
  check('refined-image-contained',fcompare(image[0],out.lower)<=0&&fcompare(out.upper,image[1])<=0);
 }else{
  check('no-result-on-failure',row.report.root===null);
  check('known-failure-status',['InvalidTransformedEvidence','Undecided','NonIsolatingImageInterval','DenominatorMayContainZero','InvalidEvidence'].includes(row.report.status));
  if(row.report.status==='DenominatorMayContainZero')check('denominator-guard',denominatorIntervalZero);
 }
 return{checks,failures};
}
export function checkExtended(){
 fieldSelfTest();const summaries={},all={};
 for(const variant of ['baseline','candidate']){
  const rows=read('results/point-extended-public-'+variant+'.stdout');assert.equal(rows.length,385);
  assert.deepEqual(rows.at(-1),{type:'terminal',cases:48,histories:4,policies:2,rows:384});
  const expected=new Set();for(let c=0;c<48;c++)for(let p=0;p<2;p++)for(let h=0;h<4;h++)expected.add([c,p,h].join(':'));
  const checks={},failures=[],statuses={};
  for(const r of rows.slice(0,-1)){
   assert.equal(r.type,'query');assert(expected.delete([r.case,r.policy,r.history].join(':')));
   const result=validate(r);failures.push(...result.failures);
   for(const[k,n]of Object.entries(result.checks))checks[k]=(checks[k]??0)+n;
   statuses[r.report.status]=(statuses[r.report.status]??0)+1;
  }
  assert.equal(expected.size,0);summaries[variant]={rows:384,statuses,checks,totalChecks:Object.values(checks).reduce((a,b)=>a+b,0),failures};all[variant]=rows;
 }
 let improved=0,unchanged=0;
 for(let i=0;i<385;i++){
  const a=all.baseline[i],b=all.candidate[i];
  if(a.report?.status==='InvalidTransformedEvidence'){
   assert.equal(a.report.message,'a collapsed refinement interval requires an exact witness');
   assert.equal(b.report.status,'Transformed');assert.deepEqual({...b,report:a.report},a);improved++;
  }else{assert.deepEqual(b,a);unchanged++;}
 }
 assert.equal(improved,128);assert.equal(unchanged,257);
 // Ensure that decoding/overlap alone cannot accept a changed witness,
 // annihilator, or endpoint. These are in-memory oracle controls, not changes
 // to any captured observation.
 const tamper=[];
 for(const kind of ['witness','polynomial','endpoint']){
  const r=structuredClone(all.candidate[0]);
  if(kind==='witness')r.report.root.exact.rational.numerator=[3];
  if(kind==='polynomial')r.report.root.polynomial[0].rational={sign:1,numerator:[1],denominator:[1]};
  if(kind==='endpoint')r.report.root.lower.rational.numerator=[3];
  const failures=validate(r).failures;assert(failures.length>0);tamper.push({kind,rejectedBy:failures.map(f=>f.name)});
 }
 const historyChanges=[];
 for(const variant of ['baseline','candidate'])for(let c=0;c<48;c++)for(let p=0;p<2;p++){
  const rows=all[variant].filter(r=>r.case===c&&r.policy===p);
  if(new Set(rows.map(r=>r.report.status)).size>1)historyChanges.push({variant,case:c,policy:p,statuses:rows.map(r=>r.report.status)});
 }
 return{status:Object.values(summaries).every(s=>s.failures.length===0)?'pass':'mathematical-fail',summaries,improved,unchanged,tamper,historyChanges,
  limits:'384 authored queries per variant, 48 operations, two policies, four ordered in-process constructed/coarse-refined/deep-refined/deserialized histories. Independent full serialized computational semantics in Q(sqrt(2),sqrt(3)) with the explicit common-argument sine/cosine identity and nonzero pi/e divisors. Full carrier polynomials, exact signs, root counts, selected values, witnesses and interval images checked without Hyper comparisons, floating-point overlap or donor arithmetic. Known analytic identities establish values of internally Unknown expressions; arbitrary transcendental nodes/divisors are rejected. Global caches are not cold processes, and no CPU/allocator/WASM/retention qualification is implied.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const result=checkExtended();console.log(JSON.stringify(result));if(result.status!=='pass')process.exitCode=1;
}
