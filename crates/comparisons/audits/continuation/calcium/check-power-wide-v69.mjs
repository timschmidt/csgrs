import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {oracle} from './power-sums-polynomial-oracle.mjs';
import {makeWideCases} from './power-wide-cases-v69.mjs';
const abs=n=>n<0n?-n:n;
function gcd(a,b){a=abs(a);b=abs(b);while(b)[a,b]=[b,a%b];return a;}
function q(n,d=1n){assert(d!==0n);if(d<0n){n=-n;d=-d;}const g=gcd(n,d);return[n/g,d/g];}
const parse=s=>{assert(/^-?\d+\/\d+$/.test(s));const r=q(...s.split('/').map(BigInt));assert.equal(r.join('/'),s);return r;};
const add=(a,b)=>q(a[0]*b[1]+b[0]*a[1],a[1]*b[1]);
const neg=a=>[-a[0],a[1]],mul=(a,b)=>q(a[0]*b[0],a[1]*b[1]);
const ops=[add,(a,b)=>add(a,neg(b)),mul,(a,b)=>q(a[0]*b[1],a[1]*b[0])];
const evaluate=(p,x)=>p.reduceRight((v,c)=>add(mul(v,x),c),q(0n));
function primitive(p){let d=1n;for(const c of p)d=d/gcd(d,c[1])*c[1];const v=p.map(c=>c[0]*(d/c[1])),g=v.reduce(gcd,0n);assert(g!==0n);return v.map(c=>(c/g).toString());}
const bits=n=>abs(BigInt(n)).toString(2).length;
const read=p=>readFileSync(p,'utf8').trimEnd().split('\n').map(JSON.parse);
export function checkWide(){
 assert.equal(ops[3](q(-7n,3n),q(2n,5n)).join('/'),'-35/6');
 assert.equal(evaluate([q(-1n),q(1n)],q(1n))[0],0n);
 const input=JSON.parse(readFileSync('power-wide-input-v69.json','utf8'));assert.deepEqual(input,makeWideCases());
 const outputs=['baseline','candidate'].map(v=>read('results/power-wide-'+v+'-run-v69.stdout'));
 const expectedTerminal={terminal:true,cases:1840,policies:2,rows:3680};
 for(const rows of outputs){assert.equal(rows.length,3681);assert.deepEqual(rows.at(-1),expectedTerminal);}
 const counts={},failures=[],statuses={baseline:{},candidate:{}},families={},heightParameters={},degreePairs={},oracleCache=new Map();
 let maxInputNumeratorBits=0,maxInputDenominatorBits=0,maxPrimitiveCoefficientBits=0,maxResultCoefficientBits=0;
 const check=(name,ok,detail)=>{counts[name]=(counts[name]??0)+1;if(!ok)failures.push({name,...detail});};
 for(const c of input){
  const source=[c.left,c.right].map((r,side)=>{
   const p=r.polynomial.map(parse),x=parse(r.point);check('authored-source-root',evaluate(p,x)[0]===0n,{id:c.id,side});
   assert.equal(p.length,(side===0?c.m:c.n)+1);assert(p.at(-1)[0]!==0n);
   for(const a of p){maxInputNumeratorBits=Math.max(maxInputNumeratorBits,bits(a[0]));maxInputDenominatorBits=Math.max(maxInputDenominatorBits,bits(a[1]));}
   const integers=primitive(p);for(const a of integers)maxPrimitiveCoefficientBits=Math.max(maxPrimitiveCoefficientBits,bits(a));
   return{p,x,integers};
  });
  assert(c.m*c.n<=9);families[c.family]=(families[c.family]??0)+1;heightParameters[c.height]=(heightParameters[c.height]??0)+1;
  degreePairs[c.m+','+c.n]=(degreePairs[c.m+','+c.n]??0)+1;
  const denominatorZero=c.op===3&&source[1].x[0]===0n;
  let polynomial=null,value=null;
  if(!denominatorZero){
   const key=JSON.stringify([source[0].integers,source[1].integers,c.op]);
   if(!oracleCache.has(key))oracleCache.set(key,oracle(source[0].integers,source[1].integers,c.op).poly);
   polynomial=oracleCache.get(key);value=ops[c.op](source[0].x,source[1].x);
   if(polynomial){check('oracle-selected-root',evaluate(polynomial.map(n=>q(BigInt(n))),value)[0]===0n,{id:c.id});
    for(const a of polynomial)maxResultCoefficientBits=Math.max(maxResultCoefficientBits,bits(a));}
  }
  for(let policy=0;policy<2;policy++){
   const idx=c.id*2+policy,rows=outputs.map(out=>out[idx]);
   check('paired-full-record',JSON.stringify(rows[0])===JSON.stringify(rows[1]),{id:c.id,policy});
   for(const [vi,row]of rows.entries()){
    const variant=vi===0?'baseline':'candidate';assert.equal(row.id,c.id);assert.equal(row.policy,policy);
    for(const [side,key]of ['left','right'].entries()){
     const r=row[key],expected=c[key];
     check('source-wire',JSON.stringify(r.polynomial)===JSON.stringify(expected.polynomial)&&r.lower===expected.point&&r.upper===expected.point&&r.exact===expected.point,{id:c.id,policy,variant,side});
     check('source-metadata',r.constraint===7&&r.symbol===(side===0?11:13)&&r.intervalIndex===3&&r.count===1&&r.validation==='Valid'&&r.validationMessage===null,{id:c.id,policy,variant,side});
    }
    const report=row.report;check('operation',report.operation===['Add','Subtract','Multiply','Divide'][c.op],{id:c.id,policy,variant});
    statuses[variant][report.status]=(statuses[variant][report.status]??0)+1;
    if(denominatorZero){check('zero-divisor-guard',report.status==='DenominatorMayContainZero'&&report.root===null,{id:c.id,policy,variant,status:report.status});continue;}
    if(polynomial===null){check('zero-resultant-control',report.status==='Undecided'&&report.root===null,{id:c.id,policy,variant,status:report.status});continue;}
    check('certified-point-result',report.status==='Transformed'&&report.root!==null&&report.message===null,{id:c.id,policy,variant,status:report.status});
    if(report.root===null)continue;const r=report.root;
    check('complete-signed-polynomial',JSON.stringify(r.polynomial)===JSON.stringify(polynomial.map(n=>n+'/1')),{id:c.id,policy,variant,actual:r.polynomial,expected:polynomial});
    check('selected-point-and-witness',r.lower===value.join('/')&&r.upper===value.join('/')&&r.exact===value.join('/'),{id:c.id,policy,variant,expected:value.join('/'),lower:r.lower,upper:r.upper,exact:r.exact});
    check('returned-polynomial-vanishes',evaluate(r.polynomial.map(parse),value)[0]===0n,{id:c.id,policy,variant});
    check('result-metadata',r.constraint===7&&r.symbol===11&&r.intervalIndex===3&&r.count===1&&r.validation==='Valid'&&r.validationMessage===null,{id:c.id,policy,variant});
   }
  }
 }
 assert.equal(Object.keys(degreePairs).length,23);assert.equal(Object.keys(families).length,4);assert.equal(Object.keys(heightParameters).length,5);
 return {checkpoint:69,status:failures.length?'mathematical-fail':'pass',cases:1840,recordsPerVariant:3681,policies:2,
  independentDeterminants:oracleCache.size,counts,totalChecks:Object.values(counts).reduce((a,b)=>a+b,0),statuses,families,heightParameters,degreePairs,
  maxInputNumeratorBits,maxInputDenominatorBits,maxPrimitiveCoefficientBits,maxResultCoefficientBits,failures,
  limits:'Authored rational point witnesses, all admitted ordered degree pairs and selected coefficient heights/carrier shapes; not exhaustive over rationals or arbitrary isolating intervals. Complete signed polynomial equality includes multiplicities, not merely roots/overlap. Zero-resultant controls preserve a separately open completeness gap. No timing/allocation/WASM/consumer/size qualification or new retention.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const r=checkWide();console.log(JSON.stringify(r));if(r.status!=='pass')process.exitCode=1;
}
