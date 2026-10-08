import {readFileSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {oracle} from './power-sums-polynomial-oracle.mjs';
const abs=n=>n<0n?-n:n;
function gcd(a,b){a=abs(a);b=abs(b);while(b)[a,b]=[b,a%b];return a;}
function q(n,d=1n){n=BigInt(n);d=BigInt(d);assert(d!==0n);if(d<0n){n=-n;d=-d;}const g=gcd(n,d);return[n/g,d/g];}
const qa=(a,b)=>q(a[0]*b[1]+b[0]*a[1],a[1]*b[1]),qn=a=>[-a[0],a[1]];
const qs=(a,b)=>qa(a,qn(b)),qm=(a,b)=>q(a[0]*b[0],a[1]*b[1]),qd=(a,b)=>q(a[0]*b[1],a[1]*b[0]);
const qc=(a,b)=>{const n=a[0]*b[1]-b[0]*a[1];return n<0n?-1:n>0n?1:0;};
const parse=s=>{assert(/^-?\d+\/\d+$/.test(s));const [n,d]=s.split('/');const a=q(n,d);assert.equal(a.join('/'),s);return a;};
const trim=p=>{while(p.length>1&&p.at(-1)[0]===0n)p.pop();return p;};
const zero=p=>p.every(c=>c[0]===0n);
const monic=p=>zero(p)?[q(0)]:p.map(c=>qd(c,p.at(-1)));
const derivative=p=>p.length===1?[q(0)]:p.slice(1).map((c,i)=>qm(c,q(i+1)));
const evaluate=(p,x)=>p.reduceRight((v,c)=>qa(qm(v,x),c),q(0));
function divrem(a,b){assert(!zero(b));let r=a.map(c=>[...c]);const quotient=Array.from({length:Math.max(1,a.length-b.length+1)},()=>q(0));
 while(!zero(r)&&r.length>=b.length){const k=r.length-b.length, c=qd(r.at(-1),b.at(-1));quotient[k]=c;
  for(let j=0;j<b.length;j++)r[j+k]=qs(r[j+k],qm(c,b[j]));trim(r);}
 return[trim(quotient),r];
}
const sfCache=new Map();
const polyKey=p=>p.map(c=>c.join('/')).join(',');
function squareFree(p){const key=polyKey(monic(p));if(sfCache.has(key))return sfCache.get(key);
 let a=p,b=derivative(p);while(!zero(b)){const r=divrem(a,b)[1];a=b;b=monic(r);}
 const [s,r]=divrem(p,monic(a));assert(zero(r));const result=monic(s);sfCache.set(key,result);return result;
}
function integers(p){let d=1n;for(const c of p)d=d/gcd(d,c[1])*c[1];let out=p.map(c=>c[0]*(d/c[1]));
 let g=out.reduce(gcd,0n);assert(g!==0n);if(out.at(-1)<0n)g=-g;return out.map(c=>c/g);}
const sturmCache=new Map(),countCache=new Map();
function rootCount(p,lo,hi){assert(qc(lo,hi)<=0);p=squareFree(p);
 const key=polyKey(p),ck=key+';'+lo.join('/')+';'+hi.join('/');if(countCache.has(ck))return countCache.get(ck);
 if(!sturmCache.has(key)){const seq=[p,derivative(p)];while(!zero(seq.at(-1))&&seq.at(-1).length>1){
   const r=divrem(seq.at(-2),seq.at(-1))[1].map(qn);if(zero(r))break;seq.push(r);}
  sturmCache.set(key,seq.filter(s=>!zero(s)));}
 const variations=x=>{let old=0,n=0;for(const s of sturmCache.get(key)){const sign=qc(evaluate(s,x),q(0));if(sign){if(old&&old!==sign)n++;old=sign;}}return n;};
 const count=variations(lo)-variations(hi)+Number(evaluate(p,lo)[0]===0n);countCache.set(ck,count);return count;
}
function image(a,b,op){
 if(op===0)return[qa(a[0],b[0]),qa(a[1],b[1])];
 if(op===1)return[qs(a[0],b[1]),qs(a[1],b[0])];
 const values=a.flatMap(x=>b.map(y=>op===2?qm(x,y):qd(x,y))).sort(qc);return[values[0],values.at(-1)];
}
function root(r){return{p:r.polynomial.map(parse),lo:parse(r.lower),hi:parse(r.upper),exact:r.exact===null?null:parse(r.exact)};}
function selfTest(){
 const p=[q(-2),q(0),q(1)];assert.equal(rootCount(p,q(1),q(2)),1);assert.equal(rootCount(p,q(-2),q(2)),2);
 const repeated=[q(1),q(-2),q(1)];assert.equal(rootCount(repeated,q(1),q(1)),1);
 assert.equal(rootCount(repeated,q(0),q(1)),1);assert.equal(rootCount(repeated,q(1),q(2)),1);
 assert.equal(rootCount(repeated,q(2),q(3)),0);
}
const key=r=>r.type==='public'?[r.type,r.i,r.j,r.op,r.scale].join(':'):r.type==='cost-case'?[r.type,r.case].join(':'):r.type;
export function checkPowerSumsPublic(baseline,candidate){
 selfTest();const read=p=>readFileSync(p,'utf8').trim().split('\n').map(s=>JSON.parse(s));
 const a=read(baseline),b=read(candidate),wanted=new Set();
 for(let i=0;i<20;i++)for(let j=0;j<20;j++)for(let op=0;op<4;op++)for(let s=0;s<4;s++)wanted.add(['public',i,j,op,s].join(':'));
 for(let i=0;i<40;i++)wanted.add('cost-case:'+i);wanted.add('terminal');
 assert.equal(a.length,6441);assert.equal(b.length,6441);
 const failures=[],counts={},checks={};
 const check=(name,ok,r,detail)=>{checks[name]=(checks[name]??0)+1;if(!ok)failures.push({key:key(r),name,detail});};
 const oracleCache=new Map();
 for(let i=0;i<a.length;i++){
  const row=b[i];assert(wanted.delete(key(row)));assert.equal(key(a[i]),key(row));
  check('paired-full-record',JSON.stringify(a[i])===JSON.stringify(row),row);
  if(row.type==='terminal'){assert.equal(row.publicRows,6400);assert.equal(row.costCases,40);continue;}
  const left=root(row.left),right=root(row.right),report=row.report;
  const op=row.type==='public'?row.op:row.case%4;
  check('source-left-isolation',rootCount(left.p,left.lo,left.hi)===1,row);
  check('source-right-isolation',rootCount(right.p,right.lo,right.hi)===1,row);
  counts[report.status]=(counts[report.status]??0)+1;
  const denominatorBlocked=op===3&&qc(right.lo,q(0))<=0&&qc(right.hi,q(0))>=0;
  if(denominatorBlocked){check('nonzero-guard',report.status==='DenominatorMayContainZero'&&report.root===null,row);continue;}
  let lp=left.p,rp=right.p;
  if((lp.length-1)*(rp.length-1)>9){lp=squareFree(lp);rp=squareFree(rp);}
  if((lp.length-1)*(rp.length-1)>9){check('degree-guard',report.status==='UnsupportedDegree'&&report.root===null,row);continue;}
  const oi=integers(lp),oj=integers(rp),okey=oi.join(',')+';'+oj.join(',')+';'+op;
  if(!oracleCache.has(okey))oracleCache.set(okey,oracle(oi,oj,op).poly);
  const expected=oracleCache.get(okey);
  if(expected===null){check('zero-resultant',report.status==='Undecided'&&report.root===null,row);continue;}
  const target=expected.map(c=>q(c));const bounds=image([left.lo,left.hi],[right.lo,right.hi],op);
  const imageRoots=rootCount(target,bounds[0],bounds[1]);
  if(report.root===null){check('nonisolating-control',imageRoots!==1&&report.status==='NonIsolatingImageInterval',row,{imageRoots,status:report.status});continue;}
  const returned=root(report.root);
  check('root-status',report.status==='Transformed'&&report.root.validation==='Valid'&&report.root.count===1,row);
  check('annihilator-root-set',JSON.stringify(integers(squareFree(returned.p)).map(String))===JSON.stringify(integers(squareFree(target)).map(String)),row);
  check('unique-arithmetic-image',imageRoots===1,row,{imageRoots});
  check('selected-root-isolation',rootCount(returned.p,returned.lo,returned.hi)===1,row);
  check('image-subinterval',qc(bounds[0],returned.lo)<=0&&qc(returned.hi,bounds[1])<=0,row);
  if(returned.exact)check('exact-root',evaluate(returned.p,returned.exact)[0]===0n&&qc(returned.lo,returned.exact)<=0&&qc(returned.exact,returned.hi)<=0,row);
 }
 assert.equal(wanted.size,0);
 return{status:failures.length?'mathematical-fail':'pass',recordsPerVariant:a.length,counts,checks,totalChecks:Object.values(checks).reduce((a,b)=>a+b,0),
  independentResultants:oracleCache.size,failures,
  limits:'Authored real-root carriers and strict-policy public queries. Independent rational Sturm/closed-interval image certificates; signed multiplicity agreement checked by paired complete records plus the separate exact kernel corpus. Not all states/platforms/features or performance qualification.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const r=checkPowerSumsPublic(process.argv[2],process.argv[3]);console.log(JSON.stringify(r));process.exitCode=r.failures.length?1:0;
}
