import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
const read=p=>readFileSync('results/arf-contract-'+p,'utf8');
const hex=s=>s.startsWith('-')?-BigInt('0x'+s.slice(1)):BigInt('0x'+s);
const abs=n=>n<0n?-n:n;
const gcd=(a,b)=>{a=abs(a);while(b){[a,b]=[b,a%b];}return a;};
const q=(n,d=1n)=>{if(d<0n){n=-n;d=-d;}const g=gcd(n,d);return[n/g,d/g];};
const dyadic=(n,e)=>e<0?q(n,1n<<BigInt(-e)):q(n<<BigInt(e));
const add=([a,b],[c,d])=>q(a*d+c*b,b*d),sub=([a,b],[c,d])=>q(a*d-c*b,b*d);
const div=([a,b],[c,d])=>q(a*d,b*c);
const msd=([n,d])=>{n=abs(n);let e=n.toString(2).length-d.toString(2).length;
 if(e>=0?n<(d<<BigInt(e)):(n<<BigInt(-e))<d)e--;return e;};
const precisions=[1,2,3,7,31,32,33,63,64,65,127,128,129,257],degrees=[1,2,3,5,7,17,2];
const pattern=(w,v)=>v===0?1n<<BigInt(w-1):v===1?(1n<<BigInt(w))-1n:
 (1n<<BigInt(w-1))|(1n<<BigInt(Math.floor(w/2)))|1n;
function fixtures() {
 const widths=[1,2,3,31,32,33,63,64,65,127,128,129,255,256,257],exps=[-257,-1,65],out=[];
 let rootId=0,arithmeticId=0;
 const root=x=>out.push({kind:'root',id:rootId++,x});
 const arithmetic=(x,y)=>out.push({kind:'arithmetic',id:arithmeticId++,x,y});
 for(let wi=0;wi<15;wi++)for(let v=0;v<3;v++)for(let ei=0;ei<3;ei++) {
  const x=pattern(widths[wi],v),y=pattern(widths[(wi+7)%15],(v+1)%3);root(dyadic(x,exps[ei]));
  for(let s=0;s<4;s++)arithmetic(dyadic(s&1?-x:x,exps[ei]),dyadic(s&2?-y:y,exps[(ei+1)%3]));
 }
 for(const k of [2,3,5,7,17])for(const odd of [3n,5n,7n])for(const delta of [-1n,0n,1n])
  root(dyadic(odd**BigInt(k)+delta,-k));
 arithmetic(q(0n),q(3n,2n));arithmetic(q(3n,2n),q(3n,2n));arithmetic(q(3n,2n),q(-3n,2n));
 assert.equal(rootId,180);assert.equal(arithmeticId,543);return out;
}
// Validate neighboring representable powers and exact halfway powers directly.
// No division/root algorithm from the C/GMP oracle is reproduced here.
function resultCheck(result,target,k,p,mode) {
 const [s,ae,ret]=result,m=hex(s);assert(Number.isInteger(ae));assert(ae>-10000&&ae<10000);assert([0,1].includes(ret));
 if(!target[0]){assert.deepEqual(result,['0',0,0]);return{exact:true,tie:false};}
 assert(m!==0n&&((m<0n)===(target[0]<0n)));assert(abs(m)%2n===1n);
 const grid=Math.floor(msd(target)/k)+1-p,shift=ae-grid;
 let v=abs(m);if(shift>=0)v<<=BigInt(shift);else{const d=1n<<BigInt(-shift);assert.equal(v%d,0n);v/=d;}
 assert(v>=(1n<<BigInt(p-1))&&v<=(1n<<BigInt(p)));
 let a=abs(target[0]),b=target[1],scale=-grid*k;
 if(scale>=0)a<<=BigInt(scale);else b<<=BigInt(-scale);
 const power=n=>n**BigInt(k)*b;
 const y=power(v),exact=y===a;assert.equal(ret,Number(!exact));
 const away=mode===1||(mode===2&&m<0n)||(mode===3&&m>0n);let tie=false;
 if(mode===4) {
  const center=a<<BigInt(k),lo=power(2n*v-1n),hi=power(2n*v+1n);
  assert(lo<=center&&center<=hi);tie=lo===center||hi===center;
  if(tie)assert.equal(v%2n,0n);
 }else if(away)assert(power(v-1n)<a&&a<=y);
 else assert(y<=a&&a<power(v+1n));
 return{exact,tie};
}
export function checkArfContracts() {
 const native=read('native.stdout');assert.equal(native,read('memcheck.stdout'));
 assert.equal(read('native.stderr'),'');const rows=native.trimEnd().split('\n').map(JSON.parse),summary=rows.pop();
 assert.deepEqual(summary,{summary:true,rootFixtures:180,arithmeticFixtures:543,checks:518490,aliases:316260,
  inputChecks:64518,exact:64860,ties:890,failures:0});
 assert.equal(rows.length,40446);let cursor=0,checked=0,exact=0,ties=0,nearest=0;
 for(const f of fixtures())for(let op=0;op<(f.kind==='root'?7:3);op++)for(const p of precisions) {
  const r=rows[cursor++],k=f.kind==='root'?degrees[op]:1;
  const target=f.kind==='root'?(op===6?div(q(1n),f.x):f.x):[add,sub,div][op](f.x,f.y);
  assert.deepEqual([r.kind,r.id,r.op,r.k,r.p],[f.kind,f.id,op,k,p]);assert.deepEqual(r.q.map(hex),target);
  assert.equal(r.results.length,5);
  for(let mode=0;mode<5;mode++) {
   const a=r.results[mode];assert.equal(a.length,f.kind==='root'?2:3);
   for(const value of a)assert.deepEqual(value,a[0]);
   const check=resultCheck(a[0],target,k,p,mode);checked+=a.length;
   exact+=Number(check.exact)*a.length;ties+=Number(check.tie)*a.length;nearest+=mode===4?a.length:0;
  }
 }
 assert.equal(cursor,40446);assert.equal(checked,summary.checks);assert.equal(exact,summary.exact);assert.equal(ties,summary.ties);
 for(const tag of ['compile','native','memcheck','linked-libraries']) {
  const g=JSON.parse(read(tag+'.json'));assert.equal(g.code,0);assert.equal(g.signal,null);assert(Date.parse(g.finished)>=Date.parse(g.started));
 }
 assert.equal(read('compile.stderr'),'');const mem=read('memcheck.stderr');
 assert.match(mem,/ERROR SUMMARY: 0 errors from 0 contexts \(suppressed: 0 from 0\)/);
 assert.match(mem,/in use at exit: 0 bytes in 0 blocks/);assert.match(mem,/All heap blocks were freed/);
 const usage=mem.match(/total heap usage: ([\d,]+) allocs, ([\d,]+) frees, ([\d,]+) bytes allocated/).slice(1).map(s=>Number(s.replaceAll(',','')));
 assert.equal(usage[0],usage[1]);
 return{summary,independentFullValueChecks:checked,independentPrimaryPowerCertificates:rows.length*5,
  nearestEvenChecks:nearest,identicalNumericalBytes:Buffer.byteLength(native),
  memory:{errors:0,suppressed:0,liveBytes:0,liveBlocks:0,allocations:usage[0],frees:usage[1],cumulativeBytesIncludingOracle:usage[2]},
  limits:'Bounded finite sequential positive-root/signed-arithmetic inputs, five modes and public whole-object aliases; native64 ADX and FE_TONEAREST only. GMP oracle uses exact rational scaling/integer roots; independent BigInt regenerates every fixture and certifies neighboring/halfway powers of complete outputs. Not every ARF operation, nearest-binary64 or integer-rounding routine, arbitrary/excessive exponent, parser/file failure, MPFR global-state, physical other-target, all-precision/history proof or donor-only timing/peak memory qualification.'};
}
if(process.argv.includes('--arf-contract-summary'))console.log(JSON.stringify(checkArfContracts()));
