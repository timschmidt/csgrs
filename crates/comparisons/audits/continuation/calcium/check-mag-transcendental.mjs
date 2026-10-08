import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
export function checkMagTranscendental(){
 const read=p=>readFileSync('results/mag-transcendental-'+p,'utf8');
 const native=read('native.stdout');assert.equal(native,read('memcheck.stdout'));
 assert.equal(read('native.stderr'),'');assert.equal(Buffer.byteLength(native),866255);
 const rows=native.trimEnd().split('\n').map(JSON.parse);assert.equal(rows.length,2303);
 const summary=rows.pop();assert.deepEqual(summary,{kind:'summary',checks:36208,qualityChecks:31652,
  inputChecks:16742,aliasChecks:16742,finiteMagResults:33488,infiniteMagResults:0,failures:0});
 const exponents=[-1075,-1001,-1000,-971,-970,-969,-61,-60,-31,-30,-29,-17,-16,-15,-14,-11,-10,-9,-1,0,1,3,4,5,6,10,23,24,25,26,29,30,31,999,1000,1001,1024];
 const mans=[536870912,536870913,536870914,536870915,671088639,671088640,671088641,1073741821,1073741822,1073741823];
 const roots=[1,2,3,4,5,7,16,31],tails=[0,1,2,3,8,31,64,128,255,256,257];
 const ops=['exp','exp_lower','expm1','expinv','expinv_lower','sinh','sinh_lower','cosh','cosh_lower',
  'atan','atan_lower','log','log_lower','neg_log','neg_log_lower','log1p',...roots.map(n=>'root-'+n)];
 let index=0,checks=0,quality=0,inputs=0,aliases=0,magResults=0;
 const histories=[];
 const normalized=r=>{
  assert.equal(r.length,3);const[m,e,inf]=r;
  assert.equal(inf,0);assert(Number.isSafeInteger(m)&&Number.isSafeInteger(e));
  assert(e>=-200000000&&e<=200000000);
  if(m===0)assert.equal(e,0);else assert(m>=2**29&&m<2**30);
  checks++;magResults++;
 };
 for(const p of [512,768]){
  const history=[],fixtures=[[0,0],...exponents.flatMap(e=>mans.map(m=>[m,e]))];
  for(let id=0;id<fixtures.length;id++){
   const[man,exp]=fixtures[id],r=rows[index++];
   assert.equal(r.kind,'unary');assert.equal(r.precision,p);assert.equal(r.id,id);assert.equal(r.man,man);assert.equal(r.exp,exp);
   const expected=ops.map((_,i)=>i).filter(i=>!(i<9&&exp>26)&&!((i===13||i===14)&&man===0));
   assert.deepEqual(r.results.map(x=>x[0]),expected);
   for(const[op,a,b]of r.results){normalized(a);normalized(b);assert.deepEqual(a,b);
    inputs++;aliases++;if(op>=9||exp<=10)quality+=2;
   }
   const{precision,...rest}=r;history.push(rest);
  }
  const tm=[0,536870912,536870912,536870912,1073741823,536870912,536870913,536870912,536870912];
  const te=[0,-59,-3,0,0,1,1,2,3];
  for(let i=0;i<tm.length;i++)for(let j=0;j<tails.length;j++){
   const r=rows[index++];assert.equal(r.kind,'tail');assert.equal(r.precision,p);assert.equal(r.id,i*11+j);
   assert.equal(r.man,tm[i]);assert.equal(r.exp,te[i]);assert.equal(r.n,tails[j]);assert.equal(r.results.length,2);
   r.results.forEach(normalized);assert.deepEqual(r.results[0],r.results[1]);inputs++;aliases++;
   const{precision,...rest}=r;history.push(rest);
  }
  const buf=new ArrayBuffer(8),view=new DataView(buf);
  const bits=x=>{view.setFloat64(0,x);return view.getBigUint64(0);};
  const hex=x=>x.toString(16).padStart(16,'0');
  const doubles=[];
  for(const s of [-1000,-31,-1,0,1,31,1000])for(let k=16;k<=47;k++){
   const b=bits(k/32*2**s);for(const d of [-1n,0n,1n])doubles.push(hex(b+d));
  }
  doubles.push('0000000000000001','000fffffffffffff','0010000000000000','0010000000000001',
   '7fefffffffffffff','3fefffffffffffff','3ff0000000000000','3ff0000000000001');
  assert.equal(doubles.length,680);
  for(let id=0;id<doubles.length;id++){
   const r=rows[index++];assert.equal(r.kind,'dlog');assert.equal(r.precision,p);assert.equal(r.id,id);assert.equal(r.x,doubles[id]);
   assert.match(r.lower,/^[0-9a-f]{16}$/);assert.match(r.upper,/^[0-9a-f]{16}$/);checks+=2;
   const{precision,...rest}=r;history.push(rest);
  }
  const r=rows[index++];assert.equal(r.kind,'pi');assert.equal(r.precision,p);assert.equal(r.results.length,2);
  r.results.forEach(normalized);quality+=2;
  assert.deepEqual(r.results,[[843314856,2,0],[843314857,2,0]]);
  const{precision,...rest}=r;history.push(rest);histories.push(history);
 }
 assert.equal(index,2302);assert.deepEqual(histories[0],histories[1]);
 assert.equal(checks,summary.checks);assert.equal(quality,summary.qualityChecks);assert.equal(inputs,summary.inputChecks);
 assert.equal(aliases,summary.aliasChecks);assert.equal(magResults,summary.finiteMagResults);
 const mem=read('memcheck.stderr');assert.match(mem,/in use at exit: 0 bytes in 0 blocks/);
 assert.match(mem,/116,716 allocs, 116,716 frees, 22,125,312 bytes allocated/);
 assert.match(mem,/All heap blocks were freed/);assert.match(mem,/ERROR SUMMARY: 0 errors from 0 contexts \(suppressed: 0 from 0\)/);
 for(const t of ['native','memcheck']){const g=JSON.parse(read(t+'.json'));assert.equal(g.code,0);assert.equal(g.signal,null);}
 return{summary,groups:2302,precisions:[512,768],unaryFixturesPerPrecision:371,unaryRoutes:ops,
  rootDegrees:roots,expTailFixturesPerPrecision:99,doubleLogFixturesPerPrecision:680,
  equalAcrossPrecisions:true,identicalNativeMemoryBytes:866255,
  memory:{errors:0,contexts:0,suppressed:0,liveBytes:0,liveBlocks:0,allocations:116716,frees:116716,cumulativeBytes:22125312},
  limits:'Valid finite public 64-bit inputs, default FE_TONEAREST only. Unary inputs use ten mantissas and 37 inline exponents -1075..1024 plus zero; exponential-growth/decay routes are restricted to exponents <=26. Zero negative-log is not a finite real value and is excluded. Supported whole-object aliases, seeded finite outputs, direct 30-bit exact MPFR imports, 512/768-bit directed reference bounds. These two precisions are not independent implementations. MPFR shares GMP integer support but does not reuse the donor magnitude polynomials or ARF conversion. Exp-tail x<=4 and N in eleven bounded positions through257 uses N..N+256 terms plus a rigorous geometric bound on all remaining terms. Quality is supplemental 1/1024 relative slack, excluded for exponential-family input exponents >10 and all tails; not correct rounding. No promoted/nonfinite/huge exponent allocation, raw helper/precondition violations, arbitrary overlap/thread, all-FENV/32-bit/ARM/libm, conversion/IO/combinatorial/other-tail qualification, new whole-stack Rust CI, matched Hyper speed/size or donor-only/peak memory claim.'};
}
if(process.argv.includes('--mag-transcendental-summary'))console.log(JSON.stringify(checkMagTranscendental()));
