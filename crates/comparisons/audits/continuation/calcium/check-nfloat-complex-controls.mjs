import {readFileSync} from 'node:fs';
import {dirname,resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url));
const read=p=>readFileSync(resolve(here,p),'utf8');

// Independent integer reconstruction of the dyadic input corpus. All four
// integers share denominator 2^(bits+80); compare squared norms without floats.
function inputs(bits,p) {
 const d=1n<<BigInt(bits+80);
 const q=Array.from({length:4},(_,i)=>((1n<<BigInt(bits))-BigInt(2*i+1))*(1n<<80n)*((p>>(i%3))&1?-1n:1n));
 switch(p) {
  case 4:q[0]=q[1]=0n;break; case 5:q[2]=q[3]=0n;break;
  case 6:q[1]=0n;break;case 7:q[0]=0n;break;case 8:q[3]=0n;break;case 9:q[2]=0n;break;
  case 10:q[1]=q[3]=0n;break;case 11:q[0]=q[2]=0n;break;case 12:q[1]=q[2]=0n;break;case 13:q[0]=q[3]=0n;break;
  case 14:q[2]=q[0];q[3]=q[1];break;case 15:q[2]=q[0];q[3]=-q[1];break;
  case 16:q[1]=q[0];q[3]=q[2];break;case 17:q[1]=-q[0];q[3]=q[2];break;
  case 18:q[1]>>=70n;break;case 19:q[0]<<=40n;q[3]>>=80n;break;
  case 20:q[2]=q[0];break;case 21:q[2]=-q[0];q[3]=-q[1];break;
  case 22:q[0]=3n*d;q[1]=4n*d;q[2]=5n*d;q[3]=0n;break;
  case 23:q.fill(0n);break;
  case 24:case 25:case 26:case 27:q[0]<<=20n;q[1]>>=60n;break;
  case 28:case 29:case 30:case 31:q[2]<<=20n;q[3]>>=60n;break;
 }
 return q;
}

export function checkComplexControls() {
 const raw=read('results/nfloat-complex-native.stdout');
 assert.equal(raw,read('results/nfloat-complex-memcheck.stdout'));
 const lines=raw.trimEnd().split('\n'),summary=JSON.parse(lines.pop());
 assert.deepEqual(summary,{suite:'nfloat-complex-controls',limb_bits:64,rows:89628,comparisons:177144,aliases:58344,skipped_zero_divisors:1188,failures:0});
 let cursor=0,arithmetic=0,cmpabs=0,skips=0;const signs={negative:0,zero:0,positive:0};
 for(let limbs=1;limbs<=66;limbs++)for(let pattern=0;pattern<32;pattern++) {
  const q=inputs(limbs*64,pattern),xzero=q[0]===0n&&q[1]===0n,yzero=q[2]===0n&&q[3]===0n;
  for(let op=0;op<14;op++) {
   if(((op===4||op===12)&&xzero)||(op===5&&yzero)){skips+=3;continue;}
   for(let alias=0;alias<3;alias++) {
    assert.equal(lines[cursor++],['arithmetic',limbs,pattern,op,alias,0,1,1,1,1].join(','));arithmetic++;
   }
  }
  const delta=q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3],sign=delta>0n?1:delta<0n?-1:0;
  assert.equal(lines[cursor++],['cmpabs',limbs,pattern,0,sign,1].join(','));cmpabs++;
  signs[sign<0?'negative':sign>0?'positive':'zero']++;
 }
 assert.equal(cursor,lines.length);assert.equal(arithmetic,87516);assert.equal(cmpabs,2112);assert.equal(skips,1188);
 const mem=read('results/nfloat-complex-memcheck.stderr');
 assert.match(mem,/ERROR SUMMARY: 0 errors from 0 contexts/);
 assert.match(mem,/in use at exit: 0 bytes in 0 blocks/);
 assert.match(mem,/2,601,723 allocs, 2,601,723 frees, 655,390,264 bytes allocated/);
 const tests=['checked_complex_operations_reject_zero_denominators','complex_display_forwards_real_formatting',
  'complex_division_preserves_exact_components_for_all_ownership_forms','complex_i_squared',
  'complex_multiplication_preserves_exact_components_for_all_ownership_forms','complex_negative_one_power_uses_reciprocal_semantics'];
 for(const profile of ['debug','release']) {
  const s=read('results/nfloat-complex-hyper-'+profile+'.stdout');
  assert.deepEqual([...s.matchAll(/^test (.+) \.\.\. ok$/gm)].map(v=>v[1]),tests);
  assert.match(s,/6 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out/);
 }
 return {summary,arithmetic,cmpabs,independentCmpabsSigns:signs,
  memory:{errors:0,liveBytes:0,allocations:2601723,frees:2601723,cumulativeBytes:655390264,includesOracle:true},hyperTests:{perProfile:6,profiles:['debug','release'],tests}};
}

export function checkFilterProbe() {
 const rows=read('results/nfloat-complex-filter-probe-native.stdout').trimEnd().split('\n').map(JSON.parse);
 assert.equal(rows.length,45);let cursor=0;const totals={};
 for(const fixture of ['rational','pi-far','pi-near'])for(const vertices of [3,4,8,16,32])for(let repeat=0;repeat<3;repeat++) {
  const row=rows[cursor++];assert.equal(row.fixture,fixture);assert.equal(row.vertices,vertices);assert.equal(row.repeat,repeat);
  const stage=fixture==='rational'?'Exact':fixture==='pi-far'?'Structural':'Refined';
  assert.equal(row.outcome,`Decided { value: Positive, certainty: Exact, stage: ${stage} }`);
  const common=[['real_op','add-ref',vertices-1],['real_op','mul-ref',2*vertices],['real_op','sub-ref',vertices]];
  const expected=fixture==='rational'?[]:fixture==='pi-far'?
   [['decide_real_sign','known-sign',1],['real','known-sign',1],...common,['resolve_real_sign','structural-real-facts',1]]:
   [['decide_real_sign','unknown',1],['real','known-sign',1],['real','refine-hit',1],...common,
    ['refine_real_sign','decided',1],['resolve_real_sign','real-refinement',1],
    ['signed_term_filter','mixed-signs',1],['signed_term_filter','zero-term',4]];
  assert.deepEqual(row.trace,expected.map(([operation,path,count])=>({operation,path,count})));
  for(const t of row.trace) {assert(t.count>0&&Number.isInteger(t.count));const key=fixture+':'+t.operation+':'+t.path;totals[key]=(totals[key]??0)+t.count;}
 }
 return {rows:rows.length,totals,qualification:'Public reachability/semantic trace only; no candidate or performance measurement.'};
}
if(process.argv.includes('--complex-summary'))console.log(JSON.stringify({controls:checkComplexControls(),probe:checkFilterProbe()}));
