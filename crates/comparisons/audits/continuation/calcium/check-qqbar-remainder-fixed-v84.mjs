import {readFileSync}from 'node:fs';
import assert from 'node:assert/strict';
import {resolve}from 'node:path';
import {fileURLToPath}from 'node:url';
import {json}from './zero-factor-retained-sources-v75.mjs';
import {checkValue,q,cmp,add,neg,mul}from './symbolic-boundary-oracle-v83.mjs';
import {scalar,integer,fzero,fadd,fneg,fmul,root2,root3}from './point-extended-field.mjs';
const raw=tag=>readFileSync('results/'+tag+'.stdout','utf8'),err=tag=>readFileSync('results/'+tag+'.stderr','utf8');
const rows=tag=>{const s=raw(tag);assert(s.length&&s.endsWith('\n'));return s.trimEnd().split('\n').map(JSON.parse);};
function gate(tag,code=0,signal=null){const g=json('results/'+tag+'.json');assert.equal(g.code,code);assert.equal(g.signal,signal);return g;}
const zero=a=>checkValue(a,{re:integer(0),im:fzero()});
function wire(a){
 assert(Array.isArray(a)&&a.length===3&&a.every(s=>typeof s==='string'&&/^-?\d+$/.test(s)&&s.length<12000));
 const e=BigInt(a[2]);assert(e>=-32768n&&e<=32768n);
 const b=a.slice(0,2).map(v=>e<0n?q(BigInt(v),1n<<-e):q(BigInt(v)<<e));
 assert(cmp(b[0],b[1])<=0);assert(cmp(add(b[1],neg(b[0])),q(1n,1n<<96n))<=0);return b;
}
function nthBound(a,degree,target,scale,sign){
 let b=wire(a);if(sign===-1)b=[neg(b[1]),neg(b[0])];b=b.map(x=>mul(x,q(scale)));
 assert(cmp(b[0],q(0))>0);const p=x=>q(x[0]**BigInt(degree),x[1]**BigInt(degree));
 assert(cmp(p(b[0]),q(target))<=0&&cmp(q(target),p(b[1]))<=0);
}
function zeroBound(a){const b=wire(a);assert(cmp(b[0],q(0))<=0&&cmp(q(0),b[1])<=0);}
export function checkCleanupRows(rs,degree,limit,count){
 assert.equal(rs.length,count+1);assert.deepEqual(rs.at(-1),{terminal:true,rows:count});let values=0;
 const success=limit>=(degree===2?4:8);
 for(let i=0;i<count;i++){
  const r=rs[i];assert.deepEqual(Object.keys(r).sort(),['family','limit','iteration','status',...(success?['roots']:[])].sort());
  assert.equal(r.family,'root-cleanup');assert.equal(r.limit,limit);assert.equal(r.iteration,i);assert.equal(r.status,success?0:2);
  if(!success)continue;assert.equal(r.roots.length,degree);const signs=[];
  for(const[j,a]of r.roots.entries()){
   assert.deepEqual(Object.keys(a).sort(),['index','poly','real','imag'].sort());assert.equal(a.index,j);
   assert.deepEqual(a.poly,Array.from({length:2*degree+1},(_,i)=>i===0?'-2':i===2*degree?'1':'0'));
   const real=wire(a.real),imag=wire(a.imag),positive=cmp(real[0],q(0))>0,imagSign=cmp(imag[0],q(0))>0?1:cmp(imag[1],q(0))<0?-1:0;
   if(degree===2){nthBound(a.real,4,2,1,positive?1:-1);zeroBound(a.imag);assert.equal(imagSign,0);}
   else if(positive){nthBound(a.real,6,2,1,1);zeroBound(a.imag);assert.equal(imagSign,0);}
   else{assert(imagSign!==0);nthBound(a.real,6,2,2,-1);nthBound(a.imag,6,54,2,imagSign);}
   signs.push([positive?1:-1,imagSign].join(','));values++;
  }
  assert.deepEqual(signs.sort(),(degree===2?['1,0','-1,0']:['1,0','-1,1','-1,-1']).sort());
 }
 return values;
}
export function normalExpected(seed){
 const x=fadd(root2(),integer(seed-4)),y=fadd(root3(),integer(1-seed)),xp=[integer(1),x,fmul(x,x)],yp=[integer(1),y,fmul(y,y)];let sum=fzero();
 for(let i=0;i<3;i++)for(let j=0;j<3;j++){
  const k=seed<8?(seed+3*i+5*j)%7-3:seed===9&&i===0&&j===0?7:0;
  sum=fadd(sum,fmul(integer(k),fmul(xp[i],yp[j])));
 }return sum;
}
export function checkNormalRows(rs){
 assert.equal(rs.length,245);assert.deepEqual(rs.at(-1),{terminal:true,rows:244});let index=0,values=0;
 for(let order=0;order<3;order++)for(let seed=0;seed<10;seed++)for(let alias=0;alias<3;alias++)for(let generic=0;generic<2;generic++){
  const r=rs[index++];assert.deepEqual(Object.keys(r).sort(),['family','order','seed','alias','generic','ok','poly','real','imag'].sort());
  for(const[k,v]of Object.entries({family:'normal-mpoly',order,seed,alias,generic,ok:1}))assert.equal(r[k],v,k);
  checkValue(r,{re:normalExpected(seed),im:fzero()});values++;
 }
 for(let wrapper=0;wrapper<3;wrapper++)for(let flags=0;flags<4;flags++)for(let n=2;n<=3;n++){
  const signs=[];for(let j=0;j<2;j++){
   const r=rs[index++];assert.deepEqual(Object.keys(r).sort(),['family','wrapper','flags','n','index','poly','real','imag'].sort());
   for(const[k,v]of Object.entries({family:'root-wrapper',wrapper,flags,n,index:j}))assert.equal(r[k],v,k);
   const sign=BigInt(r.real[0])<0n?-1:1;signs.push(sign);
   const x=fmul(scalar(q(sign,wrapper===1?3:1)),n===2?root2():root3());checkValue(r,{re:x,im:fzero()});values++;
  }assert.deepEqual(signs.sort(),[-1,1]);
 }
 for(let wrapper=0;wrapper<2;wrapper++)for(const flags of [0,2])for(let dim=0;dim<=3;dim++){
  const r=rs[index++];assert.deepEqual(Object.keys(r).sort(),['family','wrapper','flags','dim','roots'].sort());
  for(const[k,v]of Object.entries({family:'triangular',wrapper,flags,dim}))assert.equal(r[k],v,k);assert.equal(r.roots.length,dim);
  const signs=[];for(const[j,a]of r.roots.entries()){
   assert.equal(a.index,j);const negative=BigInt(a.real[0])<0n;signs.push(negative?-2:1);
   checkValue(a,{re:scalar(q(negative?-2:1,wrapper?3:1)),im:fzero()});values++;
  }assert.deepEqual(signs.sort(),Array.from({length:dim},(_,i)=>i?1:-2).sort());
 }
 assert.equal(index,244);assert.equal(values,252);return{records:245,values};
}
function memory(tag,count,leak){
 const s=err(tag);gate(tag,leak?97:0);const total=s.match(/([\d,]+) allocs, ([\d,]+) frees, ([\d,]+) bytes allocated/);assert(total);
 const n=s=>Number(s.replaceAll(',',''));const allocations=n(total[1]),frees=n(total[2]),requestedBytes=n(total[3]);
 if(leak){
  for(const[label,bytes,blocks]of [['in use at exit',408*count,4*count],['definitely lost',360*count,count],['indirectly lost',48*count,3*count]]){
   const re=new RegExp(label+':\\s*([\\d,]+) bytes in ([\\d,]+) blocks'),m=s.match(re);assert(m);assert.equal(n(m[1]),bytes);assert.equal(n(m[2]),blocks);
  }
  assert(s.includes('_gr_qqbar_poly_roots'));assert(s.includes('ERROR SUMMARY: 2 errors from 2 contexts'));assert.equal(allocations-frees,4*count);
 }else{assert(s.includes('in use at exit: 0 bytes in 0 blocks'));assert(s.includes('ERROR SUMMARY: 0 errors from 0 contexts'));assert.equal(allocations,frees);}
 return{tag,allocations,frees,requestedBytes,liveBytes:leak?408*count:0};
}
export function remainderEvidence(){
 let monomialValues=0,aborts=0;
 for(const generic of [0,1])for(const exponent of [0,30,63,64,80,256])for(const terms of [1,2]){
  const tag='qqbar-monomial-'+generic+'-'+exponent+'-'+terms+'-v84',fail=generic===0&&exponent>=64&&terms===1;
  gate(tag,fail?null:0,fail?'SIGABRT':null);
  if(fail){assert(raw(tag).includes('Exponent vector does not fit a ulong.'));aborts++;continue;}
  const rs=rows(tag);assert.equal(rs.length,1);const r=rs[0];
  for(const[k,v]of Object.entries({family:'monomial',generic,exponent,terms,ok:1}))assert.equal(r[k],v,k);
  assert.equal(r.packedBits>64,exponent>=63);checkValue(r,{re:integer(terms),im:fzero()});monomialValues++;
 }
 const overflows=[];for(const degree of [57,58,59,60,61,62,63]){
  const tag='qqbar-root-limit-sanitized-'+degree+'-v84',overflow=degree<=61;
  gate(tag,overflow?null:0,overflow?'SIGILL':null);const conjugations=1n<<BigInt(degree+1),max=(1n<<63n)-1n;
  if(overflow){assert.equal(raw(tag),'');assert(conjugations<=max&&conjugations*BigInt(degree)>max);overflows.push({degree,conjugations:String(conjugations),product:String(conjugations*BigInt(degree))});}
  else{assert(conjugations>max);assert.deepEqual(rows(tag),[{family:'root-limit',degree,bounded:0,ok:0}]);}
 }
 for(const mode of ['native','sanitized']){const tag='qqbar-root-limit-bounded-'+mode+'-v84';gate(tag);assert.deepEqual(rows(tag),[{family:'root-limit',degree:61,bounded:1,ok:0}]);}
 const debuggerTag='qqbar-root-overflow-gdb-v84';gate(debuggerTag);const trace=raw(debuggerTag);
 assert(trace.includes('Program received signal SIGILL'));assert(trace.includes('roots_poly_squarefree.c:158'));assert(trace.includes('deg_bound = conjugations * deg'));assert(trace.includes('ud2'));
 const controls=[];let rootValues=0,rootQueries=0;
 for(const degree of [2,3])for(const limit of degree===2?[1,2,4]:[4,8])for(const count of [1,32,128]){
  const native=degree===2?'qqbar-root-cleanup-'+limit+'-'+count+'-v84':'qqbar-cleanup-cubic-native-'+limit+'-'+count+'-v84',
   mem=degree===2?'qqbar-root-cleanup-mem-'+limit+'-'+count+'-v84':'qqbar-cleanup-cubic-mem-'+limit+'-'+count+'-v84';
  gate(native);assert.equal(raw(native),raw(mem));rootValues+=checkCleanupRows(rows(native),degree,limit,count);rootQueries+=count;
  controls.push({...memory(mem,count,degree===3&&limit===4),degree,limit,count});
 }
 gate('qqbar-normal-native-v84');assert.equal(raw('qqbar-normal-native-v84'),raw('qqbar-normal-mem-v84'));
 const normalRows=rows('qqbar-normal-native-v84'),normal=checkNormalRows(normalRows),normalMemory=memory('qqbar-normal-mem-v84',1,false);
 let corruptions=0;
 for(const mutate of [r=>r.pop(),r=>r[1]=r[0],r=>r[0].poly[0]='0',r=>r[0].real=['0','0','0'],r=>r[0].imag=['1','1','0'],r=>r[0].real.reverse(),r=>r[0].ok=0,r=>r.at(-1).rows--]){
  const bad=structuredClone(normalRows);mutate(bad);assert.throws(()=>checkNormalRows(bad));corruptions++;
 }
 const cubic=rows('qqbar-cleanup-cubic-native-8-1-v84');
 for(const mutate of [r=>r[0].roots.pop(),r=>r[0].roots[1]=r[0].roots[0],r=>r[0].roots[0].poly[0]='2',r=>r[0].roots[0].real=['1','2','0'],r=>r[0].roots[1].imag=['0','0','0'],r=>r[0].status=2]){
  const bad=structuredClone(cubic);mutate(bad);assert.throws(()=>checkCleanupRows(bad,3,8,1));corruptions++;
 }
 return{checkpoint:84,status:'independently-qualified-normal-values-and-three-donor-defect-families',monomialValues,aborts,overflows,
  rootQueries,rootValues,controls,normal,normalMemory,corruptions,independentAlgebraicValues:monomialValues+rootValues+normal.values,
  scope:'Pinned current donor only. Complete minimal polynomial and both selected component bounds; x^(2d)-2 irreducible by Eisenstein, signed component radicals select the required roots. Trap location checked only for degree 57; five trap inputs have exact overflow bounds. No runtime-installed UBSan report, general root-memory guarantee, production edit or benchmark.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(remainderEvidence()));
