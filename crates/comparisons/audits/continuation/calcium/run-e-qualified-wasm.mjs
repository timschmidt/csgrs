import {spawn} from 'node:child_process';
import {readFileSync,writeFileSync,appendFileSync,mkdtempSync,copyFileSync,constants,statSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sources,sha,json} from './e-plan-qualified-sources.mjs';
const mode=process.argv[2];assert(['build','check'].includes(mode));
const sourceMap=sources(),o=json('e-plan-qualified-origin.json'),inputs={};
for(const v of ['baseline','candidate']) {
 const s=readFileSync((v==='baseline'?o.baseline:o.candidate)+'/hyperreal/src/computable/approximation/constants.rs','utf8');
 assert.equal(readFileSync('e-qualified-wasm-'+v+'/kernel.rs','utf8'),'use num::{BigInt, BigUint, One};\ntype Precision = i32;\n'+s.slice(s.indexOf('fn e_terms_for_precision('))
  .replace('fn e_terms_for_precision(','pub fn e_terms_for_precision(').replace('fn e(p:','pub fn e(p:'));
 for(const name of ['Cargo.toml','kernel.rs']) {const p='e-qualified-wasm-'+v+'/'+name;inputs[p]=sha(p);}
}
for(const p of ['e-qualified-wasm.rs','run-e-qualified-wasm.mjs','prepare-e-qualified-wasm.mjs'])inputs[p]=sha(p);
async function captured(tag,command,args) {
 await new Promise((ok,fail)=>{
  const c=spawn(process.execPath,['capture.mjs',tag,'.',command,...args],{stdio:'inherit'});c.on('error',fail);
  c.on('close',(code,signal)=>code===0&&!signal?ok():fail(Error(JSON.stringify({tag,code,signal}))));
 });
}
if(mode==='build') {
 const root=mkdtempSync('/tmp/calcium-e-qualified-wasm.'),modules={};
 for(const v of ['baseline','candidate']) {
  await captured('e-qualified-wasm-build-'+v,'env',[
   'CARGO_TARGET_DIR=/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse','CARGO_INCREMENTAL=0','CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2',
   'cargo','build','--offline','--release','--target','wasm32-unknown-unknown','--manifest-path','e-qualified-wasm-'+v+'/Cargo.toml','--lib']);
  const source='/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/wasm32-unknown-unknown/release/calcium_e_qualified_wasm_'+v+'.wasm',path=root+'/'+v+'.wasm';
  copyFileSync(source,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);assert.equal(sha(path),sha(source));
  const module=new WebAssembly.Module(readFileSync(path)),imports=WebAssembly.Module.imports(module),exports=WebAssembly.Module.exports(module);
  assert.deepEqual(imports,[]);for(const name of ['plan','evaluate','answer_word','time_loop'])assert(exports.some(e=>e.name===name&&e.kind==='function'));
  modules[v]={path,sha256:sha(path),bytes:statSync(path).size,imports,exports};
  const lock='e-qualified-wasm-'+v+'/Cargo.lock';inputs[lock]=sha(lock);
 }
 assert.deepEqual(sources(),sourceMap);
 writeFileSync('e-qualified-wasm-binaries.json',JSON.stringify({root,modules,sourceMap,inputs},null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({root,modules}));
} else {
 const frozen=json('e-qualified-wasm-binaries.json');assert.deepEqual(frozen.sourceMap,sourceMap);
 for(const[p,h]of Object.entries(frozen.inputs))assert.equal(sha(p),h,p);
 const raw='results/e-qualified-wasm-check.jsonl';writeFileSync(raw,'',{flag:'wx'});
 const emit=r=>appendFileSync(raw,JSON.stringify(r)+'\n');
 const planRows=readFileSync('results/e-plan-native-baseline-plans.stdout','utf8').trim().split('\n').map(s=>JSON.parse(s)).filter(r=>r.kind==='plan');
 assert.equal(planRows.length,4151);
 let factorial=1n,k=1;
 for(const r of planRows) {const needed=r.p<0?-r.p+4:4;while(factorial.toString(2).length<=needed) {k++;factorial*=BigInt(k);}assert.equal(r.expected,k-1);}
 // Different recurrence from the binary split, with a complete positive tail.
 let n=0,num=1n,den=1n;
 while(den.toString(2).length<=262144+16) {n++;den*=BigInt(n);num=num*BigInt(n)+1n;}
 const tailDen=den*BigInt(n+1),tailNum=num*BigInt(n+1)+2n;
 const positions=[...new Set([...Array.from({length:73},(_,i)=>i-64),-127,-128,-129,-255,-256,-257,-511,-512,-513,
  -1023,-1024,-1025,-4095,-4096,-4097,-16384,-32768,-65536,-120700,-262144])].sort((a,b)=>b-a);
 assert.equal(positions.length,93);const started=new Date().toISOString(),summaries=[];
 for(const variant of ['baseline','candidate']) {
  const b=frozen.modules[variant];assert.equal(sha(b.path),b.sha256);const module=new WebAssembly.Module(readFileSync(b.path));
  const fresh=()=>new WebAssembly.Instance(module,{}).exports;const instance=fresh();let plans=0,enclosures=0;
  for(const r of planRows) {const actual=instance.plan(r.p);assert.equal(actual,r.expected);emit({variant,kind:'plan',p:r.p,actual,expected:r.expected});plans++;}
  function enclose(instance,p,route,kind) {
   const len=instance.evaluate(p,route);assert(len<=8193);let a=0n;
   for(let i=len-1;i>=0;i--)a=(a<<32n)+BigInt(instance.answer_word(i)>>>0);
   if(p<0) {assert((a-1n)*den<(num<<BigInt(-p)));assert((a+1n)*tailDen>(tailNum<<BigInt(-p)));}
   else {assert(((a-1n)*den<<BigInt(p))<num);assert(((a+1n)*tailDen<<BigInt(p))>tailNum);}
   emit({variant,kind,p,integer:a.toString(16)});enclosures++;
  }
  for(const p of positions) {enclose(instance,p,0,'kernel');enclose(instance,p,1,'public-refine');}
  for(const p of [...positions].reverse())enclose(instance,p,1,'public-coarsen');
  for(const p of [-8,-64,-128,-512,-4096,-16384,-65536,-262144])enclose(instance,p,3,'exp-one');
  for(const p of [0,-1,-8,-32,-64,-128,-512,-4096,-16384,-32768,-65536,-120700,-262144])enclose(fresh(),p,1,'fresh-instance');
  assert.equal(plans,4151);assert.equal(enclosures,300);summaries.push({variant,plans,enclosures});
 }
 assert.deepEqual(sources(),sourceMap);
 const result={started,finished:new Date().toISOString(),node:process.version,v8:process.versions.v8,oracleTerms:n,summaries,rows:8902,
  limits:'Wasm32 executes inside local Node/V8; this is not physical ARM/RISC-V execution. Exact complete output words are checked, not only fingerprints. 4151 term requests and300 scalar outputs per variant; finite corpus only. Fresh module instances isolate public constant caches for13 requests. No WASM thread/cancellation/serialization execution, general FENV claim or comparative timing in this functional gate.'};
 writeFileSync('e-qualified-wasm-check-summary.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});console.log(JSON.stringify(result));
}
