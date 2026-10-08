import {spawn} from 'node:child_process';
import {writeFileSync,appendFileSync,copyFileSync,constants,statSync,mkdtempSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sources,sha,json} from './complex-product-sources.mjs';
const mode=process.argv[2];assert(['freeze','cpu','allocation'].includes(mode));
const sourceMap=sources(),variants=['baseline','candidate'];
async function command(file,args) {
 return new Promise((ok,fail)=>{
  const c=spawn(file,args,{stdio:['ignore','pipe','pipe']});let out='',err='';
  c.stdout.on('data',s=>out+=s);c.stderr.on('data',s=>err+=s);c.on('error',fail);
  c.on('close',(code,signal)=>code===0&&!signal?ok(out):fail(Error(JSON.stringify({file,args,code,signal,out,err}))));
 });
}
if(mode==='freeze') {
 const root=mkdtempSync('/tmp/calcium-complex-product.'),binaries={};
 for(const v of variants) {
  assert.equal(json('results/complex-product-build-'+v+'.json').code,0);
  for(const kind of ['cpu','allocation','check']) {
   const source='/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/release/complex-product-'+v+'-'+kind,path=root+'/'+v+'-'+kind;
   copyFileSync(source,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
   binaries[v+'-'+kind]={path,sha256:sha(path),bytes:statSync(path).size,size:(await command('size',[path])).trim()};
   assert.equal(sha(source),sha(path));
  }
 }
 writeFileSync('complex-product-binaries.json',JSON.stringify({root,binaries,sourceMap},null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({root,binaries}));
} else {
 const frozen=json('complex-product-binaries.json');assert.deepEqual(sourceMap,frozen.sourceMap);
 for(const b of Object.values(frozen.binaries))assert.equal(sha(b.path),b.sha256);
 const started=new Date().toISOString(),raw='results/complex-product-'+mode+'.jsonl';writeFileSync(raw,'',{flag:'wx'});
 const median=a=>{const v=[...a].sort((x,y)=>x-y);return(v[Math.floor((v.length-1)/2)]+v[Math.floor(v.length/2)])/2;};
 let seed=31337;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
 const summaries=[];
 const scenarios=[['dense','integer',0],['dense','dyadic',15],['dense','odd',6],['near-cancel','dyadic',0],
  ['sum-zero','integer',2],['unbalanced','integer',0],['mixed-scale','odd',0],['zero','integer',0]];
 async function run(v,args,iterations) {
  const started=new Date().toISOString();
  const row=JSON.parse(await command('taskset',['-c','6',frozen.binaries[v+'-'+mode].path,...args.map(String),String(iterations)]));
  assert.equal(row.iterations,iterations);assert(row.ns>0);assert.equal(row.preflight,true);
  assert.deepEqual([row.bits,row.family,row.scale,row.mask,row.route,row.lifecycle],args);
  assert.equal(row.allocation===null,mode==='cpu');
  return{started,finished:new Date().toISOString(),variant:v,...row};
 }
 for(const bits of [64,192,256,1024,4096,16384])for(const [family,scale,mask]of scenarios)
 for(const route of ['product','lattice'])for(const lifecycle of ['fresh','reused']) {
  const args=[bits,family,scale,mask,route,lifecycle],pilots=[];
  for(const v of variants)pilots.push(await run(v,args,2));
  const iterations=mode==='cpu'?Math.max(4,Math.min(4096,Math.ceil(6e6/Math.max(...pilots.map(r=>r.ns/r.iterations))))):16;
  const rows=[],blocks=mode==='cpu'?12:3;
  for(let block=0;block<blocks;block++) {
   const order=mode==='cpu'?(block%2?['candidate','baseline','baseline','candidate']:['baseline','candidate','candidate','baseline']):variants;
   for(const v of order) {const row={block,...await run(v,args,iterations)};rows.push(row);appendFileSync(raw,JSON.stringify(row)+'\n');}
  }
  const summary={bits,family,scale,mask,route,lifecycle,iterations,pilots,observations:rows.length};
  if(mode==='cpu') {
   const ratios=Array.from({length:blocks},(_,b)=>{
    const clock=v=>median(rows.filter(r=>r.block===b&&r.variant===v).map(r=>r.ns));return clock('candidate')/clock('baseline');
   });
   const bootstrap=Array.from({length:5000},()=>median(ratios.map(()=>ratios[random(blocks)]))).sort((a,b)=>a-b);
   Object.assign(summary,{pairedMedianRatio:median(ratios),pairedMedianBootstrap95:[bootstrap[125],bootstrap[4875]],
    nsPerQuery:Object.fromEntries(variants.map(v=>[v,median(rows.filter(r=>r.variant===v).map(r=>r.ns/r.iterations))]))});
  } else summary.measurements=Object.fromEntries(variants.map(v=>[v,
   ['requests','requestedBytes','liveDelta','peakDelta'].map((name,i)=>{
    const a=rows.filter(r=>r.variant===v).map(r=>r.allocation[i]);return{name,min:Math.min(...a),max:Math.max(...a)};
   })]));
  summaries.push(summary);console.log(JSON.stringify(summary));
 }
 assert.equal(summaries.length,192);
 writeFileSync('complex-product-'+mode+'-summary.json',JSON.stringify({mode,started,finished:new Date().toISOString(),cpu:6,summaries,
  limits:'192 kernel groups: six widths, eight shape/scale/sign scenarios, Rational/public Hyperlattice product, separately constructed first-use or four-query-prewarmed inputs. Construction/pool lifetime/oracles excluded; outputs dropped in window. Per-process independent-input exact num-rational preflight; primary GMP correctness campaign is separate. No quotient timing, whole-application/representative binary size, peak RSS, arbitrary histories, concurrency or universal gain claim. Paired percentile bootstrap intervals are per-group and not multiplicity-adjusted.'},null,2)+'\n',{flag:'wx'});
}
