import {spawn} from 'node:child_process';
import {writeFileSync,appendFileSync,copyFileSync,constants,statSync} from 'node:fs';
import assert from 'node:assert/strict';
import {signFilterSources,sha,json} from './sign-filter-sources.mjs';
const mode=process.argv[2];assert(['cpu','allocation'].includes(mode));signFilterSources();
async function command(file,args) {
 return new Promise((ok,fail)=>{
  const c=spawn(file,args,{stdio:['ignore','pipe','pipe']});let out='',err='';
  c.stdout.on('data',s=>out+=s);c.stderr.on('data',s=>err+=s);c.on('error',fail);
  c.on('close',(code,signal)=>code===0&&!signal?ok(out):fail(Error(JSON.stringify({file,args,code,signal,out,err}))));
 });
}
const variants=['baseline','candidate'];
for(const v of variants)assert.equal(json('results/sign-filter-'+v+'-app-build.json').code,0);
let binaries;
if(mode==='cpu') {
 const root=process.argv[3];assert(/^\/tmp\/calcium-sign-filter\.[a-zA-Z0-9]+$/.test(root));binaries={};
 for(const v of variants)for(const kind of ['cpu','allocation']) {
  const source='/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/release/audit-sign-filter-'+v+'-'+kind,path=root+'/'+v+'-'+kind;
  copyFileSync(source,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
  binaries[v+'-'+kind]={path,sha256:sha(path),bytes:statSync(path).size,size:(await command('size',[path])).trim()};
  assert.equal(sha(source),sha(path));
 }
 writeFileSync('sign-filter-binaries.json',JSON.stringify(binaries,null,2)+'\n',{flag:'wx'});
}else binaries=json('sign-filter-binaries.json');
for(const b of Object.values(binaries))assert.equal(sha(b.path),b.sha256);
const started=new Date().toISOString(),raw='results/sign-filter-'+mode+'.jsonl';writeFileSync(raw,'',{flag:'wx'});
const median=a=>{const v=[...a].sort((x,y)=>x-y);return(v[Math.floor((v.length-1)/2)]+v[Math.floor(v.length/2)])/2;};
let seed=31337;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
const summaries=[];
async function run(variant,fixture,vertices,lifecycle,iterations) {
 const started=new Date().toISOString();
 const g=JSON.parse(await command('taskset',['-c','6',binaries[variant+'-'+mode].path,variant,fixture,String(vertices),lifecycle,String(iterations)]));
 for(const[k,v]of Object.entries({mode,variant,fixture,vertices,lifecycle,iterations}))assert.equal(g[k],v);
 assert(g.elapsed_ns>0);if(mode==='cpu')for(const k of ['requests','requested_bytes','live_delta','peak_delta'])assert.equal(g[k],0);
 return{started,finished:new Date().toISOString(),...g};
}
for(const fixture of ['rational','pi-far','pi-near','pi-reversed'])for(const vertices of [3,4,8,16,32,128])for(const lifecycle of ['retained','cloned-ring']) {
 const pilots=[];for(const v of variants)pilots.push(await run(v,fixture,vertices,lifecycle,2));
 const iterations=mode==='cpu'?Math.max(8,Math.min(20000,Math.ceil(6e6/Math.max(...pilots.map(g=>g.elapsed_ns/g.iterations))))):16;
 const rows=[],blocks=mode==='cpu'?12:3;
 for(let block=0;block<blocks;block++) {
  const order=mode==='cpu'?(block%2?['candidate','baseline','baseline','candidate']:['baseline','candidate','candidate','baseline']):variants;
  for(const v of order){const row={block,...await run(v,fixture,vertices,lifecycle,iterations)};rows.push(row);appendFileSync(raw,JSON.stringify(row)+'\n');}
 }
 assert.equal(new Set([...pilots,...rows].map(g=>g.outcome)).size,1);
 const summary={fixture,vertices,lifecycle,iterations,observations:rows.length,pilots,outcome:rows[0].outcome};
 if(mode==='cpu') {
  const ratios=Array.from({length:blocks},(_,b)=>{
   const clock=v=>median(rows.filter(r=>r.block===b&&r.variant===v).map(r=>r.elapsed_ns));return clock('candidate')/clock('baseline');
  });
  const bootstrap=Array.from({length:5000},()=>median(ratios.map(()=>ratios[random(blocks)]))).sort((a,b)=>a-b);
  Object.assign(summary,{pairedMedianRatio:median(ratios),pairedMedianBootstrap95:[bootstrap[125],bootstrap[4875]],
   nsPerQuery:Object.fromEntries(variants.map(v=>[v,median(rows.filter(r=>r.variant===v).map(r=>r.elapsed_ns/r.iterations))]))});
 }else summary.measurements=Object.fromEntries(variants.map(v=>[v,Object.fromEntries(['requests','requested_bytes','live_delta','peak_delta'].map(k=>{
  const a=rows.filter(r=>r.variant===v).map(r=>r[k]);return[k,{min:Math.min(...a),max:Math.max(...a)}];
 }))]));
 summaries.push(summary);console.log(JSON.stringify(summary));
}
writeFileSync('sign-filter-'+mode+'-summary.json',JSON.stringify({mode,started,finished:new Date().toISOString(),cpu:6,binaries,summaries,
 limits:'48 public ring-area groups, four fixtures, six vertex counts, retained or cloned-ring inputs. Independent exact rational shoelace and Machin alternating-series sign oracle recorded separately. Sixteen prewarm queries; cloned-ring timing includes cloning and dropping points but does not rebuild cold scalar values. Each timed query asserts the complete expected PredicateOutcome including certainty/stage. Uninstrumented CPU, separate Rust allocation requests/bytes/live/peak. Per-group paired bootstrap estimates are not multiplicity-adjusted; no universal timing, memory bound, RSS, concurrent or production-retention claim.'},null,2)+'\n',{flag:'wx'});
