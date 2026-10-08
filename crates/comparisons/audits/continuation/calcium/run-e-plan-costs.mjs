import {spawn} from 'node:child_process';
import {writeFileSync,appendFileSync,copyFileSync,constants,statSync,mkdtempSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sources,sha,json} from './e-plan-sources.mjs';
const mode=process.argv[2];assert(['freeze','cpu','allocation'].includes(mode));
const sourceMap=sources(),variants=['baseline','candidate'];
async function command(file,args) {
 return new Promise((ok,fail)=>{
  const c=spawn(file,args,{stdio:['ignore','pipe','pipe']});let out='',err='';
  c.stdout.on('data',s=>out+=s);c.stderr.on('data',s=>err+=s);c.on('error',fail);
  c.on('close',(code,signal)=>code===0&&!signal&&err===''?ok(out):fail(Error(JSON.stringify({file,args,code,signal,out,err}))));
 });
}
if(mode==='freeze') {
 const root=mkdtempSync('/tmp/calcium-e-plan.'),binaries={};
 for(const v of variants) {
  assert.equal(json('results/e-plan-build-'+v+(v==='baseline'?'-fixed':'')+'.json').code,0);
  for(const kind of ['cpu','allocation','check']) {
   const source='/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/release/e-plan-'+v+'-'+kind,path=root+'/'+v+'-'+kind;
   copyFileSync(source,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
   binaries[v+'-'+kind]={path,sha256:sha(path),bytes:statSync(path).size,size:(await command('size',[path])).trim()};
   assert.equal(sha(source),sha(path));
  }
 }
 writeFileSync('e-plan-binaries.json',JSON.stringify({root,binaries,sourceMap},null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({root,binaries}));
} else {
 const frozen=json('e-plan-binaries.json');assert.deepEqual(sourceMap,frozen.sourceMap);
 for(const b of Object.values(frozen.binaries))assert.equal(sha(b.path),b.sha256);
 const started=new Date().toISOString(),raw='results/e-plan-'+mode+'.jsonl';writeFileSync(raw,'',{flag:'wx'});
 const median=a=>{const v=[...a].sort((x,y)=>x-y);return(v[Math.floor((v.length-1)/2)]+v[Math.floor(v.length/2)])/2;};
 let seed=42819;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
 const summaries=[],scenarios=[];
 for(const p of [0,-8,-32,-64,-128,-512,-4096,-16384,-65536,-262144]) {
  for(const route of ['planner','kernel'])scenarios.push([p,route,'fresh']);
  for(const lifecycle of ['fresh','warm','coarsen',...(p<0?['refine']:[])])scenarios.push([p,'public',lifecycle]);
 }
 for(const p of [-64,-4096,-65536])for(const lifecycle of ['fresh','warm','coarsen','refine'])scenarios.push([p,'pi-control',lifecycle]);
 assert.equal(scenarios.length,71);
 async function run(v,args,iterations) {
  const started=new Date().toISOString();
  const row=JSON.parse(await command('taskset',['-c','6',frozen.binaries[v+'-'+mode].path,...args.map(String),String(iterations)]));
  assert.equal(row.iterations,iterations);assert(row.ns>0);assert.deepEqual([row.p,row.route,row.lifecycle],args);
  assert.equal(row.allocation===null,mode==='cpu');
  return{started,finished:new Date().toISOString(),variant:v,...row};
 }
 for(const args of scenarios) {
  const[p,route,lifecycle]=args,pilots=[],single=['public','pi-control'].includes(route)&&['fresh','refine'].includes(lifecycle);
  for(const v of variants)pilots.push(await run(v,args,single?1:2));
  const iterations=single?1:mode==='cpu'?Math.max(2,Math.min(1048576,Math.ceil(6e6/Math.max(...pilots.map(r=>r.ns/r.iterations))))):16;
  const rows=[],blocks=mode==='cpu'?12:3;
  for(let block=0;block<blocks;block++) {
   const order=mode==='cpu'?(block%2?['candidate','baseline','baseline','candidate']:['baseline','candidate','candidate','baseline']):variants;
   for(const v of order) {const row={block,...await run(v,args,iterations)};rows.push(row);appendFileSync(raw,JSON.stringify(row)+'\n');}
  }
  assert.equal(new Set([...pilots,...rows].map(r=>r.fingerprint+':'+r.answerBits)).size,1,'paired answers: '+args);
  const summary={p,route,lifecycle,iterations,pilots,observations:rows.length};
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
 assert.equal(summaries.length,71);assert.deepEqual(sources(),sourceMap);
 writeFileSync('e-plan-'+mode+'-summary.json',JSON.stringify({mode,started,finished:new Date().toISOString(),cpu:6,summaries,
  limits:'71 predeclared groups, ten precision levels. Planner and exact series kernel are uncached. Fresh/refine public calls are one query in each fresh process; warm/coarsen calls are repeated after explicit setup. Pi is an unchanged-path control. Wrapper/setup and postflight fingerprint are excluded, output destruction included. Allocation and CPU binaries are separate. Matched fingerprints are not a mathematical oracle; GMP/MPFR qualification is separate. 12 alternating ABBA/BAAB blocks and 5000 paired median bootstrap resamples per CPU group, intervals not multiplicity-adjusted. No whole-application timing, peak RSS, universal gain, arbitrary history or other-target claim.'},null,2)+'\n',{flag:'wx'});
}
