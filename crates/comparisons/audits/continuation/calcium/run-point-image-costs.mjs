import {spawn} from 'node:child_process';
import {readFileSync,writeFileSync,appendFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
import {summarizePointImageCosts} from './point-image-cost-statistics.mjs';
import {checkPointImageSources} from './point-image-sources.mjs';
const mode=process.argv[2];assert(['cpu','allocation'].includes(mode));
const json=p=>JSON.parse(readFileSync(p,'utf8'));
const sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const b=json('point-image-guard-binaries.json'),variants=['baseline','candidate'];
const binaries={baseline:b.baselineBinaries['baseline-'+mode],candidate:b.binaries[mode]};
for(const v of variants)assert.equal(sha(binaries[v].path),binaries[v].sha256);
checkPointImageSources();
for(const[p,h]of Object.entries(b.guardSources))assert.equal(sha('point-image-guard/'+p),h,p);
for(const tag of ['guard-public-check','guard-release','baseline-debug','baseline-release','baseline-memcheck','guard-memcheck','consumer-debug'])
 assert.equal(json('results/point-image-'+tag+'.json').code,0,tag);
const started=new Date().toISOString();
if(mode==='allocation')assert(Date.parse(json('point-image-cost-cpu-summary.json').finished)<=Date.parse(started));
const expected=Object.fromEntries(variants.map(v=>[v,new Map(readFileSync('results/'+(v==='baseline'?'power-sums-public-baseline':'point-image-guard-public')+'.stdout','utf8')
 .trim().split('\n').map(s=>JSON.parse(s)).filter(r=>r.type==='cost-case').map(r=>[r.case,r.report]))]));
const path='results/point-image-cost-'+mode+'.jsonl',pilotPath='results/point-image-cost-'+mode+'-pilots.jsonl';
writeFileSync(path,'',{flag:'wx'});writeFileSync(pilotPath,'',{flag:'wx'});
async function run(variant,which,lifecycle,iterations){
 const begin=new Date().toISOString(),args=['-c','6',binaries[variant].path,mode,String(which),lifecycle,String(iterations)];
 const output=await new Promise((ok,fail)=>{
  const child=spawn('taskset',args,{stdio:['ignore','pipe','pipe']});let out='',err='';
  child.stdout.on('data',s=>out+=s);child.stderr.on('data',s=>err+=s);child.on('error',fail);
  child.on('close',(code,signal)=>code===0&&signal===null&&err===''?ok(out):fail(Error(JSON.stringify({variant,args,code,signal,out,err}))));
 });
 const row=JSON.parse(output);
 for(const[k,v]of Object.entries({mode,case:which,lifecycle,iterations}))assert.equal(row[k],v);
 assert(row.elapsed_ns>0);assert.deepEqual(row.expected,expected[variant].get(which));
 assert.equal(row.checksum,iterations*(row.expected.root?.polynomial.length??1));
 if(mode==='cpu')for(const k of ['requests','requested_bytes','live_delta','peak_delta'])assert.equal(row[k],0);
 return{variant,started:begin,finished:new Date().toISOString(),...row};
}
const rows=[],pilots=[];
for(let which=0;which<40;which++)for(const lifecycle of ['retained','fresh']){
 const pilot=[];
 if(mode==='cpu')for(const v of variants){const row=await run(v,which,lifecycle,10);pilot.push(row);pilots.push(row);appendFileSync(pilotPath,JSON.stringify(row)+'\n');}
 const iterations=mode==='cpu'?Math.max(10,Math.min(10000,Math.ceil(8e6/Math.max(...pilot.map(r=>r.elapsed_ns/r.iterations))))):64;
 const blocks=mode==='cpu'?12:3;
 for(let block=0;block<blocks;block++){
  const order=mode==='cpu'?(block%2?['candidate','baseline','baseline','candidate']:['baseline','candidate','candidate','baseline']):variants;
  for(const v of order){const row={block,...await run(v,which,lifecycle,iterations)};rows.push(row);appendFileSync(path,JSON.stringify(row)+'\n');}
 }
 console.log(JSON.stringify({mode,case:which,lifecycle,iterations,finished:new Date().toISOString(),observations:rows.length}));
}
const result={mode,started,finished:new Date().toISOString(),cpu:6,binaries,summaries:summarizePointImageCosts(mode,rows,pilots),
 limits:'Forty authored public queries, retained/fresh inputs, eight preconditioning calls per child. CPU pinned to core 6, twelve alternating ABBA/BAAB blocks, independently instrumented allocation binary. Fresh includes input construction/drop but not cold-process caches. Exact expected report and checksum checked for every observation. Changed outcomes are additional certified work, not equal-work speedups. Bootstrap intervals are per-group, not multiplicity-adjusted or universal. Rust requested bytes/live/peak omit native allocation overhead, RSS and stacks. No production retention or representative application-size qualification at this stage.'};
writeFileSync('point-image-cost-'+mode+'-summary.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({mode,status:'pass',groups:80,observations:rows.length,pilots:pilots.length}));
