import { spawn } from 'node:child_process';
import { readFileSync, writeFileSync, appendFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url));
const variants=['baseline','sign','reuse'];
const binaries=Object.fromEntries(variants.map(v=>[v,`/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/release/calcium-reuse-live-memory-${v}`]));
const metadata={started:new Date().toISOString(),binaries:Object.fromEntries(variants.map(v=>[v,
  {path:binaries[v],sha256:createHash('sha256').update(readFileSync(binaries[v])).digest('hex')}]))};
const output=resolve(here,'results/reuse-memory.jsonl');writeFileSync(output,'',{flag:'wx'});
async function run(v,args) {
  return new Promise((ok,fail)=>{
    const child=spawn(binaries[v],args.map(String),{stdio:['ignore','pipe','pipe']});
    let out='',err='';child.stdout.on('data',x=>out+=x);child.stderr.on('data',x=>err+=x);
    child.on('error',fail);child.on('close',code=>code===0?ok(JSON.parse(out)):fail(Error(`${v} ${args}: ${code} ${err}`)));
  });
}
const rows=[];
for(const depth of [1,8]) for(const threads of [1,8,32]) for(const rounds of [1,64,512]) {
  const group=[];
  for(let repetition=0;repetition<3;repetition++) for(const variant of variants) {
    const r={variant,repetition,...await run(variant,[threads,rounds,depth])};
    assert.deepEqual([r.threads,r.rounds,r.depth],[threads,rounds,depth]);
    assert.equal(r.queries,threads*rounds);
    assert.equal(r.equal,variant==='baseline'?0:r.queries);
    r.worker_retained_bytes=r.worker_after[0]-r.worker_before[0];
    r.worker_retained_blocks=r.worker_after[1]-r.worker_before[1];
    r.post_exit_bytes=r.process_after[0]-r.process_before[0];
    r.post_exit_blocks=r.process_after[1]-r.process_before[1];
    r.peak_over_worker_start=r.peak_bytes-r.worker_before[0];
    rows.push(r);group.push(r);appendFileSync(output,JSON.stringify(r)+'\n');
  }
  console.log(JSON.stringify({depth,threads,rounds,observations:group.length,
    variants:Object.fromEntries(variants.map(v=>[v,group.filter(r=>r.variant===v)
      .map(r=>({retained:r.worker_retained_bytes,blocks:r.worker_retained_blocks,afterExit:r.post_exit_bytes,peak:r.peak_over_worker_start}))]))}));
}
writeFileSync(resolve(here,'reuse-memory-summary.json'),JSON.stringify({...metadata,finished:new Date().toISOString(),
  observations:rows.length,rows,
  limits:'Counts live allocation sizes requested through the Rust global allocator, not allocator overhead, RSS, native TLS storage, or thread stacks. Worker snapshots occur after dropping all query expressions while threads remain alive. Shared constants preinitialized on the main thread. Peaks depend on concurrent scheduling and are not CPU benchmarks. This does not measure cold TLS latency or every graph/allocator layout.'},null,2)+'\n',{flag:'wx'});
