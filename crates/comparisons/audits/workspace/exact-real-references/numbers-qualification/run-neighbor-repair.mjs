import {spawnSync} from 'node:child_process';
import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
const dir=import.meta.dirname,root=resolve(dir,'../..'),mode=process.argv[2];
const sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const records=[];
function run(bin,args,label,timeout=10000){
  const r=spawnSync('taskset',['-c','6',bin,...args],{cwd:root,encoding:'utf8',timeout,killSignal:'SIGKILL',maxBuffer:1024*1024});
  const row={label,args,status:r.status,error:r.error?.code,signal:r.signal,stdout:r.stdout,stderr:r.stderr,sha256:sha(bin),source:sha(resolve(dir,['debug','release'].includes(mode)?'neighbor_probe.rs':'neighbor_controls.rs'))};
  records.push(row);writeFileSync(resolve(dir,`neighbor-repair-${mode}-runs.json`),JSON.stringify(records,null,2)+'\n');console.log(JSON.stringify(row));
  if(r.error||r.status!==0||r.stdout.startsWith('FAIL'))throw Error('failed request '+label);
}
if(['debug','release'].includes(mode)){
  const bin=resolve(root,`.audit-numbers.rjcbha/neighbor-sample-${mode}`);
  for(const op of ['asin','atanh'])for(const terms of [0,64,256,512,700,720,736,740,744,746])for(const p of [8,32,80,160])run(bin,[op,String(terms),String(p)],`${op}-${terms}-${p}`);
}else if(['pilot','paired','repaired'].includes(mode)){
  const versions=mode==='repaired'?['after']:['before','after'];
  const families=mode==='repaired'?['repaired']:['rational','known-small','known-negative','opaque-small','warm-small','hint-small','hint-medium'];
  for(let round=0;round<(mode==='pilot'?1:9);round++)for(const op of ['asin','atanh'])for(const family of families){
    for(const version of round%2?[...versions].reverse():versions)run(resolve(root,`.audit-numbers.rjcbha/neighbor-controls-${version==='after'?'sample':version}`),[op,family,'256'],`${round}-${version}-${op}-${family}`);
  }
}else throw Error('mode');
console.log(JSON.stringify({requests:records.length,passed:records.length}));
