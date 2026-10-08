import {spawnSync} from 'node:child_process';
import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
const dir=import.meta.dirname,root=resolve(dir,'../..'),mode=process.argv[2];
if(!['pilot','paired','repaired'].includes(mode)) throw Error('mode');
const families=mode==='repaired'?['opaque-negative','hint-large']:
  ['rational','known-small','known-negative','opaque-small','opaque-positive','warm-negative','hint-small'];
const records=[];
for(let round=0;round<(mode==='pilot'?1:9);round++) for(const family of families) {
  const variants=mode==='pilot'?['before']:mode==='repaired'?['after']:round%2?['after','before']:['before','after'];
  for(const variant of variants) {
    const bin=resolve(root,`.audit-numbers.rjcbha/atan-${variant}`);
    const sha256=createHash('sha256').update(readFileSync(bin)).digest('hex');
    const args=['-c','6',bin,'bench',family,mode==='pilot'?'32':'256'];
    const r=spawnSync('taskset',args,{cwd:root,encoding:'utf8',timeout:20000,killSignal:'SIGKILL',maxBuffer:1024*1024});
    const row={round,family,variant,sha256,args,status:r.status,error:r.error?.code,signal:r.signal,stdout:r.stdout,stderr:r.stderr};
    if(r.error?.code==='EPERM') {writeFileSync(resolve(dir,`atan-${mode}-sandbox.json`),JSON.stringify(row,null,2));throw Error('subprocess permission failure');}
    records.push(row);writeFileSync(resolve(dir,`atan-${mode}-runs.json`),JSON.stringify(records,null,2)+'\n');
    if(r.status!==0||r.error) throw Error(JSON.stringify(row));
    const parts=r.stdout.trim().split('\t');
    if(parts.length!==7||parts[0]!==family||parts.slice(1).some(x=>!/^\d+$/.test(x))) throw Error('invalid output');
    console.log([round,variant,...parts].join('\t'));
  }
}
console.log(`PASS ${records.length} numerically qualified CPU6 process observations; pilot is not final evidence, round0 excluded from final analysis.`);
