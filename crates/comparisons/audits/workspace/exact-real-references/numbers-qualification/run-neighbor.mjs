import {spawnSync} from 'node:child_process';
import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
const dir=import.meta.dirname,root=resolve(dir,'../..'),mode=process.argv[2];
if(!['debug','release'].includes(mode)) throw Error('mode');
const bin=resolve(root,`.audit-numbers.rjcbha/neighbor-before-${mode}`);
const sha256=createHash('sha256').update(readFileSync(bin)).digest('hex');
const records=[];
for(const op of ['asin','atanh']) for(const terms of [0,64,256,512,700,720,736,740,744,746]) for(const p of [8,32,80,160]) {
  const args=[op,String(terms),String(p)];
  const r=spawnSync(bin,args,{cwd:root,encoding:'utf8',timeout:2000,killSignal:'SIGKILL',maxBuffer:1024*1024});
  const row={args,status:r.status,error:r.error?.code,signal:r.signal,stdout:r.stdout,stderr:r.stderr,sha256};
  if(r.error?.code==='EPERM') {writeFileSync(resolve(dir,`neighbor-${mode}-sandbox.json`),JSON.stringify(row,null,2));throw Error('subprocess permission failure');}
  records.push(row);writeFileSync(resolve(dir,`neighbor-before-${mode}-runs.json`),JSON.stringify(records,null,2)+'\n');
  console.log(JSON.stringify(row));
}
console.log(JSON.stringify({requests:records.length,finite:records.filter(r=>r.status===0&&!r.error).length,
  pass:records.filter(r=>r.status===0&&!r.error&&r.stdout.startsWith('PASS')).length,
  numericalFailure:records.filter(r=>r.status===0&&!r.error&&r.stdout.startsWith('FAIL')).length,
  exceptions:records.filter(r=>r.status!==null&&r.status!==0).length,caps:records.filter(r=>r.error==='ETIMEDOUT').length}));
