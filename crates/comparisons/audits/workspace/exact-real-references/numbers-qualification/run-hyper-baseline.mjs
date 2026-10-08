import {spawnSync} from 'node:child_process';
import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
const dir=import.meta.dirname,root=resolve(dir,'../..');
const bin=resolve(root,'.audit-numbers.rjcbha/numbers-oracle-before');
const binarySha256=createHash('sha256').update(readFileSync(bin)).digest('hex');
const requests=[];
for(const n of [100,-100,4]) for(const scale of [1,4,64]) for(const warm of [false,true])
  requests.push(['opaque-atan',String(n),String(scale),'32',warm?'warm':'cold']);
const records=[];
for(const args of requests) {
  const r=spawnSync(bin,args,{cwd:root,encoding:'utf8',timeout:2000,killSignal:'SIGKILL',maxBuffer:1024*1024});
  const row={args,status:r.status,error:r.error?.code,signal:r.signal,stdout:r.stdout,stderr:r.stderr,binarySha256};
  if(r.error?.code==='EPERM') {writeFileSync(resolve(dir,'hyper-baseline-sandbox.json'),JSON.stringify(row,null,2));throw Error('subprocess permission failure');}
  records.push(row);
  writeFileSync(resolve(dir,'hyper-baseline-runs.json'),JSON.stringify(records,null,2)+'\n');
  console.log(JSON.stringify(row));
}
console.log(`Completed ${records.length}; passed ${records.filter(x=>x.status===0&&!x.error).length}; timed out ${records.filter(x=>x.error==='ETIMEDOUT').length}. Caps are not timings.`);
