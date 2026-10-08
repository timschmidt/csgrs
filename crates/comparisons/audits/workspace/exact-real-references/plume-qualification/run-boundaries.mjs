import { spawnSync } from 'node:child_process';
import { writeFileSync } from 'node:fs';
import { resolve } from 'node:path';
const root=resolve(import.meta.dirname,'../..');
const bin=resolve(root,'.audit-plume.SZHOs8/kernel-probe');
const records=[];
for(const name of ['dy-norm-zero','dy-norm-cap','mul-zero-500','mul-zero-499','sin-zero','cos-zero','atan-zero','sqrt-zero','exp-zero','between-zero','gte-equal']) {
  const r=spawnSync(bin,['boundary',name,'+RTS','-M128m','-RTS'],{cwd:root,encoding:'utf8',timeout:4000,killSignal:'SIGKILL',maxBuffer:1024*1024});
  records.push({name,status:r.status,signal:r.signal,error:r.error?.code,stdout:r.stdout,stderr:r.stderr});
  writeFileSync(resolve(import.meta.dirname,'boundary-runs.json'),JSON.stringify(records,null,2)+'\n');
  console.log(JSON.stringify(records.at(-1)));
}
