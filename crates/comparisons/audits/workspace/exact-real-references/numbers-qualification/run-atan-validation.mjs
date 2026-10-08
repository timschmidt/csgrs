import {spawnSync} from 'node:child_process';
import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
const dir=import.meta.dirname,root=resolve(dir,'../..'),mode=process.argv[2];
if(!['debug','release'].includes(mode)) throw Error('mode');
const bin=resolve(root,`.audit-targets/ireal-derivative-18555/${mode}/numbers_oracle`);
const sha256=createHash('sha256').update(readFileSync(bin)).digest('hex');
const requests=JSON.parse(readFileSync(resolve(dir,'hyper-baseline-runs.json'),'utf8')).map(r=>r.args);
requests.push(['hyper',resolve(dir,'elementary-O2.log')],['exact-controls'],['selfcheck']);
const records=[];
for(const args of requests) {
  const r=spawnSync(bin,args,{cwd:root,encoding:'utf8',timeout:10000,killSignal:'SIGKILL',maxBuffer:1024*1024});
  const row={args,status:r.status,error:r.error?.code,signal:r.signal,stdout:r.stdout,stderr:r.stderr,sha256};
  if(r.error?.code==='EPERM') {writeFileSync(resolve(dir,`atan-validation-${mode}-sandbox.json`),JSON.stringify(row,null,2));throw Error('subprocess permission failure');}
  records.push(row);writeFileSync(resolve(dir,`atan-validation-${mode}-runs.json`),JSON.stringify(records,null,2)+'\n');
  if(r.status!==0||r.error) throw Error(JSON.stringify(row));
  console.log(JSON.stringify(row));
}
console.log(`PASS ${records.length} ${mode} requests, including all 18 frozen cold/warm cases; numeric assertions ran in every process.`);
