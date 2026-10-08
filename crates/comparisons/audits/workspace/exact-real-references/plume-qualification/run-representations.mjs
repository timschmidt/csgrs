import {spawnSync} from 'node:child_process';
import {writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
const root=resolve(import.meta.dirname,'../..');
const records=[],rows=[];
for(const rep of ['endpoint','terminating','redundant','decimal']) for(const exponent of [1,3]) for(const bits of [8,24,64]) {
  const r=spawnSync(resolve(root,'.audit-plume.SZHOs8/kernel-probe'),['representation',rep,String(exponent),String(bits),'+RTS','-M256m','-RTS'],{cwd:root,encoding:'utf8',timeout:3000,killSignal:'SIGKILL'});
  records.push({rep,exponent,bits,status:r.status,signal:r.signal,error:r.error?.code,stdout:r.stdout,stderr:r.stderr});
  if(r.status!==0) throw Error(JSON.stringify(records.at(-1)));
  rows.push(r.stdout.trim().split(/\s+/).join('\t'));
}
writeFileSync(resolve(import.meta.dirname,'representation-runs.json'),JSON.stringify(records,null,2)+'\n');
writeFileSync(resolve(import.meta.dirname,'representation-results.tsv'),rows.join('\n')+'\n');
console.log(`returned ${rows.length} representation-specific logarithm prefixes`);
