import { spawnSync } from 'node:child_process';
import { writeFileSync } from 'node:fs';
import { resolve } from 'node:path';
const root=resolve(import.meta.dirname,'../..');
const bin=resolve(root,'.audit-plume.SZHOs8/kernel-probe');
const records=[],rows=[];
for(let round=0;round<9;round++) for(const family of ['dense','sparse','terminating']) for(const bits of [32,64,128,256]) {
  const order=round%2 ? ['dyadic','signed'] : ['signed','dyadic'];
  for(const op of order) {
    const args=['-c','6',bin,'bench',op,family,String(bits),'16','+RTS','-T','-M512m','-RTS'];
    const r=spawnSync('taskset',args,{cwd:root,encoding:'utf8',timeout:15000,killSignal:'SIGKILL',maxBuffer:1024*1024});
    records.push({round,op,family,bits,args,status:r.status,signal:r.signal,error:r.error?.code,stdout:r.stdout,stderr:r.stderr});
    if(r.status!==0 || r.error) throw Error(`unqualified benchmark ${JSON.stringify(records.at(-1))}`);
    const f=r.stdout.trim().split(/\s+/);
    if(f.length!==8 || f[7]!=='PASS') throw Error('invalid benchmark output');
    rows.push([round,...f].join('\t'));
    writeFileSync(resolve(import.meta.dirname,'bench-runs.json'),JSON.stringify(records,null,2)+'\n');
    writeFileSync(resolve(import.meta.dirname,'bench-results.tsv'),'round\tvariant\tfamily\tbits\tbatch\tcpu_ps\tallocated_bytes\tchecksum\tqualification\n'+rows.join('\n')+'\n');
  }
  console.log(`${round}\t${family}\t${bits}`);
}
console.log(`PASS ${rows.length} CPU6-pinned, alternating-order, independently checked observations`);
