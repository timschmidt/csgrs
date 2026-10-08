import {spawnSync} from 'node:child_process';
import {writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
const dir=import.meta.dirname,root=resolve(dir,'../..');
const bin=resolve(root,'.audit-plume-late.4Yth3g/late-benchmark');
const records=[],rows=[];
for(let round=0;round<9;round++) for(const input of ['0.1','0.5467']) for(const iterations of [4,8,10]) {
  const order=round%2?['dy-float','sb-float']:['sb-float','dy-float'];
  for(const variant of order) {
    const args=['-c','6',bin,variant,input,String(iterations),'32','+RTS','-T','-M512m','-RTS'];
    const r=spawnSync('taskset',args,{cwd:root,encoding:'utf8',timeout:10000,killSignal:'SIGKILL',maxBuffer:1024*1024});
    const record={round,variant,input,iterations,args,status:r.status,signal:r.signal,error:r.error?.code,stdout:r.stdout,stderr:r.stderr};
    if(r.error?.code==='EPERM') {
      writeFileSync(resolve(dir,'late-bench-sandbox-run.json'),JSON.stringify(record,null,2)+'\n');
      throw Error('child permission error; retry with approval');
    }
    records.push(record);
    writeFileSync(resolve(dir,'late-bench-runs.json'),JSON.stringify(records,null,2)+'\n');
    if(r.status!==0||r.error) throw Error(`Unqualified benchmark: ${JSON.stringify(record)}`);
    const fields=r.stdout.trim().split(/\s+/);
    if(fields.length!==8||fields[7]!=='PASS') throw Error('unqualified output');
    rows.push([round,...fields].join('\t'));
    writeFileSync(resolve(dir,'late-bench-results.tsv'),'round\tvariant\tinput\titerations\tbits\tcpu_ps\tallocated_bytes\tchecksum\tqualification\n'+rows.join('\n')+'\n');
  }
  console.log(`${round}\t${input}\t${iterations}`);
}
console.log(`PASS observations=${records.length}; discard round0 warmups before analysis`);
