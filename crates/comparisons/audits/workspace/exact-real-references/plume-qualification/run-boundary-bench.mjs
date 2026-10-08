import {spawnSync} from 'node:child_process';
import {writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
const dir=import.meta.dirname,root=resolve(dir,'../..'),rows=[];
for(let round=0;round<9;round++) for(const family of ['rational','tiny-rational','tiny-radical','ordinary-radical']) {
  for(const variant of round%2?['after','before']:['before','after']) {
    const bin=resolve(root,'.audit-plume-boundary.XNOLY9/format-'+variant);
    const args=['-c','6',bin,family];
    const r=spawnSync('taskset',args,{cwd:root,encoding:'utf8',timeout:15000,killSignal:'SIGKILL'});
    const row={round,family,variant,args,status:r.status,signal:r.signal,error:r.error?.code,stdout:r.stdout,stderr:r.stderr};
    if(r.error?.code==='EPERM') {
      writeFileSync(resolve(dir,'boundary-bench-sandbox-run.json'),JSON.stringify(row,null,2)+'\n');
      throw Error('child permission failure; retry with approval');
    }
    rows.push(row);
    writeFileSync(resolve(dir,'boundary-bench-runs.json'),JSON.stringify(rows,null,2)+'\n');
    if(r.status!==0||r.error) throw Error(JSON.stringify(row));
    const parts=r.stdout.trim().split('\t');
    if(parts.length!==7||parts[0]!==family||parts[1]!=='512'||parts.slice(2).some(s=>!/^\d+$/.test(s))) throw Error('invalid record');
    console.log([round,variant,...parts].join('\t'));
  }
}
console.log(`PASS ${rows.length} independently qualified CPU6 fresh-process observations; round0 excluded by analysis.`);
