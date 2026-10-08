import { spawnSync } from 'node:child_process';
import { writeFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { createHash } from 'node:crypto';

const root = resolve(import.meta.dirname, '../..');
const bin = resolve(root, '.audit-plume-perform.LVoj1D/instrumented-probe');
if (process.argv[2]==='grids') {
  const records=[];
  for (const debug of [false,true]) for (const mode of ['grid','extra','demand','examples']) {
    const executable=bin+(debug ? '-debug' : '');
    const args=[...(mode==='grid' ? [] : [mode]),'+RTS','-M512m','-RTS'];
    const r=spawnSync(executable,args,{cwd:root,encoding:'utf8',timeout:90000,killSignal:'SIGKILL',maxBuffer:32*1024*1024});
    const output=`instrumented-${debug ? 'debug-' : ''}${mode}.${mode==='demand' ? 'tsv' : 'log'}`;
    const data=(r.stdout??'')+(r.stderr??'');
    writeFileSync(resolve(import.meta.dirname,output),data);
    records.push({debug,mode,executable,args,status:r.status,signal:r.signal,error:r.error?.code,
      output,bytes:Buffer.byteLength(data),sha256:createHash('sha256').update(data).digest('hex')});
    writeFileSync(resolve(import.meta.dirname,'instrumented-grid-runs.json'),JSON.stringify(records,null,2)+'\n');
    if(r.status!==0 || r.error) throw Error(`unqualified grid ${JSON.stringify(records.at(-1))}`);
    console.log(`${debug ? 'O0' : 'O2'}\t${mode}\tPASS process status; numerical classifications in ${output}`);
  }
  process.exit(0);
}
if (process.argv[2]==='bench') {
  const records=[],rows=[];
  for(let round=0;round<9;round++) for(const family of ['dense','sparse','terminating']) for(const bits of [32,64,128,256]) {
    const order=round%2 ? ['carry-average','average'] : ['average','carry-average'];
    for(const op of order) {
      const args=['-c','6',bin,'bench',op,family,String(bits),'256','+RTS','-T','-M512m','-RTS'];
      const r=spawnSync('taskset',args,{cwd:root,encoding:'utf8',timeout:15000,killSignal:'SIGKILL',maxBuffer:1024*1024});
      records.push({round,op,family,bits,args,status:r.status,signal:r.signal,error:r.error?.code,stdout:r.stdout,stderr:r.stderr});
      writeFileSync(resolve(import.meta.dirname,'instrumented-bench-runs.json'),JSON.stringify(records,null,2)+'\n');
      if(r.status!==0 || r.error) throw Error(`unqualified benchmark ${JSON.stringify(records.at(-1))}`);
      const f=r.stdout.trim().split(/\s+/);
      if(f.length!==8 || f[7]!=='PASS') throw Error('invalid benchmark output');
      rows.push([round,...f].join('\t'));
      writeFileSync(resolve(import.meta.dirname,'instrumented-bench-results.tsv'),'round\tvariant\tfamily\tbits\tbatch\tcpu_ps\tallocated_bytes\tchecksum\tqualification\n'+rows.join('\n')+'\n');
    }
    console.log(`${round}\t${family}\t${bits}`);
  }
  console.log(`PASS ${rows.length} CPU6-pinned, alternating-order, independently checked observations`);
  process.exit(0);
}
if (process.argv[2]==='boundaries') {
  const records=[];
  for (const name of ['float-int-div-zero','limit-upper-zero','dyadic-norm-cap']) {
    const args=['boundary',name,'+RTS','-M256m','-RTS'];
    const r=spawnSync(bin,args,{cwd:root,encoding:'utf8',timeout:3000,killSignal:'SIGKILL',maxBuffer:1024*1024});
    records.push({name,args,status:r.status,signal:r.signal,error:r.error?.code,stdout:r.stdout,stderr:r.stderr});
    writeFileSync(resolve(import.meta.dirname,'instrumented-boundary-runs.json'),JSON.stringify(records,null,2)+'\n');
    if(r.error?.code==='EPERM') throw Error('sandbox error; request approval');
    console.log(`${name}\t${r.status ?? r.error?.code ?? r.signal}`);
  }
  process.exit(0);
}
const values = {
  exp: [[-8,1],[-4,1],[-1,1],[-1,2],[0,1],[1,2],[1,1],[4,1],[8,1]],
  sin: [[-4,1],[-1,1],[-1,2],[0,1],[1,2],[1,1],[4,1]],
  cos: [[-4,1],[-1,1],[-1,2],[0,1],[1,2],[1,1],[4,1]],
  ln: [[1,16],[1,4],[1,2],[1,1],[3,2],[2,1],[8,1]],
  atan: [[-1,1],[-1,2],[0,1],[1,2],[1,1]],
  pi: [[0,1]],
};
const records = [], results = [];
for (const [op, qs] of Object.entries(values)) for (const [a,b] of qs) for (const bits of [8,24,64]) {
  const argv = ['emit',op,String(a),String(b),String(bits),'+RTS','-M256m','-RTS'];
  const t = process.hrtime.bigint();
  const run = spawnSync(bin, argv, { cwd: root, encoding: 'utf8', timeout: 3000, killSignal: 'SIGKILL', maxBuffer: 1024*1024 });
  const record = {op,a,b,bits,status:run.status,signal:run.signal,error:run.error?.code,
    elapsed_ns:String(process.hrtime.bigint()-t),stdout:run.stdout,stderr:run.stderr};
  records.push(record);
  if (run.status === 0 && !run.error) {
    const row = run.stdout.trim().split(/\s+/);
    if (row.length !== 8 || row[0] !== op || row[1] !== String(a) || row[2] !== String(b) || row[3] !== String(bits)) throw Error('invalid output');
    results.push(row.join('\t'));
  }
  writeFileSync(resolve(import.meta.dirname,'instrumented-elementary-runs.json'),JSON.stringify(records,null,2)+'\n');
  writeFileSync(resolve(import.meta.dirname,'instrumented-elementary-results.tsv'),results.join('\n')+'\n');
  console.log(`${records.length}\t${op}\t${a}/${b}\t${bits}\t${run.status ?? run.error?.code ?? run.signal}`);
  if (run.error?.code === 'EPERM') throw Error('sandbox child-process error; rerun with approval');
}
console.log(`completed ${records.length} requests; ${results.length} returned finite prefixes`);
