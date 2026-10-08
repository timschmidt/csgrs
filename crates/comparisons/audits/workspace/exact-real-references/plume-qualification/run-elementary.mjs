import { spawnSync } from 'node:child_process';
import { writeFileSync } from 'node:fs';
import { resolve } from 'node:path';

const root = resolve(import.meta.dirname, '../..');
const bin = resolve(root, '.audit-plume.SZHOs8/kernel-probe');
const values = {
  exp: [[-8,1],[-4,1],[-1,1],[-1,2],[0,1],[1,2],[1,1],[4,1],[8,1]],
  sin: [[-4,1],[-1,1],[-1,2],[0,1],[1,2],[1,1],[4,1]],
  cos: [[-4,1],[-1,1],[-1,2],[0,1],[1,2],[1,1],[4,1]],
  ln: [[1,16],[1,4],[1,2],[1,1],[3,2],[2,1],[8,1]],
  sqrt: [[0,1],[1,16],[1,4],[1,1],[2,1],[4,1],[8,1]],
  atan: [[-1,1],[-1,2],[0,1],[1,2],[1,1]],
  pi: [[0,1]],
};
const records = [];
const results = [];
for (const [op, qs] of Object.entries(values)) for (const [a,b] of qs) for (const bits of [8,24,64]) {
  const argv = ['emit',op,String(a),String(b),String(bits),'+RTS','-M256m','-RTS'];
  const t = process.hrtime.bigint();
  const run = spawnSync(bin, argv, { cwd: root, encoding: 'utf8', timeout: 3000, killSignal: 'SIGKILL', maxBuffer: 1024*1024 });
  const record = {op,a,b,bits,status:run.status,signal:run.signal,error:run.error?.code,
    elapsed_ns:String(process.hrtime.bigint()-t),stdout:run.stdout,stderr:run.stderr};
  records.push(record);
  if (run.status === 0) {
    const row = run.stdout.trim().split(/\s+/);
    if (row.length !== 8 || row[0] !== op || row[1] !== String(a) || row[2] !== String(b)) throw Error('invalid output');
    results.push(row.join('\t'));
  }
  writeFileSync(resolve(import.meta.dirname,'elementary-runs.json'),JSON.stringify(records,null,2)+'\n');
  writeFileSync(resolve(import.meta.dirname,'elementary-results.tsv'),results.join('\n')+'\n');
  console.log(`${records.length}\t${op}\t${a}/${b}\t${bits}\t${run.status ?? run.error?.code ?? run.signal}`);
}
console.log(`completed ${records.length} requests; ${results.length} returned finite prefixes`);
