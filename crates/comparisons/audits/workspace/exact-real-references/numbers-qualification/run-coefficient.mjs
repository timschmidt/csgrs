import {spawnSync} from 'node:child_process';
import {readFileSync,writeFileSync,existsSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
const dir=import.meta.dirname,root=resolve(dir,'../..'),mode=process.argv[2];
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const output=resolve(dir,`coefficient-${mode}-runs.json`);
if(existsSync(output))throw Error('refusing to overwrite '+output);
const records=[];
function run(bin,args,label,source){
  const r=spawnSync('taskset',['-c','6',bin,...args],{cwd:root,encoding:'utf8',timeout:60000,killSignal:'SIGKILL',maxBuffer:1024*1024});
  const row={label,args,status:r.status,error:r.error?.code,signal:r.signal,stdout:r.stdout,stderr:r.stderr,sha256:hash(bin),source:hash(resolve(dir,source))};
  records.push(row);writeFileSync(output,JSON.stringify(records,null,2)+'\n');console.log(JSON.stringify(row));
  if(r.error||r.status!==0||/^FAIL /m.test(r.stdout))throw Error('failed request '+label);
}
if(['pilot-before','pilot-after','paired'].includes(mode)){
  const cases=[['asin','tiny'],['asin','radical'],['asin','opaque'],['asin','large'],['asinh','tiny'],['asinh','radical'],['asinh','opaque'],['asinh','large'],['atanh','tiny'],['asin','high'],['asinh','high']];
  for(let round=0;round<(mode==='paired'?9:1);round++)for(const args of cases){
    const versions=mode==='paired'?(round%2?['after','before']:['before','after']):[mode.slice(6)];
    for(const version of versions)run(resolve(root,`.audit-numbers.rjcbha/coefficient-controls-${version}`),args,`${round}-${version}-${args.join('-')}`,'coefficient_controls.rs');
  }
}else if(['debug','release'].includes(mode)){
  for(const bits of [184000,192000,256000,524288])for(const op of ['asin','asinh']){
    run(resolve(root,`.audit-numbers.rjcbha/coefficient-series-after-${mode}`),[op,String(bits)],`${mode}-${op}-${bits}`,'series_limit.rs');
  }
}else throw Error('mode');
console.log(JSON.stringify({requests:records.length,passed:records.length}));
