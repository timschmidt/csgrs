import {spawnSync} from 'node:child_process';
import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
const dir=import.meta.dirname,root=resolve(dir,'../..'),mode=process.argv[2];
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const records=[];
function run(bin,args,label,source){
  const r=spawnSync('taskset',['-c','6',bin,...args],{cwd:root,encoding:'utf8',timeout:10000,killSignal:'SIGKILL',maxBuffer:1024*1024});
  const row={label,args,status:r.status,error:r.error?.code,signal:r.signal,stdout:r.stdout,stderr:r.stderr,sha256:hash(bin),source:hash(resolve(dir,source))};
  records.push(row);writeFileSync(resolve(dir,`log-repair-${mode}-runs.json`),JSON.stringify(records,null,2)+'\n');console.log(JSON.stringify(row));
  if(r.error||r.status!==0||/^FAIL /m.test(r.stdout))throw Error('failed request '+label);
}
if(['debug','release'].includes(mode)){
  run(resolve(root,`.audit-numbers.rjcbha/log-boundary-after-${mode}`),[],mode,'log_boundary.rs');
}else if(['neighbor-debug','neighbor-release'].includes(mode)){
  const profile=mode.slice('neighbor-'.length);
  const before=JSON.parse(readFileSync(resolve(dir,`neighbor-before-${profile}-runs.json`),'utf8'));
  for(const {args} of before)run(resolve(root,`.audit-numbers.rjcbha/log-neighbor-after-${profile}`),args,args.join('-'),'neighbor_probe.rs');
}else if(['pilot','paired'].includes(mode)){
  const families=['ln-rational','ln-radical','ln-near-one','asinh-small','asinh-negative','asinh-warm','asinh-moderate','asinh-large','acosh-near-one','acosh-moderate','acosh-large'];
  for(let round=0;round<(mode==='pilot'?1:9);round++)for(const family of families){
    for(const version of round%2?['after','before']:['before','after'])run(resolve(root,`.audit-numbers.rjcbha/log-controls-${version}`),[family,'256'],`${round}-${version}-${family}`,'log_controls.rs');
  }
}else throw Error('mode');
console.log(JSON.stringify({requests:records.length,passed:records.length}));
