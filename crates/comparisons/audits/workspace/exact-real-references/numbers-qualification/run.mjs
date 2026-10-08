import {spawnSync} from 'node:child_process';
import {writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
const dir=import.meta.dirname,root=resolve(dir,'../..');
const mode=process.argv[2],opt=process.argv[3]??'O2';
if(!['O0','O2'].includes(opt)) throw Error('optimization');
const requests=[];
if(mode==='basic') for(const m of ['grid','decisions','derivatives','boundary']) requests.push({args:[m],timeout:60000});
else if(['elementary','fixed'].includes(mode)) {
  const sets={
    sqrt:[[0,1],[1,16],[2,1],[9,4],[1000,1]],
    exp:[[-16,1],[-2,1],[-1,4],[0,1],[1,2],[2,1],[16,1]],
    log:[[1,1024],[1,2],[7,8],[1,1],[9,8],[2,1],[1000,1]],
    sin:[[-1000,1],[-2,1],[-1,2],[0,1],[1,2],[2,1],[1000,1]],
    cos:[[-1000,1],[-2,1],[-1,2],[0,1],[1,2],[2,1],[1000,1]],
    atan:[[-10,1],[-11,8],[-21,16],[-5,4],[-19,16],[-9,8],[-1,1],[-1,2],[0,1],[1,2],[1,1],[5,4],[10,1]],
    asin:[[-1,1],[-7,8],[-1,2],[0,1],[1,2],[7,8],[1,1]],
    acos:[[-1,1],[-7,8],[-1,2],[0,1],[1,2],[7,8],[1,1]],
    asinh:[[-10,1],[-1,1],[0,1],[1,1],[10,1]],
    acosh:[[1,1],[9,8],[2,1],[100,1]],
    atanh:[[-7,8],[-1,2],[0,1],[1,2],[7,8]],
  };
  for(const [f,inputs] of Object.entries(sets)) for(const [n,d] of inputs) for(const p of [8,32,80,160])
    requests.push({args:[mode,f,String(n),String(d),String(p)],timeout:1500});
} else throw Error('mode');
const records=[];
for(const request of requests) {
  const args=[...request.args,'+RTS','-M256m','-RTS'];
  const r=spawnSync(resolve(root,`.audit-numbers.rjcbha/probe-${opt}`),args,
    {cwd:root,encoding:'utf8',timeout:request.timeout,killSignal:'SIGKILL',maxBuffer:8*1024*1024});
  const row={...request,status:r.status,signal:r.signal,error:r.error?.code,stdout:r.stdout,stderr:r.stderr};
  if(r.error?.code==='EPERM') {
    writeFileSync(resolve(dir,`${mode}-${opt}-sandbox.json`),JSON.stringify(row,null,2)+'\n');
    throw Error('child permission failure; retry with approval');
  }
  records.push(row);
  writeFileSync(resolve(dir,`${mode}-${opt}-runs.json`),JSON.stringify(records,null,2)+'\n');
  writeFileSync(resolve(dir,`${mode}-${opt}.log`),records.filter(x=>x.status===0&&!x.error).map(x=>x.stdout).join(''));
  console.log(JSON.stringify({index:records.length,args:request.args,status:r.status,error:r.error?.code,bytes:r.stdout?.length}));
}
console.log(`Complete ${mode}/${opt}: ${records.length} requests; finite ${records.filter(r=>r.status===0&&!r.error).length}. Numeric qualification is separate.`);
