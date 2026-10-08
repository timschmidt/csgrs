import {spawnSync} from 'node:child_process';
import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
const dir=import.meta.dirname, root=resolve(dir,'../..');
const mode=process.argv[2]??'functional', debug=process.argv.includes('--debug');
const suffix=debug?'-debug':'';
const bin=resolve(root,'.audit-plume-late.4Yth3g/late-probe'+suffix);
const variants=['sb-float','cross-float','dy-float','sb-unnormalized','sb-stream','cross-stream','dy-stream'];
const inputs=[[0,1],[1,4],[1,2],[3,4],[1,1],[1,10],[5467,10000]];
const requests=[];
if(mode==='functional') {
  for(const name of ['quadratic-min','quadratic-max','square-integral','reciprocal-max']) for(const bits of [0,4,6,8])
    requests.push({name,bits,args:['functional',name,String(bits)],timeout:3000});
} else if(mode==='logistic') {
  for(const variant of variants) for(const [a,b] of inputs) {
    const reps=b===10?['greedy','decimal-0.1']:b===10000?['greedy','decimal-0.5467']:['greedy','delayed'];
    for(const rep of reps) for(const iterations of [0,1,2,3]) {
      const bits=12;
      requests.push({variant,rep,a,b,iterations,bits,args:['logistic',variant,rep,a,b,iterations,bits].map(String),timeout:750});
    }
  }
} else if(mode==='long') {
  for(const variant of variants) for(const [a,b,rep] of [[1,10,'decimal-0.1'],[5467,10000,'decimal-0.5467']])
    for(const iterations of [10,40,60]) {
      const bits=32;
      requests.push({variant,rep,a,b,iterations,bits,args:['logistic',variant,rep,a,b,iterations,bits].map(String),timeout:2000});
    }
} else if(mode==='first-digit') {
  for(const rep of ['greedy','delayed']) for(const iterations of [0,1,2,10])
    requests.push({rep,iterations,args:['first-digit',rep,String(iterations)],timeout:4000});
} else throw Error('unknown mode');
const records=[], rows=[];
for(const request of requests) {
  const args=[...request.args,'+RTS','-M256m','-RTS'];
  const r=spawnSync(bin,args,{cwd:root,encoding:'utf8',timeout:request.timeout,killSignal:'SIGKILL',maxBuffer:1024*1024});
  const row={...request,args,status:r.status,signal:r.signal,error:r.error?.code,stdout:r.stdout,stderr:r.stderr};
  if(r.error?.code==='EPERM') {
    writeFileSync(resolve(dir,`late-${mode}${suffix}-sandbox-run.json`),JSON.stringify(row,null,2)+'\n');
    throw Error('child permission error; retry with approval');
  }
  records.push(row);
  if(r.status===0&&!r.error) {
    const result=r.stdout.trim().split(/\s+/);
    if(result.length!==(mode==='first-digit'?2:6)||result.some(x=>!/^[-]?\d+$/.test(x))) throw Error('invalid output record');
    const metadata=mode==='functional'?[request.name,request.bits]:mode==='first-digit'?[request.rep,request.iterations]:
      [request.variant,request.rep,request.a,request.b,request.iterations,request.bits];
    rows.push([...metadata,...result].join('\t'));
  }
  writeFileSync(resolve(dir,`late-${mode}${suffix}-runs.json`),JSON.stringify(records,null,2)+'\n');
  writeFileSync(resolve(dir,`late-${mode}${suffix}-results.tsv`),rows.join('\n')+(rows.length?'\n':''));
  console.log(`${records.length}/${requests.length}\t${request.args.join(' ')}\t${r.status===0&&!r.error?'prefix':r.error?.code??r.signal??r.status}`);
}
console.log(`Requests ${records.length}; finite results ${rows.length}; numerical qualification is separate.`);
