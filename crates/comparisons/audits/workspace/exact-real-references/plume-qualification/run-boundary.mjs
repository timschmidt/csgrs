import {spawnSync} from 'node:child_process';
import {writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
const dir=import.meta.dirname,root=resolve(dir,'../..');
const mode=process.argv[2],debug=process.argv.includes('--debug'),suffix=debug?'-debug':'';
const bin=resolve(root,'.audit-plume-boundary.XNOLY9/boundary-probe'+suffix);
const requests=[];
if(['grid','format','printed'].includes(mode)) requests.push({args:[mode],timeout:60000});
else if(mode==='productive') {
  for(const op of ['sb-max','sb-extra','dy-max']) for(const n of [0,1,2,8])
    requests.push({args:[mode,op,String(n)],timeout:750});
  requests.push({args:[mode,'sb-unbounded','0'],timeout:750});
  for(const op of ['no-neg','no-neg-extra']) for(const n of [-4,-1,0,1,500])
    requests.push({args:[mode,op,String(n)],timeout:750});
  for(const op of ['mul-zero','mul-nonzero']) for(const n of [498,499,500,501])
    requests.push({args:[mode,op,String(n)],timeout:750});
} else if(mode==='literal') {
  for(const literal of ['', '+','-','.','+.','-.','0.','1.','-1.','.0','+.5','-.5',
    '0','-0','+01.250','00.000','1e2','1..2','++1','--1',' 1','1 ','NaN','Infinity','١','0.5'])
    requests.push({args:[mode,literal],timeout:750});
} else throw Error('unknown mode');
const records=[];
for(const request of requests) {
  const args=[...request.args,'+RTS','-M256m','-RTS'];
  const r=spawnSync(bin,args,{cwd:root,encoding:'utf8',timeout:request.timeout,killSignal:'SIGKILL',maxBuffer:8*1024*1024});
  const row={...request,args,status:r.status,signal:r.signal,error:r.error?.code,stdout:r.stdout,stderr:r.stderr};
  if(r.error?.code==='EPERM') {
    writeFileSync(resolve(dir,`boundary-${mode}${suffix}-sandbox-run.json`),JSON.stringify(row,null,2)+'\n');
    throw Error('child permission failure; retry with approval');
  }
  records.push(row);
  writeFileSync(resolve(dir,`boundary-${mode}${suffix}-runs.json`),JSON.stringify(records,null,2)+'\n');
  writeFileSync(resolve(dir,`boundary-${mode}${suffix}.log`),records.map(x=>x.stdout).join(''));
  console.log(JSON.stringify({args:request.args,status:r.status,error:r.error?.code,signal:r.signal,bytes:r.stdout?.length}));
}
console.log(`Requests ${records.length}; status-zero ${records.filter(x=>x.status===0&&!x.error).length}; numeric qualification is separate.`);
