import { spawnSync } from 'node:child_process';
import { writeFileSync } from 'node:fs';
import { resolve } from 'node:path';
const root=resolve(import.meta.dirname,'../..');
const records=[];
const cases=[['1+2*3',7,1],['(1+2)*3',9,1],['1/3',1,3],['-7/3',-7,3],
  ['max(-1,0.5)',1,2],['min(-1,0.5)',-1,1],['abs(1)',null,null],
  ['2^3',8,1],['sqrt(4)',2,1],['sin(0)',0,1],['cos(0)',1,1],
  ['x:=2\nx+x',4,1],['x:=2\ny:=x+1\nx:=4\ny',5,1],
  ['missing',null,null],['1+@',null,null],['',null,null]];
for(const [expression,n,d] of cases) {
  const input=`Digits:=8\n${expression}\nexit\n`;
  const r=spawnSync(resolve(root,'.audit-plume.SZHOs8/plume-calc'),[],{cwd:root,input,encoding:'utf8',timeout:2000,killSignal:'SIGKILL',maxBuffer:1024*1024});
  const record={expression,n,d,status:r.status,signal:r.signal,error:r.error?.code,stdout:r.stdout,stderr:r.stderr};
  if(n!==null && r.status===0) {
    const matches=[...r.stdout.matchAll(/^=\s+(-?\d+(?:\.\d*)?)/gm)];
    if(matches.length) {
      const s=matches.at(-1)[1],places=s.includes('.')?s.split('.')[1].length:0;
      const mant=BigInt(s.replace('.','')),scale=10n**BigInt(places);
      const diff=mant*BigInt(d)-BigInt(n)*scale;
      // One requested last-place unit, with exact integer comparisons.
      record.exact_decimal_check=(diff<0n?-diff:diff)*100000000n<=scale*BigInt(d);
    } else record.exact_decimal_check=false;
  }
  records.push(record);
  writeFileSync(resolve(import.meta.dirname,'ui-runs.json'),JSON.stringify(records,null,2)+'\n');
  console.log(JSON.stringify(record));
}
