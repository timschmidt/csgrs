import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
const read=p=>readFileSync(p,'utf8').trimEnd().split('\n').map(JSON.parse);
export function costCases(){
 const cases=[],add=(label,source,row)=>cases.push({id:cases.length,label,source,left:row.left,right:row.right,operation:row.report.operation});
 const old=read('results/power-rebased-baseline-public-v68.stdout').filter(r=>r.type==='cost-case');assert.equal(old.length,40);
 for(const row of old)add('old-'+row.case,{corpus:'public',case:row.case},row);
 const degree=read('results/zero-factor-degree-baseline-v70.stdout').filter(r=>r.policy===0);assert.equal(degree.length,66);
 for(const row of degree)add('degree-'+row.id,{corpus:'degree',id:row.id},row);
 const authored=JSON.parse(readFileSync('power-wide-input-v69.json','utf8')),
  wide=read('results/power-wide-baseline-run-v69.stdout').filter(r=>r.policy===0);
 const selected=authored.filter(c=>[65,257].includes(c.height)&&c.m===3&&c.n===3&&['distinct','unused-zero'].includes(c.family)&&[0,3].includes(c.op));
 assert.equal(selected.length,8);
 for(const c of selected)add('wide-'+c.id,{corpus:'wide',id:c.id,height:c.height,family:c.family},wide.find(r=>r.id===c.id));
 assert.equal(cases.length,114);return cases;
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const cases=costCases();writeFileSync('zero-factor-cost-input-v71.json',JSON.stringify(cases)+'\n',{flag:'wx'});
 console.log(JSON.stringify({cases:cases.length,groups:cases.length*4}));
}
