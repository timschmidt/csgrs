import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
const root=import.meta.dirname,workspace=resolve(root,'../../..');
const assert=(ok,message)=>{if(!ok)throw Error(message);};
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
assert(hash(root+'/../hyper_field_memory.rs')==='795bcc3813a6934976b6b10fb1f7d59b66d2b4e7ecbc7de7119754962fb2d1eb','source changed');
for(const [mode,want] of Object.entries({before:'4908f913d6ad16cce2d612162aed5f2dec0e83a2c66b284f870887d12ce8242e',after:'3fb69b14ec0725070db7af10931f6b3c04af8ca80f3c85c358b8b1052a93cb69'}))assert(hash(workspace+'/.audit-constructible-build.l4UDoe/fractional-memory-'+mode)===want,'memory binary changed');
const result=[];
for(const [group,count] of [['quadratic',1168],['tower-1',28],['tower-2',28],['tower-3',28],['tower-4',28],['tower-5',28],['independent',16],['transverse-20',940],['transverse-60',940]]){
 const records={};
 for(const mode of ['before','after']){
  const samples=[];
  for(let sample=0;sample<3;sample++){
   const [bench,mem,...rest]=readFileSync(root+`/memory-${sample}-${group}-${mode}.log`,'utf8').trimEnd().split('\n');
   const b=bench.split('\t'),m=mem.split('\t');
   assert(rest.length===0&&b.length===5&&b[0]==='BENCH'&&+b[1]===1&&+b[2]===count&&+b[3]>0&&+b[4]>0,'memory check coverage');
   assert(m.length===4&&m[0]==='MEMORY'&&m.slice(1).every(x=>Number.isSafeInteger(+x)&&+x>0),'memory contract');samples.push(m.slice(1).map(Number));
  }
  assert(samples.every(x=>JSON.stringify(x)===JSON.stringify(samples[0])),'nondeterministic allocation observations');
  const[calls,allocatedBytes,peakLiveBytes]=samples[0];records[mode]={calls,allocatedBytes,peakLiveBytes};
 }
 result.push({group,count,...records,afterOverBeforeAllocated:records.after.allocatedBytes/records.before.allocatedBytes,afterOverBeforePeak:records.after.peakLiveBytes/records.before.peakLiveBytes});
}
const report={observations:54,verifiedComparisons:result.reduce((n,x)=>n+6*x.count,0),result,limitations:['Counts include arguments, cached-file ingestion, parser, construction, comparison and benchmark output allocation.','A one-byte before/after argument-path length difference is not a substantive memory saving.','Global counting-allocator clocks are not performance measurements.','Peak live bytes are tracked requested allocation sizes, not RSS or allocator-reserved physical memory.']};
writeFileSync(root+'/memory-analysis.json',JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report,null,2));
