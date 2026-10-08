import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
const root=import.meta.dirname;
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const expected={
  'field-corpus.tsv':'9ff02bde757b516d384b3460c20ee3fd2daab2b9d6582e9bf88705c04c45ebff',
  'FieldProbe.hs':'04db066a3336b7a7a1d296c6d57a627ebd3f626581824c33593077f98486ee75',
  'hyper_field_probe.rs':'ac4d8afd755066495e4e77ec7165e1bfad9b92e24b80dab02f703d5d9bb73fea',
  'FieldBench.hs':'7356e68ad8b58ade4feaf6db5f8db356c9b2e2895b7a1499c2abf18919c66a79',
  'hyper_field_bench.rs':'b6e54507949e28c2ccfe020fcec848f9868cc29a915b4d6ae882b1647eaff5ca',
  'ContractProbe.hs':'16c830bbd909bdc893fd43d372c1f3f097c71dc9e7610c9eefdc6c6162cfd2f0',
};
const assert=(ok,message)=>{if(!ok)throw Error(message);};
for(const [path,want] of Object.entries(expected))assert(hash(root+'/'+path)===want,'source changed: '+path);
const corpus=readFileSync(root+'/field-corpus.tsv','utf8').trimEnd().split('\n').slice(1).map(s=>s.split('\t'));
assert(corpus.length===1324,'corpus count');
const corpusMap=new Map(corpus.map(r=>[r[0]+'\t'+r[1],r]));assert(corpusMap.size===1324,'duplicate case');
for(const [file,expectedPass,expectedUnknown] of [
  ['field-O0-all.log',1324,0],['field-O2-all.log',1324,0],
  ['hyper-field-debug-all-2048.log',1310,14],['hyper-field-release-all-2048.log',1310,14],
  ['hyper-field-release-tower5-4096.log',24,4],['hyper-field-release-tower5-16384.log',28,0],
  ['hyper-field-debug-tower5-16384.log',28,0],
]){
  const rows=readFileSync(root+'/'+file,'utf8').trimEnd().split('\n').map(s=>s.split('\t'));
  const summary=rows.pop(),seen=new Set();let passed=0,unknown=0;
  for(const r of rows){const key=r[1]+'\t'+r[2],want=corpusMap.get(key);assert(want&&!seen.has(key),'bad probe case');seen.add(key);
    if(r[0]==='PASS'){assert(r[3]===want[2],'wrong proof');passed++;}
    else{assert(r[0]==='UNKNOWN'&&r[3]==='UNKNOWN'&&r[1]==='tower-5'&&r[2].startsWith('inverse-'),'unexpected failure');unknown++;}
  }
  assert(passed===expectedPass&&unknown===expectedUnknown,'probe counts '+file);
  assert(summary[0]==='SUMMARY'&&+summary[1]===passed&&+summary.at(-1)===rows.length,'summary '+file);
}
for(const opt of ['O0','O2']){
  const lines=readFileSync(root+'/contracts-corrected-'+opt+'.log','utf8').trimEnd().split('\n');
  assert(lines.at(-1)==='SUMMARY\t126\t126'&&lines.filter(s=>s.startsWith('PASS\t')).length===126,'contracts '+opt);
  const golden=readFileSync(root+'/golden-corrected-'+opt+'.log','utf8');assert(golden.includes('PASS\tgolden\t')&&golden.endsWith('SUMMARY\t1\t1\n'),'golden '+opt);
  for(const mode of ['agm','unity17','conversion']){
    const text=readFileSync(root+'/'+mode+(opt==='O0'?'-corrected':'')+'-'+opt+'.log','utf8');
    assert(text.includes('PASS\t'+mode+'\t')&&text.endsWith('SUMMARY\t1\t1\n'),'doc '+mode+' '+opt);
  }
}
const observations=readFileSync(root+'/field-bench-approved-observations.jsonl','utf8').trimEnd().split('\n').map(JSON.parse);
assert(observations.length===126,'benchmark observations');
const families=[['quadratic',16,1168],['tower-1',400,28],['tower-2',300,28],['tower-3',200,28],['tower-4',100,28],['tower-5',64,28],['independent',800,16]];
const sorted=xs=>xs.slice().sort((a,b)=>a-b),median=xs=>{const x=sorted(xs),n=x.length;return n%2?x[n>>1]:(x[n/2-1]+x[n/2])/2;};
let state=0x183957a3;const random=()=>{state^=state<<13;state^=state>>>17;state^=state<<5;return(state>>>0)/2**32;};
function interval(ratios){const samples=[];for(let i=0;i<10000;i++)samples.push(median(ratios.map(()=>ratios[Math.floor(random()*ratios.length)])));samples.sort((a,b)=>a-b);return[samples[249],samples[9749]];}
const result=[];
for(const [group,rounds,rows] of families){
  const ratios={cpu:[],wall:[]},samples={native:[],hyper:[]};
  for(let sample=0;sample<=8;sample++){
    const pair={};
    for(const mode of ['native','hyper']){
      const matches=observations.filter(x=>x.group===group&&x.sample===sample&&x.mode===mode);assert(matches.length===1,'missing/duplicate observation');
      const x=matches[0];assert(x.status===0&&x.rounds===rounds&&x.checks===rows*rounds&&x.cpu_ns>0&&x.wall_ns>0&&x.rss_kib>0,'observation contract');
      const prefix=`field-bench-approved-${sample}-${group}-${mode}`;
      assert(readFileSync(root+'/'+prefix+'.stdout','utf8')===`BENCH\t${rounds}\t${rows*rounds}\t${x.cpu_ns}\t${x.wall_ns}\n`,'stdout changed');
      const stderr=readFileSync(root+'/'+prefix+'.stderr','utf8');assert(stderr.includes('AUDIT_RSS_KIB '+x.rss_kib),'RSS changed');
      pair[mode]=x;if(sample>0)samples[mode].push(x);
    }
    if(sample>0){ratios.cpu.push(pair.hyper.cpu_ns/pair.native.cpu_ns);ratios.wall.push(pair.hyper.wall_ns/pair.native.wall_ns);}
  }
  result.push({group,rounds,checks_per_observation:rows*rounds,pairs:8,hyper_over_native_cpu_median:median(ratios.cpu),cpu_bootstrap_95:interval(ratios.cpu),hyper_over_native_wall_median:median(ratios.wall),wall_bootstrap_95:interval(ratios.wall),native_cpu_ms:median(samples.native.map(x=>x.cpu_ns/1e6)),hyper_cpu_ms:median(samples.hyper.map(x=>x.cpu_ns/1e6)),native_rss_kib:median(samples.native.map(x=>x.rss_kib)),hyper_rss_kib:median(samples.hyper.map(x=>x.rss_kib)),native_heap_bytes:median(samples.native.map(x=>x.native_heap_bytes))});
}
const report={observations:126,postwarmup:112,checks:observations.reduce((n,x)=>n+x.checks,0),result};
writeFileSync(root+'/field-bench-analysis.json',JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report,null,2));
