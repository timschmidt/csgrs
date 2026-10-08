import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
const root=import.meta.dirname,build=resolve(root,'../../.audit-constructible-build.l4UDoe');
const assert=(ok,message)=>{if(!ok)throw Error(message);};
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
for(const [path,want] of Object.entries({
  'ApiProbe.hs':'986dd265e1ef2363b4c3654becdfd110e3737e39e8bc12dc693ccf99e8a385fc',
  'FloatProbe.hs':'2432a1263ead3b7065643a86ac1a336e713b9131629fc39c70dfea2a4519783b',
  'float_oracle.rs':'4231672af009549533e07234082f11600dd22a501ea626c614a91c987f39badb',
}))assert(hash(root+'/'+path)===want,'source changed '+path);
for(const [path,want] of Object.entries({
  'api-before-O0':'1a4f80a3ebabc3658449b3562296db1969c4a869e1ab2cbed878a60dfa9fb314',
  'api-before-O2':'a897bbe22ea0344c98a2408e43c96430ebda5ddf6c8a41d29c36a44cf0883338',
  'float-before-O0':'8f4b8c6644e578e70a649c98313d55646d66693cfd9aff70cbe533357649d5e7',
  'float-before-O2':'dc810f9b6d29a659c44d242a7056017850052f4b2cb576520e5ce3e4f5aaf550',
  'float-oracle-debug':'7cc704c5624f4ae6060c33149bbd8101f686587a520cb475e6d5d5ac044aaf23',
  'float-oracle-release':'d0996a5eb954196ea378b99abcf0535c6dc37055def559e7fbfce76197c48504',
}))assert(hash(build+'/'+path)===want,'binary changed '+path);
const corpus=readFileSync(root+'/field-corpus.tsv','utf8').trimEnd().split('\n').slice(1).map(s=>s.split('\t'));
const expected=[];
for(const r of corpus)for(const side of ['lhs','rhs'])for(const test of ['show','negative-show','deconstruct'])expected.push(`${r[0]}-${r[1]}-${side}-${test}`);
for(let i=0;i<23;i++)for(const test of ['succ','pred','from','then','to','descending','zero-step'])expected.push(i+'-'+test);
expected.push(...['precedence-add','precedence-sub','precedence-div','precedence-neg','precedence-root','trailing-garbage','list-read','unsupported-logBase']);
assert(expected.length===8113,'API expectation count');
for(const opt of ['O0','O2']){
  const lines=readFileSync(root+'/api-'+opt+'.log','utf8').trimEnd().split('\n');assert(lines.pop()==='SUMMARY\t8113\t8113','API summary');
  assert(lines.length===8113,'API count');
  lines.forEach((line,i)=>{const r=line.split('\t');assert(r[0]==='PASS'&&r[1]===expected[i]&&(r[2]==='True'||r[1]==='unsupported-logBase'),'API result');});
  const path=root+'/float-'+opt+'.log';assert(hash(path)==='2533c29c98e9faa916f3b9353d6f7c943d583d9666372afdda3efc363e3467f8','float output changed');
  const records=readFileSync(path,'utf8').trimEnd().split('\n');assert(records.pop()==='SUMMARY 194','float summary');assert(records.length===194,'float count');
  const wanted=[];for(const sign of [-1,1])wanted.push(['close',0,sign,0n,0n]);
  let p=1n,q=1n;for(let i=1;i<=96;i++){assert((p*p-2n*q*q)**2n===1n,'Pell norm');for(const sign of [-1,1])wanted.push(['pell',i,sign,p,q]);[p,q]=[p+2n*q,p+q];}
  records.forEach((line,i)=>{const r=line.split(' ');assert(r.length===8&&r.slice(0,5).join(' ')===wanted[i].join(' '),'float case identity');assert(BigInt(r[6])>0n&&Number.isFinite(Number(r[7])),'finite float');});
  for(const mode of ['debug','release']){
    const lines=readFileSync(root+`/float-oracle-${mode}-${opt}.log`,'utf8').trimEnd().split('\n');assert(lines.pop()==='SUMMARY\t194\t582','oracle summary');assert(lines.length===194,'oracle count');
    lines.forEach((line,i)=>assert(line===['PASS',...wanted[i].slice(0,3)].join('\t'),'oracle case'));
  }
}
console.log(JSON.stringify({apiPerBuild:8113,nativeBuilds:['O0','O2'],floatCasesPerNativeBuild:194,mpfrChecksPerCombination:582,combinations:4,referenceBits:4096,hyperApproximationBits:512,relativeFloatTolerance:'2^-48',newProductionChanges:0,targetStatus:'OPEN'},null,2));
