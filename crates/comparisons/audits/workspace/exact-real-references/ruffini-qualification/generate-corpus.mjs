import {writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
const rows=['id\top\tbits\tan\tad\tbn\tbd\tdecimal'];
let id=0;
for(const bits of [0,1,8,16,17,32,64,128,256]) {
 for(const [an,ad] of [[0,1],[1,3],[-1,3],[17,7],[-17,7],[1000000,1],[-1000000,1],[65537,3],[-65537,3]]) {
  rows.push([id++,'neg',bits,an,ad,0,1,'-'].join('\t'));
  if(an>0) rows.push([id++,'inv',bits,an,ad,0,1,'-'].join('\t'));
  for(const [bn,bd] of [[1,10],[-1,10],[1,7],[-1,7],[1000000,3],[-1000000,3]])
   for(const op of ['add','mul']) rows.push([id++,op,bits,an,ad,bn,bd,'-'].join('\t'));
 }
 for(const a of [-1000000,-65537,-17,-1,0,1,17,65537,1000000])
  for(const d of ['0.1','-0.1','0.3','-0.3','0.5','-0.5']) {
   const n=BigInt(d.replace('.',''));
   rows.push([id++,'public-mul',bits,a,1,n,10,d].join('\t'));
  }
}
const text=rows.join('\n')+'\n';
writeFileSync(import.meta.dirname+'/rational-corpus.tsv',text);
console.log(JSON.stringify({cases:id,sha256:createHash('sha256').update(text).digest('hex')}));
