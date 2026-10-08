import {readFileSync,writeFileSync,existsSync} from 'node:fs';
import {createHash} from 'node:crypto';
const root=import.meta.dirname, bytes=readFileSync(root+'/../field-corpus.tsv');
if(createHash('sha256').update(bytes).digest('hex')!=='9ff02bde757b516d384b3460c20ee3fd2daab2b9d6582e9bf88705c04c45ebff')throw Error('corpus changed');
const [header,...rows]=bytes.toString().trimEnd().split('\n');
for(const bits of [20,60]){
  const group='transverse-'+bits;
  const controls=rows.map(x=>x.split('\t')).filter(r=>r[2]==='EQ').map(([old,label,_,lhs,rhs])=>[group,old+'-'+label,'GT',`+ ${lhs} q 1 ${1n<<BigInt(bits)}`,rhs].join('\t'));
  const path=root+'/'+group+'.tsv',body=header+'\n'+controls.join('\n')+'\n';
  if(existsSync(path)){if(readFileSync(path,'utf8')!==body)throw Error('control changed');}else writeFileSync(path,body,{flag:'wx'});
  console.log(group,controls.length,createHash('sha256').update(body).digest('hex'));
}
