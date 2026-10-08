import {readFileSync,writeFileSync,existsSync} from 'node:fs';
import {createHash} from 'node:crypto';
const root=import.meta.dirname, corpus=readFileSync(root+'/field-corpus.tsv');
if(createHash('sha256').update(corpus).digest('hex')!=='9ff02bde757b516d384b3460c20ee3fd2daab2b9d6582e9bf88705c04c45ebff')throw Error('corpus changed');
const [header,...rows]=corpus.toString().trimEnd().split('\n');
for(const group of new Set(rows.map(row=>row.split('\t')[0]))){
  const path=root+'/field-'+group+'.tsv',body=header+'\n'+rows.filter(row=>row.startsWith(group+'\t')).join('\n')+'\n';
  if(existsSync(path)){if(readFileSync(path,'utf8')!==body)throw Error('frozen partition changed');}
  else writeFileSync(path,body,{flag:'wx'});
  console.log(group,body.split('\n').length-2,createHash('sha256').update(body).digest('hex'));
}
