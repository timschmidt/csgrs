import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
const dir=import.meta.dirname;
const rows=readFileSync(resolve(dir,'../PLUME_FILE_INVENTORY.tsv'),'utf8').trim().split('\n').slice(1).map(x=>x.split('\t'));
const sources=new Map();
for (const [path,,bytes,lines,hash] of rows) {
  const data=readFileSync(resolve(dir,'../Plume',path));
  if(data.length!==+bytes||createHash('sha256').update(data).digest('hex')!==hash) throw Error(`source changed: ${path}`);
  sources.set(path,{data,lines:+lines,hash});
}
const result={performance:[],cgi:[]};
for(const [group,prefix,other] of [['performance','performance/perform/','versioned/perform/'],['cgi','cgi/','versioned/v1.2/']]) {
  for(const [path,source] of sources) if(path.startsWith(prefix)) {
    const candidate=other+path.slice(prefix.length), match=sources.get(candidate);
    result[group].push({path,lines:source.lines,sha256:source.hash,counterpart:match ? candidate : null,
      byte_identical:match ? source.data.equals(match.data) : false});
  }
}
if(result.performance.length!==36||result.performance.some(x=>!x.byte_identical)) throw Error('unexpected performance copy difference');
writeFileSync(resolve(dir,'source-copy-map.json'),JSON.stringify(result,null,2)+'\n');
for(const [group,entries] of Object.entries(result)) console.log(group,JSON.stringify({files:entries.length,lines:entries.reduce((n,x)=>n+x.lines,0),identical:entries.filter(x=>x.byte_identical).length,different_or_new:entries.filter(x=>!x.byte_identical).map(x=>x.path)}));
console.log('Byte identity links prior qualification evidence; it does not itself grant line-read coverage.');
