import {readFileSync,writeFileSync,existsSync,mkdirSync,copyFileSync} from 'node:fs';
import {dirname} from 'node:path';
import {createHash} from 'node:crypto';
const root=import.meta.dirname,dest=root+'/hypersolve-diagonal';
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
if(existsSync(dest))throw Error('refusing to overwrite candidate checkout');
const snapshot=JSON.parse(readFileSync(root+'/snapshot.json','utf8')),copied=[];
for(const[p,h]of snapshot.files){
 if(hash(root+'/snapshot/'+p)!==h)throw Error('frozen source drift '+p);
 if(!p.startsWith('hypersolve/'))continue;
 const path=p.slice('hypersolve/'.length),out=dest+'/'+path;mkdirSync(dirname(out),{recursive:true});copyFileSync(root+'/snapshot/'+p,out);if(hash(out)!==h)throw Error('copy mismatch');copied.push([path,h]);
}
writeFileSync(root+'/public-copy-origin.json',JSON.stringify({created:new Date().toISOString(),files:copied},null,2)+'\n');console.log(JSON.stringify({copied:copied.length}));
