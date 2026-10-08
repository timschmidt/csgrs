import {readFileSync,writeFileSync,existsSync,mkdirSync,copyFileSync} from 'node:fs';
import {spawnSync} from 'node:child_process';
import {resolve,dirname} from 'node:path';
import {createHash} from 'node:crypto';
const root=import.meta.dirname,ws=resolve(root,'../../..');
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
if(existsSync(root+'/snapshot'))throw Error('refusing to replace frozen source');
const rows=[],heads={};
for(const crate of ['hyperreal','hyperlattice','hyperlimit','hypersolve']) {
 const listed=spawnSync('git',['-C',ws+'/'+crate,'ls-files','--cached','--others','--exclude-standard','-z'],{encoding:'utf8'});
 if(listed.error||listed.status!==0)throw Error('listing failed');
 const state=spawnSync('git',['-C',ws+'/'+crate,'status','--short'],{encoding:'utf8'});
 const head=spawnSync('git',['-C',ws+'/'+crate,'rev-parse','HEAD'],{encoding:'utf8'});
 if(state.error||state.status!==0||head.error||head.status!==0)throw Error('git state failed');
 heads[crate]={head:head.stdout.trim(),status:state.stdout};
 for(const p of [...new Set(listed.stdout.split('\0').filter(p=>p.endsWith('.rs')||p==='Cargo.toml'||p==='README.md'))].sort()) {
  if(p.startsWith('/')||p.split('/').includes('..'))throw Error('unsafe path');
  const name=crate+'/'+p,original=ws+'/'+name,destination=root+'/snapshot/'+name,before=hash(original);
  mkdirSync(dirname(destination),{recursive:true});copyFileSync(original,destination);
  if(hash(destination)!==before||hash(original)!==before)throw Error('source drift while copying '+name);
  rows.push([name,before]);
 }
}
for(const[p,h]of rows)if(hash(ws+'/'+p)!==h)throw Error('source drift during freeze '+p);
const original=readFileSync(root+'/snapshot/hypersolve/src/curve_resultant.rs','utf8');
const extract=name=>{const start=original.indexOf('fn '+name+'(');if(start<0)throw Error(name);const end=original.indexOf('\n}\n',start);if(end<0)throw Error(name);return original.slice(start,end+3);};
const generated=['multiply_exact_polynomials','exact_real_is_zero'].map(extract).join('\n');
writeFileSync(root+'/baseline.rs',generated);
writeFileSync(root+'/snapshot.json',JSON.stringify({created:new Date().toISOString(),heads,files:rows,baselineSha256:hash(root+'/baseline.rs')},null,2)+'\n');
console.log(JSON.stringify({files:rows.length,heads,baselineSha256:hash(root+'/baseline.rs')}));
