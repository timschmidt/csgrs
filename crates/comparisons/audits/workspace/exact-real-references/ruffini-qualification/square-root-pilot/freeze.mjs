import {readFileSync,writeFileSync,existsSync,mkdirSync,copyFileSync} from 'node:fs';
import {spawnSync} from 'node:child_process';
import {resolve,dirname} from 'node:path';
import {createHash} from 'node:crypto';
const root=import.meta.dirname,ws=resolve(root,'../../..');
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
if(existsSync(root+'/snapshot'))throw Error('refusing to replace frozen evidence');
const files=[],heads={};
for(const crate of ['hyperreal','hyperlattice','hyperlimit','hypersolve']){
 const git=args=>{const r=spawnSync('git',['-C',ws+'/'+crate,...args],{encoding:'utf8'});if(r.error||r.status!==0)throw Error('git failed');return r.stdout;};
 heads[crate]={head:git(['rev-parse','HEAD']).trim(),status:git(['status','--short'])};
 const paths=[...new Set(git(['ls-files','--cached','--others','--exclude-standard','-z']).split('\0').filter(p=>p.endsWith('.rs')||['Cargo.toml','README.md','PERFORMANCE.md'].includes(p)))].sort();
 for(const path of paths){
  if(path.startsWith('/')||path.split('/').includes('..'))throw Error('unsafe source path');
  const name=crate+'/'+path,source=ws+'/'+name,dest=root+'/snapshot/'+name,h=hash(source);
  mkdirSync(dirname(dest),{recursive:true});copyFileSync(source,dest);
  if(hash(source)!==h||hash(dest)!==h)throw Error('source changed during copy');files.push([name,h]);
 }
}
for(const[p,h]of files)if(hash(ws+'/'+p)!==h)throw Error('source changed during freeze '+p);
const source=readFileSync(root+'/snapshot/hypersolve/src/curve_resultant.rs','utf8');
const extract=name=>{const start=source.indexOf('fn '+name+'('),end=source.indexOf('\n}\n',start);if(start<0||end<start)throw Error(name);return source.slice(start,end+3);};
writeFileSync(root+'/baseline.rs',extract('exact_polynomial_square_root'));
writeFileSync(root+'/shared.rs',['add_exact_polynomials','multiply_exact_polynomials','exact_polynomial_is_zero','strict_reciprocal','exact_real_is_zero','scale_exact_polynomial','subtract_exact_polynomials'].map(extract).join('\n'));
const result={created:new Date().toISOString(),heads,files,baselineSha256:hash(root+'/baseline.rs'),sharedSha256:hash(root+'/shared.rs')};
writeFileSync(root+'/snapshot.json',JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify({...result,files:files.length},null,2));
