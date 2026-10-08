import {readFileSync,writeFileSync,readdirSync,mkdirSync,existsSync} from 'node:fs';
import {spawnSync} from 'node:child_process';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
const root=import.meta.dirname,repo=resolve(root,'../Ruffini'),build=resolve(root,'../../.audit-ruffini-build.LmZgYM'),out=build+'/elliptic-coordinates';
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
if(existsSync(root+'/elliptic-coordinate-runs.json'))throw Error('refusing to overwrite evidence');
const sources=[root+'/EllipticCoordinates.java',...['elements/AffinePoint','elements/EdwardsPoint','elements/JacobianPoint','elements/ProjectivePoint','structures/ShortWeierstrassCurveAffine','structures/ShortWeierstrassCurveProjective','structures/MontgomeryCurve'].map(p=>repo+'/elliptic/src/main/java/dk/jonaslindstrom/ruffini/elliptic/'+p+'.java')];
const before=sources.map(p=>[p,hash(p)]);mkdirSync(out,{recursive:true});
const cp=out+':'+build+'/polynomial:'+build+'/matrix:'+build+'/classes:'+build+'/deps/*';
const compiled=spawnSync('javac',['--release','16','-encoding','UTF-8','-cp',cp,'-d',out,...sources],{encoding:'utf8',timeout:60000});
writeFileSync(root+'/elliptic-coordinate-build.log',(compiled.stdout??'')+(compiled.stderr??'')+'\n'+JSON.stringify({status:compiled.status,error:compiled.error?.code??null})+'\n');
if(compiled.error||compiled.status!==0)throw Error('coordinate compilation failed');
const classes=readdirSync(out,{recursive:true}).filter(p=>p.endsWith('.class')).sort().map(p=>[p,hash(out+'/'+p)]);
writeFileSync(root+'/elliptic-coordinate-build-manifest.json',JSON.stringify({sources:before,classes,release:16},null,2)+'\n');
const results=[];
for(const mode of ['jit','interpreter']){
 const args=['-ea','-Xmx256m',...(mode==='interpreter'?['-Xint']:[]),'-cp',cp,'EllipticCoordinates'];
 const started=new Date().toISOString();const r=spawnSync('java',args,{encoding:'utf8',timeout:60000,killSignal:'SIGKILL',maxBuffer:8*1024*1024});
 const file=mode+'-elliptic-coordinates.log';writeFileSync(root+'/'+file,r.stdout??'');writeFileSync(root+'/'+file+'.stderr',r.stderr??'');
 const record={mode,args,started,finished:new Date().toISOString(),status:r.status,error:r.error?.code??null,signal:r.signal,file,sha256:hash(root+'/'+file),stderrSha256:hash(root+'/'+file+'.stderr')};results.push(record);writeFileSync(root+'/elliptic-coordinate-runs.json',JSON.stringify(results,null,2)+'\n');console.log(JSON.stringify(record));
 if(r.error||r.status!==0)throw Error('coordinate probe failed');
}
for(const[p,h]of before)if(hash(p)!==h)throw Error('source drift '+p);
