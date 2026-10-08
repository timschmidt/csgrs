import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const root=import.meta.dirname,base=root+'/../Ruffini/demos/src/main/java/demo/poseidon/poseidon_constants/';
const modulus=21888242871839275222246405745257275088548364400416034343698204186575808495617n;
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const mod=x=>(x%modulus+modulus)%modulus;
function gcd(a,b){a=a<0n?-a:a;while(b)[a,b]=[b,a%b];return a;}
function inverse(x){
 let a=mod(x),b=modulus,u=1n,v=0n;
 while(b){const q=a/b;[a,b]=[b,a-q*b];[u,v]=[v,u-q*v];}
 assert.equal(a,1n,'pivot must be a unit; no primality assumption');return mod(u);
}
function bareiss(input){
 const a=input.map(r=>r.slice()),n=a.length;if(!n)return 1n;let previous=1n,sign=1n;
 for(let k=0;k<n-1;k++){
  let p=k;while(p<n&&a[p][k]===0n)p++;if(p===n)return 0n;
  if(p!==k){[a[k],a[p]]=[a[p],a[k]];sign=-sign;}
  const pivot=a[k][k];for(let i=k+1;i<n;i++){
   for(let j=k+1;j<n;j++){const t=pivot*a[i][j]-a[i][k]*a[k][j];assert.equal(t%previous,0n);a[i][j]=t/previous;}
   a[i][k]=0n;
  }previous=pivot;
 }return sign*a[n-1][n-1];
}
function modularDeterminant(input){
 const a=input.map(r=>r.map(mod)),n=a.length;let d=1n;
 for(let k=0;k<n;k++){
  let p=k;while(p<n&&gcd(a[p][k],modulus)!==1n)p++;
  if(p===n){if(a.slice(k).every(r=>r[k]===0n))return 0n;throw Error('no unit pivot; not a singularity proof');}
  if(p!==k){[a[k],a[p]]=[a[p],a[k]];d=-d;}
  d=mod(d*a[k][k]);const inv=inverse(a[k][k]);
  for(let i=k+1;i<n;i++){const factor=mod(a[i][k]*inv);for(let j=k+1;j<n;j++)a[i][j]=mod(a[i][j]-factor*a[k][j]);a[i][k]=0n;}
 }return mod(d);
}
function laplace(a){return a.length?a[0].reduce((s,x,j)=>s+(j%2?-1n:1n)*x*laplace(a.slice(1).map(r=>r.filter((_,k)=>k!==j))),0n):1n;}
let controls=0;for(let n=0;n<=5;n++)for(let seed=0;seed<20;seed++){
 const a=Array.from({length:n},(_,i)=>Array.from({length:n},(_,j)=>BigInt((17*i+13*j+seed*(i+1)*(j+1))%11-5)));
 const d=laplace(a);assert.equal(bareiss(a),d);assert.equal(modularDeterminant(a),mod(d));controls++;
}
const inventory=new Map(readFileSync(root+'/../RUFFINI_FILE_INVENTORY.tsv','utf8').trimEnd().split('\n').slice(1).map(s=>{const r=s.split('\t');return[r[0],r];}));
const matrices=[];
for(let index=4;index<16;index++){
 const name='matrix'+String(index).padStart(2,'0'),path=base+name,text=readFileSync(path,'utf8'),lines=text.split('\n'),n=index+2;
 if(text.endsWith('\n'))lines.pop();
 const row=inventory.get('demos/src/main/java/demo/poseidon/poseidon_constants/'+name);
 assert.equal(row[4],'READ');assert.equal(hash(path),row[3]);assert.equal(lines.length,+row[2]);assert.equal(lines.length,n*n);
 assert(lines.every(s=>/^(0|[1-9][0-9]*)$/.test(s)));const v=lines.map(BigInt);assert(v.every(x=>x>=0n&&x<modulus));
 const a=Array.from({length:n},(_,i)=>v.slice(i*n,(i+1)*n)),d=mod(bareiss(a));
 assert.equal(modularDeterminant(a),d);assert.equal(gcd(d,modulus),1n);
 let twoByTwo=0;for(let i=0;i<n;i++)for(let j=i+1;j<n;j++)for(let k=0;k<n;k++)for(let l=k+1;l<n;l++){
  assert.equal(gcd(a[i][k]*a[j][l]-a[i][l]*a[j][k],modulus),1n);twoByTwo++;
 }
 assert(v.every(x=>gcd(x,modulus)===1n));
 matrices.push({name,n,physicalLines:lines.length,sha256:hash(path),finalNewline:text.endsWith('\n'),determinantModuloAdvertisedModulus:String(d),oneByOneMinors:v.length,twoByTwoMinors:twoByTwo,checkedDeterminantsAreUnits:true});
}
const result={sourceSha256:hash(import.meta.filename),controls,matrices,entries:matrices.reduce((s,m)=>s+m.physicalLines,0),twoByTwoMinors:matrices.reduce((s,m)=>s+m.twoByTwoMinors,0),fullDeterminants:matrices.length,scope:'format, range, unit-entry, two-by-two and full-determinant checks only; no all-minor, primality or cryptographic qualification; no read credit from execution'};
writeFileSync(root+'/remaining-matrices-analysis.json',JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result,null,2));
