import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const root=import.meta.dirname,base=root+'/../Ruffini/demos/src/main/java/demo/poseidon/poseidon_constants/';
const modulus=21888242871839275222246405745257275088548364400416034343698204186575808495617n;
const sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
function det(input){
 const a=input.map(r=>r.slice()),n=a.length;if(n===0)return 1n;let previous=1n,sign=1n;
 for(let k=0;k<n-1;k++){
  let pivot=k;while(pivot<n&&a[pivot][k]===0n)pivot++;if(pivot===n)return 0n;
  if(pivot!==k){[a[k],a[pivot]]=[a[pivot],a[k]];sign=-sign;}
  const p=a[k][k];for(let i=k+1;i<n;i++){for(let j=k+1;j<n;j++){
   const numerator=p*a[i][j]-a[i][k]*a[k][j];assert.equal(numerator%previous,0n);a[i][j]=numerator/previous;
  }a[i][k]=0n;}previous=p;
 }
 return sign*a[n-1][n-1];
}
function laplace(a){if(a.length===0)return 1n;return a[0].reduce((sum,x,j)=>sum+(j%2?-1n:1n)*x*laplace(a.slice(1).map(r=>r.filter((_,k)=>k!==j))),0n);}
let oracleControls=0;for(let n=0;n<=5;n++)for(let seed=0;seed<20;seed++){
 const a=Array.from({length:n},(_,i)=>Array.from({length:n},(_,j)=>BigInt((i*17+j*13+seed*(i+1)*(j+1))%11-5)));
 assert.equal(det(a),laplace(a));oracleControls++;
}
function gcd(a,b){a=a<0n?-a:a;while(b){[a,b]=[b,a%b];}return a;}
function subsets(n,k){const result=[];for(let mask=0;mask<1<<n;mask++){const indices=Array.from({length:n},(_,i)=>i).filter(i=>mask&(1<<i));if(indices.length===k)result.push(indices);}return result;}
const matrices=[];
for(let index=0;index<4;index++){
 const name='matrix'+String(index).padStart(2,'0'),path=base+name,contents=readFileSync(path,'utf8'),lines=contents.trimEnd().split('\n'),n=index+2;
 assert.equal(lines.length,n*n);assert(lines.every(s=>/^[0-9]+$/.test(s)));const values=lines.map(BigInt);assert(values.every(v=>v>=0n&&v<modulus));
 const a=Array.from({length:n},(_,i)=>values.slice(i*n,(i+1)*n));let testedMinors=0;
 for(let k=1;k<=n;k++)for(const rows of subsets(n,k))for(const cols of subsets(n,k)){
  const minor=rows.map(i=>cols.map(j=>a[i][j]));const d=det(minor);assert.equal(d,laplace(minor));assert.equal(gcd(d,modulus),1n);testedMinors++;
 }
 matrices.push({name,n,physicalLines:lines.length,sha256:sha(path),finalNewline:contents.endsWith('\n'),testedMinors,allDeterminantsUnitsModuloAdvertisedModulus:true});
}
const result={sourceSha256:sha(import.meta.filename),oracleControls,matrices,totalReadEntries:matrices.reduce((s,m)=>s+m.physicalLines,0),totalTestedMinors:matrices.reduce((s,m)=>s+m.testedMinors,0),scope:'only four explicitly read matrix resources; no primality, cryptographic or unread-data qualification claimed'};
writeFileSync(root+'/read-matrices-analysis.json',JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result,null,2));
