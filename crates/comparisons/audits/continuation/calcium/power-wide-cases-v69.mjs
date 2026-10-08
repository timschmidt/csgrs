import {writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
const abs=n=>n<0n?-n:n;
function gcd(a,b){a=abs(a);b=abs(b);while(b)[a,b]=[b,a%b];return a;}
const q=(n,d=1n)=>{assert(d!==0n);if(d<0n){n=-n;d=-d;}const g=gcd(n,d);return[n/g,d/g];};
const wire=r=>r.join('/');
function multiply(a,b){const p=Array(a.length+b.length-1).fill(0n);for(let i=0;i<a.length;i++)for(let j=0;j<b.length;j++)p[i+j]+=a[i]*b[j];return p;}
const linear=r=>[-r[0],r[1]];
function carrier(degree,point,height,family,side){
 const t=1n<<BigInt(height);let p=linear(point),used=1;
 while(used<degree){
  if(family===3&&used+2<=degree){p=multiply(p,[t+3n,0n,(t-1n)*(t-1n)]);used+=2;continue;}
  const r=family===1?point:family===2&&used===1?q(0n):q(BigInt(side===0?1:-1)*(t+BigInt(2*used+3)),t+BigInt(2*used+1));
  p=multiply(p,linear(r));used++;
 }assert.equal(p.length,degree+1);assert(p.at(-1)!==0n);return p;
}
export function makeWideCases(){
 const pairs=[];for(let m=1;m<=9;m++)for(let n=1;n<=9;n++)if(m*n<=9)pairs.push([m,n]);assert.equal(pairs.length,23);
 const heights=[1,31,65,129,257],families=['distinct','repeated','unused-zero','complex-conjugates'],rows=[];
 for(const [pair,[m,n]]of pairs.entries())for(const [hi,height]of heights.entries())for(let family=0;family<4;family++){
  const t=1n<<BigInt(height),a=family===3&&pair%2===0?q(0n):q(t+1n,t-1n),
   b=family===1?a:family===3&&pair%3===0?q(0n):q(-(t+3n),t+1n);
  const signs=[(pair+hi+family)%2===0?1n:-1n,(pair+2*hi+family)%4<2?1n:-1n];
  if(family===1&&m===n)signs[1]=signs[0];
  const source=[a,b].map((point,side)=>{
   const p=carrier(side===0?m:n,point,height,family,side),scale=q(signs[side]*(t+7n),t+5n);
   return{point:wire(point),polynomial:p.map(c=>wire(q(c*scale[0],scale[1])))};
  });
  for(let op=0;op<4;op++)rows.push({id:rows.length,pair,m,n,height,family:families[family],op,left:source[0],right:source[1]});
 }
 assert.equal(rows.length,1840);return rows;
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const rows=makeWideCases();writeFileSync('power-wide-input-v69.json',JSON.stringify(rows)+'\n',{flag:'wx'});
 console.log(JSON.stringify({cases:rows.length,policies:2,degreePairs:23,heights:[1,31,65,129,257],families:4,
  note:'Deterministic authored public inputs, not random/statistically representative samples. Coefficient heights grow with carrier degree; height is a construction parameter.'}));
}
