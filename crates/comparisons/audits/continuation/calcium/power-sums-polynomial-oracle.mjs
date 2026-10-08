// Independent polynomial-ring determinant oracle: subset/Leibniz expansion.
// No determinant sampling, interpolation, Newton recurrence, FLINT or Hyper oracle.
import {readFileSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
const carriers=[[-1,1],[1,2],[0,1],[-2,0,1],[-3,0,1],[1,-2,1],[0,-1,1],[-2,1,1],
 [-2,0,0,1],[-3,0,0,2],[-1,3,-3,1],[0,-2,0,1],[2,-3,0,1],[1,0,-10,0,1],
 [4,0,-4,0,1],[0,0,1],[-2,0,0,0,0,0,0,0,0,1],[-7,0,0,0,0,0,0,1],[6,-5,1],[1,0,1]];
const trim=p=>{while(p.length>1&&p.at(-1)===0n)p.pop();return p;};
const isZero=p=>p.every(c=>c===0n);
function add(a,b,sign=1n){const c=Array.from({length:Math.max(a.length,b.length)},(_,i)=>(a[i]??0n)+sign*(b[i]??0n));return trim(c);}
function mul(a,b){const c=Array(a.length+b.length-1).fill(0n);for(let i=0;i<a.length;i++)for(let j=0;j<b.length;j++)c[i+j]+=a[i]*b[j];return trim(c);}
function monomial(c,n){const p=Array(n+1).fill(0n);p[n]=c;return trim(p);}
function choose(n,k){let c=1n;for(let i=1;i<=k;i++)c=c*BigInt(n-i+1)/BigInt(i);return c;}
const bits=n=>{let k=0;for(;n;n>>>=1)k+=n&1;return k;};
function determinant(matrix){
 const n=matrix.length;assert(n<=10);const dp=Array(1<<n);dp[0]=[1n];
 for(let mask=0;mask<dp.length;mask++){
  const row=bits(mask);if(row===n||!dp[mask]||isZero(dp[mask]))continue;
  for(let col=0;col<n;col++)if(!(mask&(1<<col))&&!isZero(matrix[row][col])){
   const next=mask|(1<<col),sign=bits(mask>>>(col+1))%2?-1n:1n;
   dp[next]=add(dp[next]??[0n],mul(dp[mask],matrix[row][col]),sign);
  }
 }
 return dp.at(-1)??[0n];
}
const abs=n=>n<0n?-n:n;
function gcd(a,b){a=abs(a);b=abs(b);while(b)[a,b]=[b,a%b];return a;}
function primitive(p){p=trim(p);if(isZero(p))return null;const g=p.reduce(gcd,0n);return p.map(c=>(c/g).toString());}
function oracle(left,right,op){
 const a=left.map(BigInt),b=right.map(BigInt),m=a.length-1,n=b.length-1;
 const inX=Array.from({length:n+1},()=>[0n]);
 for(let k=0;k<=n;k++){
  if(op<2)for(let j=k;j<=n;j++){
   const negative=(op===0?k:j-k)%2;
   inX[k]=add(inX[k],monomial(b[j]*choose(j,k)*(negative?-1n:1n),j-k));
  }
  else if(op===2)inX[k]=monomial(b[n-k],n-k);
  else inX[k]=monomial(b[k],n-k);
 }
 while(inX.length>1&&isZero(inX.at(-1)))inX.pop();
 const effectiveN=inX.length-1,size=m+effectiveN;
 const matrix=Array.from({length:size},()=>Array.from({length:size},()=>[0n]));
 for(let row=0;row<effectiveN;row++)for(let k=0;k<=m;k++)matrix[row][row+m-k]=[a[k]];
 for(let row=0;row<m;row++)for(let k=0;k<=effectiveN;k++)matrix[effectiveN+row][row+effectiveN-k]=inX[k];
 return {poly:primitive(determinant(matrix)),effectiveN};
}
function selfTest(){
 assert.deepEqual(determinant([[[1n],[2n]],[[3n],[4n]]]),[-2n]);
 assert.deepEqual(oracle([-2,0,1],[-3,0,1],0).poly,['1','0','-10','0','1']);
 assert.deepEqual(oracle([-2,0,1],[-3,0,1],2).poly,['36','0','-12','0','1']);
 assert.deepEqual(oracle([-2,0,1],[-3,0,1],3).poly,['4','0','-12','0','9']);
 assert.deepEqual(oracle([-1,1],[-2,1],1).poly,['-1','-1']);
 assert.deepEqual(oracle([-2,0,1],[0,-1,1],3).poly,['2','0','-1']);
}

selfTest();
export {oracle};
