import assert from 'node:assert/strict';
const abs=x=>x<0n?-x:x,gcd=(a,b)=>{a=abs(a);b=abs(b);while(b)[a,b]=[b,a%b];return a;};
const q=(n,d=1n)=>{n=BigInt(n);d=BigInt(d);assert(d!==0n);if(d<0n){n=-n;d=-d;}const g=gcd(n,d);return[n/g,d/g];};
const add=(a,b)=>q(a[0]*b[1]+b[0]*a[1],a[1]*b[1]),neg=a=>[-a[0],a[1]],sub=(a,b)=>add(a,neg(b));
const mul=(a,b)=>q(a[0]*b[0],a[1]*b[1]),div=(a,b)=>q(a[0]*b[1],a[1]*b[0]);
const compare=(a,b)=>{const v=a[0]*b[1]-b[0]*a[1];return v<0n?-1:v>0n?1:0;};
const text=a=>a.map(String).join('/');
// Machin identity: tan(4 atan(1/5) - atan(1/239)) = 1. The angle is
// positive and <4/5<pi/2, so it is pi/4, not a different tangent branch.
const tan2=t=>div(mul(q(2),t),sub(q(1),mul(t,t)));
const tan4=tan2(tan2(q(1,5)));
assert.deepEqual(tan4,q(120,119));
assert.deepEqual(div(sub(tan4,q(1,239)),add(q(1),mul(tan4,q(1,239)))),q(1));
// Alternating atan series, positive decreasing terms. An even number of
// terms gives a lower bound; adding the next term gives an upper bound.
function atanBounds(den,n) {
 assert(n%2===0);let sum=q(0);
 for(let k=0;k<n;k++)sum=add(sum,q(k%2?-1:1,BigInt(2*k+1)*BigInt(den)**BigInt(2*k+1)));
 return[sum,add(sum,q(1,BigInt(2*n+1)*BigInt(den)**BigInt(2*n+1)))];
}
const a=atanBounds(5,12),b=atanBounds(239,12);
const pi=[sub(mul(q(16),a[0]),mul(q(4),b[1])),sub(mul(q(16),a[1]),mul(q(4),b[0]))];
assert(compare(pi[0],pi[1])<0);assert(compare(pi[0],q(103993,33102))>0);
const rows=[];
for(const fixture of ['rational','pi-far','pi-near','pi-reversed'])for(const n of [3,4,8,16,32,128]) {
 const rational=fixture==='rational',left=rational?[q(0),q(3)]:[q(1),q(0)];
 const right=fixture==='rational'?q(2):fixture==='pi-far'?q(3):q(103993,33102);
 const points=Array.from({length:n-1},(_,i)=>{const t=q(i,n-2);return {x:left.map(v=>mul(v,t)),y:t};});
 points.push({x:[q(0),right],y:q(1)});if(fixture==='pi-reversed')points.reverse();
 const sum=[q(0),q(0)];
 for(let i=0;i<n;i++)for(let j=0;j<2;j++) {
  const p=points[i],next=points[(i+1)%n];sum[j]=add(sum[j],sub(mul(p.x[j],next.y),mul(p.y,next.x[j])));
 }
 const sign=fixture==='pi-reversed'?-1:1;
 assert.deepEqual(sum,[mul(q(sign),left[0]),mul(q(sign),sub(left[1],right))]);
 const bounds=pi.map(p=>add(mul(sum[0],p),sum[1]));
 assert(bounds.every(v=>compare(v,q(0))===sign));
 rows.push({fixture,vertices:n,twiceAreaPiCoefficient:text(sum[0]),twiceAreaConstant:text(sum[1]),sign});
}
console.log(JSON.stringify({method:'exact BigInt rational shoelace and alternating-series Machin bounds',
 piBounds:pi.map(text),cases:rows.length,rows}));
