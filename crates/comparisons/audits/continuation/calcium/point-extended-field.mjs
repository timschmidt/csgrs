// Independent exact interpreter for this corpus, not a general Real decoder.
// Coefficients lie in Q(sqrt(2),sqrt(3)); formal variables P,E,S,C represent
// pi,e,sin(e-pi),cos(e-pi), with C^2 = 1-S^2. No numerical approximation,
// Hyper equality, class-only value assertion, FLINT or interval overlap is used.
import assert from 'node:assert/strict';
const abs=n=>n<0n?-n:n;
function gcd(a,b){a=abs(a);b=abs(b);while(b)[a,b]=[b,a%b];return a;}
export function q(n,d=1n){n=BigInt(n);d=BigInt(d);assert(d);if(d<0n){n=-n;d=-d;}const g=gcd(n,d);return[n/g,d/g];}
const qa=(a,b)=>q(a[0]*b[1]+b[0]*a[1],a[1]*b[1]),qn=a=>[-a[0],a[1]],qm=(a,b)=>q(a[0]*b[0],a[1]*b[1]);
const qd=(a,b)=>q(a[0]*b[1],a[1]*b[0]);
const qs=a=>a[0]<0n?-1:a[0]>0n?1:0;
export const fzero=()=>[q(0),q(0),q(0),q(0)];
export const scalar=a=>[a,q(0),q(0),q(0)],integer=n=>scalar(q(n));
export const fadd=(a,b)=>a.map((v,i)=>qa(v,b[i])),fneg=a=>a.map(qn),fsub=(a,b)=>fadd(a,fneg(b));
export const fkey=a=>a.map(v=>v.join('/')).join(',');
export const feq=(a,b)=>fkey(a)===fkey(b),fz=a=>a.every(v=>v[0]===0n);
export function fmul(a,b){const out=fzero();for(let i=0;i<4;i++)if(a[i][0])for(let j=0;j<4;j++)if(b[j][0]){
 const common=i&j,factor=((common&1)?2:1)*((common&2)?3:1);
 out[i^j]=qa(out[i^j],qm(qm(a[i],b[j]),q(factor)));
}return out;}
const conj=(a,m)=>a.map((v,i)=>(((i&m)&1)^(((i&m)>>1)&1))?qn(v):v);
export function finv(a){const n=fmul(fmul(conj(a,1),conj(a,2)),conj(a,3)),d=fmul(a,n);assert(d[0][0]&&d.slice(1).every(v=>!v[0]));return n.map(v=>qd(v,d[0]));}
export const fdiv=(a,b)=>fmul(a,finv(b));
function quadraticSign(a,b){const x=qs(a),y=qs(b);if(!x)return y;if(!y||x===y)return x;return x*qs(qa(qm(a,a),qn(qm(q(2),qm(b,b)))));}
export function fsign(a){
 const A=[a[0],a[1],q(0),q(0)],B=[a[2],a[3],q(0),q(0)];
 const x=quadraticSign(...A),y=quadraticSign(...B);if(!x)return y;if(!y||x===y)return x;
 const d=fsub(fmul(A,A),fmul(integer(3),fmul(B,B)));return x*quadraticSign(d[0],d[1]);
}
export const fcompare=(a,b)=>fsign(fsub(a,b));
export function sqrtFloor(n){assert(n>=0n);if(n<2n)return n;let x=1n<<BigInt(Math.ceil(n.toString(2).length/2));for(;;){const y=(x+n/x)>>1n;if(y>=x)return x;x=y;}}
export function fsqrt(a){
 assert(a.slice(1).every(v=>!v[0])&&a[0][0]>=0n);
 for(const [i,d]of [1,2,3,6].entries()){
  const t=qd(a[0],q(d)),n=sqrtFloor(t[0]),den=sqrtFloor(t[1]);
  if(n*n===t[0]&&den*den===t[1]){const r=fzero();r[i]=q(n,den);return r;}
 }
 throw Error('Square root outside authored field: '+fkey(a));
}
export const root2=()=>fsqrt(integer(2)),root3=()=>fsqrt(integer(3));
export const evaluate=(poly,x)=>poly.reduceRight((v,c)=>fadd(fmul(v,x),c),fzero());
export function productRoots(roots){let p=[integer(1)];for(const r of roots){const out=Array.from({length:p.length+1},fzero);for(let i=0;i<p.length;i++){out[i]=fsub(out[i],fmul(p[i],r));out[i+1]=fadd(out[i+1],p[i]);}p=out;}return p;}
export const monic=p=>p.map(c=>fdiv(c,p.at(-1)));

const key=e=>e.join(','),constant='0,0,0,0';
function term(p,e,c){
 if(fz(c))return;
 if(e[3]>=2){const a=e.slice();a[3]-=2;term(p,a,c);const b=a.slice();b[2]+=2;term(p,b,fneg(c));return;}
 const k=key(e),v=p.has(k)?fadd(p.get(k),c):c;if(fz(v))p.delete(k);else p.set(k,v);
}
const pconstant=c=>fz(c)?new Map():new Map([[constant,c]]);
const pa=(a,b)=>{const p=new Map(a);for(const[k,c]of b)term(p,k.split(',').map(Number),c);return p;};
const pn=a=>new Map([...a].map(([k,c])=>[k,fneg(c)]));
function pm(a,b){const p=new Map();for(const[k,c]of a)for(const[l,d]of b){const e=k.split(',').map(Number),f=l.split(',').map(Number);term(p,e.map((v,i)=>v+f[i]),fmul(c,d));}return p;}
const peq=(a,b)=>a.size===b.size&&[...a].every(([k,v])=>b.has(k)&&feq(v,b.get(k)));
function quotientField(n,d){
 assert(d.size);if(!n.size)return fzero();if(n.size!==d.size)return null;
 const [k,c]=d.entries().next().value;if(!n.has(k))return null;const v=fdiv(n.get(k),c);
 return [...d].every(([k,c])=>n.has(k)&&feq(n.get(k),fmul(v,c)))?v:null;
}
const rf=(n,d=pconstant(integer(1)))=>{const f=quotientField(n,d);return f===null?{n,d}:{n:pconstant(f),d:pconstant(integer(1))};};
const literal=f=>rf(pconstant(f));
const variable=i=>{const e=[0,0,0,0];e[i]=1;return rf(new Map([[key(e),integer(1)]]));};
const ra=(a,b)=>rf(pa(pm(a.n,b.d),pm(b.n,a.d)),pm(a.d,b.d));
const rn=a=>rf(pn(a.n),a.d),rm=(a,b)=>rf(pm(a.n,b.n),pm(a.d,b.d));
const req=(a,b)=>peq(pm(a.n,b.d),pm(b.n,a.d));
function ri(a){
 const f=quotientField(a.n,a.d);
 if(f!==null)return literal(finv(f));
 // The only nonalgebraic divisors admitted here are nonzero monomials in
 // positive pi/e. Do not assume an arbitrary formal denominator is nonzero.
 assert.equal(a.n.size,1);const[k,c]=a.n.entries().next().value,e=k.split(',').map(Number);
 assert(e[2]===0&&e[3]===0&&!fz(c));return rf(a.d,a.n);
}
function asField(a){const f=quotientField(a.n,a.d);assert(f!==null,'Unreduced transcendental output in authored corpus');return f;}
function natural(words){assert(Array.isArray(words));let n=0n;for(let i=words.length-1;i>=0;i--){assert(Number.isInteger(words[i])&&words[i]>=0&&words[i]<2**32);n=(n<<32n)+BigInt(words[i]);}return n;}
function rational(r){assert([-1,0,1].includes(r.sign));const n=natural(r.numerator),d=natural(r.denominator);assert(d>0n);assert.equal(n===0n,r.sign===0);return q(BigInt(r.sign)*n,d);}
const cache=new Map();
function node(c){
 const fingerprint=JSON.stringify(c);if(cache.has(fingerprint))return cache.get(fingerprint);
 assert.deepEqual(Object.keys(c),['internal']);const n=c.internal;
 const [type,value]=typeof n==='string'?[n,undefined]:Object.entries(n)[0];let out;
 if(typeof n!=='string')assert.equal(Object.keys(n).length,1);
 switch(type){
  case 'One':out=literal(integer(1));break;
  case 'Int':assert([-1,0,1].includes(value[0]));out=literal(scalar(q(BigInt(value[0])*natural(value[1]))));break;
  case 'Ratio':out=literal(scalar(rational(value)));break;
  case 'Constant':
   assert(['Pi','E','InvPi','Sqrt2'].includes(value));
   out=value==='InvPi'?ri(variable(0)):value==='Sqrt2'?literal(root2()):variable(value==='Pi'?0:1);break;
  case 'Negate':out=rn(node(value));break;
  case 'Add':out=ra(node(value[0]),node(value[1]));break;
  case 'Multiply':out=rm(node(value[0]),node(value[1]));break;
  case 'Square':out=rm(node(value),node(value));break;
  case 'Inverse':out=ri(node(value));break;
  case 'Offset':{const e=value[1];assert(Number.isInteger(e)&&Math.abs(e)<20000);out=rm(node(value[0]),literal(scalar(e<0?q(1n,1n<<BigInt(-e)):q(1n<<BigInt(e)))));break;}
  case 'Sqrt':out=literal(fsqrt(asField(node(value))));break;
  case 'PrescaledSin':case 'PrescaledCos':
   assert(req(node(value),ra(variable(1),rn(variable(0)))),'Only the shared finite e-pi argument is admitted');
   out=variable(type==='PrescaledSin'?2:3);break;
  default:throw Error('Uncovered serialized operation: '+type);
 }
 cache.set(fingerprint,out);return out;
}
export function realValue(r){
 const scale=scalar(rational(r.rational));let value;
 if(r.computable===null){assert.equal(r.class,'One');value=integer(1);}else value=asField(node(r.computable));
 if(r.class==='One')assert(feq(value,integer(1)));
 else if(r.class!=='Irrational'){
  assert.deepEqual(Object.keys(r.class),['Sqrt']);assert(feq(value,fsqrt(scalar(rational(r.class.Sqrt)))));
 }
 return fmul(scale,value);
}
export function fieldSelfTest(){
 const a=root2(),b=root3();assert(feq(fmul(a,a),integer(2)));assert(feq(fmul(b,b),integer(3)));
 for(const x of [a,b,fadd(a,b),fsub(a,b),fadd(integer(1),fadd(a,b))])assert(feq(fmul(x,finv(x)),integer(1)));
 assert.equal(fsign(fsub(b,a)),1);assert.equal(fsign(fsub(integer(3),fmul(integer(2),a))),1);
 assert.equal(fsign(fsub(fmul(a,b),fadd(a,b))),-1);
 assert.equal(fsign(scalar(q(1n,1n<<6000n))),1);
 const s=variable(2),c=variable(3);assert(req(ra(rm(s,s),rm(c,c)),literal(integer(1))));
 assert(req(rm(variable(0),ri(variable(0))),literal(integer(1))));
 assert.throws(()=>ri(s));assert.throws(()=>finv(fzero()));
 for(let n=0n;n<100n;n++){const k=sqrtFloor(n);assert(k*k<=n&&(k+1n)*(k+1n)>n);}
 return{field:'Q(sqrt(2),sqrt(3))',transcendentalRelation:'sin(e-pi)^2 + cos(e-pi)^2 = 1',status:'pass'};
}
