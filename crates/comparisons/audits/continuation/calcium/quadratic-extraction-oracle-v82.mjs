import assert from 'node:assert/strict';
import {q,abs,gcd,primitive,cmp} from './qqbar-inverse-oracle-v80.mjs';
import {endpoints81,endpointSelfTest} from './scalar-boundary-endpoints-v81.mjs';
export {q,gcd,abs,cmp};
export function cases(){
 const rows=[],push=(d,s,a,b,den,family)=>rows.push({id:rows.length,family,d,s:String(s),a:String(a),b:String(b),q:String(den)});
 for(const d of [-5,-3,-2,-1,0,1,2,3,5,23])for(const s of [1,2,9,19,257,65537])
  for(const a of [-3,0,5])for(const b of [-2,1])for(const den of [1,6])push(d,s,a,b,den,'grid');
 for(const d of [-2,2])for(const s of [1,65537])for(const sign of [-1n,1n])for(const b of [-1,1])
  for(const den of [1n,1n<<128n])push(d,s,sign*(1n<<256n),b,den,'large');
 for(const d of [-2,2])for(const sign of [-1n,1n])for(const b of [-1,1])push(d,1,sign*(1n<<256n),b,1n<<1024n,'close');
 for(const a of [-(1n<<256n),-7n,0n,7n,1n<<256n])for(const den of [1n,6n,1n<<128n])push(2,19,a,0,den,'rational');
 assert.equal(rows.length,775);return rows;
}
export function expected(c){
 const a=BigInt(c.a),b=BigInt(c.b),s=BigInt(c.s),den=BigInt(c.q),d=BigInt(c.d),rad=d*s*s;
 const rational=!b||d===0n||d===1n,rationalNum=a+(d===1n?b*s:0n),zero={kind:'quadratic',a:q(0),b:q(0),d:1};
 const poly=rational?primitive([q(-rationalNum),q(den)]):primitive([q(a*a-b*b*rad),q(-2n*a*den),q(den*den)]);
 const real=rational?{...zero,a:q(rationalNum,den)}:{kind:'quadratic',a:q(a,den),b:q(d>0n?b*s:0n,den),d:Number(abs(d))};
 const imag=d<0n&&b?{kind:'quadratic',a:q(0),b:q(b*s,den),d:Number(-d)}:zero;
 return{a,b,s,den,d,rad,rational,rationalValue:q(rationalNum,den),poly,real,imag};
}
function integer(s){assert(typeof s==='string'&&/^-?\d+$/.test(s)&&s.length<=4096);return BigInt(s);}
export function checkRow(row,c,mode,phase){
 assert.deepEqual(Object.keys(row).sort(),['id','mode','phase','a','b','c','q','poly','real','imag','reconstructedPoly','reconstructedReal','reconstructedImag'].sort());
 assert.equal(row.id,c.id);assert.equal(row.mode,mode);assert.equal(row.phase,phase);
 const e=expected(c),a=integer(row.a),b=integer(row.b),d=integer(row.c),den=integer(row.q);let checks=0;
 const check=x=>{assert(x,`input ${c.id} mode ${mode} phase ${phase}`);checks++;};
 check(den>0n);check(gcd(gcd(a,b),den)===1n);
 if(e.rational){check(b===0n);check(d===0n);check(cmp(q(a,den),e.rationalValue)===0);}
 else{
  check(b!==0n&&((b>0n)===(e.b>0n)));check((d>0n)===(e.d>0n));
  check(a*e.den===e.a*den);check(b*b*d*e.den*e.den===e.b*e.b*e.rad*den*den);
  check(abs(d)%4n!==0n);
  if(e.d===-1n||mode===1)check(d===e.d);
  // Smooth factoring is heuristic above its documented trial limit. Do not
  // require its radicand to be minimal, or identical to complete factoring.
 }
 for(const name of ['poly','reconstructedPoly']){assert.deepEqual(row[name],e.poly.map(String));checks++;}
 for(const [name,value]of [['real',e.real],['imag',e.imag],['reconstructedReal',e.real],['reconstructedImag',e.imag]]){
  const r=endpoints81(value,row[name]);check(r.ordered);check(r.contains);check(r.width);
 }
 return checks;
}
export function selfTest(){
 endpointSelfTest();
 assert.deepEqual(expected({a:'1',b:'1',s:'1',d:2,q:'1'}).poly,[-1n,-2n,1n]);
 assert.deepEqual(expected({a:'1',b:'-1',s:'3',d:-1,q:'2'}).poly,[5n,-2n,2n]);
 assert.deepEqual(expected({a:'3',b:'-2',s:'9',d:1,q:'6'}).poly,[5n,2n]);
 const squarefree=[-5,-3,-2,-1,2,3,5,23];for(const d of squarefree)for(let p=2;p*p<=Math.abs(d);p++)assert(Math.abs(d)%(p*p)!==0);
 return{status:'pass',inputs:775,rows:4650,field:'Known squarefree quadratic fields, including Q(i); exact integer identities and selected-component enclosure containment',
  minimality:'Required only for complete factoring and the mandatory Gaussian-rational special case. Smooth factoring is not assumed canonical.'};
}
