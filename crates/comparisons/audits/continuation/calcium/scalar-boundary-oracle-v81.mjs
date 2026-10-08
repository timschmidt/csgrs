import assert from 'node:assert/strict';
import {q,add,neg,mul,cmp,primitive,expected as inverseExpected,minimal,endpoints} from './qqbar-inverse-oracle-v80.mjs';
import {fzero,scalar,finv,fz,fsign,fsub} from './point-extended-field.mjs';
export {q,add,neg,mul,cmp,primitive,endpoints};
export const complexBits=['0000000000000000','8000000000000000','0000000000000001','8000000000000001',
 '000fffffffffffff','0010000000000000','3fd5555555555555','bfe0000000000000','3ff0000000000000','7fefffffffffffff',
 'ffefffffffffffff','7ff0000000000000','fff0000000000000','7ff8000000000000','7ff0000000000001','fff8000000001234'];
const field=a=>({kind:'field',a}),real=a=>field([a,q(0),q(0),q(0)]);
export function binaryValue(width,hex){
 assert(width===32||width===64);assert(/^[0-9a-f]{16}$/.test(hex));
 const bits=BigInt('0x'+hex),s=width===32?23n:52n,e=width===32?8n:11n,bias=(1n<<(e-1n))-1n;
 assert(bits<(1n<<BigInt(width)));const negative=(bits>>BigInt(width-1))!==0n,
 exponent=(bits>>s)&((1n<<e)-1n),fraction=bits&((1n<<s)-1n);
 if(exponent===(1n<<e)-1n)return{value:null,error:fraction?'NotANumber':'Infinity'};
 const significand=exponent?fraction+(1n<<s):fraction,shift=(exponent?exponent:1n)-bias-s;
 const n=(negative?-1n:1n)*significand,value=shift<0n?q(n,1n<<-shift):q(n<<shift);
 return{value,error:null};
}
export function floatCases(){
 const out=[];
 for(const width of [32,64])for(const sign of [0,1])for(let exp=0;exp<(width===32?256:2048);exp++){
  const s=width===32?23n:52n;
  for(const fraction of [0n,1n,1n<<(s-1n),(1n<<s)-1n]){
   const bits=((BigInt(sign)<<BigInt(width-1))|(BigInt(exp)<<s)|fraction).toString(16).padStart(16,'0');
   out.push({family:'float',width,bits});
  }
 }
 assert.equal(out.length,18432);return out;
}
const polyMul=(a,b)=>{const out=Array(a.length+b.length-1).fill(null).map(()=>q(0));for(let i=0;i<a.length;i++)for(let j=0;j<b.length;j++)out[i+j]=add(out[i+j],mul(a[i],b[j]));return out;};
function translate(poly,a){
 let out=[q(0)];for(let i=poly.length-1;i>=0;i--){out=polyMul(out,[neg(a),q(1)]);out[0]=add(out[0],poly[i]);}
 while(out.length>1&&!out.at(-1)[0])out.pop();return primitive(out);
}
export function simpleComplex(a,b){
 return{poly:b[0]===0n?[-a[0],a[1]]:primitive([add(mul(a,a),mul(b,b)),mul(q(-2),a),q(1)]),real:real(a),imag:real(b)};
}
export function roundingValue({b,kind,imaginary=0}){
 assert(b>=0&&b<=6&&kind>=0&&kind<23);let a=q([-(1n<<256n),-3n,-1n,0n,1n,3n,1n<<256n][b]),coefficient=q(0);
 if(kind){
  const sign=kind%2?1:-1;
  if(kind<=8)a=add(a,q(sign,1n<<BigInt([1,8,128,1024][(kind-1)>>1])));
  else{
   const exp=kind<=16?[0,8,128,1024][(kind-9)>>1]:[8,128,1024][(kind-17)>>1];
   coefficient=q(sign,1n<<BigInt(exp));if(kind>=17)a=add(a,q(1,2));
  }
 }
 let p;
 if(!coefficient[0])p=imaginary?[q(3),q(0),q(1)]:[q(0),q(1)];
 else{const b2=mul(coefficient,coefficient);
  p=imaginary?[mul(add(mul(q(2),b2),q(3)),add(mul(q(2),b2),q(3))),q(0),add(q(6),mul(q(-4),b2)),q(0),q(1)]:[mul(q(-2),b2),q(0),q(1)];}
 return{poly:translate(p,a),real:field([a,coefficient,q(0),q(0)]),imag:field([q(0),q(0),q(imaginary),q(0)])};
}
export function compareInteger(value,n){return fsign(fsub(value.real.a,scalar(q(n))));}
export function integers(value){
 const a=value.real.a[0];let center=a[0]/a[1];if(a[0]<0n&&a[0]%a[1])center--;
 for(let n=center-2n;n<=center+2n;n++)if(compareInteger(value,n)>=0&&compareInteger(value,n+1n)<0)
  return{floor:n,ceil:compareInteger(value,n)===0?n:n+1n};
 throw Error('Authored bounded quadratic offset escaped independent floor search');
}
export function reciprocal(c){
 const base=inverseExpected({family:'angle',op:c.op?'asin':'acos',k:c.k});
 if(fz(base.real.a))return null;const a=finv(base.real.a);return{poly:minimal(a),real:field(a),imag:field(fzero()),angle:base.angle};
}
export function controls(c){
 if(c.k===5)return{...simpleComplex(q(0),q(1)),angle:null};
 const a=[q(0),q(1),q(-1),q(1,2),q(-1,2)][c.k];
 return{...simpleComplex(a,q(0)),angle:c.k===1?(c.op?q(1,2):q(0)):c.k===2?(c.op?q(-1,2):q(1)):null};
}
export function nativeCases(){
 const out=floatCases();
 for(let i=0;i<16;i++)for(let j=0;j<16;j++)out.push({family:'complex-float',i,j});
 for(const op of [0,1])for(let k=0;k<24;k++)for(const shift of [-3,0,5])for(const scale of [1,3])for(const phase of [0,1])out.push({family:'inverse',op,k,shift,scale,phase});
 for(const op of [0,1])for(let k=0;k<6;k++)out.push({family:'inverse-control',op,k});
 for(let b=0;b<7;b++)for(let kind=0;kind<23;kind++)for(const imaginary of [0,1])for(const phase of [0,1])out.push({family:'round',b,kind,imaginary,phase});
 out.push({family:'swap-integer'},{family:'phi'},{family:'rational-extraction'});assert.equal(out.length,19923);return out;
}
export function selfTest(){
 assert.deepEqual(binaryValue(64,'0000000000000001').value,q(1n,1n<<1074n));
 assert.deepEqual(binaryValue(64,'0010000000000000').value,q(1n,1n<<1022n));
 assert.deepEqual(binaryValue(64,'8000000000000000').value,q(0));
 assert.deepEqual(binaryValue(32,'0000000000000001').value,q(1n,1n<<149n));
 assert.equal(binaryValue(64,'7ff0000000000001').error,'NotANumber');
 assert.deepEqual(roundingValue({b:3,kind:9,imaginary:1}).poly,[25n,0n,2n,0n,1n]);
 assert.deepEqual(roundingValue({b:3,kind:9}).poly,[-2n,0n,1n]);
 assert.deepEqual(integers(roundingValue({b:3,kind:10})),{floor:-2n,ceil:-1n});
 return{status:'pass',binaryOracle:'IEEE significand/exponent to exact rational, no floating evaluation',
  roundingOracle:'Exact Q(sqrt(2)) signs and independent biquadratic complex minimal polynomials'};
}
