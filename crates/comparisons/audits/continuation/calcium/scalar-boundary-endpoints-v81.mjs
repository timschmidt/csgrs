// The frozen v80 decoder bounds exponent magnitude at 8192. Valid public
// near-boundary outputs here reach -16383; widen only the wire resource bound.
// Containment, endpoint ordering and <=2^-96 width requirements are unchanged.
import assert from 'node:assert/strict';
import {q,add,neg,mul,cmp,endpoints as oldEndpoints} from './qqbar-inverse-oracle-v80.mjs';
import {scalar,fsub,fsign} from './point-extended-field.mjs';
function decode(wire){
 assert(Array.isArray(wire)&&wire.length===3);for(const v of wire)assert(typeof v==='string'&&/^-?\d+$/.test(v)&&v.length<=12000);
 const e=BigInt(wire[2]);assert(e>=-32768n&&e<=32768n);
 return wire.slice(0,2).map(s=>e<0n?q(BigInt(s),1n<<-e):q(BigInt(s)<<e));
}
const quadraticSign=(a,b,d)=>{const x=cmp(a,q(0)),y=cmp(b,q(0));if(!x)return y;if(!y||x===y)return x;return x*cmp(mul(a,a),mul(q(d),mul(b,b)));};
export function endpoints81(value,wire){
 const[lo,hi]=decode(wire);let contains;
 if(value.kind==='field')contains=fsign(fsub(value.a,scalar(lo)))>=0&&fsign(fsub(value.a,scalar(hi)))<=0;
 else{assert.equal(value.kind,'quadratic');contains=quadraticSign(add(value.a,neg(lo)),value.b,value.d)>=0&&quadraticSign(add(value.a,neg(hi)),value.b,value.d)<=0;}
 return{ordered:cmp(lo,hi)<=0,contains,width:cmp(add(hi,neg(lo)),q(1n,1n<<96n))<=0};
}
export function endpointSelfTest(){
 let comparisons=0;for(const e of [-8192,-128,-96,-1,0,128])for(const n of [-17,-1,0,1,19]){
  const w=[String(n-1),String(n+1),String(e)],a=decode([String(n),String(n),String(e)])[0],value={kind:'field',a:scalar(a)};
  assert.deepEqual(endpoints81(value,w),oldEndpoints(value,w));comparisons++;
 }
 const zero={kind:'field',a:scalar(q(0))};assert.deepEqual(endpoints81(zero,['-1','1','-16383']),{ordered:true,contains:true,width:true});
 assert.throws(()=>endpoints81(zero,['0','0','-32769']));assert.throws(()=>endpoints81(zero,['x','0','0']));
 assert.throws(()=>endpoints81(zero,['1'.repeat(12001),'0','0']));
 assert.equal(endpoints81(zero,['1','-1','0']).ordered,false);
 return{status:'pass',sharedRangeComparisons:comparisons,maxExponentMagnitude:32768,maxWireDigits:12000,
  note:'Resource decoding bound expanded; mathematical checks unchanged. Original failing checker and capture preserved.'};
}
