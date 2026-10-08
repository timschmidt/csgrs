import {readFileSync} from 'node:fs';
const m=await import('./'+(process.argv[2]??'point-extended-field.mjs'));
console.log(JSON.stringify(m.fieldSelfTest()));let count=0;
for(const variant of ['baseline','candidate'])for(const r of readFileSync('results/point-extended-public-'+variant+'.stdout','utf8').trim().split('\n').map(JSON.parse)){
 if(r.type!=='query')continue;
 for(const [side,root]of [['left',r.left],['right',r.right],['output',r.report.root]])if(root){
  for(const[field,value]of [['lower',root.lower],['upper',root.upper],['exact',root.exact],...root.polynomial.map((v,i)=>['coefficient-'+i,v])])if(value){
   try{m.realValue(value);count++;}catch(error){console.error(JSON.stringify({variant,case:r.case,policy:r.policy,history:r.history,side,field,decodedBeforeFailure:count}));throw error;}
  }
 }
}
console.log(JSON.stringify({status:'all-values-decoded',values:count,limits:'Exact decoding only; root arithmetic and interval certificate checks remain separate.'}));
