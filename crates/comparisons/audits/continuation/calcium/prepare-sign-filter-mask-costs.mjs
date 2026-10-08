import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
const read=p=>readFileSync(p,'utf8');
let runner=read('run-sign-filter-costs.mjs');
function replace(old,value){assert.equal(runner.split(old).length,2,old);runner=runner.replace(old,value);}
replace("from './sign-filter-sources.mjs'","from './sign-filter-mask-sources.mjs'");
replace("for(const v of variants)assert.equal(json('results/sign-filter-'+v+'-app-build.json').code,0);",
 "for(const v of variants)assert.equal(json('results/sign-filter-'+(v==='candidate'?'mask':v)+'-app-build.json').code,0);");
replace('calcium-sign-filter\\.','calcium-sign-filter-mask\\.');
replace("  const source='/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/release/audit-sign-filter-'+v+'-'+kind,path=root+'/'+v+'-'+kind;",
 "  if(v==='baseline') { binaries[v+'-'+kind]=json('sign-filter-binaries.json')[v+'-'+kind]; continue; }\n"+
 "  const source='/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/release/audit-sign-filter-mask-'+kind,path=root+'/'+v+'-'+kind;");
runner=runner.replaceAll("'sign-filter-binaries.json'","'sign-filter-mask-binaries.json'");
replace("json('sign-filter-mask-binaries.json')[v+'-'+kind]","json('sign-filter-binaries.json')[v+'-'+kind]");
replace("raw='results/sign-filter-'+mode+'.jsonl'","raw='results/sign-filter-mask-'+mode+'.jsonl'");
replace("writeFileSync('sign-filter-'+mode+'-summary.json'","writeFileSync('sign-filter-mask-'+mode+'-summary.json'");
const app=read('sign-filter-app-baseline/Cargo.toml').replaceAll('baseline','mask');
const files={'sign-filter-app-mask/Cargo.toml':app,'run-sign-filter-mask-costs.mjs':runner};
console.log('*** Begin Patch\n'+Object.entries(files).map(([p,text])=>'*** Add File: '+resolve(p)+'\n'+text.trimEnd().split('\n').map(s=>'+'+s).join('\n')).join('\n')+'\n*** End Patch');
