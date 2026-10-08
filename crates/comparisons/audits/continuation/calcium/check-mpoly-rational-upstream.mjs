import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json} from './e-plan-qualified-sources.mjs';
export function checkMpolyRationalUpstream() {
 const names=['add','add_fmpq','add_fmpz','div','div_fmpq','div_fmpz','get_set_str','inv',
  'mul','mul_fmpq','mul_fmpz','randtest','sub','sub_fmpq','sub_fmpz'].map(s=>'fmpz_mpoly_q_'+s);
 const stdout=readFileSync('results/mpoly-rational-upstream-native.stdout','utf8');
 const clean=stdout.replace(/\x1b\[[0-9;]*m/g,''),lines=clean.trimEnd().split('\n');assert.equal(lines.length,30);
 for(let i=0;i<names.length;i++){assert.equal(lines[2*i],names[i]+'...');
  assert(new RegExp('^'+names[i]+' +[0-9]+\\.[0-9]+ +\\(PASS\\)$').test(lines[2*i+1]));}
 for(const tag of ['compile','native','linked-libraries']) {
  const p='results/mpoly-rational-upstream-'+tag,r=json(p+'.json');assert.equal(r.code,0);assert.equal(r.signal,null);
  assert.equal(readFileSync(p+'.stderr','utf8'),'');assert(Date.parse(r.finished)>=Date.parse(r.started));
 }
 const g=json('results/mpoly-rational-upstream-native.json');assert.equal(g.command,'env');
 assert.deepEqual(g.args,['FLINT_TEST_MULTIPLIER=1','/tmp/calcium-mpoly-rational.Hmk8Is/upstream']);
 const linked=readFileSync('results/mpoly-rational-upstream-linked-libraries.stdout','utf8');
 const libraries=Object.fromEntries([...linked.matchAll(/=> (\/[^ ]+) \(/g)].map(m=>[m[1],sha(m[1])]));
 assert.deepEqual(libraries,json('arf-conversion-experiment.json').libraries);
 return{passed:15,failed:0,skipped:0,names,multiplier:1,stdoutBytes:Buffer.byteLength(stdout),
  limits:'All15 registered current-FLINT rational-function tests executed without filters at explicit multiplier1; their random contexts use ORD_LEX and their mathematical references share polynomial/GCD kernels. Valid generated string roundtrips run; malformed inputs do not. This is native execution, not a full upstream Memcheck, all-target proof or independent oracle. The separate exact-polynomial corpus supplies independent certificates and its own focused memory check.'};
}
if(process.argv.includes('--mpoly-rational-upstream-summary'))console.log(JSON.stringify(checkMpolyRationalUpstream()));
