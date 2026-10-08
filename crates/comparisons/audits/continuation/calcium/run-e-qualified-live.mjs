import {spawn} from 'node:child_process';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sources,sha} from './e-plan-qualified-sources.mjs';
import {specifications} from './e-qualified-gates.mjs';
const initial=sources(false);
const check=()=>{for(const[p,h]of Object.entries(initial.candidate))assert.equal(sha(resolve('../../../..',p)),h,p);};
check();
for(const g of specifications(true).filter(g=>g.tag.startsWith('e-qualified-retained-'))) {
 await new Promise((ok,fail)=>{
  const c=spawn(process.execPath,['capture.mjs',g.tag,g.cwd,g.command,...g.args],{stdio:'inherit'});c.on('error',fail);
  c.on('close',(code,signal)=>code===0&&!signal?ok():fail(Error(JSON.stringify({tag:g.tag,code,signal}))));
 });
}
check();assert.deepEqual(sources(false),initial);
