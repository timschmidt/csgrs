import { readFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url));
export const json=p=>JSON.parse(readFileSync(resolve(here,p),'utf8'));
export const sha=p=>createHash('sha256').update(readFileSync(resolve(here,p))).digest('hex');
export function sources() {
  const origin=json('derivative-demand-origin.json'), out={};
  const f='hypercurve/src/rational_bezier_general.rs';
  const original=readFileSync(resolve(here,origin.origin,f),'utf8');
  const base=original.replace('fn evaluate_power_polynomial_derivatives(',
    '#[cfg(test)]\n#[path = "derivative_demand_tests.rs"]\nmod derivative_demand_tests;\n\nfn evaluate_power_polynomial_derivatives(');
  let candidate=base;
  const old='    for coefficient in coefficients.iter().rev() {\n        for order in (1..=max_order).rev() {';
  assert.equal(base.split(old).length,3);
  for(const comment of ['This Horner prefix has degree at most processed; higher orders stay zero.',
    'Preserve every requested output while skipping the still-zero tail.']) {
    candidate=candidate.replace(old,'    for (processed, coefficient) in coefficients.iter().rev().enumerate() {\n        // '+comment+'\n        for order in (1..=max_order.min(processed)).rev() {');
  }
  for(const variant of ['baseline','candidate']) {
    const root='derivative-demand-'+variant; out[variant]={};
    for(const[p,h]of Object.entries(origin.sourceHashes)) {
      assert.equal(sha(resolve(here,origin.origin,p)),h);
      const path=root+'/'+p;
      if(p===f)assert.equal(readFileSync(resolve(here,path),'utf8'),variant==='baseline'?base:candidate);
      else assert.equal(sha(path),h,path);
      out[variant][p]=sha(path);
    }
    const test='hypercurve/src/derivative_demand_tests.rs';out[variant][test]=sha(root+'/'+test);
  }
  assert.equal(out.baseline['hypercurve/src/derivative_demand_tests.rs'],out.candidate['hypercurve/src/derivative_demand_tests.rs']);
  return out;
}
