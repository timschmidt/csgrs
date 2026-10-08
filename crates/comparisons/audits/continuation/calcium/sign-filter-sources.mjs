import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './derivative-demand-sources.mjs';
export {sha,json};
export function signFilterSources() {
 const origin=json('sign-filter-origin.json'),out={};
 assert.deepEqual(origin.sourceHashes,json('nfloat-complex-experiment.json').candidateSources.candidate);
 const f='hyperlimit/src/resolve.rs',old=readFileSync(origin.origin+'/'+f,'utf8');
 const registration='#[cfg(test)]\n#[path = "sign_filter_summary_tests.rs"]\nmod sign_filter_summary_tests;\n\n';
 const baseline=old.replace('#[cfg(test)]\nmod tests {',registration+'#[cfg(test)]\nmod tests {');assert.notEqual(baseline,old);
 const before=`fn signed_term_filter_dynamic(terms: &[(&Real, Sign)]) -> Option<PredicateOutcome<Sign>> {
    let mut nonzero = Vec::with_capacity(terms.len());

    for (term, multiplier) in terms {
        let Some(sign) = signed_nonzero_term(term, *multiplier)? else {
            continue;
        };
        nonzero.push(sign);
    }

    finish_signed_term_filter(&nonzero)
}`;
 const after=`fn signed_term_filter_dynamic(terms: &[(&Real, Sign)]) -> Option<PredicateOutcome<Sign>> {
    // One representative suffices for a same-sign sum; two distinct signs
    // suffice for a mixed sum. Still inspect every term in order: a later
    // unknown must keep its existing short-circuit and trace behavior.
    let mut nonzero = [Sign::Zero; 2];
    let mut nonzero_len = 0;

    for (term, multiplier) in terms {
        let Some(sign) = signed_nonzero_term(term, *multiplier)? else {
            continue;
        };
        if nonzero_len == 0 {
            nonzero[0] = sign;
            nonzero_len = 1;
        } else if sign != nonzero[0] {
            nonzero[1] = sign;
            nonzero_len = 2;
        }
    }

    finish_signed_term_filter(&nonzero[..nonzero_len])
}`;
 assert.equal(baseline.split(before).length,2);const candidate=baseline.replace(before,after);
 for(const variant of ['baseline','candidate']) {
  const root='sign-filter-'+variant;out[variant]={};
  for(const[p,h]of Object.entries(origin.sourceHashes)) {
   assert.equal(sha(origin.origin+'/'+p),h,p);
   if(p===f)assert.equal(readFileSync(root+'/'+p,'utf8'),variant==='baseline'?baseline:candidate);
   else assert.equal(sha(root+'/'+p),h,p);
   out[variant][p]=sha(root+'/'+p);
  }
  const test='hyperlimit/src/sign_filter_summary_tests.rs';assert.equal(sha(root+'/'+test),sha('sign_filter_summary_tests.rs'));
  out[variant][test]=sha(root+'/'+test);
 }
 return out;
}
if(process.argv.includes('--sign-filter-sources'))console.log(JSON.stringify(Object.fromEntries(Object.entries(signFilterSources()).map(([v,m])=>[v,Object.keys(m).length]))));
