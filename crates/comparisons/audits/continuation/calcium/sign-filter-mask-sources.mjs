import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {signFilterSources as firstSources,sha,json} from './sign-filter-sources.mjs';
export {sha,json};
export function signFilterSources() {
 const previous=firstSources();assert.deepEqual(previous,json('sign-filter-experiment.json').sourceMaps);
 const f='hyperlimit/src/resolve.rs',baseline=readFileSync('sign-filter-baseline/'+f,'utf8');
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
    // Only presence of each nonzero sign matters. Keep visiting terms after
    // both signs appear so later Unknown and trace behavior remain unchanged.
    let mut signs = 0_u8;

    for (term, multiplier) in terms {
        let Some(sign) = signed_nonzero_term(term, *multiplier)? else {
            continue;
        };
        signs |= if sign == Sign::Positive { 1 } else { 2 };
    }

    match signs {
        0 => finish_signed_term_filter(&[]),
        1 => finish_signed_term_filter(&[Sign::Positive]),
        2 => finish_signed_term_filter(&[Sign::Negative]),
        _ => finish_signed_term_filter(&[Sign::Positive, Sign::Negative]),
    }
}`;
 assert.equal(baseline.split(before).length,2);
 const candidate={};
 for(const[p,h]of Object.entries(previous.baseline)) {
  const source='sign-filter-mask/'+p;
  if(p===f)assert.equal(readFileSync(source,'utf8'),baseline.replace(before,after));
  else assert.equal(sha(source),h,p);
  candidate[p]=sha(source);
 }
 return{baseline:previous.baseline,candidate};
}
if(process.argv.includes('--sign-filter-mask-sources'))console.log(JSON.stringify(Object.fromEntries(Object.entries(signFilterSources()).map(([v,m])=>[v,Object.keys(m).length]))));
