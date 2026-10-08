from pathlib import Path
import json

A=Path(__file__).resolve().parent
assert json.loads((A/'retained-structural-pair-20260928-v625-reaped.json').read_text())['outer_exit_code']==1
source=(A/'finite-root-envelope-candidate-v625.rs').read_text()
start=source.index('    pub(crate) fn finite_roots(')
end=source.index('\nfn parameter_is_in_ordered_range(',start)
helper=source[start:end]
helper=helper.replace('return Ok(Classification::Uncertain(reason));', '''eprintln!("FINITE_ROOTS uncertain line={} reason={reason:?}", line!());
                    eprintln!("FINITE_ROOTS rational_coefficients={:?}", polynomial.coefficients().iter().map(|value| value.exact_rational_ref().map(|value| { let text=value.to_string(); if text.len()<200 {text} else {format!("{} digits",text.len())} })).collect::<Vec<_>>());
                    return Ok(Classification::Uncertain(reason));''')
helper=helper.replace('let lower_bound = rational_bound(outer_lower, false);','''let lower_bound = rational_bound(outer_lower, false);''')
helper=helper.replace('let mut retained = Vec::with_capacity(roots.len());','''eprintln!("FINITE_ROOTS isolated count={}", roots.len());
            let mut retained = Vec::with_capacity(roots.len());''')
source=source[:start]+helper+source[end:]
(A/'retained-structural-split-diagnostic-v627.rs').write_text(source)
source=(A/'retained-structural-parameter-diagnostic-v624.rs').read_text()
old='eprintln!("INTERVAL_ROOTS endpoint_sign degree={}", coefficients.len().saturating_sub(1));'
new=old+'''
                    eprintln!("INTERVAL_ROOTS rational_coefficients={:?}", coefficients.iter().map(|value| value.exact_rational_ref().map(|value| { let text=value.to_string(); if text.len()<200 {text} else {format!("{} digits",text.len())} })).collect::<Vec<_>>());
                    eprintln!("INTERVAL_ROOTS endpoint_bounds={:?}", endpoint.certified_rational_interval(-24).map(|bounds| bounds.map(|value| {let text=value.to_string();if text.len()<200 {text}else{format!("{} digits",text.len())}})));'''
assert source.count(old)==1
(A/'retained-structural-parameter-diagnostic-v627.rs').write_text(source.replace(old,new))
driver=(A/'probe-retained-structural-pair-diagnostic-20260928-v624.py').read_text()
driver=driver.replace('retained-structural-pair-diagnostic-20260928-v624','retained-structural-pair-diagnostic-20260928-v627')
driver=driver.replace('retained-structural-split-diagnostic-v624.rs','retained-structural-split-diagnostic-v627.rs').replace('retained-structural-parameter-diagnostic-v624.rs','retained-structural-parameter-diagnostic-v627.rs')
driver=driver.replace('composition=prior',"assert json.loads((A/'retained-structural-pair-20260928-v625-reaped.json').read_text())['outer_exit_code']==1\ncomposition=prior")
(A/'probe-retained-structural-pair-diagnostic-20260928-v627.py').write_text(driver)
print('Prepared combined V627 root-envelope diagnostic')
