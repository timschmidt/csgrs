#!/usr/bin/env python3
"""Retain read-only follow-up findings without claiming new measured failures."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT/'target/scalar-evaluation-candidate/source'
record = E/'typed-api-prototype-v1101-budget-verus/source-hashes.json.gz'
sha = lambda data: hashlib.sha256(data).hexdigest()
assert sha(record.read_bytes()) == '824a7ecd2822e2f06c5ee58d6090ce60aa8e69098d5774ecc0c7113f50caa54a'
sources = json.loads(gzip.decompress(record.read_bytes()))
prefix = 'crates/hyperreal/src/computable/node/'
reviewed = {}
for name,ranges in {
    'algebra.rs':[(1,820),(895,1015),(1220,1542),(1645,1730),(1910,2068),(2145,2410)],
    'domain_certificates.rs':[(1,181)],
    'evaluation_budget.rs':[(1,62)],
    'approximation_queries.rs':[(1,200),(410,580)],
    'structural_analysis.rs':[(1270,1370),(1670,1820)],
    'exp_trig.rs':[(180,282),(550,627)],
    'normal_form_budget_tests.rs':[(1,180)],
}.items():
    path = SOURCE/(prefix+name)
    data = path.read_bytes()
    assert sha(data) == sources[prefix+name]
    lines = data.splitlines(keepends=True)
    reviewed[prefix+name] = {
        'source_sha256':sha(data),'total_lines':len(lines),
        'reviewed_slices':[{'first_line':a,'last_line':b,'sha256':sha(b''.join(lines[a-1:b]))} for a,b in ranges],
        'scope':'Only the listed inclusive line ranges were read for this supplemental review.'
    }
searches = []
for pattern in [
    'inverse_trig_linear_sign\\(|fn immediate_sign|fn exact_sign\\(|fn approx\\(',
    'fn has_finite_real_certificate|fn ensure_finite_real|fn bounded_rational_value',
    'bounded_laurent_rational|compact_pi_laurent|collapsed_inverse_trig_linear_sum|rational_offset_parts',
]:
    command = ['rg','-n','--glob','*.rs','--glob','!**/*test*',pattern,'crates/hyperreal/src/computable/node']
    result = subprocess.run(command,cwd=SOURCE,text=True,capture_output=True)
    assert result.returncode == 0 and not result.stderr
    searches.append({'command':command,'exit_code':0,'stdout':result.stdout,
        'scope':'Textual inventory only; inline tests may appear. Search hits do not establish execution.'})
out = E/'algebraic-preflight-supplemental-review-v1133.json'
assert not out.exists()
out.write_text(json.dumps({
    'status':'Read-only follow-up review; no new runtime change or measured failure',
    'source_record':str(record.relative_to(E)),'source_record_sha256':sha(record.read_bytes()),
    'reviewed_source_slices':reviewed,'text_searches':searches,
    'findings':[
        {
            'kind':'candidate for a frozen allocation regression',
            'location':prefix+'structural_analysis.rs:1734',
            'finding':'bounded_rational_value tests Ratio numerator/denominator widths with canonicalizing accessors before meter.read. The retained algebraic preflight repair does not alter this separate recognizer.',
            'next_measurement':'Use fresh lazy ratios with canonical values on both sides of the existing 4096-bit recognition boundary. Measure the named helper at insufficient work, preserve generous/unlimited answers, then trace domain-validation callers separately.'
        },
        {
            'kind':'static call path requiring dynamic coverage',
            'path':['sign_until_with_budget','sign_until','exact_sign on Add',
                'inverse_trig_linear_sign_with_work::<true>',
                'two-atan argument difference or acos endpoint difference',
                'Computable::negate and Computable::add'],
            'finding':'The metered inverse-trig sign implementation constructs argument differences with general constructors. add invokes unlimited internal_structural_eq and rational_offset_parts; exact_rational clones integer payloads. This establishes a source-level constructor path, not that a particular resource failure is reachable with a fixed public allowance.',
            'next_measurement':'Freeze valid public-query fixtures whose argument equality rejects cheaply, including different integer bit lengths, then measure wide argument negation/addition and retry proof retention. Preserve all constructor exactness and existing proof paths.'
        },
        {
            'kind':'bounded allocation ordering candidate',
            'location':prefix+'algebra.rs:321',
            'finding':'PiLaurentPolynomial::clone_with_work charges one visit and allocates its term vector before each RadicalCoefficient clone performs coefficient preflight. The vector is bounded by eight polynomial terms; no unbounded allocation or public resource regression is established.',
            'next_measurement':'Measure allowances just after the visit and before the first unavailable coefficient copy, alongside successful-copy and no-allowance behavior. Keep the eight-term limit and recognition semantics.'
        },
        {
            'kind':'existing bounds and distinct remaining work',
            'finding':'Laurent recognition already caps aggregate node visits at 512, polynomial terms at eight and radical terms at eight, with charged atom comparisons. The reviewed inverse-trig recognizers use a depth bound and query-local shared-node memoization; their unshared terms still need separate growth measurements. AlgebraicSeparation has its existing 256-node/16-generator/4096-combination guards. These are not native exact-operation time or memory bounds.'
        },
        {
            'kind':'separate domain-certificate payload review',
            'location':prefix+'domain_certificates.rs:35',
            'finding':'Several rational domain cases compare canonical numerator/denominator components after a node debit. Construction-time finite-real hints and cached certificates may bypass these branches; their remaining lazy-reduction and wide-scan exposure needs branch-specific measurement.'
        }
    ],
    'limits':[
        'No new benchmark, allocator measurement, failing assertion, public trace or whole-file audit is claimed beyond the listed reads.',
        'The source-level route into Computable constructors does not establish a route into the separately reviewed direct Real adapters.',
        'No existing fixture, recognition limit, runtime source, property status or admission requirement changed during this review.'
    ]
},indent=2)+'\n')
print(json.dumps({'reviewed_files':len(reviewed),'source_slices':sum(len(r['reviewed_slices']) for r in reviewed.values()),'text_searches':len(searches),'record':out.name}))
