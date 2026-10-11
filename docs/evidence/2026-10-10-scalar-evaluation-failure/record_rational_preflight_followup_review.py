#!/usr/bin/env python3
"""Retain a source-only fixture design without claiming a measured failure."""
import gzip
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT/'target/scalar-evaluation-candidate/source'
record = E/'typed-api-prototype-v1162-budget-verus/source-hashes.json.gz'
sha = lambda data:hashlib.sha256(data).hexdigest()
assert sha(record.read_bytes()) == '2f5184c00b655901e7a89b44ccb63824a7aad7ce3790a11f7e82e7ca5da5192f'
sources = json.loads(gzip.decompress(record.read_bytes()))
prefix = 'crates/hyperreal/src/computable/node/'
reviewed = {}
for name,ranges in {
    'algebra.rs':[(75,115),(308,339),(437,615),(1220,1488),(1500,1635),(1670,1735),(2110,2425)],
    'roots_inverse_hyperbolic.rs':[(155,285)],
    'exp_trig.rs':[(545,648)],
    'primitive_constructors.rs':[(1,67),(945,998)],
    'approximation_queries.rs':[(434,503)],
    'evaluation_budget.rs':[(1,62)],
    'trig_operands.rs':[(225,252)],
    'representation.rs':[(370,438)],
    'structural_analysis.rs':[(1250,1315)],
}.items():
    data = (SOURCE/(prefix+name)).read_bytes()
    assert sha(data) == sources[prefix+name]
    lines = data.splitlines(keepends=True)
    reviewed[prefix+name] = {'source_sha256':sha(data),'total_lines':len(lines),
        'reviewed_slices':[{'first_line':a,'last_line':min(b,len(lines)),
            'sha256':sha(b''.join(lines[a-1:b]))} for a,b in ranges],
        'scope':'Only the listed inclusive ranges were read for this supplemental review.'}
out = E/'rational-preflight-supplemental-review-v1195.json'
assert not out.exists()
out.write_text(json.dumps({
    'status':'Read-only candidate fixture design; no new measured resource failure',
    'source_record':str(record.relative_to(E)),'source_record_sha256':sha(record.read_bytes()),
    'reviewed_source_slices':reviewed,
    'retained_prior_review':'algebraic-preflight-supplemental-review-v1133-corrected.json',
    'findings':[
        {'kind':'distinguish constructor representations before measuring',
         'finding':'Public Computable::atan recognizes exact rational arguments and constructs AtanRational, whose argument extraction charges a rational read. A raw AtanDeferred(Int) fixture can isolate the private path but does not demonstrate that ordinary construction creates that same shape.',
         'next_measurement':'Keep raw-shape helper measurements separate from complete public constructor/query schedules. Inspect the constructed root shape and retained facts before claiming a cold public path.'},
        {'kind':'public constructor fixture candidate',
         'expression':'atan(n + pi) - atan(m + pi), with positive integers n and m of different bit lengths',
         'path':['Computable::rational and Computable::pi','add and atan constructors',
             'sign_until_with_budget','sign_until','exact_sign(Add)',
             'inverse_trig_linear_sign_with_work::<true>','pi_atan_terms_form_with_work::<true>',
             'general negate/add constructors for the two argument difference'],
         'finding':'Deferred atan argument extraction borrows Computable nodes by Arc clone. The term combiner uses budgeted structural equality, whose integer comparison can reject different bit lengths after a node debit. The inverse-trig sign path then constructs an argument difference. General add invokes optional metered inverse-trig collapse, followed by exact_rational and rational_offset_parts; rational_offset_parts can copy exact integer offsets without a shared payload debit. The source ordering motivates checking copies after an unsuccessful optional collapse, but no allowance that exposes such a failure has been measured.',
         'next_measurement':'Construct fresh public expressions outside the allocation window; verify no retained root certificate first. Sweep small allowances on fresh graphs, compare with small-width controls, retain full receipts and measure only the synchronous sign query. Use strict monotonicity of atan for the sign oracle, then check generous/unlimited retries and zero-work proof reuse. Include both operand orders and preserve exact cancellation separately.'},
        {'kind':'bounded allocation ordering candidate remains separate',
         'location':prefix+'algebra.rs:321',
         'finding':'PiLaurentPolynomial::clone_with_work charges one visit and reserves the at-most-eight-term vector before RadicalCoefficient::clone_with_work checks all coefficient reads. The coefficient helper itself preflights its rational, radical and atom coefficients before cloning.',
         'next_measurement':'Freeze allowance cutpoints immediately before coefficient copy, plus funded copies and retained contents. Do not treat the bounded vector as an established unbounded resource failure.'},
        {'kind':'completed certificates and constructor setup',
         'finding':'sign_until_with_budget returns an immediate root sign before introducing the allowance scope. atan_deferred propagates a known argument sign; add can retain same-sign or constant-plus-rational facts. Cold-query fixtures must account for such legitimate setup-time facts. NodeFacts::new stores hints and supplied bound/sign facts, not a finite-real flag.'}
    ],
    'limits':[
        'No new compiled fixture, allocation measurement, benchmark, public trace or failing assertion is claimed.',
        'The suggested public expression and allowance schedule remain candidates until construction and query behavior are measured.',
        'Only the recorded source slices were reviewed; no whole-file or whole-call-graph audit is claimed.',
        'No runtime source, existing test, local recognition limit, property status or admission requirement changed during this review.'
    ]
},indent=2)+'\n')
print(json.dumps({'record':out.name,'reviewed_files':len(reviewed),
    'source_slices':sum(len(r['reviewed_slices']) for r in reviewed.values())}))
