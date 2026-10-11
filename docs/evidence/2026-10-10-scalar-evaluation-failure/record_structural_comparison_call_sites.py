#!/usr/bin/env python3
"""Retain scoped call-site evidence without claiming a complete call graph."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT/'target/scalar-evaluation-candidate/source'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    record = E/'typed-api-prototype-v1043-budget-verus/source-hashes.json.gz'
    expected = 'a0d1d543babfb7ae724d0cfdebb6a294c33b59423b94223a0468716413a1b00b'
    assert sha(record) == expected
    source = json.loads(gzip.decompress(record.read_bytes()))
    prefix = 'crates/hyperreal/src/'
    scopes = {
        'computable/node/evaluation_budget.rs':'Complete two public allowance wrappers and local charge helpers.',
        'computable/node/exp_trig.rs':'atan_argument/atan_rational_argument and adjacent asin/acos argument accessors only.',
        'computable/node/algebra.rs':'Complete negate constructor only in this follow-up.',
        'real/arithmetic/elementary_functions.rs':'retained_atan_argument and adjacent inverse-trig adapters; initial sin, cos and cot branches that invoke it. No whole-file audit.',
        'real/arithmetic/quadratic_tower_sign.rs':'All production adapters before the test module; selected adjacent tests.',
        'computable/node/quadratic_tower.rs':'Private sign, branch, annihilator and polynomial adapter methods around lines 811-877. No whole-file audit.',
        'computable/node/radical_tower.rs':'Complete radical_tower_sign adapter only.',
        'computable/mod.rs':'Complete module facade.',
        'computable/node.rs':'Complete imports/includes facade.',
        'computable/approximation.rs':'Complete imports/includes facade and adjacent pi tests.',
        'computable/approximation/representation.rs':'Complete Approximation enum, LinearCombination3 and NormalQuantileData payload definitions; adjacent serialization guard.',
        'computable/node/representation.rs':'Computable/Node fields and initial constructor dispatch only.',
        'computable/node/bounds.rs':'Signal alias and initial should_stop implementation only.',
        'rational/arithmetic.rs':'Complete imports/includes facade.',
        'trace.rs':'Complete trace macro definitions.',
        'dispatch_trace.rs':'Initial report structures plus active-key/hash/state/recording/record implementation around lines 435-563; no whole-file audit.'
    }
    files = {}
    for short,scope in scopes.items():
        name = prefix+short
        assert sha(SOURCE/name) == source[name]
        files[name] = {'sha256':source[name],'scope':scope}
    searches = []
    for pattern,paths in [
        (r'BudgetScope::enter|atan_rational_argument\(|retained_atan_argument\(', [prefix]),
        (r'\bReal\b|crate::real|super::real|real::', [prefix+'computable',prefix+'rational']),
        (r'quadratic_tower_parts|quadratic_tower_positive_rational_branch|quadratic_tower_annihilating_polynomial|sign_polynomial_at_rational_square_root', [prefix]),
    ]:
        command = ['rg','-n',pattern,*paths,'--glob','*.rs','--glob','!**/*test*']
        result = subprocess.run(command,cwd=SOURCE,text=True,capture_output=True)
        assert result.returncode in [0,1] and not result.stderr
        searches.append({'command':command,'exit_code':result.returncode,'stdout':result.stdout})
    out = E/'structural-comparison-call-sites-v1074.json'
    assert not out.exists()
    out.write_text(json.dumps({
        'status':'Supplemental scoped source review; no complete transitive callgraph, resource proof or runtime change',
        'previous_review':'algebraic-leaf-reachability-review-v927.json',
        'source_record':str(record.relative_to(E)),'source_record_sha256':expected,
        'reviewed_source_slices':files,'searches':searches,
        'observed_edges':[
            'Real::sin, Real::cos and Real::cot call retained_atan_argument, which calls Computable::atan_rational_argument or its quadratic-surd argument accessor.',
            'Real tower adapters call private Computable tower recognizers. The inspected sign adapters preserve the outer rational sign; compaction, polynomial and branch adapters may construct or scale exact payloads.',
            'The only production BudgetScope::enter matches in this scan are the two Computable approximation/sign allowance wrappers.',
            'The inspected graph payloads contain concrete scalar data and Computable children; Signal is Arc<AtomicBool>. No user numeric callback is stored in those fields.',
            'The trace macro calls dispatch_trace::record, which updates a thread-local count map. Its recording wrapper invokes the caller closure externally, and record does not dispatch a stored callback.',
            'Computable::negate cancels a pre-existing Negate node at construction. A synthetic deep raw Negate chain therefore does not by itself demonstrate an ordinary constructor-chain failure.'
        ],
        'inference':'The additional reviewed call sites continue to support a Real-to-Computable adapter direction. They do not establish a route from a public Computable allowance scope back into the Real adapters. Adding allowance charges to those adapters under such a reachability claim remains unjustified by this evidence.',
        'limits':[
            'Search results are retained as searches, not credited as complete source reading. Embedded test modules may appear in files whose names do not contain test.',
            'No compiler-derived whole-program callgraph or formal non-reachability proof was produced.',
            'Serialization, ordinary direct Real construction resources, unshared forms and exact-operation native resources remain separate follow-up coverage.',
            'No existing runtime, fixture, process cap, property status, target or future-work scope changed.'
        ]
    },indent=2)+'\n')
    print(json.dumps({'record':out.name,'reviewed_source_slices':len(files),'retained_searches':len(searches)}))


if __name__ == '__main__':
    main()
