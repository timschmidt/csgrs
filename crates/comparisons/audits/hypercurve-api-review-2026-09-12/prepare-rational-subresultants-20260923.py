from pathlib import Path
from fractions import Fraction as Q
import io,json,shutil,subprocess,tarfile
workspace=Path('/home/tim/Documents/GitHub/workspace');audit=Path(__file__).resolve().parent
trial=Path('/tmp/hypercurve-gcd-replay-2026-09-23')
root=Path('/tmp/hypercurve-rational-subresultants-2026-09-23');root.mkdir()
for p in trial.iterdir():
    if p.is_dir() and p.name not in ('hypercurve','hypersolve','dumps'):
        (root/p.name).symlink_to(p.resolve(),target_is_directory=True)
repo=root/'hypersolve';repo.mkdir()
archive=subprocess.check_output(['git','archive','8bccea3d36b2803824129e648656126bdd9e3553'],cwd=workspace/'hypersolve')
with tarfile.open(fileobj=io.BytesIO(archive)) as stream:stream.extractall(repo,filter='data')
for name in ['src/algebraic_fiber.rs','src/integer_interpolation.rs','src/integer_interpolation/bivariate.rs']:
    shutil.copy2(trial/'hypersolve'/name,repo/name)
p=repo/'src/algebraic_fiber.rs';s=p.read_text(); original=(workspace/'hypersolve/src/algebraic_fiber.rs').read_text()
a=s.index('fn count_common_fiber_roots('); b=s.index('\nfn local_fiber_polynomial(',a)
a0=original.index('fn count_common_fiber_roots(');b0=original.index('\nfn local_fiber_polynomial(',a0)
s=s[:a]+original[a0:b0]+s[b:]
s=s.replace('assert!(rational_local_sturm_rows(&local, &mut field).is_some());','''let derivative = derivative_local_polynomial(&local, &field).unwrap();
                    assert!(rational_local_subresultant_rows(&local, &derivative, &mut field).is_some());''')
s=s.replace('fn rational_sturm_declines_degree_changes_and_general_exact_coefficients()', 'fn rational_sturm_preserves_degree_changes_and_general_exact_coefficients()')
# The generalized PRS now admits global gaps for GCD; Sturm's mathematical
# root-count assertions continue to cover its signed fallback.
s=s.replace('''                let mut field = LocalAlgebraicField::new(&root, policy).unwrap();
                let local =
                    local_fiber_polynomial(&fiber, CurveResultantParameter::First, &mut field)
                        .unwrap();
                assert!(rational_local_sturm_rows(&local, &mut field).is_none());
''','')
s=s.replace('fn rational_sturm_isolates_degree_41_fillet_contact_in_both_orientations()', 'fn rational_subresultants_replay_degree_41_fillet_contact_in_both_orientations()')
# Share the contact fixture, adding the distinct degree-six predicate.
pdata=repo/'tests/data/nonph_fillet_circle_contact.json';d=json.loads(pdata.read_text());g=json.loads((trial/'hypersolve/examples/contact_gcd_probe_20260923.json').read_text())
assert list(map(Q,d['base']))==list(map(Q,g['base']))
grows=g['second'];d['predicate']=[[row[i] if i<len(row) else '0/1' for row in grows] for i in range(max(map(len,grows)))];d['common_root_interval']=g['fiber_interval']
pdata.write_text(json.dumps(d,indent=2)+'\n')
a=s.index('        let coefficients = data["incidence"]',s.index('fn rational_subresultants_replay_degree_41'))
b=s.index('        for policy in ',a)
s=s[:a]+'''        let polynomial = |key: &str, transposed: bool| {
            let coefficients = data[key].as_array().unwrap().iter().map(|row| {
                row.as_array().unwrap().iter().map(&read).collect::<Vec<_>>()
            }).collect::<Vec<_>>();
            if !transposed {
                return BivariatePolynomial::new(coefficients);
            }
            let mut swapped = vec![vec![Real::zero(); coefficients.len()];
                coefficients.iter().map(Vec::len).max().unwrap()];
            for (i,row) in coefficients.iter().enumerate() {
                for (j,value) in row.iter().enumerate() {
                    swapped[j][i] = value.clone();
                }
            }
            BivariatePolynomial::new(swapped)
        };
        let fibers = [
            (polynomial("incidence", false), polynomial("predicate", false), CurveResultantParameter::First),
            (polynomial("incidence", true), polynomial("predicate", true), CurveResultantParameter::Second),
        ];
'''+s[b:]
s=s.replace('            for (fiber, orientation) in &fibers {','            for (fiber, predicate, orientation) in &fibers {',1)
a=s.index('                assert_eq!(report.certainty, Certainty::Exact);',a)+len('                assert_eq!(report.certainty, Certainty::Exact);')
s=s[:a]+'''
                // Incidence and the branch predicate share one root in this
                // isolator. Their degrees differ by two; forcing monic local
                // remainders used to expand expensive algebraic inverses.
                for (first, second) in [(fiber, predicate), (predicate, fiber)] {
                    let common = count_bivariate_common_fiber_roots_at_algebraic_parameter(
                        first, second, *orientation, &root,
                        &read(&data["common_root_interval"][0]),
                        &read(&data["common_root_interval"][1]), policy,
                    );
                    assert_eq!(common.status, AlgebraicFiberRootCountStatus::Counted);
                    assert_eq!(common.distinct_root_count, Some(1));
                    assert_eq!(common.certainty, Certainty::Exact);
                }'''+s[a:]
a=s.index('    #[test]\n    fn local_field_sturm_counts_even_multiplicity_in_both_orientations()')
s=s[:a]+'''    #[test]
    fn rational_subresultants_preserve_common_roots_across_degree_gaps() {
        // Both pairs have coprime rational cofactors. Their common factor is
        // (u^2-t)^multiplicity. The second pair has equal initial degrees and
        // an abnormal 5 -> 3 (or 7 -> 5) drop inside the PRS.
        let pairs = [
            (vec![1,0,2,0,0,1], vec![1,0,0,1]),
            (vec![1,1,0,1], vec![1,0,0,1]),
        ];
        let polynomial = |factor: &[i64], multiplicity, scale: Real| {
            let factor = factor.iter().map(|&value| real(value) * &scale).collect::<Vec<_>>();
            let shift = |places, values: Vec<Real>| {
                let mut row = vec![Real::zero(); places]; row.extend(values); row
            };
            BivariatePolynomial::new(if multiplicity == 1 {
                vec![shift(2, factor.clone()), factor.iter().map(|value| -value).collect()]
            } else {
                vec![shift(4, factor.clone()),
                     shift(2, factor.iter().map(|value| real(-2)*value).collect()), factor]
            })
        };
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            for (lower,upper,count) in [(-2,-1,0), (1,2,2)] {
                // (t^2-2)*(t-3) keeps a foreign factor in the presentation.
                let root = represented_root(vec![real(6),real(-2),real(-3),real(1)], real(lower), real(upper), policy);
                for multiplicity in [1,2] {
                    for (a,b) in &pairs {
                        let first = polynomial(a, multiplicity, rational(2,7));
                        let second = polynomial(b, multiplicity, rational(-3,5));
                        for (first,second) in [(&first,&second),(&second,&first)] {
                            let report = count_bivariate_common_fiber_roots_at_algebraic_parameter(
                                first, second, CurveResultantParameter::First, &root,
                                &real(-2), &real(2), policy,
                            );
                            assert_eq!(report.status, AlgebraicFiberRootCountStatus::Counted);
                            assert_eq!(report.distinct_root_count, Some(count));
                            assert_eq!(report.certainty, Certainty::Exact);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn rational_subresultants_preserve_selected_degree_changes_and_exact_coefficients() {
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let root = represented_root(vec![real(-3),real(0),real(1)],real(1),real(2),policy);
            let sqrt_three = real(3).sqrt().unwrap();
            let cases = [
                // F=u^3+t*u^2+u, G=dF/du. At sqrt(3) the linear PRS row
                // loses degree but remains nonzero; the inputs are coprime.
                (BivariatePolynomial::new(vec![vec![real(0),real(1),real(0),real(1)],vec![real(0),real(0),real(1)]]),
                 BivariatePolynomial::new(vec![vec![real(1),real(0),real(3)],vec![real(0),real(2)]]),0),
                // The common root sqrt(3) uses general exact Real payloads.
                (BivariatePolynomial::new(vec![vec![-sqrt_three.clone(), Real::one()-&sqrt_three,Real::one()]]),
                 BivariatePolynomial::new(vec![vec![real(2)*&sqrt_three, -real(2)-&sqrt_three,Real::one()]]),1),
            ];
            for (first,second,count) in cases {
                let mut field = LocalAlgebraicField::new(&root, policy).unwrap();
                let local_first = local_fiber_polynomial(&first, CurveResultantParameter::First, &mut field).unwrap();
                let local_second = local_fiber_polynomial(&second, CurveResultantParameter::First, &mut field).unwrap();
                assert!(rational_local_subresultant_rows(&local_first, &local_second, &mut field).is_none());
                let report = count_bivariate_common_fiber_roots_at_algebraic_parameter(
                    &first, &second, CurveResultantParameter::First, &root, &real(-3), &real(3), policy,
                );
                assert_eq!(report.status, AlgebraicFiberRootCountStatus::Counted);
                assert_eq!(report.distinct_root_count, Some(count));
                assert_eq!(report.certainty, Certainty::Exact);
            }
        }
    }

'''+s[a:]
assert 'rational_local_sturm_rows' not in s
assert 'TRACE_GCD' not in s
p.write_text(s)
s=(audit/'qualify-rational-sturm-20260923.py').read_text().replace('/tmp/hypercurve-contact-isolation-2026-09-23',str(root)).replace('rational-sturm-20260923-','rational-subresultants-20260923-').replace("if '::rational_sturm_' in n]; assert len(focused)==3,focused", "if '::rational_sturm_' in n or '::rational_subresultants_' in n]; assert len(focused)==5,focused")
(audit/'qualify-rational-subresultants-20260923.py').write_text(s)
