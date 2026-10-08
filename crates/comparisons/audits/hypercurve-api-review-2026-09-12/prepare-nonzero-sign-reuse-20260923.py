from pathlib import Path
from fractions import Fraction as Q
import io,json,math,shutil,subprocess,tarfile
workspace=Path('/home/tim/Documents/GitHub/workspace');audit=Path(__file__).resolve().parent
source=Path('/tmp/hypercurve-endpoint-sign-2026-09-23');root=Path('/tmp/hypercurve-nonzero-sign-reuse-2026-09-23');root.mkdir()
for p in source.iterdir():
 if p.is_dir() and p.name not in ['hypercurve','hypersolve','dumps']:(root/p.name).symlink_to(p.resolve(),target_is_directory=True)
repo=root/'hypersolve';repo.mkdir();archive=subprocess.check_output(['git','archive','74ad6b857e042c4f8d18f67a89f08b6668a8ad41'],cwd=workspace/'hypersolve')
with tarfile.open(fileobj=io.BytesIO(archive)) as stream:stream.extractall(repo,filter='data')
p=repo/'src/algebraic_fiber.rs';s=p.read_text();candidate=(source/'hypersolve/src/algebraic_fiber.rs').read_text()
a=s.index('    fn sign_reduced_polynomial(');b=s.index('    /// Refines the already-certified singleton interval',a)
x=candidate.index('    fn sign_reduced_polynomial(');y=candidate.index('    /// Refines the already-certified singleton interval',x)
s=s[:a]+candidate[x:y]+s[b:]
d=json.loads((source/'hypersolve/examples/endpoint_sign_probe_20260923.json').read_text())[0]
fixture=json.loads((repo/'tests/data/nonph_fillet_circle_contact.json').read_text());assert list(map(Q,d['base']))==list(map(Q,fixture['base']))
coefficients=list(map(Q,d['predicate']));den=math.lcm(*(q.denominator for q in coefficients));ints=[q.numerator*(den//q.denominator) for q in coefficients];content=math.gcd(*ints);assert content>0
(repo/'tests/data/nonph_fillet_endpoint_sign.json').write_text(json.dumps(dict(predicate=[str(c//content) for c in ints],interval=d['interval']),indent=2)+'\n')
a=s.index('    #[test]\n    fn rational_subresultants_replay_degree_41_fillet_contact')
s=s[:a]+'''    #[test]
    fn certified_nonzero_signs_share_the_refined_degree_41_endpoint() {
        // Boolean normalization compares a selected circle contact with its
        // ordinary algebraic endpoint. Its linear common factor leaves this
        // nonzero coefficient over the retained degree-41 presentation.
        let source: serde_json::Value = serde_json::from_str(include_str!(
            "../tests/data/nonph_fillet_circle_contact.json"
        )).unwrap();
        let data: serde_json::Value = serde_json::from_str(include_str!(
            "../tests/data/nonph_fillet_endpoint_sign.json"
        )).unwrap();
        let read = |value: &serde_json::Value| Real::new(value.as_str().unwrap().parse::<hyperreal::Rational>().unwrap());
        let predicate = data["predicate"].as_array().unwrap().iter().map(&read).collect::<Vec<_>>();
        let opposite = predicate.iter().map(|value| -value).collect::<Vec<_>>();
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let root = represented_root(source["base"].as_array().unwrap().iter().map(&read).collect(),
                read(&data["interval"][0]), read(&data["interval"][1]), policy);
            let mut field = LocalAlgebraicField::new(&root, policy).unwrap();
            assert_eq!(field.sign_polynomial(&predicate).unwrap(), Ordering::Less);
            let refinements = field.refinement_steps;
            // A related query uses the same certified singleton instead of
            // rebuilding a second sign proof over the same broad interval.
            assert_eq!(field.sign_polynomial(&opposite).unwrap(), Ordering::Greater);
            assert_eq!(field.refinement_steps, refinements);
            assert_eq!(field.certainty, Certainty::Exact);
        }
    }

    #[test]
    fn certified_nonzero_signs_distinguish_small_values_from_selected_zeros() {
        let epsilon = (Real::one() / Real::new(hyperreal::Rational::from_bigint(
            num::BigInt::from(1) << 192_usize
        ))).unwrap();
        let zero = vec![real(-2), real(0), real(1)];
        let positive = vec![real(-2) + &epsilon, real(0), real(1)];
        let negative = positive.iter().map(|value| -value).collect::<Vec<_>>();
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            for (lower,upper) in [(-2,-1),(1,2)] {
                // Both selected conjugates have Q(alpha)=2^-192, despite
                // the foreign factor t-3 in their defining polynomial.
                let root = represented_root(vec![real(6),real(-2),real(-3),real(1)],real(lower),real(upper),policy);
                let mut field = LocalAlgebraicField::new(&root,policy).unwrap();
                assert_eq!(field.sign_polynomial(&zero).unwrap(),Ordering::Equal);
                assert_eq!(field.sign_polynomial(&positive).unwrap(),Ordering::Greater);
                let refinements = field.refinement_steps;
                assert_eq!(field.sign_polynomial(&negative).unwrap(),Ordering::Less);
                assert_eq!(field.refinement_steps, refinements);
                assert_eq!(field.certainty,Certainty::Exact);
            }
        }
    }

'''+s[a:]
p.write_text(s)
s=(audit/'qualify-rational-subresultants-20260923.py').read_text().replace('/tmp/hypercurve-rational-subresultants-2026-09-23',str(root)).replace('rational-subresultants-20260923-','nonzero-sign-reuse-20260923-')
s=s.replace("if '::rational_sturm_' in n or '::rational_subresultants_' in n]; assert len(focused)==5,focused", "if '::certified_nonzero_signs_' in n]; assert len(focused)==2,focused")
(audit/'qualify-nonzero-sign-reuse-20260923.py').write_text(s)
print('Prepared clean candidate with exact captured coefficient fixture and refinement-reuse regressions.')
