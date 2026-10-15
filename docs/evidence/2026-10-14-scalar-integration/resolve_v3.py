"""Resolve the four integration conflicts in a fresh copy; keep both ancestors."""
import importlib.util
import json
from pathlib import Path
import re
import shutil
import sys

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location('prepare_v1', E / 'prepare_v1.py')
p = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(p)
SOURCE = p.ROOT / 'target/scalar-main-integration-v2/source'
DEST = p.ROOT / 'target/scalar-main-integration-v3/source'


def main():
    expected = p.read_map(E / 'preparation-v2/source-hashes.json.gz')
    assert p.inventory(SOURCE) == expected and not DEST.exists()
    shutil.copytree(SOURCE, DEST)
    prefix = 'crates/hyperreal/'
    pattern = re.compile(r'<<<<<<< current-main\n(.*?)=======\n(.*?)>>>>>>> fallible-prototype\n', re.S)
    resolutions = {}
    paths = {
        'fuzz/fuzz_targets/structural_representations.rs': 1,
        'src/computable/node/certified_planning_tests.rs': 1,
        'src/computable/node/tests.rs': 3,
        'src/real/arithmetic/facts.rs': 1,
    }
    for name, count in paths.items():
        path = DEST / (prefix + name)
        text = path.read_text()

        def resolve(match):
            current, candidate = match.groups()
            if name.startswith('fuzz/'):
                assert 'let cold_sign = graph.sign_until(0);' in current
                return '    let cold_sign = graph.sign_until(0);\n' + candidate
            if name.endswith('facts.rs'):
                assert not current
                return ''
            cleanup = '        crate::mpfr_test::cleanup_on_thread_exit();\n'
            assert cleanup in current
            if candidate.startswith('        use '):
                first, rest = candidate.split('\n', 1)
                return first + '\n' + cleanup + rest
            return cleanup + candidate

        text, changed = pattern.subn(resolve, text)
        assert changed == count, (name, changed)
        if name.endswith('facts.rs'):
            old = 'let approximation = self.computable_clone().approx(precision);'
            assert text.count(old) == 1
            text = text.replace(old, 'let approximation = self.computable_clone().approx(precision).ok()?;')
            old = '    /// Returns `None` if evaluation was aborted; aborted approximations are not\n    /// certificates.'
            assert text.count(old) == 1
            text = text.replace(old, '    /// Returns `None` if evaluation fails, including cancellation or exhausted\n    /// precision resources. Failed approximations are never certificates.')
        path.write_text(text)
        resolutions[prefix + name] = {'conflicts': changed, 'sha256': p.sha(path)}
    tests = DEST / (prefix + 'src/real/tests.rs')
    text = tests.read_text()
    anchor = '    #[test]\n    fn cached_rational_interval_rejects_an_aborted_warm_value()'
    assert text.count(anchor) == 1
    addition = '''    #[test]
    fn failed_interval_refinement_preserves_retained_certificates() {
        // The extreme request previously published a zero in the shared pi
        // cache. Both the ordinary and cache-only interval APIs must retain
        // the completed enclosure and sign after a typed evaluation failure.
        let pi = Real::pi();
        let scale = Rational::fraction(-3, 7).unwrap();
        for value in [pi.clone(), &pi * Real::new(scale)] {
            let fine = value.certified_rational_interval(-96).unwrap();
            let coarse = value.cached_rational_interval(-32).unwrap();
            let sign = value.certified_sign_until(0);
            assert!(matches!(sign, CertifiedRealSign::Known { .. }));
            assert!(value.certified_rational_interval(i32::MIN).is_none());
            assert!(value.cached_rational_interval(i32::MIN).is_none());
            assert_eq!(value.cached_rational_interval(-96), Some(fine.clone()));
            assert_eq!(value.certified_rational_interval(-96), Some(fine));
            assert_eq!(value.cached_rational_interval(-32), Some(coarse));
            assert_eq!(value.certified_sign_until(0), sign);
        }
        // Independent rational bounds exclude the historical poisoned zero.
        let [lower, upper] = Real::pi().certified_rational_interval(-96).unwrap();
        assert!(Rational::fraction(333, 106).unwrap() < lower);
        assert!(upper < Rational::fraction(355, 113).unwrap());
    }

'''
    tests.write_text(text.replace(anchor, addition + anchor))
    assert p.inventory(SOURCE) == expected
    p.dump(E / 'resolution-v3.json', {
        'source_directory': str(DEST), 'source_parent': 'preparation-v2/source-hashes.json.gz',
        'source_parent_sha256': p.sha(E / 'preparation-v2/source-hashes.json.gz'),
        'runner_sha256': p.sha(Path(__file__)), 'resolutions': resolutions,
        'new_regression': prefix + 'src/real/tests.rs',
        'new_regression_sha256_before_formatting': p.sha(tests),
        'scope': 'Preserve P4 fuzz checks, MPFR thread cleanup and cache-only interval semantics while adapting the ordinary interval caller to Result. Formatting and all qualification remain pending.',
        'runtime_admitted': False,
    })
    print(json.dumps({'resolved_conflicts': sum(paths.values()), 'regressions_added': 1}))


if __name__ == '__main__':
    main()
