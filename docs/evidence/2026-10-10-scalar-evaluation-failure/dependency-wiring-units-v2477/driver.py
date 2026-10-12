import importlib.util
import pathlib
import sys
import unittest
sys.dont_write_bytecode = True
proposal, legacy = map(pathlib.Path, sys.argv[1:])
sys.path.insert(0, str(proposal))
import verus_dependencies
assert pathlib.Path(verus_dependencies.__file__).resolve() == (proposal / 'verus_dependencies.py').resolve()
import test_verified_export
spec = importlib.util.spec_from_file_location('legacy_identity', legacy)
old = importlib.util.module_from_spec(spec)
spec.loader.exec_module(old)
assert old.dependencies is verus_dependencies
suite = unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromModule(old), unittest.defaultTestLoader.loadTestsFromModule(test_verified_export)])
assert suite.countTestCases() == 44, suite.countTestCases()
result = unittest.TextTestRunner(verbosity=2).run(suite)
raise SystemExit(0 if result.wasSuccessful() else 1)
