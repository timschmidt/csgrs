"""Timing completeness and assertion-mode rejection witnesses."""
import copy
from pathlib import Path
import subprocess
import sys
import unittest

import run


class RunnerContract(unittest.TestCase):
    def test_optimized_python_cannot_disable_the_acceptance_oracle(self):
        result = subprocess.run([sys.executable, "-O", "-c", "from validate import validate; validate([])"],
                                cwd=Path(__file__).parent, capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("assertions enabled", result.stderr)

    def test_each_case_mode_sample_and_actual_work_must_be_present(self):
        cases = ["constant_point", "time_growth_box", "two_rate_point"]
        rows = [{"kind": "timing", "solver": "hyperdynamics", "case": c, "mode": mode,
                 "sample": sample, "iterations": 5, "total_ns": 1000}
                for c in cases for mode in ("reuse", "fresh") for sample in range(2)]
        run.validate_samples(rows, "hyperdynamics", cases, 5, 2)
        mutations = [lambda r: r.pop(), lambda r: r.__setitem__(0, copy.deepcopy(r[1])),
                     lambda r: r[0].__setitem__("solver", "capd"),
                     lambda r: r[0].__setitem__("case", "unknown"),
                     lambda r: r[0].__setitem__("mode", "other"),
                     lambda r: r[0].__setitem__("sample", 2),
                     lambda r: r[0].__setitem__("sample", False),
                     lambda r: r[0].__setitem__("iterations", 4),
                     lambda r: r[0].__setitem__("total_ns", 0),
                     lambda r: r[0].__setitem__("total_ns", -1),
                     lambda r: r[0].__setitem__("total_ns", True)]
        for mutate in mutations:
            invalid = copy.deepcopy(rows)
            mutate(invalid)
            with self.assertRaises(AssertionError):
                run.validate_samples(invalid, "hyperdynamics", cases, 5, 2)


if __name__ == "__main__":
    unittest.main()
