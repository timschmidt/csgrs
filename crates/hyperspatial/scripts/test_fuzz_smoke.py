"""Reject successful libFuzzer exits that never leave seed initialization."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("fuzz_smoke", Path(__file__).with_name("fuzz-smoke.py"))
fuzz_smoke = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fuzz_smoke)


class MutationEvidenceTests(unittest.TestCase):
    def test_actual_seed_only_epsilon_run_is_rejected(self):
        log = "#310\tINITED cov: 10006 ft: 33292 corp: 182/1092b\n#310\tDONE cov: 10006 ft: 33292 corp: 182/1092b\nDone 310 runs in 1912 second(s)\n"
        self.assertEqual(fuzz_smoke.mutation_evidence(log), {
            "initialized_runs": 310, "completed_runs": 310, "mutated": False,
        })

    def test_completed_mutation_is_recorded(self):
        log = "#261\tINITED cov: 9253 ft: 30656\n#512\tNEW cov: 9261 ft: 31999\n#746\tDONE cov: 9261 ft: 31999\nDone 746 runs in 1804 second(s)\n"
        self.assertEqual(fuzz_smoke.mutation_evidence(log), {
            "initialized_runs": 261, "completed_runs": 746, "mutated": True,
        })

    def test_incomplete_fixed_input_and_out_of_order_logs_are_rejected(self):
        for log in ["", "Executed fixed input in 12084 ms", "#16 pulse cov: 100", "#18 INITED cov: 100", "#42 DONE cov: 100", "#19 DONE\n#18 INITED", "#42 INITED\n#41 DONE"]:
            with self.subTest(log=log):
                self.assertFalse(fuzz_smoke.mutation_evidence(log)["mutated"])


if __name__ == "__main__":
    unittest.main()
