"""Regress independent changed-function denominators, not production execution."""
from pathlib import Path
import unittest
import coverage_common as coverage

ROOT = Path(__file__).resolve().parents[3]


class FunctionCoverageTests(unittest.TestCase):
    def report(self, missing=None, uncovered=None):
        ranges = coverage.native_function_ranges(ROOT)
        texts = {}
        for label, entries in ranges.items():
            for index, entry in enumerate(entries):
                if (label, index) == missing:
                    continue
                text = texts.setdefault(entry["path"], [])
                text.append(f'{entry["start"]}|1|fn body')
                text.append(f'{entry["start"] + 1}|{0 if (label,index) == uncovered else 1}|body')
                text.append(f'{entry["end_exclusive"]}|10000|unrelated outside the function')
        annotated = "\n".join(path+":\n"+"\n".join(lines) for path,lines in texts.items())
        return coverage.count_function_lines(annotated, ranges, ROOT)

    def test_functions_are_independent_of_unrelated_file_lines(self):
        report = self.report()
        self.assertEqual(report["native_atan2"]["physical_executable_lines"], {"count":4,"covered":4,"percent":100.0})
        self.assertEqual(report["native_atan_deferred"]["physical_executable_lines"], {"count":2,"covered":2,"percent":100.0})
        coverage.check_native_function_coverage(report)

    def test_each_floor_rejects_its_own_regression(self):
        for label in coverage.native_function_ranges(ROOT):
            with self.subTest(label=label):
                with self.assertRaisesRegex(SystemExit,label):
                    coverage.check_native_function_coverage(self.report(uncovered=(label,0)))

    def test_missing_function_measurements_cannot_be_hidden_by_the_other_function(self):
        for label,entries in coverage.native_function_ranges(ROOT).items():
            for index in range(len(entries)):
                with self.subTest(label=label,index=index):
                    with self.assertRaisesRegex(SystemExit,label):
                        self.report(missing=(label,index))


if __name__ == "__main__":
    unittest.main()
