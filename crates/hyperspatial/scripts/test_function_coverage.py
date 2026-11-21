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

    def test_separate_executables_union_their_measurements(self):
        ranges = coverage.native_function_ranges(ROOT)
        entry = ranges["native_atan2"][0]
        body = entry["start"] + 1
        # Every other function is fully executed in a third executable.
        others = [(label, index) for label, entries in ranges.items()
                  for index in range(len(entries)) if (label, index) != ("native_atan2", 0)]
        rest = "\n".join(
            ranges[label][index]["path"] + ":\n" + f'{ranges[label][index]["start"]}|1|fn body'
            for label, index in others)
        def listing(hit, extra=()):
            rows = [f'{entry["start"]}|1|fn body', f'{body}|{hit}|body', *extra]
            return entry["path"] + ":\n" + "\n".join(rows)
        def row(listings):
            files = coverage.count_function_lines(listings + [rest], ranges, ROOT)["native_atan2"]["files"]
            measured = next(r for rows in files.values() for r in rows if r["start"] == entry["start"])
            return measured["count"], measured["covered"], measured["uncovered"]
        # An unexecuted copy in one executable does not mask the executed
        # copy in another, in either order.
        self.assertEqual(row([listing(0), listing(3)]), (2, 2, []))
        self.assertEqual(row([listing(3), listing(0)]), (2, 2, []))
        # A line mapped only by an unexecuted copy still joins the denominator.
        extra = [f'{body + 1}|0|unexecuted elsewhere']
        self.assertEqual(row([listing(1), listing(1, extra)]), (3, 2, [body + 1]))

    def test_missing_function_measurements_cannot_be_hidden_by_the_other_function(self):
        for label,entries in coverage.native_function_ranges(ROOT).items():
            for index in range(len(entries)):
                with self.subTest(label=label,index=index):
                    with self.assertRaisesRegex(SystemExit,label):
                        self.report(missing=(label,index))


if __name__ == "__main__":
    unittest.main()
