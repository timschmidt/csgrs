"""Negative controls for the benchmark admission checker."""

from dataclasses import replace
import unittest

from benchmark_receipt import compare, expected_cases, read_receipt


class BenchmarkReceiptTests(unittest.TestCase):
    source = '''
        println!("spline_kernel/multi_section_c0_loft: {N} iterations in {t:?}");
        println!("spline_kernel/area-pair: {N} paired iterations in {t:?}");
    '''
    log = '''    Finished `bench` profile
spline_kernel/multi_section_c0_loft: 10 iterations in 10µs (1µs/iter), checksum=20
spline_kernel/area-pair: 2 paired iterations in 3ms (1.5ms/pair), checksum=4
'''

    def setUp(self):
        self.names = expected_cases(self.source)
        self.cases = read_receipt(self.log, self.names)

    def test_digits_and_punctuation_in_labels_are_retained(self):
        self.assertEqual(len(self.cases), 2)
        self.assertEqual(self.cases[0].name, 'spline_kernel/multi_section_c0_loft')
        self.assertEqual(self.cases[1].per_iteration_ns, 1_500_000)

    def test_missing_row_fails(self):
        with self.assertRaises(ValueError):
            read_receipt('\n'.join(self.log.splitlines()[:-1]), self.names)

    def test_duplicate_row_fails(self):
        with self.assertRaises(ValueError):
            read_receipt(self.log + self.log.splitlines()[1] + '\n', self.names)

    def test_extra_row_fails(self):
        with self.assertRaises(ValueError):
            read_receipt(self.log + 'extra/case: 1 iterations in 1ns (1ns/iter), checksum=0\n', self.names)

    def test_malformed_row_fails(self):
        with self.assertRaises(ValueError):
            read_receipt(self.log.replace('checksum=20', 'checksum=not-a-number'), self.names)

    def test_reordered_rows_fail(self):
        with self.assertRaises(ValueError):
            read_receipt('\n'.join(reversed(self.log.splitlines()[1:])), self.names)

    def test_changed_counts_units_and_checksums_fail(self):
        for field, value in [('iterations', 11), ('unit', 'pair'), ('checksum', 21)]:
            with self.subTest(field=field), self.assertRaises(ValueError):
                compare(self.cases, [replace(self.cases[0], **{field: value}), self.cases[1]])

    def test_inconsistent_duration_fails(self):
        with self.assertRaises(ValueError):
            read_receipt(self.log.replace('(1µs/iter)', '(2µs/iter)'), self.names)


if __name__ == '__main__':
    unittest.main()
