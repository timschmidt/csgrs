"""Independent contract witnesses for the maintained comparison oracle."""
import copy
from fractions import Fraction as Q
import unittest

from validate import CASES, horizon, oracle, series, sqrt_bounds, validate

FIELDS = ("constant", "exponential", "riccati", "oscillator", "time_growth",
          "turnaround", "noncommuting", "two_rate", "oscillator_long", "riccati_long")


def dyadic(pair):
    scale = 1 << 160
    lo, hi = pair
    return [str(lo * scale // 1), str(-(-hi * scale // 1)), -160]


def synthetic_corpus():
    """Outward dyadic oracle hulls for validator tests, never solver certificates."""
    rows = []
    for kind, box in CASES:
        initial, end, tube = oracle(kind, box)
        end_time, cap, _ = horizon(kind)
        pieces = int(end_time / cap)
        rows.append({"kind": "certificate", "solver": "test_only", "case": f"{kind}_{'box' if box else 'point'}",
                     "status": "certified", "encoding": "dyadic", "pieces": pieces,
                     "start": dyadic((Q(0), Q(0))), "end_time": dyadic((end_time, end_time)),
                     "time_domains": [dyadic((i * cap, i * cap)) for i in range(pieces + 1)],
                     "initial": [dyadic(v) for v in initial], "end": [dyadic(v) for v in end],
                     "tube": [dyadic(v) for v in tube]})
    return rows


class OracleContract(unittest.TestCase):
    def test_all_original_and_extended_fields_have_both_initial_sets(self):
        self.assertEqual(CASES, [(kind, box) for kind in FIELDS for box in (False, True)])

    def test_turnaround_endpoint_does_not_cover_interior_maximum(self):
        for box in (False, True):
            initial, end, tube = oracle("turnaround", box)
            self.assertEqual(end, initial)
            self.assertEqual(tube, [(initial[0][0], initial[0][1] + Q(1, 16))])

    def test_noncommuting_polynomial_has_an_interior_minimum(self):
        initial, end, tube = oracle("noncommuting", False)
        self.assertEqual(initial, [(1, 1), (0, 0), (1, 1)])
        self.assertEqual(end, [(Q(385, 384), Q(385, 384)), (Q(1, 32), Q(1, 32)), (1, 1)])
        self.assertEqual(tube, [(1, Q(385, 384)), (0, Q(1, 32)), (1, 1)])
        initial, end, tube = oracle("noncommuting", True)
        a, b = initial[0]
        r, t = Q(1, 1024), Q(1, 32)
        interior = a - r * t + a * t**3 / 6
        self.assertLess(interior, a)
        self.assertLessEqual(tube[0][0], interior)
        self.assertGreater(end[0][0], a)
        self.assertEqual(end[0], (a-r/4+a/Q(384), b+r/4+b/Q(384)))

    def test_time_growth_and_two_decay_rates(self):
        initial, end, tube = oracle("time_growth", False)
        self.assertEqual(initial, [(1, 1)])
        self.assertTrue(1 + Q(1, 32) < end[0][0] <= end[0][1] < Q(32, 31))
        self.assertEqual(tube, [(1, end[0][1])])
        initial, end, tube = oracle("two_rate", False)
        self.assertEqual(initial, [(1, 1), (1, 1)])
        self.assertTrue(Q(3, 4) < end[0][0] <= end[0][1] < Q(4, 5))
        self.assertTrue(Q(1, 8) < end[1][0] <= end[1][1] < Q(1, 7))
        self.assertEqual(tube, [(v[0], 1) for v in end])

    def test_original_series_identities_and_sqrt_bounds(self):
        self.assertEqual(series("exp", Q(0)), (1, 1))
        self.assertEqual(series("cos", Q(0)), (1, 1))
        self.assertEqual(series("sin", Q(0)), (0, 0))
        lo, hi = series("exp", Q(2))
        self.assertTrue(7 < lo <= hi < 8)
        for numerator in range(50):
            for denominator in range(1, 31):
                x = Q(numerator, denominator)
                lo, hi = sqrt_bounds(x)
                self.assertTrue(lo >= 0 and lo*lo <= x <= hi*hi)
                self.assertLessEqual(hi-lo, Q(1, 1 << 180))

    def test_validator_requires_new_dimensions_extrema_and_step_cap(self):
        rows = synthetic_corpus()
        self.assertEqual(len(validate(rows)), 20)
        cases = {row["case"]: i for i, row in enumerate(rows)}
        for name in ("turnaround_point", "noncommuting_box"):
            broken = copy.deepcopy(rows)
            row = broken[cases[name]]
            initial, end, _ = oracle(name.rsplit('_', 1)[0], name.endswith('box'))
            row["tube"][0] = dyadic((min(initial[0][0], end[0][0]), max(initial[0][1], end[0][1])))
            with self.assertRaises(AssertionError):
                validate(broken)
        broken = copy.deepcopy(rows)
        broken[cases["noncommuting_point"]]["end"].pop()
        with self.assertRaises(AssertionError):
            validate(broken)
        broken = copy.deepcopy(rows)
        row = broken[cases["two_rate_point"]]
        row["pieces"] = 4
        row["time_domains"] = [dyadic((Q(i, 16), Q(i, 16))) for i in range(5)]
        with self.assertRaises(AssertionError):
            validate(broken)

    def test_long_rotation_crosses_a_full_turn_and_long_steps_are_capped(self):
        initial, end, tube = oracle("oscillator_long", False)
        self.assertEqual(initial, [(1, 1), (0, 0)])
        cos, sin = series("cos", Q(8)), series("sin", Q(8))
        self.assertTrue(Q(-15, 100) < cos[0] <= cos[1] < Q(-14, 100))
        self.assertTrue(Q(98, 100) < sin[0] <= sin[1] < Q(99, 100))
        self.assertEqual(end, [cos, (-sin[1], -sin[0])])
        self.assertEqual(tube, [(-1, 1), (-1, 1)])
        rows = synthetic_corpus()
        cases = {row["case"]: i for i, row in enumerate(rows)}
        broken = copy.deepcopy(rows)
        row = broken[cases["oscillator_long_box"]]
        # A hull of the initial and final sets misses the turn's extrema.
        row["tube"] = [dyadic((min(a[0], b[0]), max(a[1], b[1])))
                       for a, b in zip(*oracle("oscillator_long", True)[:2])]
        with self.assertRaises(AssertionError):
            validate(broken)
        broken = copy.deepcopy(rows)
        row = broken[cases["riccati_long_point"]]
        row["pieces"] = 4
        row["time_domains"] = [dyadic((Q(i, 8), Q(i, 8))) for i in range(5)]
        with self.assertRaises(AssertionError):
            validate(broken)

    def test_unknown_field_is_not_silently_an_oscillator(self):
        with self.assertRaises(ValueError):
            oracle("unknown", False)


if __name__ == "__main__":
    unittest.main()
