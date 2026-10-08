#!/usr/bin/env python3

import builtins
import sys

from pyariadne import *


def semantic_probe():
    one = RoundedFloatDP(Rational(1), dp)
    two_value = RoundedFloatDP(Rational(2), dp)
    print("rounded_max", max(one, two_value))
    print("rounded_min", min(one, two_value))

    scalar = FloatDPApproximation(10.0, dp)
    differential = FloatDPApproximationDifferential
    vector = FloatDPApproximationDifferentialVector(2, 1, 1, dp)
    vector[0] = differential.constant(1, 1, scalar)
    vector[1] = differential.constant(1, 1, scalar)
    offset = FloatDPApproximationVector([1.0, 2.0], dp)
    print("vector_plus", (vector + offset).value())
    print("vector_minus", (vector - offset).value())
    print("vector_reverse_minus", (offset - vector).value())

    decimals = [Decimal("1.2") for _ in range(8)]
    decimal_hashes = [hash(value) for value in decimals]
    print("decimal_equal", all(bool(decimals[0] == value) for value in decimals))
    print("decimal_hashes", decimal_hashes)
    print("decimal_distinct_hashes", len(set(decimal_hashes)))
    print("decimal_dict_size", len({value: index for index, value in enumerate(decimals)}))

    events = [DiscreteEvent("same") for _ in range(8)]
    event_hashes = [hash(value) for value in events]
    print("event_equal", all(bool(events[0] == value) for value in events))
    print("event_hashes", event_hashes)
    print("event_distinct_hashes", len(set(event_hashes)))
    print("event_dict_size", len({value: index for index, value in enumerate(events)}))

    for _ in range(10000):
        Float[DP](Dyadic(1), dp)
        Bounds[FloatDP](ValidatedNumber(Rational(1)), dp)
    print("generic_constructor_stress", "passed")


def invalid_negative_index_probe():
    vector = RationalVector([1, 2])
    print("valid_negative_index", vector[-1], flush=True)
    print("invalid_negative_index", vector[-3], flush=True)


if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1] == "invalid-index":
        invalid_negative_index_probe()
    else:
        semantic_probe()
