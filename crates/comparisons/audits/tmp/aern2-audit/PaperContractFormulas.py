"""Independent exact-arithmetic checks of printed formulas, not a donor build.

ERC v10 Examples 2.1(7), 2.2(6); CCA2016 slide 17.
"""
from fractions import Fraction as Q
from itertools import product


def x_name(n):
    return -1 if n == 1 else 1 if n == 2 else 0


def y_name(n):
    return 0 if n == 0 else 2 if n == 1 else 1 if n == 2 else 2 ** (n - 1)


def compare_digit(n):
    x, y = x_name(n), y_name(n)
    return 1 if x + 2 < y else -1 if y + 2 < x else 0


def sticky_name(digits):
    decided = 0
    for d in digits:
        if decided:
            assert d in (0, decided)
        elif d:
            decided = d
        yield decided


def is_k_prefix(digits):
    decided = 0
    for d in digits:
        if decided and d != decided:
            return False
        if d:
            decided = d
    return True


# For n >= 3 the formulas give EXACT names of x=0, y=1/2;
# this finite check supplements that analytic tail, not an inferred tail.
for n in range(24):
    scale = Q(1, 2**n)
    assert abs(Q(0) - x_name(n) * scale) <= scale
    assert abs(Q(1, 2) - y_name(n) * scale) <= scale
raw = [compare_digit(n) for n in range(24)]
fixed = list(sticky_name(raw))
assert raw[:5] == [0, 1, 0, 1, 1]
assert not is_k_prefix(raw)
assert is_k_prefix(fixed)
print("ERC valid real names: raw Kleenean prefix", raw[:8])
print("ERC latched prefix satisfying stated delta_K shape:", fixed[:8])

# A counterexample to the stated intermediate inference, not a failed
# execution of an ERC compiler or a disproof of computability of Kond.
bottom = object()
p, q, r = {0}, {bottom}, {True}
kond_true = p
assert bottom not in kond_true
assert bottom in q
print("Kond(true,{0},{bottom})={0}: total selected output does not imply both branches are defined")

# The iteration squares the residual, NOT an unscaled absolute error.
f, inverse, proposal = Q(4), Q(1, 4), Q(3, 10)
next_proposal = 2 * proposal - proposal**2 * f
old_error = abs(proposal - inverse)
new_error = abs(next_proposal - inverse)
assert new_error == f * old_error**2
assert new_error > old_error**2
assert 1 - f * next_proposal == (1 - f * proposal)**2
print("Newton inverse absolute errors:", old_error, "->", new_error,
      "; raw error squared:", old_error**2, "; residual identity holds")

# Algorithm 1 prints y=1 for the full domain x>=0. At x=4 both the
# alleged initial bracket and the exit guarantee fail, without ambiguity
# about which nondeterministic option is available.
x, y, p = Q(4), Q(1), -4
z = x / y
tolerance = Q(1, 16)
choices = [i for i, valid in enumerate((y-z < tolerance, tolerance/2 < y-z)) if valid]
assert choices == [0]
assert abs(y - Q(2)) > tolerance
print("Printed Heron at x=4,p=-4: choose", choices, "returns", y,
      "; expected sqrt=2 within", tolerance)


def graph(values):
    """Definition 6.3: no graph values if ANY branch is bottom."""
    return set() if bottom in values else values


def composed_values(values, images):
    result = set()
    for value in values:
        result.update({bottom} if value is bottom else images[value])
    return result


def printed_composed_graph(values, images):
    result = set()
    for value in graph(values):
        result.update(graph(images[value]))
    return result


def guarded_composed_graph(values, images):
    valid_inputs = graph(values)
    if not valid_inputs or any(not graph(images[x]) for x in valid_inputs):
        return set()
    return printed_composed_graph(values, images)


subsets = [{x for i, x in enumerate((0, 1, bottom)) if mask & (1 << i)}
           for mask in range(1, 8)]
total, failures = 0, 0
for values, at_zero, at_one in product(subsets, repeat=3):
    images = {0: at_zero, 1: at_one}
    actual = graph(composed_values(values, images))
    proposed = printed_composed_graph(values, images)
    failures += actual != proposed
    assert guarded_composed_graph(values, images) == actual
    total += 1
values, images = {0, 1}, {0: {bottom}, 1: {1}}
assert composed_values(values, images) == {bottom, 1}
assert printed_composed_graph(values, images) == {1}
assert graph(composed_values(values, images)) == set()
print("Lemma 6.4 finite composition cases:", total,
      "; printed existential formula mismatches:", failures,
      "; universally guarded formula mismatches: 0")

# The printed continuity antecedent imposes x+delta <= b. Choosing
# delta > b-a makes it false for EVERY x>=a, independently of f/epsilon.
a, b = Q(0), Q(1)
delta = b-a+1
assert a + delta > b
print("Printed continuity predicate: delta=b-a+1 makes its antecedent vacuous")

# Section 7 variant decrease L=2^(p-2) is too large for an admissible
# continue choice in the overlap region. Both inner choices are valid for
# this continuous linear function, so the counterexample uses no partial f.
a, b, root = Q(0), Q(5, 8), Q(1, 4)
f = lambda x: x-root
assert f(a) < 0 < f(b)
assert Q(1, 2) < b-a < 1  # p=0, both outer choices admissible
assert f((2*a+b)/3) * f(b) < 0
assert f(a) * f((a+2*b)/3) < 0
old_variant = b-a-Q(1, 2)
new_variant = (2*(b-a)/3)-Q(1, 2)
printed_decrease = Q(1, 4)
assert new_variant > old_variant - printed_decrease
assert new_variant <= old_variant - Q(1, 8)
print("Trisection p=0,width=5/8: decrease", old_variant-new_variant,
      "is less than printed L=1/4; L=1/8 works")

# Lemma A.1 C uses the initial sigma in its guard while B (and the later
# proof) use the current delta. Compare on a two-state terminating loop.
initial = 1
live, escaped_printed, escaped_corrected = {initial}, set(), set()
for _ in range(2):
    next_live = set()
    for current in live:
        next_state = max(0, current-1)
        if current > 0:
            next_live.add(next_state)
        if not initial > 0:
            escaped_printed.add(current)
        if not current > 0:
            escaped_corrected.add(current)
    live = next_live
assert not live and not escaped_printed and escaped_corrected == {0}
print("Lemma A.1 after two unfoldings of while x>0: x:=max(0,x-1):",
      "printed escaped set empty; current-state recurrence gives {0}")
