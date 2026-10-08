# Checkpoint 80 — inverse rational-angle recognition

Read both pins' asin_pi, acos_pi, atan_pi, acot_pi and log_pi_i source/tests
independently. Both log implementations and both manual sections are rereads.
The private declaration header adds 21 new lines: nineteen new complete files,
1,738 uniquely new lines (878 archived, 860 current). Do not credit declarations
as completion of the other declared routines or infer archived execution.

Reuse the existing FLINT/GMP/MPFR libraries and bind source/configuration hashes
before compilation. No production/candidate/donor change or library rebuild.
Use two small executables in one dedicated temporary directory.

Main deterministic corpus: 1,081 input cases, each in initial and explicitly
256-bit cached states, giving 2,162 rows plus a terminal record. The 1,008 angle
cases use sin/cos/exp at all 24 pi/12 residues, tan/cot at all 48 pi/24 residues,
period offsets -3/0/5 and unreduced factors 1/3. Add eight golden-ratio inputs,
twelve cubic-cosine inputs, eight signed 2^-100 near-matches, and 45 rational,
quadratic, imaginary and domain controls. Check all poles directly.

Independent oracles: Q(sqrt(2),sqrt(3)) Galois products and exact signs; Q(sqrt(5))
quadratic norms/signs; and the irreducible cubic 8X^3+4X^2-4X-1 with three disjoint
rational isolating intervals and the seventh-root cyclotomic trace identity.
Check complete primitive minimal polynomials, both enclosure components, width
<=2^-96, canonical reduced fractions and the specified principal branches.
For cubic inputs, endpoint polynomial signs independently bracket the selected
root. Root-of-unity polynomials come from exact division of X^n-1. No donor
inverse/forward round trip or approximate overlap is the correctness oracle.

Near-match inputs differ exactly from the authored rational-angle values. Record
the finite 64-bit candidate stage and its containment flag; check that exact
recognition rejects the perturbed value even if the candidate stage overlaps.
Never inspect p/q after a failed inverse call; only root-of-unity recognition,
not these wrappers, explicitly permits NULL output pointers. Main native and
Memcheck outputs must match; failed/corrupted records stay recorded.

Separate completeness hypothesis: x=tan(pi/3360), tested through the public
qqbar_atan_pi API. Bound the native process to 1 GiB address space, 30 CPU seconds
and 45 wall seconds. Independently prove its exact angle by checking that its
polynomial divides Im((1+iX)^3360), and that its narrow real enclosure brackets
a polynomial root in (1/1200,1/1000). The interval contains only tan(pi/3360),
using 3<pi<22/7, monotonicity of tan, sin(t)<=t and cos(t)>=1-t^2/2. A zero
recognition flag is not accepted as proof of a nonrational angle. Do not broaden
this into large-degree, near-word-boundary or corrupted-representation testing.

Compare corresponding live Hyper inverse certificates and bounded predicates,
including explicit radical inputs and the 3360-denominator case. No standalone
Real acot API was found; any tested reciprocal-atan construction must be labeled
as such, with the donor's zero/negative branch made explicit. Performance and
retention claims require their own matched campaigns; these collectors are
correctness/completeness probes, not benchmarks. Full ecosystem scope stays open.
