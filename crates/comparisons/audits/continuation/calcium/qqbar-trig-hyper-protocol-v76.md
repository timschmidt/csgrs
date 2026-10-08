# Hyper completeness comparison — checkpoint 76 supplement

Use the current, unchanged checkpoint-75 scalar source, not a candidate. Compare
sin/cos/tan/cot at all 24 pi/12 residues with separately authored radical formulas
and with direct periodic equivalents. Add 1/1024 unequal controls. For each,
use two bounded certified-equality budgets (-64 and -256), and exact period
shifts 0, 5 and 2^1024. A positive predicate must not be confused with structural
PartialEq. Check poles against the explicit NotANumber contract.

There are 1,728 ordered CSV observations per debug/release run, including 72
repeated pole observations. Build/run the small audit package in the existing
shared two-job, nonincremental target. No live source modification or broad copy.
Compare full records across profiles; record any Unknown results as unresolved
completeness observations, not errors, proved inequalities, or independent bugs.
Run formatting and warnings-denied Clippy for this audit package. This is a
bounded capability probe, not full Hyper regression, benchmark, or a proposal
to replace SinPi/TanPi with eagerly constructed radical expressions.
