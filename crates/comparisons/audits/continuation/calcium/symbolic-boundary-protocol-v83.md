# Checkpoint 83 — source audit and expression-boundary diagnostics

Twenty-six new files are read in full: both pins' abs/im/re/re_im/sgn and their
abs/abs2/re_im/sgn tests, get_fexpr/set_fexpr and get_fexpr/get_fexpr_formula tests.
Read coverage is distinct from executing every upstream test or path. Current
FLINT and Hyper sources remain unchanged; reuse pinned native libraries/builds.

Prioritize suspected false accepted values at the public qqbar_set_fexpr boundary.
The main native corpus contains 224 rational-Pi angle cases (seven operations,
eight exponent sizes, two signs, two initialized destinations) and 84 explicit/
absent-Pi controls. Large coefficients are 2^e, or (2^e+1)/2 for cot/csc, so exact
period reduction gives a cheap independent oracle without constructing large
cyclotomic fields. Operations are Sin/Cos/Tan/Cot/Sec/Csc and Exp(i*x).

Every successful output supplies its minimal polynomial and both component
enclosures. Failed outputs are never consumed. For absent-Pi nonzero arguments,
use exact rational Taylor bounds at |x|<=1 to distinguish actual radian values
from Pi-scaled results. Such expressions are outside the listed algebraic subset:
the defect sought is a false successful conversion, not failure to support them.
Large explicit-Pi expressions are within the listed algebraic input form.

Separately test Pow(1,2), Pow(1,2^80) and Pow(1,1/2^80) under Memcheck at bounded
repetition counts. Denied exponent budgets may legitimately return failure, but
must not leak temporary big integers. Preserve expected diagnostic nonzero exits
and their exact allocation/loss records; never label a leak report a clean gate.

Malformed AlgebraicNumberSerialized inputs are not executed: the manual explicitly
requires valid data without checking. The nested Pow arity check is recorded as a
source-level trust-boundary concern, not a demonstrated supported-input defect.
No private parser is called. No library rebuild, source copy or production edit.
Use 60 wall seconds / 30 CPU seconds / 1 GiB for native cases, 120 wall seconds
for Memcheck without an instrumentation-hostile address-space cap.

Additional component/roundtrip and Hyper capability gates, if run, must be bound
separately. This protocol does not predeclare successful results, a production
transfer, complete ecosystem coverage or a benchmark win.
