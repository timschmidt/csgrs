# Checkpoint 84 — remaining qqbar code and generic support

Read all remaining 35 qqbar files (16 archived / 19 current) completely, then
the delegated current generic multivariate evaluator and its test, and finish
the previously partial gr/qqbar.c adapter. This is 37 new complete files and one
partial file completed, with 5,324 newly read lines. Prior ranges are not counted
twice. Source coverage does not imply every randomized upstream test was run.

Prioritize three source-derived, bounded diagnostics:

1. Compare qqbar_evaluate_fmpz_mpoly with gr_fmpz_mpoly_evaluate on a single
   monomial with a large exact exponent and x=1. The generic dispatcher guards
   exponent packing width; the qqbar wrapper only tests term count. Also exercise
   two terms and machine-sized controls, with exact successful outputs checked.
2. Instrument the unmodified roots_poly_squarefree.c translation unit with
   nonrecovering undefined-behavior sanitization, using its documented squarefree
   input contract. All coefficients sqrt(2), degree d: sqrt(2)*(x^(d+1)-1)/(x-1)
   has d distinct roots. The coefficient-conjugate product is 2^(d+1); multiplying
   by d may overflow a signed machine word. Stop at the diagnostic; never attempt
   the enormous product without instrumentation. Include finite degree-limit
   controls that reject before multiplication. No full donor library rebuild.
3. Check public gr_poly_roots failure cleanup on x^2-sqrt(2), with bounded degree
   policies and repeated calls under Memcheck. A budget failure is legitimate;
   leaked temporary root vectors are not. Do not consume failed outputs.

Reuse pinned native libraries, disable core dumps, bound diagnostic wall/CPU
time and native address space. Preserve every compiler, sanitizer, timeout and
memory failure separately from passing mathematical checks. No production edits
or performance improvements are predeclared. Add normal-input root/matrix and
multivariate checks and Hyper comparisons as evidence develops; bind those
separately. The whole original ecosystem inventory remains open.
