# Checkpoint 84 instrumentation correction

The original compiler gates failed before any numerical diagnostic ran:
the first harness had a const-pointer mismatch; the corrected harness compiled.
Compiling the unmodified donor translation unit then exposed a signedness warning
under `-Werror`; demoting only that warning exposed a missing system UBSan shared
runtime (`/usr/lib64/libubsan.so.1.0.0`). All failed captures remain preserved.

Use GCC's undefined-behavior trap instrumentation in place of the unavailable
reporting runtime. This still instruments the unmodified donor translation unit
and stops execution before undefined arithmetic. It does not print a UBSan
report, so a trap alone will not establish the source location: obtain debugger
evidence before classifying the suspected overflow. Preserve the original
protocol, keep the finite-limit controls, and do not run enormous unbounded
inputs on the uninstrumented binary. No system package installation, full
library rebuild or production edit is required.
