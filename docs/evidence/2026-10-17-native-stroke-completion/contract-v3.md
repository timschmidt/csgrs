# Preserve the single-line construction before general arrangement

Candidate v1's area-only reuse is rejected: the original curve regressions
fall from six passes to four, including previously certified rational-length
and budget-recovery area comparisons. Mathematical equivalence alone does not
justify weakening a supported downstream proof. All source and raw failures
remain frozen. Do not move that change into another wrapper or lower API.

Candidate v2 instead constructs a certified distinct single line's butt/round
stroke as its two exact parallels and two endpoint caps, using existing native
line, arc, contour and regularization implementations. Connectivity, positive
half-width, corner-style validity and certified endpoint distinctness are
preconditions. The cap radius and antipodal sweep follow directly from the
normal displacement construction. The private helper can retain those facts.
Square caps, multiple spans and unresolved distinctness keep their original
general route. No global measurement algorithm changes.

This evaluates the archived compact construction independently. It must pass
every original curve case and preserve the existing suite and consumers;
artifact boundary subdivisions may intentionally differ, so expose that
representation migration separately from mathematical equality. It still does
not establish completed exact KiCad ingestion or admission of the full archived
integration. No source fixture, expectation, policy or resource gate is relaxed.
