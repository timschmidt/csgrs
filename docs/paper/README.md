# Hyper stack scientific manuscript

[LaTeX source](hyper-stack.tex) and [generated tables](results.tex) for
*Exact Geometry as a Compositional Contract: Design and Admission in the Hyper
Stack*. This is a working scientific and educational draft, not a submitted or
publication-ready paper. Authorship, venue, independent technical review,
a distributable reproducibility bundle and broader replication remain open.

The nine-page draft distinguishes desired contracts, inspected implementation,
elementary proofs and historical experimental observations. Its tables were
recomputed from 476 retained process observations. Unfavorable measurements,
interrupted runs, missing historical artifacts, rejected native changes and
open downstream/release work remain explicit.

From this directory, create a fresh output directory and run:

```sh
pdflatex -no-shell-escape -halt-on-error -interaction=nonstopmode \
  -output-directory=NEW_OUTPUT hyper-stack.tex
```

Repeat in the same open build directory to resolve references; inspect the final
log before accepting the render. Byte-identical PDF metadata is not promised.
The rendered PDF and previews remain in the physical locations pinned by the
[document evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/docs/evidence/2026-10-17-scientific-manuscript/README.md).
They are not duplicated in Git. Rendering this paper does not validate geometry,
admit the cache candidate or complete the plan.
