#!/bin/sh
set -e
(cd aern-order; cabal install --force-reinstalls)
(cd aern-real; cabal install --force-reinstalls)
(cd aern-interval; cabal install --force-reinstalls)
(cd aern-double; cabal install --force-reinstalls)
(cd aern-realfn; cabal install --force-reinstalls)
(cd aern-realfn-plot-gtk; cabal install --force-reinstalls)
(cd aern-poly; cabal install --force-reinstalls)
(cd aern-poly-plot-gtk; cabal install --force-reinstalls)
(cd aern-ivp; cabal install --force-reinstalls)
