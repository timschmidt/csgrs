module Data.Number.Dyadic (
  module Data.Number.MPFR,
  pow2
) where

import Data.Number.MPFR 

pow2 :: Int -> Dyadic
pow2 = int2i Near minPrec 1