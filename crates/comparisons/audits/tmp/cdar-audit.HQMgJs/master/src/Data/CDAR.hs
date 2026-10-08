{-| Convenience module importing and reexporting the 'CR' data type of
computable real numbers and the underlying data type including the centred
dyadic approximations 'Approx'.

Modules imported are:

- "Data.CDAR.Approx"
- "Data.CDAR.Classes"
- "Data.CDAR.Dyadic"
- "Data.CDAR.Extended"
- "Data.CDAR.IntegerLog"
-}
module Data.CDAR (module Approx
                 ,module Classes
                 ,module Dyadic
                 ,module Extended
                 ,module IntegerLog) where

import           Data.CDAR.Approx as Approx
import           Data.CDAR.Classes as Classes
import           Data.CDAR.Dyadic as Dyadic
import           Data.CDAR.Extended as Extended
import           Data.CDAR.IntegerLog as IntegerLog
