-- |
-- Module:      Math.NumberTheory.Roots
-- Copyright:   (c) 2011 Daniel Fischer, 2016-2020 Andrew Lelechenko
-- Licence:     MIT
-- Maintainer:  Andrew Lelechenko <andrew.lelechenko@gmail.com>
--
-- Calculating integer roots and testing perfect powers.
--
module Math.NumberTheory.Roots
  ( -- * Square roots
    integerSquareRoot
  , isSquare
  , exactSquareRoot
  -- * Cube roots
  , integerCubeRoot
  , isCube
  , exactCubeRoot
  -- * /k/-th roots
  , integerRoot
  , isKthPower
  , exactRoot
  -- * Perfect powers
  , isPerfectPower
  , highestPower
  ) where

import Math.NumberTheory.Roots.Squares
import Math.NumberTheory.Roots.Cubes
import Math.NumberTheory.Roots.General
