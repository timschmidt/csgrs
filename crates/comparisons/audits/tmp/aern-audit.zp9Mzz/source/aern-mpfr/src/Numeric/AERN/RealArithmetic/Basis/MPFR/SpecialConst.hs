{-# LANGUAGE TypeFamilies #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.Basis.MPFR.SpecialConst
    Description :  common constants such as pi
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable

    Common constants such as pi.
-}

module Numeric.AERN.RealArithmetic.Basis.MPFR.SpecialConst where

import Numeric.AERN.RealArithmetic.Basis.MPFR.Effort

import Numeric.AERN.RealArithmetic.NumericOrderRounding

import qualified Data.Number.MPFR as M
import Data.Number.MPFR (MPFR)
import Data.Number.MPFR.Instances.Up

instance  
    RoundedSpecialConstEffort MPFR
    where
    type SpecialConstEffortIndicator MPFR = M.Precision
    specialConstDefaultEffort _ = 100

instance 
    RoundedSpecialConst MPFR
    where
    piUpEff prec _ = M.pi M.Up prec
    piDnEff prec _ = M.pi M.Down prec
    eUpEff prec _ = M.exp M.Up prec 1
    eDnEff prec _ = M.exp M.Down prec 1
