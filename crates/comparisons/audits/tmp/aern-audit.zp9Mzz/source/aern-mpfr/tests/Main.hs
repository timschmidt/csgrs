{-# LANGUAGE TypeFamilies #-}
{-|
    Module      :  Main
    Description :  run all tests defined in the AERN-Real package  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
-}
module Main where

import Numeric.AERN.RealArithmetic.Basis.MPFR
import Numeric.AERN.RealArithmetic.Interval.MPFR
import Numeric.AERN.RealArithmetic.Interval
-- import Numeric.AERN.RealArithmetic.Interval.ElementaryDirect
import Numeric.AERN.RealArithmetic.Interval.ElementaryFromBasis
import Numeric.AERN.Basics.Interval

import Numeric.AERN.Basics.Consistency
import Numeric.AERN.Basics.Arbitrary (areaWhole)
import qualified Numeric.AERN.NumericOrder as NumOrd
import qualified Numeric.AERN.RefinementOrder as RefOrd

import Numeric.AERN.RealArithmetic.Measures
import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut

import Test.Framework (defaultMain)

main =
    do
    defaultMain tests

tests = testsMPFR ++ testsMI

testsMPFR =
    [
--       NumOrd.testsArbitraryTuple ("MPFR", sampleM, NumOrd.compare),
       NumOrd.testsPartialComparison ("MPFR", sampleM) areaM,
       NumOrd.testsRoundedLatticeDistributive ("MPFR", sampleM) areaM,
       testsDistance ("MPFR", sampleM),
       ArithUpDn.testsConvert ("MPFR", sampleM, "Integer", sampleI),
       ArithUpDn.testsConvert ("Integer", sampleI, "MPFR", sampleM),
       ArithUpDn.testsConvert ("MPFR", sampleM, "Rational", sampleR),
       ArithUpDn.testsConvert ("Rational", sampleR, "MPFR", sampleM),
       ArithUpDn.testsConvert ("Double", sampleD, "MPFR", sampleM),
       ArithUpDn.testsConvert ("MPFR", sampleM, "Double", sampleD),
       ArithUpDn.testsUpDnAdd ("MPFR", sampleM),
       ArithUpDn.testsUpDnSubtr ("MPFR", sampleM),
       ArithUpDn.testsUpDnAbs ("MPFR", sampleM),
       ArithUpDn.testsUpDnMult ("MPFR", sampleM),
       ArithUpDn.testsUpDnIntPower ("MPFR", sampleM),
       ArithUpDn.testsUpDnDiv ("MPFR", sampleM),
       ArithUpDn.testsUpDnMixedFieldOps ("MPFR", sampleM) ("Integer", sampleI),
       ArithUpDn.testsUpDnMixedFieldOps ("MPFR", sampleM) ("Rational", sampleR),
       ArithUpDn.testsUpDnMixedFieldOps ("MPFR", sampleM) ("Double", sampleD),
       ArithUpDn.testsUpDnExp ("MPFR", sampleM)
    ]

testsMI =
    [
       testsConsistency ("MI", sampleMI),
       NumOrd.testsPartialComparison ("MI", sampleMI) areaMI,
       NumOrd.testsRefinementRoundedLatticeDistributiveMonotone ("MI", sampleMI) areaMI,
       RefOrd.testsPartialComparison  ("MI", sampleMI) areaMI, 
       RefOrd.testsRoundedBasis ("MI", sampleMI),
       RefOrd.testsRoundedLatticeDistributive ("MI", sampleMI) areaMI,
       testsDistance ("MI", sampleMI),
       testsImprecision ("MI", sampleMI),
       ArithInOut.testsConvertNumOrd ("Integer", sampleI, "MI", sampleMI),
       ArithInOut.testsConvertNumOrd ("Double", sampleD, "MI", sampleMI),
       ArithInOut.testsConvertNumOrd ("Rational", sampleR, "MI", sampleMI),
       ArithInOut.testsInOutAdd ("MI", sampleMI) areaMI,
       ArithInOut.testsInOutSubtr ("MI", sampleMI) areaMI,
       ArithInOut.testsInOutAbs ("MI", sampleMI) areaMI,
       ArithInOut.testsInOutMult ("MI", sampleMI) areaMI,
       ArithInOut.testsInOutIntPower ("MI", sampleMI) areaMI,
       ArithInOut.testsInOutDiv ("MI", sampleMI) areaMI,
       ArithInOut.testsInOutMixedFieldOps ("MI", sampleMI) ("Integer", sampleI) areaMI,
       ArithInOut.testsInOutMixedFieldOps ("MI", sampleMI) ("Rational", sampleR) areaMI,
       ArithInOut.testsInOutMixedFieldOps ("MI", sampleMI) ("Double", sampleD) areaMI
--       ,
--       ArithInOut.testsInOutExp ("MI", sampleMI),
--       ArithInOut.testsInOutSqrt ("MI", sampleMI) unPositiveMI
    ]

areaM = areaWhole sampleM
areaMI = areaWhole sampleMI

sampleD = 1 :: Double
sampleI = 1 :: Integer
sampleR = 1 :: Rational
