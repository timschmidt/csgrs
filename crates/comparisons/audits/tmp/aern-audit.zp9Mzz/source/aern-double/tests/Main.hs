{-# LANGUAGE TypeFamilies #-}

{-|
    Module      :  Main
    Description :  run all tests defined in the AERN-Real package  
    Copyright   :  (c) Michal Konecny, Jan Duracz
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
-}
module Main where

import Numeric.AERN.RealArithmetic.Basis.Double
import Numeric.AERN.RealArithmetic.Interval.Double
import Numeric.AERN.RealArithmetic.Interval
import Numeric.AERN.RealArithmetic.Interval.ElementaryFromFieldOps
import Numeric.AERN.Basics.Interval

import Numeric.AERN.Basics.Consistency
import Numeric.AERN.Basics.Arbitrary
import qualified Numeric.AERN.NumericOrder as NumOrd
import qualified Numeric.AERN.RefinementOrder as RefOrd

import Numeric.AERN.RealArithmetic.Measures
import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut

import Test.Framework (defaultMain)

main =
    do
    defaultMain tests

tests = testsDouble ++ testsDI

testsDouble =
    [
--       NumOrd.testsArbitraryTuple ("Double", sampleD, NumOrd.compare),
       NumOrd.testsPartialComparison ("Double", sampleD) areaD,
       NumOrd.testsRoundedLatticeDistributive ("Double", sampleD) areaD,
       testsDistance ("Double", sampleD),
       ArithUpDn.testsConvert ("Double", sampleD, "Integer", sampleI),
       ArithUpDn.testsConvert ("Integer", sampleI, "Double", sampleD),
       ArithUpDn.testsConvert ("Double", sampleD, "Rational", sampleR),
       ArithUpDn.testsConvert ("Rational", sampleR, "Double", sampleD),
       ArithUpDn.testsConvert ("Double", sampleD, "Double", sampleD),
       ArithUpDn.testsUpDnAdd ("Double", sampleD),
       ArithUpDn.testsUpDnSubtr ("Double", sampleD),
       ArithUpDn.testsUpDnAbs ("Double", sampleD),
       ArithUpDn.testsUpDnMult ("Double", sampleD),
       ArithUpDn.testsUpDnIntPower ("Double", sampleD),
       ArithUpDn.testsUpDnDiv ("Double", sampleD),
       ArithUpDn.testsUpDnMixedFieldOps ("Double", sampleD) ("Integer", sampleI),
       ArithUpDn.testsUpDnMixedFieldOps ("Double", sampleD) ("Rational", sampleR),
       ArithUpDn.testsUpDnMixedFieldOps ("Double", sampleD) ("Double", sampleD)
    ]

testsDI =
    [
       testsConsistency ("DI", sampleDI),
       NumOrd.testsPartialComparison ("DI", sampleDI) areaDI,
       NumOrd.testsRefinementRoundedLatticeDistributiveMonotone  ("DI", sampleDI) areaDI,
       RefOrd.testsPartialComparison  ("DI", sampleDI) areaDI, 
       RefOrd.testsRoundedBasis ("DI", sampleDI),
       RefOrd.testsRoundedLatticeDistributive ("DI", sampleDI) areaDI,
       testsDistance ("DI", sampleDI),
       testsImprecision ("DI", sampleDI),
       ArithInOut.testsConvertNumOrd ("Integer", sampleI, "DI", sampleDI),
       ArithInOut.testsConvertNumOrd ("Double", sampleD, "DI", sampleDI),
       ArithInOut.testsConvertNumOrd ("Rational", sampleR, "DI", sampleDI),
       ArithInOut.testsInOutAdd ("DI", sampleDI) areaDI,
       ArithInOut.testsInOutSubtr ("DI", sampleDI) areaDI,
       ArithInOut.testsInOutAbs ("DI", sampleDI) areaDI,
       ArithInOut.testsInOutMult ("DI", sampleDI) areaDI,
       ArithInOut.testsInOutIntPower ("DI", sampleDI) areaDI,
       ArithInOut.testsInOutDiv ("DI", sampleDI) areaDI,
       ArithInOut.testsInOutMixedFieldOps ("DI", sampleDI) ("Integer", sampleI) areaDI,
       ArithInOut.testsInOutMixedFieldOps ("DI", sampleDI) ("Rational", sampleR) areaDI,
       ArithInOut.testsInOutMixedFieldOps ("DI", sampleDI) ("Double", sampleD) areaDI,
       ArithInOut.testsInOutExp ("DI", sampleDI),
       ArithInOut.testsInOutSqrt ("DI", sampleDI) unPositiveDI
    ]

areaD = areaWhole sampleD
areaDI = areaWhole sampleDI

sampleI = 1 :: Integer
sampleR = 1 :: Rational

