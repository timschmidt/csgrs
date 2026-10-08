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

import Numeric.AERN.Poly.IntPoly
import Numeric.AERN.Poly.IntPoly.Interval

import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.Laws (testsEval, testsFnNumCompare, testsFieldPointwise)
--import Numeric.AERN.RmToRn.Evaluation

import Numeric.AERN.RealArithmetic.Basis.Double ()
import Numeric.AERN.RealArithmetic.Basis.MPFR ()
import Numeric.AERN.Basics.Interval

-- import Numeric.AERN.RealArithmetic.Interval
-- import Numeric.AERN.RealArithmetic.Interval.Mutable
-- import Numeric.AERN.RealArithmetic.Interval.ElementaryDirect
--import Numeric.AERN.RealArithmetic.Interval.ElementaryFromBasis
--import Numeric.AERN.Basics.Interval

--import Numeric.AERN.Basics.Consistency
import Numeric.AERN.Basics.Arbitrary
--import qualified Numeric.AERN.NumericOrder as NumOrd
--import qualified Numeric.AERN.RefinementOrder as RefOrd
import Numeric.AERN.RefinementOrder.Operators

--import Numeric.AERN.RealArithmetic.Measures
--import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn
--import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
--import Numeric.AERN.RealArithmetic.RefinementOrderRounding.OpsDefaultEffort

import Test.Framework (defaultMain, Test)

--import qualified Data.Map as Map

--type CF = Interval MPFR
type CF = Interval Double
type Poly = IntPoly String CF
type PI = Interval Poly

polyTypeName :: String
polyTypeName = "P(DI)"

polyIntervalTypeName :: String
polyIntervalTypeName = "PI(DI)"

main :: IO ()
main =
    do
    defaultMain tests

tests :: [Test]
tests = testsPoly

testsPoly :: [Test]
testsPoly =
    [
        testsEval (polyTypeName, samplePoly) areaPoly
        ,
        testsEval (polyIntervalTypeName, samplePI) areaPI
        ,
        testsFnNumCompare (polyTypeName, samplePoly) areaPoly
        ,
        testsFieldPointwise (polyIntervalTypeName, samplePI) areaPI
    ]

areaPI :: Area PI
areaPI = (areaPoly, AreaMaybeAllowOnlyWithConsistencyStatus Nothing)

areaPoly :: Area Poly
areaPoly = areaWhole samplePoly

--sampleD = 1 :: Double
sampleI :: Integer
sampleI = 1
sampleR :: Rational
sampleR = 1

samplePoly :: Poly
samplePoly = newConstFn limits varDoms 0

samplePI :: PI
samplePI = Interval samplePoly samplePoly

limits :: IntPolySizeLimits CF
limits =
    (defaultIntPolySizeLimits 0 () 2)
    {
        ipolylimits_cf_limits = (),
        ipolylimits_maxdeg = 4,
        ipolylimits_maxsize = 30
    } 

varDoms :: [(Var Poly, CF)]
varDoms = zip vars doms

vars :: [Var Poly]
vars = ["x", "y"]

doms :: [CF]
doms = [(0 </\> 1), 0 </\> 1]

--doms = [(0 </\> 1)]
--vars = ["x"]
        