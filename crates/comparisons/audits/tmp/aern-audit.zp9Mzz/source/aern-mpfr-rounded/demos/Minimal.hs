{-# LANGUAGE ExistentialQuantification #-}

module Main where

import Numeric.AERN.Basics.Effort

import qualified Numeric.AERN.NumericOrder as NumOrd
import Numeric.AERN.NumericOrder.Operators

import qualified Numeric.AERN.RefinementOrder as RefOrd
import Numeric.AERN.RefinementOrder.Operators

import Numeric.AERN.RealArithmetic.ExactOps

import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn
import Numeric.AERN.RealArithmetic.NumericOrderRounding.Operators

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators

import Numeric.AERN.RealArithmetic.Basis.MPFR
import Numeric.AERN.RealArithmetic.Interval.MPFR
--import Numeric.AERN.RealArithmetic.Interval.ElementaryFromBasis
import Numeric.AERN.RealArithmetic.Interval.ElementaryFromFieldOps

import Numeric.AERN.Basics.Interval
import Numeric.AERN.Basics.ShowInternals
import Numeric.AERN.Basics.SizeLimits

import Control.Concurrent

main =
    do
    putStrLn $ "oo - oo = " ++ showInternals shouldShowInternals inftyMinusInftyOut
    putStrLn $ "0</>0 = " ++ showInternals shouldShowInternals zeroOverZeroOut
    putStrLn $ "0>/<0 = " ++ showInternals shouldShowInternals zeroOverZeroIn
    putStrLn $ "exp 1 = \n" ++ (unlines $ map (showInternals shouldShowInternals) exp1efforts)
    putStrLn $ "exp 3 = \n" ++ (unlines $ map (showInternals shouldShowInternals) exp3efforts)
    putStrLn $ "exp 10 = \n" ++ (unlines $ map (showInternals shouldShowInternals) exp10efforts)
    putStrLn $ "exp 100 = \n" ++ (unlines $ map (showInternals shouldShowInternals) exp100efforts)
    putStrLn $ "exp 1000 = \n" ++ (unlines $ map (showInternals shouldShowInternals) exp1000efforts)
    return ()

shouldShowInternals = (Just 100, False)

precision :: MPFRPrec
precision = 100

sample :: MI
sample = sampleMIWithPrec precision

inftyMinusInftyOut :: MI
inftyMinusInftyOut =
    plusInfinity sample <-> plusInfinity sample

zeroOverZeroOut :: MI
zeroOverZeroOut =
    zero sample </> zero sample

zeroOverZeroIn :: MI
zeroOverZeroIn =
    zero sample >/< zero sample

oneOverZeroOut :: MI
oneOverZeroOut =
    one sample </> zero sample

oneOverZeroIn :: MI
oneOverZeroIn =
    one sample >/< zero sample

exp1efforts :: [MI]
exp1efforts =
    map (\n -> ArithInOut.expOutEff (expEffort n) (one sample)) [1..20]
    
exp3efforts :: [MI]
exp3efforts =
    map (\n -> ArithInOut.expOutEff (expEffort n) three) [1..20]

exp10efforts :: [MI]
exp10efforts =
    map (\n -> ArithInOut.expOutEff (expEffort n) ten) [1..20]

exp100efforts :: [MI]
exp100efforts =
    map (\n -> ArithInOut.expOutEff (expEffort n) hundred) [1..20]

exp1000efforts :: [MI]
exp1000efforts =
    map (\n -> ArithInOut.expOutEff (expEffort n) thousand) [1..20]

three, ten, hundred, thousand :: MI

[three, ten, hundred, thousand] = 
    map (ArithInOut.convertOutEff () sample) ([3,10,100,1000] :: [Int])

    
expEffort n =
    expEffortDefault
    {
        expeff_taylorDeg = n
    }

expEffortDefault = 
   ArithInOut.expDefaultEffort three


   