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

import Numeric.AERN.RealArithmetic.Basis.Double
import Numeric.AERN.RealArithmetic.Interval.Double
import Numeric.AERN.RealArithmetic.Interval.ElementaryFromFieldOps

import Numeric.AERN.Basics.ShowInternals

import Control.Concurrent

main =
    do
--    setMachineRoundingModeUp 
    -- not sure why the above is sometimes needed...
    -- the unsafePerformIO embedded in rounded ops sometimes 
    -- does not work when run standalone, 
    -- but they seem to work when run from ghci 
--    putStrLn $ "big = " ++ showInternals shouldShowInternals big
--    putStrLn $ "t1 = " ++ showInternals shouldShowInternals t1
--    putStrLn $ "t2 = " ++ showInternals shouldShowInternals t2
    putStrLn $ "exp 1 = \n" ++ (unlines $ map (showInternals shouldShowInternals) exp1efforts)
    putStrLn $ "exp 3 = \n" ++ (unlines $ map (showInternals shouldShowInternals) exp3efforts)
    putStrLn $ "exp 10 = \n" ++ (unlines $ map (showInternals shouldShowInternals) exp10efforts)
    putStrLn $ "exp 100 = \n" ++ (unlines $ map (showInternals shouldShowInternals) exp100efforts)
    putStrLn $ "exp 1000 = \n" ++ (unlines $ map (showInternals shouldShowInternals) exp1000efforts)
    return ()

shouldShowInternals = False

exp1efforts :: [DI]
exp1efforts =
    map (\n -> ArithInOut.expOutEff (expEffort n) c1) [1..20]
    
exp3efforts :: [DI]
exp3efforts =
    map (\n -> ArithInOut.expOutEff (expEffort n) c3) [1..20]

exp10efforts :: [DI]
exp10efforts =
    map (\n -> ArithInOut.expOutEff (expEffort n) c10) [1..20]

exp100efforts :: [DI]
exp100efforts =
    map (\n -> ArithInOut.expOutEff (expEffort n) c100) [1..20]

exp1000efforts :: [DI]
exp1000efforts =
    map (\n -> ArithInOut.expOutEff (expEffort n) c1000) [1..20]

c1, c3, c10, c100, c1000 :: DI

[c1,c3,c10,c100,c1000] = 
    map (ArithInOut.convertOutEff () 0) ([1,3,10,100,1000] :: [Int])

    
expEffort n =
    expEffortDefault
    {
        expeff_taylorDeg = n
    }

expEffortDefault = 
   ArithInOut.expDefaultEffort t2

big = 10E200

t1 :: Double
t1 = 1 +^ big

t2 :: DI
t2 = 
   (1 :: Int)
   |<+> 
   (ArithInOut.convertOutEff () 0 big)

   