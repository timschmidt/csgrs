{-# OPTIONS_GHC -F -pgmF htfpp #-}

import Test.Framework
import Control.Exception
import Distribution.TestSuite
import Data.Approximate.Floating.MPFR
import Data.Approximate.ApproximateField
import Data.Reals.Reals
import Data.Reals.Space

assertIn a b x = assertBoolVerbose 
     ("Result " ++ show x ++ " in interval [" ++ show a ++ "," ++ show b ++ "]")
      (x > a && x < b)


test_1 = do
       putStrLn $ "a = " ++ show a
       putStrLn $ "a * (1-a) = " ++ show (a * (1-a))
       assertIn (-0.4) (-0.38) $ a * (1-a)

       --putStrLn $ "a^200 = " ++ show (a ^ 200)
       assertIn 6.144e22 6.149e22 (a ^ 200)
       
  where
       prod = a * (1-a)
       a = 1.3 :: RealNum

test_2 = do
      putStrLn $ "a = " ++ show a
      putStrLn $ "b = " ++ show b
      assertBoolVerbose "forall x:[0,1].x*(1-x)<0.26" (force a) 
      assertBoolVerbose "forall x:[0,1].x*(1-x)<0.24" (not $ force b)      
    where
      i = ClosedInterval (appFromInteger 0, appFromInteger 1)
      a = forall i $ \x -> (x * (1 - x)) `less` (0.26 :: RealNum)
      b = forall i $ \x -> (x * (1 - x)) `less` (0.24 :: RealNum)



main = htfMain htf_thisModulesTests

