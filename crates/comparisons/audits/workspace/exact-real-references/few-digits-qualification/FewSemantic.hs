{-# LANGUAGE BangPatterns #-}
module Main where
import qualified Data.Real.CReal as C
import qualified Data.Real.ICReal as I
import Data.Ratio
import Control.Exception
import Control.Monad
import System.IO

check :: String -> (Rational -> Rational) -> Rational -> IO ()
check label approximation truth = do
  forM_ [8,16,32,64] $ \bits -> do
    let eps=1%(2^bits)
        actual=approximation eps
    result<-try(evaluate(abs(actual-truth)<=eps)) :: IO (Either SomeException Bool)
    putStrLn $ label++"\t"++show bits++"\t"++either displayException show result++"\t"++show actual
main :: IO ()
main=do
  hSetBuffering stdout LineBuffering
  check "CReal-cos-zero" (C.approx(cos(C.inject 0))) 1
  check "ICReal-rational-cos-zero" (I.approx(cos(I.inject 0))) 1
  check "ICReal-opaque-cos-zero" (I.approx(cos(I.unsafeMkCReal(const 0)))) 1
  let x=(I.unsafeMkCReal(const(5%4))+1)/16
      root=sqrt x
  check "ICReal-sqrt-nine-sixty-fourths" (I.approx root) (3%8)
  check "ICReal-square-after-sqrt" (I.approx(I.realPowerInt root 2)) (9%64)
  check "ICReal-product-after-sqrt" (I.approx(root*I.unsafeMkCReal(const 1))) (3%8)
  check "CReal-exp-zero" (C.approx(exp(C.inject 0))) 1
  check "CReal-log-one" (C.approx(log(C.inject 1))) 0
  check "CReal-empty-sum" (C.approx(C.sumRealList [])) 0
  check "CReal-positive-cancellation-sum" (C.approx(C.sumRealList [C.inject 3,C.inject(-3)])) 0
