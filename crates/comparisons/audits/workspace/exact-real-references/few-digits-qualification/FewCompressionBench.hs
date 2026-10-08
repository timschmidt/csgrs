{-# LANGUAGE BangPatterns #-}
module Main where
import qualified Data.Real.CReal as C
import qualified Data.Real.Base as B
import Data.Ratio
import Control.Exception
import Control.Monad
import Data.List (foldl')
import System.Environment
import System.CPUTime
import System.Mem
import GHC.Stats
import Text.Printf

fibPair :: Int -> (Integer,Integer)
fibPair n=foldl' (\(!a,!b) _ -> (b,a+b)) (0,1) [1..n]
main :: IO ()
main=do
  [variant,ns,bs,ss]<-getArgs
  let n=read ns; bits=read bs::Int; seed=read ss
      (a,b)=fibPair(n+seed)
      q=a%b
      eps=1%(2^bits)
  evaluate(numerator q `seq` denominator q `seq` ())
  performGC
  before<-getRTSStats
  start<-getCPUTime
  result<-evaluate(C.approx(sin(C.inject q)) eps)
  evaluate(numerator result `seq` denominator result `seq` ())
  end<-getCPUTime
  performGC
  after<-getRTSStats
  -- This additional, untimed dyadic projection is only for compact reporting.
  -- The oracle reserves its certified error eps/65536 from the allowed budget.
  let reported=B.approxBase result (eps/65536)
  printf "%s\t%d\t%d\t%d\t%.3f\t%d\t%s\t%s\t%s\t%s\t%d\n" variant n bits seed
    (fromIntegral(end-start)/1e6::Double)
    (allocated_bytes after-allocated_bytes before)
    (show(numerator q)) (show(denominator q))
    (show(numerator reported)) (show(denominator reported))
    (length(show(denominator result)))
