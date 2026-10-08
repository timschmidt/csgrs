{-# LANGUAGE BangPatterns #-}
module Main where
import qualified Data.Real.Base as B
import Data.Ratio
import Data.Bits
import Data.List (foldl')
import Control.Exception (evaluate)
import Control.Monad
import System.Environment
import System.CPUTime
import System.Mem
import GHC.Stats
import Text.Printf

inputs :: String -> Int -> Int -> Int -> [Rational]
inputs kind n bits seed = [num i % den i | i<-[1..n]] where
  common = 2^bits-1
  noise i = (6364136223846793005 * fromIntegral (i+seed*100003) + 1442695040888963407) .&. (2^bits-1)
  num i = if even(i+seed) then fromIntegral(i+seed) else negate(fromIntegral(i+seed))
  den i = case kind of
    "equal" -> common
    "dyadic" -> 2^(1+(i*13+seed) `mod` bits)
    "nested" -> 3^(1+(i*13+seed) `mod` bits)
    "mixed" -> (2^(bits `div` 2)+1)*(2^(bits-1)+noise i .|. 1)
    "independent" -> 2^(bits-1)+noise i .|. 1
    _ -> error "unknown denominator family"

forceQ :: Rational -> ()
forceQ q = numerator q `seq` denominator q `seq` ()
balanced :: (a -> a -> a) -> a -> [a] -> a
balanced _ zero []=zero
balanced _ _ [x]=x
balanced f zero xs=let(a,b)=splitAt(length xs `div` 2) xs
                   in f(balanced f zero a)(balanced f zero b)
sequentialLCM :: [Rational] -> Rational
sequentialLCM xs = let d=foldl' lcm 1(map denominator xs)
                  in foldl' (\s q->s+numerator q*(d `div` denominator q)) 0 xs % d
operation :: String -> [Rational] -> Rational
operation "donor" = B.sumBase
operation "seq-lcm" = sequentialLCM
operation "left" = foldl' (+) 0
operation "balanced-add" = balanced (+) 0
operation _ = error "unknown algorithm"

main :: IO ()
main=do
  [algorithm,kind,ns,bs,ss]<-getArgs
  let n=read ns; bits=read bs; seed=read ss
      xs=inputs kind n bits seed
  evaluate(foldl' (\() q->forceQ q) () xs)
  performGC
  before<-getRTSStats
  start<-getCPUTime
  result<-evaluate(operation algorithm xs)
  evaluate(forceQ result)
  end<-getCPUTime
  performGC
  after<-getRTSStats
  let oracle=foldl' (+) 0 xs
  unless(result==oracle)(error "exact sum mismatch")
  printf "%s\t%s\t%d\t%d\t%d\t%.3f\t%d\t%d\n" algorithm kind n bits seed
    (fromIntegral(end-start)/1e6::Double)
    (allocated_bytes after-allocated_bytes before)
    (length(show(denominator result)))
