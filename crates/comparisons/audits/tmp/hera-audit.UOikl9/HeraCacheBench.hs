{-# LANGUAGE BangPatterns #-}
module Main where
import qualified Data.Number.MPFR as D
import qualified Data.Number.Real as R
import Data.Bits
import Data.Ratio
import Data.List (foldl')
import Control.Exception (evaluate)
import Control.Monad
import System.Environment
import System.CPUTime
import System.Mem
import GHC.Stats
import Text.Printf

rat :: D.Dyadic -> Rational
rat d | d == D.zero = 0
      | D.isNaN d || D.isInfinite d = error "nonfinite approximation"
      | e>=0 = fromInteger (m `shiftL` e)
      | otherwise = m % (1 `shiftL` negate e)
  where (m,e)=D.decompose d
{-# NOINLINE query #-}
query :: R.CReal -> Word -> IO Integer
query x digits = case R.approx x digits of
  Left _ -> error "requested accuracy not reached"
  Right d -> evaluate (D.getMantissa d `mod` 1000000007)
{-# NOINLINE build #-}
build :: Int -> R.CReal
build offset = foldl1 (+) [R.sqrt(R.fromDyadic(D.fromInt D.Near 256 (i*i+1))) | i<-[offset+1..offset+32]]
validate :: R.CReal -> Word -> Int -> IO ()
validate x digits offset = do
  let p = ceiling (fromIntegral digits * logBase 2 10 :: Double)+256
      lo = foldl' (D.add D.Down p) D.zero [D.sqrt D.Down p (D.fromInt D.Near p (i*i+1))|i<-[offset+1..offset+32]]
      hi = foldl' (D.add D.Up p) D.zero [D.sqrt D.Up p (D.fromInt D.Near p (i*i+1))|i<-[offset+1..offset+32]]
      epsilon = 1 % (10^digits)
  case R.approx x digits of
    Left _ -> error "validation did not reach requested accuracy"
    Right d -> unless (rat d-epsilon<=rat lo && rat hi<=rat d+epsilon) (error "independent oracle enclosure failed")
main :: IO ()
main = do
  [variant,mode,ds,rs,os] <- getArgs
  let high=read ds; reps=read rs :: Int; offset=read os
      low=8
      x=build offset
  -- Validation is outside timing and warms the retained expression.
  validate x high offset
  validate x low offset
  _ <- query x high
  performGC
  before <- getRTSStats
  start <- getCPUTime
  let loop !j !acc | j==reps = return acc
                   | otherwise = do
                     let object = if mode=="cold" then build (offset+j `mod` 7) else x
                     a <- query object (if mode=="mixed" && even j then low else high)
                     loop (j+1) (acc+a)
  checksum <- loop 0 0
  stop <- getCPUTime
  performGC
  after <- getRTSStats
  -- Keep x live through both GCs, and recheck numerical validity after history.
  validate x high offset
  validate x low offset
  printf "%s\t%s\t%d\t%d\t%d\t%.3f\t%d\t%d\t%d\n"
    variant mode high reps offset
    (fromIntegral(stop-start)/1000/fromIntegral reps :: Double)
    (allocated_bytes after-allocated_bytes before)
    (gcdetails_live_bytes (gc after)) checksum
