{-# LANGUAGE BangPatterns #-}
module Main where
import qualified Data.Number.MPFR as D
import qualified Data.Number.Ball as B
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
      | e>=0 = fromInteger(m `shiftL` e)
      | otherwise = m % (1 `shiftL` negate e)
  where (m,e)=D.decompose d
-- The full-radius control reconstructs Ball.add's unchanged formula with
-- radius precision p instead of 32. It is not another native donor API.
fullAdd :: Word -> B.Ball -> B.Ball -> B.Ball
fullAdd p (B.Ball c r) (B.Ball c' r') = B.Ball cen rad
  where (cen,e)=D.add_ D.Near p c c'
        eps=D.int2i D.Zero p 1 (if cen==0 then 0 else D.getExp cen-fromIntegral(D.getPrec cen)-1)
        corrected=if e==0 then r else D.add D.Up (D.getPrec r) r eps
        rad=D.add D.Up p r' corrected
{-# NOINLINE source #-}
source :: Word -> Int -> [(D.Dyadic,D.Dyadic)]
source p offset = [(D.div2i D.Near p (D.fromIntegerA D.Near p c) (fromIntegral p-5),
                   D.div2i D.Up p (D.fromIntegerA D.Near p r) (2*fromIntegral p)) |
                   j<-[offset+1..offset+128],
                   let high=1 `shiftL` (fromIntegral p-1) :: Integer,
                   let c=(high+fromIntegral(j*65537))*(if odd j then 1 else -1),
                   let r=high+fromIntegral(j*7919)]
{-# NOINLINE input #-}
input :: String -> Word -> Int -> [B.Ball]
input variant p offset = [B.Ball c (D.set D.Up rp r) | (c,r)<-source p offset]
  where rp=if variant=="fixed32" then 32 else p
{-# NOINLINE run #-}
run :: String -> Word -> [B.Ball] -> B.Ball
run variant p = foldl1 (if variant=="fixed32" then B.add p else fullAdd p)
validate :: String -> Word -> Int -> IO ()
validate variant p offset = do
  let xs=source p offset
      lo=sum [rat c-rat r|(c,r)<-xs]
      hi=sum [rat c+rat r|(c,r)<-xs]
      B.Ball c r=run variant p (input variant p offset)
  unless (rat r>=0 && rat c-rat r<=lo && rat c+rat r>=hi) (error "exact endpoint oracle failed")
main :: IO ()
main = do
  [variant,ps,rs,os]<-getArgs
  let p=read ps; reps=read rs :: Int; offset=read os
      inputs=[input variant p (offset+j)|j<-[0..6]]
  mapM_ (validate variant p) [offset..offset+6]
  -- Preconstruct and force every source payload outside the timing.
  _ <- evaluate (sum [D.getMantissa c+D.getMantissa r|xs<-inputs,B.Ball c r<-xs])
  performGC
  before<-getRTSStats
  start<-getCPUTime
  let loop !j !acc | j==reps = return acc
                   | otherwise = do
                      let B.Ball c r=run variant p (inputs!!(j `mod` 7))
                      n<-evaluate ((D.getMantissa c+D.getMantissa r) `mod` 1000000007)
                      loop (j+1) (acc+n)
  checksum<-loop 0 0
  stop<-getCPUTime
  performGC
  after<-getRTSStats
  _ <- evaluate (sum [D.getMantissa c+D.getMantissa r|xs<-inputs,B.Ball c r<-xs])
  printf "%s\t%d\t%d\t%d\t%.3f\t%d\t%d\t%d\n" variant p reps offset
    (fromIntegral(stop-start)/1000/fromIntegral reps :: Double)
    (allocated_bytes after-allocated_bytes before)
    (gcdetails_live_bytes(gc after)) checksum
