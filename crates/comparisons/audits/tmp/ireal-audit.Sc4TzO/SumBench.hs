{-# LANGUAGE BangPatterns #-}
module Main where
import Data.Number.IReal
import Data.Number.IReal.IReal (appr)
import Data.Number.IReal.IntegerInterval
import Data.Ratio
import Data.Bits
import Data.List (foldl')
import Control.Exception
import System.CPUTime
import System.Environment
import GHC.Stats
import System.Mem
import Text.Printf
import Data.IORef

-- A seed affects each fresh graph. Both endpoints are forced; 'force' alone
-- is not a valid evaluation barrier for this donor.
{-# NOINLINE value #-}
value :: String -> Int -> Integer -> [IReal]
value kind n seed =
  let qs=[((k+seed) `mod` 97+1)%((k `mod` 29)+31) | k<-[1..toInteger n]]
      xs=map fromRational qs
  in if kind=="independent" then map sin xs else tail (scanl (+) 0 (map sin xs))
{-# NOINLINE work #-}
work :: String -> String -> Int -> Int -> Integer -> Integer
work mode kind n p seed =
  let xs=value kind n seed
      x=case mode of "sum"->sum xs;"bsum"->bsum xs;"isum"->isumN' (toInteger n) xs
      I(l,u)=appr x p
  in l `seq` u `seq` if u-l==2 then l+u else error "benchmark did not achieve a thin point enclosure"
main = do
  [mode,kind,ns,ps]<-getArgs
  let n=read ns :: Int;p=read ps :: Int
  evaluate (work mode kind n p 1)
  seedRef<-newIORef (1::Integer)
  performGC
  startStats<-getRTSStats
  start<-getCPUTime
  let loop !count !checksum = do
        seed<-readIORef seedRef
        v<-evaluate (work mode kind n p seed)
        now<-getCPUTime
        let new=checksum+v
        new `seq` if now-start>=200000000000 then return (count+1,new,now)
                  else loop (count+1) new
  (count,checksum,end)<-loop (0::Int) (0::Integer)
  performGC
  endStats<-getRTSStats
  printf "%s\t%s\t%d\t%d\t%d\t%.3f\t%.1f\t%d\t%d\n" mode kind n p count
    (fromIntegral (end-start)/1e3/fromIntegral count :: Double)
    (fromIntegral (allocated_bytes endStats-allocated_bytes startStats)/fromIntegral count :: Double)
    (max_live_bytes endStats) (checksum `mod` 1000003)
