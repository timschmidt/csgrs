{-# LANGUAGE BangPatterns #-}
module Main where
import qualified Data.Approximate.ApproximateField as A
import qualified Data.Reals.Staged as S
import Control.Exception
import GHC.Stats
import System.Mem
import System.CPUTime
import System.Environment
import Text.Printf

{-# NOINLINE queryFun #-}
queryFun :: S.StagedWithFun Int -> Int -> Int
queryFun x p = S.approximate x (A.precDown p)
{-# NOINLINE queryList #-}
queryList :: S.StagedWithList Int -> Int -> Int
queryList x p = S.approximate x (A.precDown p)

loop :: (Int -> Int) -> Int -> Int -> Int -> IO Int
loop f p n !acc | n == 0 = pure acc
               | otherwise = do
                   v <- evaluate (f (p + n `mod` 2))
                   loop f p (n-1) (acc+v)

bench :: String -> Int -> IO ()
bench variant p = do
  performGC
  before <- getRTSStats
  let xf = S.limit A.precision :: S.StagedWithFun Int
      xl = S.limit A.precision :: S.StagedWithList Int
      f = if variant == "list" then queryList xl else queryFun xf
  _ <- evaluate (f (p+1))
  performGC
  after <- getRTSStats
  let count = if variant == "fun" then 10000000 else max 128 (min 1000000 (100000000 `div` max 1 p))
  start <- getCPUTime
  checksum <- loop f p count 0
  end <- getCPUTime
  performGC -- Flush nursery allocation accounting after, not inside, timing.
  final <- getRTSStats
  printf "%s\t%d\t%d\t%.3f\t%d\t%d\t%d\t%d\n" variant p count
    (fromIntegral (end-start) / fromIntegral count / 1000 :: Double)
    (allocated_bytes after - allocated_bytes before)
    (gcdetails_live_bytes (gc after))
    (allocated_bytes final - allocated_bytes after) checksum

main :: IO ()
main = do
  args <- getArgs
  case args of
    [variant,p] -> bench variant (read p)
    _ -> error "usage: stage-bench list|fun PRECISION +RTS -T"
