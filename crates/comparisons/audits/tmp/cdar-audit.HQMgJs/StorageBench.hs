{-# LANGUAGE CPP, BangPatterns #-}
module Main where
import Control.DeepSeq (force)
import Control.Exception (evaluate)
import Control.Monad (forM_)
import Data.Bits (shiftL)
import Data.CDAR
import GHC.Stats
import System.CPUTime (getCPUTime)
import System.Environment (getArgs)
import System.Mem (performGC)

make :: Int -> Integer -> Integer -> Int -> Approx
#ifdef MBOUND
make = approxMB
#else
make _ = Approx
#endif

bounds :: Approx -> (Rational,Rational)
#ifdef MBOUND
bounds (Approx _ m e s) = (fromInteger (m-e)*2^^s,fromInteger (m+e)*2^^s)
#else
bounds (Approx m e s) = (fromInteger (m-e)*2^^s,fromInteger (m+e)*2^^s)
#endif
bounds Bottom = error "unexpected Bottom"

bits :: Integer -> Int
bits = go 0 . abs where
  go !n 0 = n
  go !n a = go (n+1) (a `div` 2)

shape :: Approx -> (Int,Int,Int)
#ifdef MBOUND
shape (Approx _ m e s) = (bits m,bits e,s)
#else
shape (Approx m e s) = (bits m,bits e,s)
#endif
shape Bottom = error "unexpected Bottom"

fingerprint :: Approx -> Integer
#ifdef MBOUND
fingerprint (Approx _ m e s) = (m `mod` 65521)+e+fromIntegral s
#else
fingerprint (Approx m e s) = (m `mod` 65521)+e+fromIntegral s
#endif
fingerprint Bottom = error "unexpected Bottom"

iterations :: String -> Int
iterations "square" = 12
iterations "multiply" = 256
iterations "add" = 1024
iterations _ = error "unknown workload"

{-# NOINLINE compute #-}
compute :: String -> String -> Int -> Int -> Approx
compute work policy p seed = go (iterations work) x
  where
  m = (1 `shiftL` 32)+toInteger(1+seed `mod` 1024)
  x = make p m 0 (-32)
  finish = if policy=="limited" then limitAndBound p else id
  step a = finish $ case work of
    "square" -> sqrA a
    "multiply" -> a*x
    "add" -> a+x
    _ -> error "unknown workload"
  go 0 !a = a
  go n !a = go (n-1) (step a)

expected :: String -> Int -> Rational
expected work seed = case work of
  "square" -> x^(2^iterations work :: Int)
  "multiply" -> x^(iterations work+1)
  "add" -> fromIntegral(iterations work+1)*x
  _ -> error "unknown workload"
  where x=fromInteger((1 `shiftL` 32)+toInteger(1+seed `mod` 1024))*2^^(-32::Int)

main :: IO ()
main = do
  [work,policy,pText] <- getArgs
  let p=read pText
  forM_ [1,7,31] $ \seed -> do
    a <- evaluate(force(compute work policy p seed))
    let (l,u)=bounds a; exactValue=expected work seed
    if l<=exactValue && exactValue<=u && u-l<=2^^(-p+32)
      then pure () else error ("oracle failure "++show(work,policy,seed,a))
  performGC
  before <- getRTSStats
  started <- getCPUTime
  let batch !seed !count !checksum = do
        (!next,!sum') <- inner seed 20 checksum
        now <- getCPUTime
        if now-started >= 200000000000
          then pure(count+20,sum',now,next)
          else batch next (count+20) sum'
      inner !seed 0 !checksum = pure(seed,checksum)
      inner !seed n !checksum = do
        a <- evaluate(force(compute work policy p seed))
        c <- evaluate(fingerprint a)
        inner (seed+1) (n-1) (checksum+c)
  (count,checksum,ended,_) <- batch 100 0 0
  performGC
  after <- getRTSStats
  let elapsed=fromIntegral(ended-started)/1e3 :: Double
      allocated=allocated_bytes after-allocated_bytes before
      sh=shape(compute work policy p 31)
  putStrLn("{\"work\":"++show work++",\"policy\":"++show policy++
    ",\"precision\":"++show p++",\"iterations\":"++show(count::Int)++
    ",\"ns_per_iteration\":"++show(elapsed/fromIntegral count)++
    ",\"allocated_per_iteration\":"++show(fromIntegral allocated/fromIntegral count::Double)++
    ",\"max_live_bytes\":"++show(max_live_bytes after)++
    ",\"shape\":"++show(show sh)++",\"checksum\":"++show checksum++"}")
