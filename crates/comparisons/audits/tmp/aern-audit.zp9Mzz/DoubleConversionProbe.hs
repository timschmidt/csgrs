module Main where

import Control.DeepSeq (force)
import Control.Exception (SomeException, evaluate, try)
import Control.Monad (forM, unless)
import Data.Ratio ((%))
import System.Environment (getArgs)
import Numeric.IEEE.RoundMode
import Numeric.AERN.RealArithmetic.Basis.Double ()
import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as R
import Numeric.AERN.RealArithmetic.Interval.Double (bisect, DI)
import Numeric.AERN.Basics.Interval (Interval(..))

lower :: Rational -> Double -> Bool
lower r x = not (isNaN x) && (if isInfinite x then x < 0 else toRational x <= r)
upper :: Rational -> Double -> Bool
upper r x = not (isNaN x) && (if isInfinite x then x > 0 else toRational x >= r)

pow2 :: Int -> Rational
pow2 e | e >= 0 = fromInteger (2^e)
       | otherwise = 1 % (2^(-e))

-- Labels keep extreme exact values readable; all oracles use Rational, not Double.
cases :: [((Integer,Integer,Int),Rational)]
cases = [((n,d,e),(n%d)*pow2 e) | n <- [-16..16], d <- [1,3,7],
  e <- [-1120,-1080,-1077,-1076,-1075,-1074,-1073,-1072,
        -1024,-1023,-1022,-1021,-1,0,1,53,63,64,1022,1023,1024,1100]]

runCase (label,r) = do
  result <- try (evaluate (force check)) :: IO (Either SomeException (Maybe Double,Maybe Double,Bool))
  return (label,result)
  where
    lo = R.convertDnEff () (0::Double) r
    hi = R.convertUpEff () (0::Double) r
    check = (lo,hi,maybe True (lower r) lo && maybe True (upper r) hi)

main :: IO ()
main = do
  args <- getArgs
  let mode = if args == ["up"] then Upward else ToNearest
  ok <- setRound mode
  unless ok (error "cannot set requested rounding mode")
  getRound >>= print
  results <- mapM runCase cases
  let bad = [(label,lo,hi) | (label,Right (lo,hi,False)) <- results]
      exceptions = [(label,take 140 (show err)) | (label,Left err) <- results]
  putStrLn ("rational conversions: " ++ show (length results) ++
    "; unsound enclosures: " ++ show (length bad) ++ "; exceptions: " ++ show (length exceptions))
  mapM_ print (take 12 bad)
  mapM_ print (take 4 exceptions)
  intResults <- forM [-(2^80),-(2^64),-(2^63)-2048,2^63,2^64,2^80] $ \n -> do
    let d = fromInteger n :: Double
        lo = R.convertDnEff () (0::Int) d
        hi = R.convertUpEff () (0::Int) d
    evaluate (force (n,lo,hi,maybe True ((<=toRational d).fromIntegral) lo,
                              maybe True ((>=toRational d).fromIntegral) hi))
  putStrLn "out-of-range Int conversions (directional bounds, not exact casts):"
  mapM_ print intResults
  let huge = encodeFloat (2^53-1) 971 :: Double
      tiny = encodeFloat 1 (-1074) :: Double
      samples = [Interval huge huge, Interval (-huge) huge, Interval 0 tiny] :: [DI]
  putStrLn "bisection boundary samples:"
  mapM_ (\i -> print (i,bisect Nothing i)) samples
  putStrLn "Probe completed; counts are observations, not library qualification."
