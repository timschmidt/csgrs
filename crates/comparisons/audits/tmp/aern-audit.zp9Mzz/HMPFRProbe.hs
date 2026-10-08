module Main where

import Control.DeepSeq (force)
import Control.Exception (evaluate)
import Control.Monad (forM, forM_, unless)
import Data.Ratio ((%))
import System.Environment (getArgs)
import Numeric.IEEE.RoundMode
import Numeric.AERN.RealArithmetic.Basis.MPFR ()
import Numeric.AERN.Basics.Interval (Interval(..))
import qualified Numeric.AERN.MPFRBasis.Interval as I
import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as R
import qualified Data.Number.MPFR as M
import Data.Number.MPFR.Instances.Up ()

valid :: Bool -> Rational -> Double -> Bool
valid up exact value = not (isNaN value) &&
  if isInfinite value then if up then value > 0 else value < 0
  else if up then toRational value >= exact else toRational value <= exact

fromQ :: M.Precision -> Rational -> M.MPFR
fromQ p q = fromRational q `at` p
  where at x prec = M.set M.Near prec x

main :: IO ()
main = do
  args <- getArgs
  let mode = if args == ["up"] then Upward else ToNearest
      dyadic n e = if e >= 0 then (n * 2^e) % 1 else n % (2^(-e))
      qs = [dyadic n e | n <- [-17..17],
            e <- [-1120,-1100,-1076,-1075,-1074,-1073,-1072,-1050,-1023,-1022,-1021,-53,-1,0,1,52,1022,1023,1024,1100]]
      -- Every source dyadic has at most five significant bits, so this
      -- constructor is exact; verify it before using it as an oracle input.
      values = [(q,fromQ 128 q) | q <- qs]
  forM_ values $ \(q,v) -> unless (toRational v == q) (error "inexact probe input")
  forM_ ["generic conversion","direct doubleBounds"] $ \name -> do
    results <- forM [(q,v,u) | (q,v) <- values, u <- [False,True]] $ \(q,v,u) -> do
      ok <- setRound mode
      unless ok (error "rounding mode unavailable")
      let Just generic = (if u then R.convertUpEff else R.convertDnEff) () (0::Double) v
          direct = (if u then snd else fst) (I.doubleBounds (Interval v v))
      result <- evaluate (if name == "generic conversion" then generic else direct)
      pure (q,u,result,valid u q result)
    let bad = [r | r@(_,_,_,False) <- results]
    putStrLn (name ++ " initial=" ++ show mode ++ " checks=" ++ show (length results)
      ++ " invalid=" ++ show (length bad))
    mapM_ print (take 12 bad)
    whenDirect name (null bad)

  let precCases = [(n % (2^k),p) | n <- [-256..256], k <- [1,6,32], p <- [2,3,5,17,53]]
  checks <- forM precCases $ \(q,p) -> do
    let x = fromQ 128 q
        Interval lo hi = I.setPrecOut p (Interval x x)
    pure (toRational lo <= q && q <= toRational hi)
  putStrLn ("setPrecOut checks=" ++ show (length checks) ++ " invalid=" ++ show (length (filter not checks)))
  unless (and checks) (error "precision containment failed")
  let minInt = minBound :: Int
      sample = M.fromInt M.Near 100 0
      Just minDown = R.convertDnEff 100 sample minInt
  print ("Int minBound downward",toRational minDown,toRational minDown <= toRational minInt)
  unless (toRational minDown <= toRational minInt) (error "minBound direction failed")
  forM_ [("expDn(1)",R.expDnEff () (M.fromInt M.Near 100 1)),
         ("sqrtDn(2)",R.sqrtDnEff () (M.fromInt M.Near 100 2)),
         ("eDn",R.eDnEff 100 sample)] $ \(name,x) -> do
    rendered <- evaluate (force (show x))
    putStrLn (name ++ ": " ++ rendered)
  let posInf = M.fromDouble M.Near 100 (1/0)
      negInf = -1/0 :: Double
  print ("opposed-infinity fallback addDn/addUp",
    R.mixedAddDnEff () posInf negInf,R.mixedAddUpEff () posInf negInf)
  where
    whenDirect name good = unless (name /= "direct doubleBounds" || good)
      (error "direct MPFR export failed exact-rational containment")
