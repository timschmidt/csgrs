{-# LANGUAGE ScopedTypeVariables #-}
module Main where

import Control.DeepSeq (force)
import Control.Exception (SomeException, evaluate, try)
import Control.Monad (forM, forM_, unless)
import Data.Ratio ((%))
import Numeric.AERN.RealArithmetic.Basis.MPFR
import Numeric.AERN.Basics.Interval (Interval(..))
import qualified Numeric.AERN.MPFRBasis.Interval as I
import qualified Numeric.AERN.RealArithmetic.Interval.MPFR as IM
import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as R

report :: Show a => String -> a -> IO ()
report label value = do
  result <- try (evaluate (force (show value))) :: IO (Either SomeException String)
  putStrLn (label ++ ": " ++ either (\e -> "EXCEPTION " ++ show e) id result)

main :: IO ()
main = do
  let sample = withPrec 100 0
      one = withPrec 100 1
      two = withPrec 100 2
  report "expUp(1)" (R.expUpEff () one)
  report "expDn(1)" (R.expDnEff () one)
  report "sqrtUp(2)" (R.sqrtUpEff () two)
  report "sqrtDn(2)" (R.sqrtDnEff () two)
  report "eUp" (R.eUpEff 100 sample)
  report "eDn" (R.eDnEff 100 sample)
  report "piDn" (R.piDnEff 100 sample)
  report "piUp" (R.piUpEff 100 sample)
  report "bisect default" (IM.bisect Nothing (Interval sample two))
  report "bisect explicit" (IM.bisect (Just one) (Interval sample two))
  report "MPFR -> Double upward" (R.convertUpEff () (0::Double) one)
  report "mixed precision comparison" (one == withPrec 53 1)

  let ints = [minBound, minBound+1, -1, 0, 1, maxBound-1, maxBound] :: [Int]
      intCases = [(n,up) | n <- ints, up <- [False,True]]
  intResults <- forM intCases $ \(n,up) -> do
    let result = (if up then R.convertUpEff else R.convertDnEff) () sample n
        exact = toRational n
        good = case result of
          Nothing -> False
          Just x -> if up then toRational x >= exact else toRational x <= exact
    pure (n,up,fmap toRational result,good)
  let badInts = [r | r@(_,_,_,False) <- intResults]
  putStrLn ("Int directed conversion: checks=" ++ show (length intCases) ++
    " invalid=" ++ show (length badInts))
  mapM_ print badInts

  let precCases = [(n % (2^k),p) | n <- [-256..256], k <- [1,6,32], p <- [2,3,5,17,53]]
  precResults <- forM precCases $ \(q,p) -> do
    let x = withPrec 128 (fromRational q)
        original = toRational x
        Interval lo hi = I.setPrecOut p (Interval x x)
        lower = toRational lo
        upper = toRational hi
    unless (original == q) (error "probe source must be exactly representable")
    pure (q,p,lower,upper,lower <= original && original <= upper)
  let badPrec = [r | r@(_,_,_,_,False) <- precResults]
  putStrLn ("setPrecOut exact singleton: checks=" ++ show (length precCases) ++
    " invalid=" ++ show (length badPrec))
  mapM_ print (take 8 badPrec)

  forM_ [2,24,53,100] $ \p -> do
    let qs = [n % d | n <- [-17,-3,-1,0,1,3,17], d <- [1,3,16]]
        values = [withPrec p (fromRational q) | q <- qs]
        ops = [("add",(+),R.addDnEff (),R.addUpEff ()),
               ("multiply",(*),R.multDnEff (),R.multUpEff ()),
               ("divide",(/),R.divDnEff (),R.divUpEff ())]
    forM_ ops $ \(name,exactOp,dn,up) -> do
      let pairs = [(a,b,u) | a <- values, b <- values,
                   name /= "divide" || toRational b /= 0, u <- [False,True]]
      results <- forM pairs $ \(a,b,u) -> do
        let qa = toRational a
            qb = toRational b
            exact = exactOp qa qb
            actual = toRational ((if u then up else dn) a b)
        pure (qa,qb,u,actual,if u then actual >= exact else actual <= exact)
      let bad = [r | r@(_,_,_,_,False) <- results]
      putStrLn (name ++ " precision=" ++ show p ++ " checks=" ++ show (length pairs)
        ++ " invalid=" ++ show (length bad))
      mapM_ print (take 4 bad)
      unless (null bad) (error "directed field enclosure failed")
