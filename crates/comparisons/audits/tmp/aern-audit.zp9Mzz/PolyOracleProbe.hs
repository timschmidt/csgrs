{-# LANGUAGE ScopedTypeVariables #-}
module Main where
import Control.DeepSeq (force)
import Control.Exception (SomeException,evaluate,try)
import Control.Monad (forM,forM_,unless)
import qualified Data.Map as Map
import Numeric.AERN.RealArithmetic.Basis.Double ()
import Numeric.AERN.RealArithmetic.Interval.Double ()
import Numeric.AERN.Basics.Interval (Interval(..))
import Numeric.AERN.Basics.SizeLimits
import Numeric.AERN.Poly.IntPoly
import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Evaluation
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as R
import qualified Numeric.AERN.NumericOrder as N
import Numeric.IEEE.RoundMode
import System.Environment (getArgs)
import System.IO (stdout,hSetBuffering,BufferMode(LineBuffering))
import System.Timeout (timeout)

type P = IntPoly String (Interval Double)
type Case = (String, P, Rational -> Rational)
cf :: Double -> Interval Double
cf x = Interval x x
limits :: IntPolySizeLimits (Interval Double)
limits = (defaultIntPolySizeLimits (cf 0) () 1)
  {ipolylimits_maxdeg = 12, ipolylimits_maxsize = 24}
projection :: (Double,Double) -> P
projection (l,u) = newProjection limits [("x",Interval l u)] "x"
constant :: P -> Int -> P
constant p n = newConstFnFromSample p (cf (fromIntegral n))
quadratic :: P -> (Int,Int,Int) -> P
quadratic x (a,b,c) = R.mixedAddOut (R.multOut x
  (R.mixedAddOut (R.mixedMultOut x c) b)) a
qvalue :: (Int,Int,Int) -> Rational -> Rational
qvalue (a,b,c) x = (fromIntegral c*x+fromIntegral b)*x+fromIntegral a
at :: P -> Double -> (Double,Double)
at p x = let Interval l u = evalAtPointOut (Map.singleton "x" (cf x)) p in (l,u)

cases :: P -> [Case]
cases x =
  [("projection",x,id)] ++
  concat [[("add",R.addOut p q,\t -> fp t+fq t),
           ("subtract",R.subtrOut p q,\t -> fp t-fq t),
           ("multiply",R.multOut p q,\t -> fp t*fq t)]
         | (ac,bc) <- zip coeffs (reverse coeffs),
           let p=quadratic x ac; q=quadratic x bc; fp=qvalue ac; fq=qvalue bc] ++
  [("reduce power",changeSizeLimitsOut small (R.powerToNonnegIntOut x n),\t -> t^n)
     | n <- [2,3,5,8], d <- [0,1,2,5], count <- [1,2,6],
       let small=limits {ipolylimits_maxdeg=d,ipolylimits_maxsize=count}] ++
  [("reciprocal",R.divOutEff eff (constant x 1) (R.mixedAddOut x (3::Int)),\t -> 1/(t+3))
     | n <- [2,3,5,8], let eff=(R.divDefaultEffort x) {ipolyeff_recipTauDegree=n}] ++
  concat [[("max",N.maxOutEff eff shifted (constant x 0),\t -> max (t-toRational h) 0),
           ("min",N.minOutEff eff shifted (constant x 0),\t -> min (t-toRational h) 0)]
     | n <- [2,3,5,8], h <- [-0.5,0,0.5,2.5],
       let shifted=R.mixedAddOut x (negate h::Double)
           eff=(N.minmaxInOutDefaultEffort x) {ipolyeff_minmaxBernsteinDegree=n}]
  where coeffs = [(a,b,c) | (a,b,c) <- [(-2,1,1),(1,-3,2),(0,0,1),(3,2,-1),(-1,-1,-1),(2,0,0)]]

main :: IO ()
main = do
  hSetBuffering stdout LineBuffering
  args <- getArgs
  let mode=if args==["up"] then Upward else ToNearest
  forM_ [(-1,1),(0,1),(2,4)] $ \dom@(dl,du) -> do
    points <- evaluate (force [dl+(du-dl)*fromIntegral k/8 | k <- [0..8::Int]])
    results <- forM (zip [0::Int ..] (cases (projection dom))) $ \(i,(label,p,f)) -> do
      expected <- evaluate (force [f (toRational v) | v <- points])
      ok <- setRound mode
      unless ok (error "rounding mode unavailable")
      measured <- timeout 2000000 (try (evaluate (force (map (at p) points)))
        :: IO (Either SomeException [(Double,Double)]))
      let bad = case measured of
            Just (Right values) -> [(v,q,b) | (v,q,b@(l,u)) <- zip3 points expected values,
              not (encloses l u q)]
            _ -> []
          status = case measured of
            Nothing -> "timeout"
            Just (Left _) -> "exception"
            Just (Right _) -> if null bad then "pass" else "invalid"
      unless (status=="pass") $ putStrLn (show (dom,i,label,status,take 2 bad))
      case measured of
        Just (Left e) -> putStrLn (take 350 (show e))
        _ -> pure ()
      pure (label,status)
    forM_ ["projection","add","subtract","multiply","reduce power","reciprocal","max","min"] $ \label -> do
      let statuses=[s | (l,s) <- results,l==label]
      print ("summary",show mode,dom,label,length statuses,
        [(s,length (filter (==s) statuses)) | s <- ["pass","invalid","exception","timeout"]])
  where
    encloses l u q = not (isNaN l || isNaN u) && l<=u &&
      (if isInfinite l then l<0 else toRational l<=q) &&
      (if isInfinite u then u>0 else q<=toRational u)
