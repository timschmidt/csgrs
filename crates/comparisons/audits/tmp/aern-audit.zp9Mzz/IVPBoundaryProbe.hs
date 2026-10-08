{-# LANGUAGE ScopedTypeVariables #-}
module Main where
import Control.DeepSeq (force)
import Control.Exception (SomeException, evaluate, try)
import Control.Monad (forM_, unless)
import qualified Data.Map as Map
import qualified Data.Set as Set
import Numeric.AERN.RealArithmetic.Interval.Double ()
import Numeric.AERN.RealArithmetic.Basis.Double ()
import Numeric.AERN.Basics.Interval (Interval(..))
import Numeric.AERN.Basics.SizeLimits
import Numeric.AERN.Poly.IntPoly
import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.Evaluation
import qualified Numeric.AERN.NumericOrder as N
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as R
import Numeric.AERN.IVP.Solver.Bisection
import Numeric.AERN.IVP.Solver.Events.Locate
import System.IO (stdout,hSetBuffering,BufferMode(LineBuffering))
import System.Timeout (timeout)

type D = Interval Double
type P = IntPoly String D
cf :: Double -> D
cf x = Interval x x
limits = defaultIntPolySizeLimits (cf 0) (defaultSizeLimits (cf 0)) 1
projection l r = newProjection limits [("x",Interval l r)] "x" :: P
at :: P -> Double -> (Double,Double)
at p x = let Interval l r = evalAtPointOut (Map.singleton "x" (cf x)) p in (l,r)

main :: IO ()
main = do
  hSetBuffering stdout LineBuffering
  let effort = R.roundedRealDefaultEffort (cf 0)
      tree = BisectionSplit () (BisectionNoSplit (Just (projection 2 3)))
                 (Just (BisectionNoSplit (Just (projection 3 4))))
  forM_ [2,2.5,3,3.25,3.5,4] $ \cut -> do
    let trim Nothing = Nothing
        trim (Just p) = Just $ adjustDomain p "x" (N.minOut dom (cf cut))
          where Just dom = Map.lookup "x" (getDomainBox p)
        trimmed = bisectionInfoTrimAt effort trim (const Nothing) tree (cf 2,cf 4) (cf cut)
        leaves = bisectionInfoGetLeafSegInfoSequence trimmed
    forM_ leaves $ \leaf -> case leaf of
      Nothing -> pure ()
      Just p -> do
        let Just (Interval l r) = Map.lookup "x" (getDomainBox p)
        forM_ [l,(l+r)/2,r] $ \x -> do
          let (a,b) = at p x
          unless (a <= x && x <= b) $ error ("trim enclosure violated " ++ show (cut,x,a,b))
    putStrLn ("right trim at " ++ show cut ++ ": PASS " ++ show (map (fmap getDomainBox) leaves))
  let locate step lo hi possible violated poor =
        locateFirstDipAmongMultipleFns (cf step) (const possible) (const violated) (const poor) (cf lo,cf hi)
        :: LocateDipResult D Int
  putStrLn ("no-events: " ++ show (locate (1/16) 0 1 Set.empty False False))
  putStrLn ("possible-event: " ++ show (locate (1/16) 0 1 (Set.singleton 1) False False))
  putStrLn ("certain-event: " ++ show (locate (1/16) 0 1 (Set.singleton 1) True False))
  putStrLn ("poor-enclosure: " ++ show (locate (1/16) 0 1 (Set.singleton 1) False True))
  let adjacent = encodeFloat 4503599627370497 (-52) :: Double
      result = show (locate (encodeFloat 1 (-60)) 1 adjacent (Set.singleton 1) False False)
      Interval midpointLower midpointUpper = R.mixedDivOut (R.addOut (cf 1) (cf adjacent)) (2::Int)
  putStrLn ("adjacent endpoints and midpoint: " ++ show (1::Double,adjacent,midpointLower,midpointUpper))
  unless (midpointLower == 1 || midpointLower == adjacent) $ error "midpoint unexpectedly made progress"
  putStrLn ("adjacent-step control: " ++ show (locate (encodeFloat 1 (-52)) 1 adjacent (Set.singleton 1) False False))
  r <- timeout 2000000 (try (evaluate (force result)) :: IO (Either SomeException String))
  putStrLn ("adjacent-binary64 positive-step: " ++ case r of
    Nothing -> "TIMEOUT"
    Just (Left e) -> "EXCEPTION " ++ show e
    Just (Right s) -> s)
