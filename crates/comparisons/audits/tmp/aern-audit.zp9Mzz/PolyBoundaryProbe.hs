{-# LANGUAGE ScopedTypeVariables #-}
module Main where
import Control.DeepSeq (force)
import Control.Exception (SomeException,evaluate,try)
import Control.Monad (forM_)
import qualified Data.IntMap as IM
import qualified Data.Map as Map
import Numeric.AERN.RealArithmetic.Interval.Double ()
import Numeric.AERN.RealArithmetic.Basis.Double ()
import Numeric.AERN.Basics.Interval (Interval(..))
import Numeric.AERN.Basics.SizeLimits
import Numeric.AERN.Poly.IntPoly
import Numeric.AERN.RmToRn.Differentiation
import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.Evaluation
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as R
import System.IO (stdout,hSetBuffering,BufferMode(LineBuffering))
import System.Timeout (timeout)

type P = IntPoly String (Interval Double)
cf :: Double -> Interval Double
cf x = Interval x x
limits = defaultIntPolySizeLimits (cf 0) (defaultSizeLimits (cf 0)) 1
at :: P -> Double -> (Double,Double)
at p x = let Interval l u = evalAtPointOut (Map.singleton "x" (cf x)) p in (l,u)
probe :: String -> String -> IO ()
probe label result = do
  r <- timeout 2000000 (try (evaluate (force result)) :: IO (Either SomeException String))
  putStrLn (label ++ ": " ++ case r of
    Nothing -> "TIMEOUT"
    Just (Left e) -> "EXCEPTION " ++ show e
    Just (Right s) -> s)

main :: IO ()
main = do
  hSetBuffering stdout LineBuffering
  let projection = newProjection limits [("x",Interval 2 4)] "x" :: P
      cfg = intpoly_cfg projection
      control = IntPoly cfg (IntPolyV "x" (IM.fromList [(0,IntPolyC (cf 2)),(1,IntPolyC (cf 1))]))
      collect p = termsCollectCoeffsWith (\ds (Interval l u) -> (ds,l,u)) (intpoly_terms p)
  probe "projection traversal" (show (collect projection))
  probe "control traversal" (show (collect control))
  case intpoly_terms projection of
    IntPolyV _ powers -> probe "projection lookup zero/one" (show (IM.member 0 powers,IM.member 1 powers))
    _ -> pure ()
  forM_ [2,3,4] $ \x -> do
    probe ("projection at " ++ show x) (show (at projection x))
    probe ("control at " ++ show x) (show (at control x))
  probe "adjust control domain [3,4], evaluate at 3" (show (at (adjustDomain control "x" (Interval 3 4)) 3))
  let cfg0 = intpoly_cfg (newProjection limits [("x",Interval 0 1)] "x" :: P)
      square = IntPoly cfg0 (IntPolyV "x" (IM.fromList [(0,IntPolyC (cf 0)),(2,IntPolyC (cf 1))]))
      deriv = fakePartialDerivativeOutEff (fakeDerivativeDefaultEffort square) square "x"
  probe "derivative x^2 traversal" (show (collect deriv))
  probe "derivative x^2 at 1" (show (at deriv 1))
  probe "derivative x^2 plus 2 at 1" (show (at (R.mixedAddOut deriv (2::Int)) 1))
  let dense = IntPoly cfg0 (IntPolyV "x" (IM.fromList [(k,IntPolyC (cf 1)) | k <- [0..5]]))
      smallLimits = limits {ipolylimits_maxsize = 2}
  probe "degree-five sum reduced to maxsize 2" (show (collect (changeSizeLimitsOut smallLimits dense)))
  let noVarsLimits = defaultIntPolySizeLimits (cf 0) (defaultSizeLimits (cf 0)) 0
      c = newConstFn noVarsLimits [] (cf 1) :: P
  probe "zero-variable default maxsize" (show (ipolylimits_maxsize noVarsLimits))
  probe "zero-variable constant addition" (show (collect (R.addOut c c)))
