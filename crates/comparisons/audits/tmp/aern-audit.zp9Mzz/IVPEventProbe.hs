module Main where
import Control.DeepSeq (force)
import Control.Exception (SomeException,evaluate,try)
import qualified Data.Map as Map
import Numeric.AERN.RealArithmetic.Basis.Double ()
import Numeric.AERN.RealArithmetic.Interval.Double ()
import Numeric.AERN.Basics.Interval (Interval(..))
import Numeric.AERN.Basics.SizeLimits
import Numeric.AERN.Poly.IntPoly
import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.Evaluation
import Numeric.AERN.RmToRn.Integration
import Numeric.AERN.RmToRn.Differentiation
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as R
import qualified Numeric.AERN.RefinementOrder as Q
import qualified Numeric.AERN.NumericOrder as N
import Numeric.AERN.IVP.Specification.ODE
import Numeric.AERN.IVP.Specification.Hybrid
import Numeric.AERN.IVP.Solver.Events.EventTree
import Numeric.AERN.IVP.Solver.Picard.UncertainValue
import System.Timeout (timeout)
import System.IO (stdout,hSetBuffering,BufferMode(LineBuffering))
type D = Interval Double
type P = IntPoly String D
cf :: Double -> D
cf x = Interval x x
limits = (defaultIntPolySizeLimits (cf 0) () 2) {ipolylimits_maxdeg=4,ipolylimits_maxsize=32}
sample = newConstFn limits [("t",Interval 0 1),("u",Interval 0 1)] (cf 0) :: P
effEval = evaluationDefaultEffort sample
initial size t dom =
  let u = newProjection size [(t,dom),("u",Interval 0 1)] "u" :: P
  in [R.multOut u (R.subtrOut (newConstFnFromSample u (cf 1)) u)]
ivp prune = ODEIVP
  { odeivp_description="zero ODE with nonlinear initial parameter"
  , odeivp_componentNames=["x"]
  , odeivp_field=map (\p -> newConstFnFromSample p (cf 0))
  , odeivp_tVar="t", odeivp_tStart=cf 0, odeivp_tEnd=cf 1
  , odeivp_makeInitialValueFnVec=initial, odeivp_t0End=cf 0
  , odeivp_intersectDomain=prune, odeivp_valuePlotExtents=[(cf 0,cf 1)]
  , odeivp_enclosureRangeWidthLimit=cf 1000, odeivp_maybeExactValuesAtTEnd=Nothing }
run wrap prune = fst $ solveODEIVPUncertainValueExactTime_UsingPicard_Bisect wrap False
  limits (sizeLimitsChangeDefaultEffort sample) (compositionDefaultEffort sample)
  effEval (integrationDefaultEffort sample) (fakeDerivativeDefaultEffort sample)
  (Q.pCompareDefaultEffort sample) (R.addDefaultEffort sample) (R.multDefaultEffort sample)
  (R.absDefaultEffort sample) (N.minmaxInOutDefaultEffort sample)
  (R.mixedDivDefaultEffort sample (0::Int)) (R.mixedAddDefaultEffort sample (cf 0))
  (R.mixedMultDefaultEffort sample (cf 0)) (R.roundedRealDefaultEffort (cf 0))
  (cf 1) 3 (cf 0.125) (cf 0.5) (cf 0) (ivp prune)
prune [Interval l r] | r < 0.125 = Nothing
                    | otherwise = Just [Interval (max l 0.125) r]
prune _ = error "unexpected dimension"
probe label result = do
  value <- timeout 10000000 (try (evaluate (force result)) :: IO (Either SomeException String))
  putStrLn (label ++ ": " ++ case value of
    Nothing -> "TIMEOUT"
    Just (Left e) -> "EXCEPTION " ++ show e
    Just (Right s) -> s)
main = do
  hSetBuffering stdout LineBuffering
  let p = newConstFn limits [("t",Interval 0 1)] (cf 1) :: P
      state = (HybSysMode "mode",[p])
      collect child = eventInfoCollectFinalStates (evaluationDefaultEffort p) "t" (cf 1)
        (EventNextMaybe state (Map.singleton (HybSysEventKind "event") child))
  probe "possible event with inconsistent child" (show (collect EventInconsistent))
  probe "possible event with unknown child" (show (collect EventGivenUp))
  probe "possible event with fixed-point child" (show (collect (EventFixedPoint state)))
  let [q] = initial limits "t" (Interval 0 1)
      sampled = foldl1 (Q.</\>) (map head (evalSamplesAlongEdgesEff effEval 1 (getDomainBox q) (getVars (getDomainBox q)) [q]))
      midpoint = evalAtPointOutEff effEval (Map.fromList [("t",cf 0.5),("u",cf 0.5)]) q
  probe "initial-value sampled hull vs interior" (show (sampled,midpoint))
  probe "unwrapped unrestricted IVP" (show (run False Just))
  probe "unwrapped invariant-pruned IVP" (show (run False prune))
  probe "wrapped invariant-pruned IVP" (show (run True prune))
