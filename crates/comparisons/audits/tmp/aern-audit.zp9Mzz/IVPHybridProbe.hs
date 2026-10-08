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
import Numeric.AERN.RmToRn.Evaluation
import Numeric.AERN.RmToRn.Integration
import Numeric.AERN.RmToRn.Differentiation
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as R
import qualified Numeric.AERN.RefinementOrder as Q
import qualified Numeric.AERN.NumericOrder as N
import Numeric.AERN.IVP.Specification.Hybrid
import Numeric.AERN.IVP.Solver.Events.EventTree
import Numeric.AERN.IVP.Solver.Events.SplitNearEvents
import System.Timeout (timeout)
import System.IO (stdout,hSetBuffering,BufferMode(LineBuffering))
type D = Interval Double
type P = IntPoly String D
cf :: Double -> D
cf x = Interval x x
limits = (defaultIntPolySizeLimits (cf 0) () 2) {ipolylimits_maxdeg=4,ipolylimits_maxsize=32}
sample = newConstFn limits [("t",Interval 0 1),("x",cf 1)] (cf 0) :: P
effEval = evaluationDefaultEffort sample
effCompose = compositionDefaultEffort sample
effInteg = integrationDefaultEffort sample
effIncl = Q.pCompareDefaultEffort sample
effAdd = R.addDefaultEffort sample
effMult = R.multDefaultEffort sample
effAddDom = R.mixedAddDefaultEffort sample (cf 0)
effDom = R.roundedRealDefaultEffort (cf 0)
good = HybSysMode "constant"
bad = HybSysMode "exponential"
ivp :: [HybSysMode] -> HybridIVP P
ivp modes = HybridIVP
  { hybivp_description="two valid event-free modes, one cannot be enclosed with the selected Picard effort"
  , hybivp_system=HybridSystem
      { hybsys_componentNames=["x"]
      , hybsys_modeFields=Map.fromList
          [(good,map (\p -> newConstFnFromSample p (cf 0)))
          ,(bad,map (\p -> R.mixedMultOut p (100::Int)))]
      , hybsys_modeInvariants=Map.fromList [(good,Just),(bad,Just)]
      , hybsys_eventSpecification=const Map.empty }
  , hybivp_tVar="t", hybivp_tStart=cf 0, hybivp_tEnd=cf 1
  , hybivp_initialStateEnclosure=Map.fromList [(mode,[cf 1])|mode<-modes]
  , hybivp_maybeExactStateAtTEnd=Nothing }
noSplit modes = fst $ solveHybridIVP_UsingPicardAndEventTree
  limits (partialEvaluationDefaultEffort sample) effCompose effEval effInteg effIncl
  effAdd effMult effAddDom effDom 8 (cf 1) 3 "t0" (ivp modes)
split modes = fst $ solveHybridIVP_UsingPicardAndEventTree_SplitNearEvents
  limits (sizeLimitsChangeDefaultEffort sample) (partialEvaluationDefaultEffort sample)
  effCompose effEval effInteg (fakeDerivativeDefaultEffort sample) effIncl effAdd effMult
  (R.absDefaultEffort sample) (N.minmaxInOutDefaultEffort sample)
  (R.mixedDivDefaultEffort sample (0::Int)) effAddDom (R.mixedMultDefaultEffort sample (cf 0))
  effDom (cf 1) (cf 1) 8 (cf 1) 3 "t0" (cf 1) (cf 1) (cf 0) (ivp modes)
probe label result = do
  value <- timeout 10000000 (try (evaluate (force result)) :: IO (Either SomeException String))
  putStrLn (label ++ ": " ++ case value of
    Nothing -> "TIMEOUT"
    Just (Left e) -> "EXCEPTION " ++ show e
    Just (Right s) -> s)
main = do
  hSetBuffering stdout LineBuffering
  probe "event-tree constant only" (show (noSplit [good]))
  probe "event-tree exponential only" (show (noSplit [bad]))
  probe "event-tree both" (show (noSplit [good,bad]))
  probe "split-near-events constant only" (show (split [good]))
  probe "split-near-events exponential only" (show (split [bad]))
  probe "split-near-events both" (show (split [good,bad]))
