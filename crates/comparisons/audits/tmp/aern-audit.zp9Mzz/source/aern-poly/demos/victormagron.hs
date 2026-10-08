module Main where

import Numeric.AERN.Poly.IntPoly

import Numeric.AERN.RmToRn

import Numeric.AERN.RealArithmetic.Basis.Double ()
--import Numeric.AERN.RealArithmetic.Basis.MPFR
import Numeric.AERN.Basics.Interval

import Numeric.AERN.RealArithmetic.NumericOrderRounding.ElementaryFromFieldOps.Sqrt (SqrtThinEffortIndicator(..))

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators

--import Numeric.AERN.RealArithmetic.ExactOps
--import Numeric.AERN.RealArithmetic.Measures 

import qualified Numeric.AERN.RefinementOrder as RefOrd
import Numeric.AERN.RefinementOrder.Operators

import qualified Numeric.AERN.NumericOrder as NumOrd
--import Numeric.AERN.NumericOrder.OpsDefaultEffort

import Numeric.AERN.Basics.SizeLimits
--import Numeric.AERN.Basics.Consistency
import Numeric.AERN.Basics.PartialOrdering
import Numeric.AERN.Basics.ShowInternals

--import System.IO

--import qualified Data.Map as Map
--import qualified Data.List as List

--type CF = Interval MPFR
type CF = Interval Double
type Poly = IntPoly String CF


main :: IO ()
main =
    do
    return ()

{------

Results over [4,6.3504]^6 for various choices of params: 

"(6,20,1,100) -> [_0.15211827195519825,2.21712006899124^]"
(0.79 secs, 733925428 bytes)

"(10,100,1,100) -> [_0.18368014851264433,2.112064768342252^]"
(3.84 secs, 3732886624 bytes)

"(10,1000,1,200) -> [_0.18368029513801096,2.112064768342252^]"
(6.76 secs, 6307827848 bytes)



Results over [4, 6.3504]^3 x [6.3504, 8] x [4, 6.3504]^2 for various choices of params: 

"(6,20,1,100) -> [_7.528559837267282e-2,1.9364386211716682^]"
(0.34 secs, 346256664 bytes)

"(6,100,1,100) -> [_8.771971942510443e-2,1.7822440874987577^]"
(2.42 secs, 2533208868 bytes)


------}

params@(maxdeg, maxsize, recipdeg, evalsplitsize) = (6, 100, 1, 200) 

result =
    show params 
    ++ " -> " ++ show r_flyspeckXXX


{- 
    For x=x1=x2=...=x6 we get:
    
     f_delta4DeltaX = 2x^2
     
     f_DeltaX = 2x^3
     
     (r_flyspeckXXX)^2 = 4x^4 / 8 x^4 = 0.5
-}

r_flyspeckXXX = r_delta4DeltaX / (ArithInOut.sqrtOutEff effCfSqrt r_4x1DeltaX)

r_4x1DeltaX = rangeOut f_4x1DeltaX

f_4x1DeltaX = (4::Int) |* (x1 * f_DeltaX)

r_delta4DeltaX = rangeOut f_delta4DeltaX

rangeOut fn =
    evalAtPointOut (getDomainBox fn) fn

f_delta4DeltaX = 
    x1 * (-x1 + x2 + x3 - x4 + x5 + x6) + x2 * x5 + x3 * x6 - x2 * x3 - x5 * x6

f_DeltaX = 
    x1*x4*(-x1 + x2 + x3 - x4 + x5 + x6) + 
    x2*x5*(x1 - x2 + x3 + x4 - x5 + x6) +
    x3*x6*(x1 + x2 - x3 + x4 + x5 - x6)
    - x2*(x3*x4+x1*x6)
    - x5*(x1*x3+x4*x6) 


x1,x2,x3,x4,x5,x6 :: Poly
x1 = newProjection limits varDoms "x1"
x2 = newProjection limits varDoms "x2"
x3 = newProjection limits varDoms "x3"
x4 = newProjection limits varDoms "x4"
x5 = newProjection limits varDoms "x5"
x6 = newProjection limits varDoms "x6"

--effSqrt =
    

effIP :: IntPolyEffort CF
effIP =
    IntPolyEffort
    {
        ipolyeff_sampleCf = 0,
        ipolyeff_cfRoundedRealEffort = effCf,
        ipolyeff_cfGetEndpointsEffort = RefOrd.getEndpointsDefaultEffort sampleCf,
        ipolyeff_cfFromEndpointsEffort = RefOrd.fromEndpointsDefaultEffort sampleCf,
        ipolyeff_evalMaxSplitSize = evalsplitsize,
        ipolyeff_minmaxBernsteinDegree = bernsteinDegree,
        ipolyeff_recipTauDegree = tauDegree,
        ipolyeff_counterExampleSearchSampleCount = 4 * arity
    }
    where
    bernsteinDegree = 4
    tauDegree = recipdeg
    arity = 2
    sampleCf = 0 :: CF
    
effCf :: ArithInOut.RoundedRealEffortIndicator CF
--effCf = (100, (100,())) -- MPFR with explicit precision
effCf = ArithInOut.roundedRealDefaultEffort (0:: CF)

effCfSqrt = (ArithInOut.sqrtDefaultEffort (0 :: CF))
    {
        sqrteff_newtonIters = 14
    }

effIPsplitSize :: Int -> IntPolyEffort CF
effIPsplitSize maxsplitSize =
    effIP { ipolyeff_evalMaxSplitSize = maxsplitSize }

minmaxUpDnEff :: IntPolyEffort CF
minmaxUpDnEff = effIP { ipolyeff_minmaxBernsteinDegree = 10 }

minmaxInOutEff :: IntPolyEffort CF
minmaxInOutEff = effIP { ipolyeff_minmaxBernsteinDegree = 10 }

--evalOpsOutCf :: PolyEvalOps String CF CF
--evalOpsOutCf = evalOpsEff effIP x1 (0::CF)

limits :: IntPolySizeLimits CF
limits =
    IntPolySizeLimits
    {
        ipolylimits_cf_limits = (),
        ipolylimits_maxdeg = maxdeg,
        ipolylimits_maxsize = maxsize,
        ipolylimits_effort = effIP
    } 

varDoms :: [(Var Poly, CF)]
varDoms =
    [
        ("x1", 4 </\> (6.3504)),
        ("x2", 4 </\> (6.3504)),
        ("x3", 4 </\> (6.3504)),
--        ("x4", 4 </\> (6.3504)),
        ("x4", (6.3504) </\> 8),
        ("x5", 4 </\> (6.3504)),
        ("x6", 4 </\> (6.3504))
    ] 
--    take 6 $ zip vars $ repeat dom 

--vars :: [Var Poly]
--vars = map (\n -> "x" ++ show n) [1..]
--
--dom = 4 </\> (6.3504)

