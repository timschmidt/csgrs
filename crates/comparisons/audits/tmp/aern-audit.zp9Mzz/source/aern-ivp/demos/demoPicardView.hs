module Main where

import Numeric.AERN.RmToRn.Basis.Polynomial.IntPoly

import qualified Numeric.AERN.RmToRn.Plot.PicardView as PV
import Numeric.AERN.RmToRn.Plot.CairoDrawable

import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Evaluation

import Numeric.AERN.RealArithmetic.Basis.Double
--import Numeric.AERN.RealArithmetic.Basis.MPFR
import Numeric.AERN.Basics.Interval

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.OpsDefaultEffort

import Numeric.AERN.RealArithmetic.ExactOps

import qualified Numeric.AERN.RefinementOrder as RefOrd
import Numeric.AERN.RefinementOrder.OpsDefaultEffort

import qualified Numeric.AERN.NumericOrder as NumOrd
import Numeric.AERN.NumericOrder.OpsDefaultEffort

import Numeric.AERN.Basics.Consistency
import Numeric.AERN.Basics.ShowInternals

import Numeric.AERN.Misc.Debug

import qualified Graphics.UI.Gtk as Gtk

import qualified Control.Concurrent as Concurrent
import Control.Concurrent.STM

import qualified Data.Map as Map
import qualified Data.List as List


--type CF = Interval MPFR
type CF = Interval Double
type Poly = IntPoly String CF

main =
    do
--    Gtk.initGUI
    -- enable multithreaded GUI:
    Gtk.unsafeInitGUIForThreadedRTS
    Gtk.timeoutAddFull 
        (Concurrent.yield >> Concurrent.yield >> Concurrent.yield >> return True) 
        Gtk.priorityDefaultIdle 20
--    putStrLn "plot main: calling PV.new"
    let ivp = ivpPendulum
    PV.new samplePoly effDrawFn eff eff ivp Nothing
--    putStrLn "plot main: PV.new completed"
    Gtk.mainGUI

ivpPendulum :: PV.IVP Poly
ivpPendulum =
    PV.IVP
    {
        PV.ivpInitialValues = [cf0],
        PV.ivpVectorField = id 
    }
    
showP p = showPoly id show p -- ++ " [" ++ show p ++ "]"

c1,c0,x,c01,c10,cHalf,cHalf1,cOneOver16,cSevenOver16,cOneMinusOneOver16, samplePoly :: Poly
x = newProjection cfg dombox "x"
c0 = newConstFn cfg dombox 0
c1 = newConstFn cfg dombox 1
cHalf = newConstFn cfg dombox 0.5
cHalf1 = newConstFn cfg dombox $ 0.5 </\> 1
cOneOver16 = newConstFn cfg dombox $ 0.5^4
cSevenOver16 = newConstFn cfg dombox $ 7 * 0.5^4
cOneMinusOneOver16 = newConstFn cfg dombox $ 15 * 0.5^4
c01 = newConstFn cfg dombox $ 0 </\> 1
c10 = newConstFn cfg dombox $ 1 <\/> 0

samplePoly = x

--eff = (100, (100,())) -- MPFR
eff = ArithInOut.roundedRealDefaultEffort (0:: CF)
effNumComp = NumOrd.pCompareDefaultEffort x
effRefComp = RefOrd.pCompareDefaultEffort x
minmaxUpDnEff = minmaxUpDnDefaultEffortIntPolyWithBezierDegree 10 x
minmaxInOutEff = minmaxInOutDefaultEffortIntPolyWithBezierDegree 10 x
effDrawFn = cairoDrawFnDefaultEffort samplePoly


evalOpsOutCf = evalOpsOut eff x (0::CF)

cfg =
    IntPolyCfg
        {
            ipolycfg_vars = vars,
            ipolycfg_domsLZ = doms,
            ipolycfg_domsLE = replicate (length vars) (0 :: CF),
            ipolycfg_sample_cf = 0 :: CF,
            ipolycfg_maxdeg = 4,
            ipolycfg_maxsize = 30
        }

dombox = Map.fromList $ zip vars doms

vars = ["x"]

doms :: [CF]
doms = [(0 </\> 1)]

constructCF :: Double -> Double -> CF
constructCF l r =
    RefOrd.fromEndpointsOut (cf0 <+>| l, cf0 <+>| r)
cf0 = 0 :: CF
    