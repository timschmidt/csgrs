module Main where

---- Imports -----

import Numeric.AERN.Poly.IntPoly (IntPoly(..), defaultIntPolySizeLimits, IntPolySizeLimits(..))
import Numeric.AERN.Poly.IntPoly.Plot ()
import Numeric.AERN.Basics.Interval 
    (Interval, IntervalApprox, intervalApproxIncludedIn, intervalApproxUnion)
import Numeric.AERN.RealArithmetic.Interval
        (ExpThinEffortIndicator(..))

import Numeric.AERN.RmToRn 
        (getVarDoms, 
         newProjection,
         evaluationDefaultEffort)

import Numeric.AERN.RealArithmetic.Basis.Double ()
--import Numeric.AERN.RealArithmetic.Basis.MPFR

-- real arithmetic operators and imprecision measure:
import Numeric.AERN.RealArithmetic.RefinementOrderRounding 
    (RoundedReal, roundedRealDefaultEffort,
     expOutEff, expDefaultEffort)

import Numeric.AERN.RefinementOrder ((/\))

import qualified 
       Numeric.AERN.RmToRn.Plot.FnView 
       as FV
import Numeric.AERN.RmToRn.Plot.CairoDrawable (cairoDrawFnDefaultEffort)

import Numeric.AERN.RealArithmetic.Measures (iterateUntilAccurate)

import Numeric.AERN.Basics.Effort (Int1To100(..), Int1To10(..))

import qualified 
       Graphics.UI.Gtk 
       as Gtk (unsafeInitGUIForThreadedRTS, mainGUI)

import Control.Concurrent.STM (atomically, newTVar)

import System.IO (stdout, hSetBuffering, BufferMode(LineBuffering))
import System.Environment (getArgs)

---- End of imports -----


--type CF = Interval MPFR
type CF = Interval Double
type Poly = IntPoly String CF
type PIX = IntervalApprox Poly
type Fn = PIX

usage :: String
usage = "Usage: Inclusion <maxdeg> <exp taylor degree>"

processArgs :: [String] -> Maybe (Int, Int)
processArgs [] = Nothing
processArgs [maxdegS, expTaylorS] =
    case (reads maxdegS, reads expTaylorS) of
        ((maxdeg, []):_,(expTaylor, []):_) -> Just (maxdeg, expTaylor)
        _ -> error $ usage
processArgs _ = error $ usage

main :: IO ()
main =
    do
    -- set console to print each line asap:
    hSetBuffering stdout LineBuffering
    -- process command line arguments:
    args <- getArgs
    case processArgs args of
        Just (maxdeg, expTaylor) ->
            do
            plot $ inclusionFunctions maxdeg expTaylor
        _ ->
            do
            results <- mapM reportAttempt inclusionResult
            let ((Int1To100 maxdeg, Int1To10 expTaylor), _, _) = last results
            plot $ inclusionFunctions maxdeg expTaylor
    where
    reportAttempt result@(eff, res, _) =
        do
        putStrLn $ show eff ++ ": " ++ show res
        return result
            
inclusionResult :: [((Int1To100, Int1To10), Maybe Bool, Maybe ())]
inclusionResult =
    iterateUntilAccurate initEffort maxAttempts maxImprecision $ 
        \ eff -> testInclusion eff
        where
        initEffort = (Int1To100 initMaxdeg, Int1To10 initExpTaylorDeg)
        initMaxdeg = 3
        initExpTaylorDeg = 1
        maxImprecision = Just ()
        maxAttempts = 100
        
        testInclusion (Int1To100 maxdeg, Int1To10 expTaylorDeg) =
            fn1 `intervalApproxIncludedIn` fn2
            where
            ([[fn1, fn2]], _) = inclusionFunctions maxdeg expTaylorDeg


inclusionFunctions :: Int -> Int -> ([[Fn]], FV.FnMetaData Fn)
inclusionFunctions maxdeg expTaylorDeg = (fnGroups, fnmeta)
    where
    fnGroups = [[fn1, fn2]]

    fn1Name = "+-0.2x + exp(x)"
    fn1 = (pmDelta) * x + expOutEff effExp x
    
--    fn1Name = "+-0.8x + exp(x)"
--    fn1 = (pmDelta + pmDelta + pmDelta + pmDelta) * x + expOutEff effExp x
    
    fn2Name = "(exp(x+-0.2))"
    fn2 = expOutEff effExp (x + pmDelta)
    
    pmDelta = (- delta) `intervalApproxUnion` delta
    
    x = newProjection limits doms "x"
    delta = newProjection limits doms "delta"
    doms = [("x", 0 /\ 1), 
            ("delta", 0.2 /\ 0.2)]
--            ("delta", 0.2 /\ 1)]

    limits = 
        (defaultIntPolySizeLimits sampleCF cfLimits arity)
        {
            ipolylimits_maxdeg = maxdeg
        }
        where
        sampleCF = 0
        cfLimits = ()
        arity = 1

    effExp = 
        (expDefaultEffort x)
        {
            expeff_taylorDeg = expTaylorDeg            
        }

    fnmeta =
        FV.simpleFnMetaData
            sampleFn 
            (FV.Rectangle  3 (0) (0) 1) -- initial plotting region
            (Just (1,1,1,1))
            200 -- samplesPerUnit
            "x"
            [("fn1 ⊆ fn2 ?", functionInfos)]
        where
        sampleFn = x
        functionInfos =
            zip3 
                [fn1Name, fn2Name]
                [green, blue, red] -- styles 
                (repeat True) -- show everything by default


plot :: ([[Fn]], FV.FnMetaData Fn) -> IO ()
plot (fns, fnmeta) =
    do
    -- enable multithreaded GUI:
    _ <- Gtk.unsafeInitGUIForThreadedRTS
    fnDataTV <- atomically $ newTVar $ FV.FnData $ addPlotVar fns
    fnMetaTV <- atomically $ newTVar $ fnmeta
    _ <- FV.new sampleFn effDrawFn effCF effEval (fnDataTV, fnMetaTV) Nothing
    Gtk.mainGUI
    where
    ((sampleFn :_) :_) = fns 
    effDrawFn = 
        (cairoDrawFnDefaultEffort sampleFn)
    effEval = 
        (evaluationDefaultEffort sampleFn)
    effCF = roundedRealDefaultEffort (0:: CF)
    --effCF = (100, (100,())) -- MPFR

addPlotVar :: [[Fn]] -> [[(FV.GraphOrParamPlotFn Fn, String)]]
addPlotVar fns =
    map (map addV) fns
    where
    addV fn = (FV.GraphPlotFn [fn], plotVar)
        where
        (plotVar : _) = vars
        vars = map fst $ getVarDoms fn   

black :: FV.FnPlotStyle
black = FV.defaultFnPlotStyle
blue :: FV.FnPlotStyle
blue = FV.defaultFnPlotStyle 
    { 
        FV.styleOutlineColour = Just (0.1,0.1,0.8,1), 
        FV.styleFillColour = Just (0.1,0.1,0.8,0.1) 
    } 
green :: FV.FnPlotStyle
green = FV.defaultFnPlotStyle 
    { 
        FV.styleOutlineColour = Just (0.1,0.8,0.1,1), 
        FV.styleFillColour = Just (0.1,0.8,0.1,0.1) 
    } 
red :: FV.FnPlotStyle
red = FV.defaultFnPlotStyle 
    { 
        FV.styleOutlineColour = Just (0.8,0.2,0.2,1), 
        FV.styleFillColour = Just (0.8,0.2,0.2,0.1) 
    } 

