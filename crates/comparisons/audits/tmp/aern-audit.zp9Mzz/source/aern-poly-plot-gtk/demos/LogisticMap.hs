module Main where

import Numeric.AERN.Poly.IntPoly 
    (IntPoly(..), 
     defaultIntPolySizeLimits, IntPolySizeLimits(..), IntPolyEffort(..))
import Numeric.AERN.Poly.IntPoly.Plot ()
import Numeric.AERN.Basics.Interval (Interval)

import Numeric.AERN.RmToRn 
    (Domain, newProjection, getVarDoms, evaluationDefaultEffort)

--import Numeric.AERN.RealArithmetic.Basis.Double ()
import Numeric.AERN.RealArithmetic.Basis.MPFR (MPFR, Precision)

-- real arithmetic operators and imprecision measure:
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators ((|*))
import Numeric.AERN.RealArithmetic.RefinementOrderRounding (RoundedReal, roundedRealDefaultEffort)
import Numeric.AERN.RealArithmetic.Measures (imprecisionOf, iterateUntilAccurate)

-- ability to change precision of numbers:
import Numeric.AERN.Basics.SizeLimits (changeSizeLimitsOut)

import Numeric.AERN.RefinementOrder ((/\))


import qualified 
       Numeric.AERN.RmToRn.Plot.FnView 
       as FV
import Numeric.AERN.RmToRn.Plot.CairoDrawable (cairoDrawFnDefaultEffort)

import qualified 
       Graphics.UI.Gtk 
       as Gtk (unsafeInitGUIForThreadedRTS, mainGUI)

import Control.Concurrent.STM (atomically, newTVar)

import System.IO (stdout, hSetBuffering, BufferMode(LineBuffering))
import System.Environment (getArgs)


type CF = Interval MPFR
--type CF = Interval Double
type Poly = IntPoly String CF
--type PI = Interval Poly
type Fn = Poly 
--type Fn = PI

usage :: String
usage = "Usage: LogisticMap <number of iterations> <result digits>"

processArgs :: [String] -> (Int, Int, Maybe (Int, Precision))
processArgs [itersS, digitsS] =
    case (reads itersS, reads digitsS) of
        ((iters, []):_,(digits, []):_) -> (iters, digits, Nothing)
        _ -> error $ usage
processArgs [itersS, digitsS, maxdegS, precS] =
    case (reads itersS, reads digitsS, reads maxdegS, reads precS) of
        ((iters, []):_,(digits, []):_, (maxdeg, []):_, (prec, []):_) -> 
            (iters, digits, Just (maxdeg, fromInteger prec))
        _ -> error $ usage
processArgs _ = error $ usage

rArg :: Rational
rArg = 375 / 100

x0Domain :: Domain Fn
x0Domain = 0 /\ 1 -- ie interval covering both 0 and 1

main :: IO ()
main =
    do
    -- set console to print each line asap:
    hSetBuffering stdout LineBuffering
    -- process command line arguments:
    args <- getArgs
    let (iters, digits, maybeEff) = processArgs args 

    -- compute the iterations using various efforts and report progress:
    functions <-
        case maybeEff of
            Nothing -> 
                mapM reportAttempt $ attemptsWithIncreasingEffort iters digits
            Just eff ->
                return [logisticMapIterateNTimes rArg (identityFunctionWithMaxdeg eff x0Domain) iters]

    -- plot the enclosures for all iterations computed in the last attempt:
    plot $ addPlotMetainfo (last functions :: [Fn])

    where
    -- print the result of an attempt
    reportAttempt (prec, res, imprecision) =
        do    
        putStrLn formattedRes
        return res 
        where
        formattedRes =
            show prec ++ ": " 
            ++ "; prec = " ++ (show imprecision)
--            ++ "\n res = " ++ (show $ last res)
--            ++ "\n 1 + res = " ++ (show $ 1 + (last res))

    attemptsWithIncreasingEffort iters digits =
        -- invoke an iRRAM-style procedure for automatic precision/effort incrementing 
        --    on the computation of iters-many iterations of the logistic map:
        iterateUntilAccurate initEff maxAttempts maxImprecision $ 
            \ eff -> logisticMapIterateNTimes rArg (identityFunctionWithMaxdeg (initMaxdeg, eff) x0Domain) iters
        where
        --        initEff = (initMaxdeg, initPrec)
        initEff = initPrec
        --            initMaxdeg = 3
        initMaxdeg = 2^iters
        initPrec = 50
        maxAttempts = 100 -- try to increase precision 100 times before giving up
        maxImprecision = 10^^(-digits) -- target result precision
        
identityFunctionWithMaxdeg (maxdeg, prec) dom =
    newProjection limits [("x", domP)] "x"
    where
    domP = changeSizeLimitsOut prec dom
    limits =
        defaultLimits
        {
            ipolylimits_maxdeg = maxdeg,
            ipolylimits_maxsize = maxdeg + 1,
            ipolylimits_effort = effortIncreaseEvaluationEffort
        }
        where
        defaultLimits =
            defaultIntPolySizeLimits sampleCF cfLimits arity 
        effortIncreaseEvaluationEffort =
            (ipolylimits_effort defaultLimits)
            {
                ipolyeff_evalMaxSplitSize = 1000,
                ipolyeff_minmaxBernsteinDegree = 4
            }
        sampleCF = changeSizeLimitsOut prec 0
        cfLimits = prec
        arity = 1

logisticMapIterateNTimes ::
    (Num real, RoundedReal real)
    =>
    Rational {-^ @r@ scaling constant -} -> 
    real {-^ @x0@ initial value  -} ->
    Int {-^ @n@ number of iterations -} ->
    [real] {-^ @logisticMap^n(x0)@ -} 

logisticMapIterateNTimes r x0 n = 
    take (n+1) $ (iterate (logisticMap r) x0)
    
logisticMap ::
    (Num real, RoundedReal real)
    =>
    Rational {-^ scaling constant r -} -> 
    real {-^ previous value x -} ->
    real {-^ rx(1-x) -} 

logisticMap r xPrev =
    r |* (xPrev * (1 - xPrev))
        
addPlotMetainfo ::
    [Fn] ->
    ([[Fn]], FV.FnMetaData Fn)
addPlotMetainfo fns =
    ([fns], fnMeta)
    where
    fnMeta =  
        FV.simpleFnMetaData
            sampleFn 
            (FV.Rectangle  1 (0) 0 1) -- initial plotting region
            (Just (1,1,1,1))
            500 -- samplesPerUnit
            "x"
            [("LM iterations", functionInfos)]
        where
        functionInfos =
            zip3 
                ["x" ++ show i | i <- [1..iters]]
                (cycle [black, blue, red, green]) -- styles 
                ((replicate (iters - 1) False) ++ [True]) -- show only the last iteration
--                (repeat True) -- show all iterations
        iters = length fns
    sampleFn : _ = fns

plot :: ([[Fn]], FV.FnMetaData Fn) -> IO ()
plot (fns, fnmeta) =
    do
--    putStrLn $ "last fn  = " ++ show lastFn
    putStrLn $ "impresision of iterations = " ++ show (map imprecisionOf $ head fns)
    -- enable multithreaded GUI:
    _ <- Gtk.unsafeInitGUIForThreadedRTS
    fnDataTV <- atomically $ newTVar $ FV.FnData $ addPlotVar fns
    fnMetaTV <- atomically $ newTVar $ fnmeta
    _ <- FV.new sampleFn effDrawFn effCF effEval (fnDataTV, fnMetaTV) Nothing
--    Concurrent.forkIO $ signalFn fnMetaTV
    Gtk.mainGUI
    where
    ((sampleFn :_) :_) = fns 
    lastFn = head $ reverse $ head fns 
    effDrawFn = 
        (cairoDrawFnDefaultEffort sampleFn)
--        {
--            draweff_evalF = effEval
--        }
    effEval = 
        (evaluationDefaultEffort sampleFn)
--        (ipolyeff2, othereff)
--        where
--        ipolyeff2 =
--            ipolyeff
--            {
--                ipolyeff_evalMaxSplitSize = 1000
--            }
--        (ipolyeff, othereff) =
--            (evaluationDefaultEffort sampleFn)
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
red :: FV.FnPlotStyle
red = FV.defaultFnPlotStyle 
    { 
        FV.styleOutlineColour = Just (0.8,0.2,0.2,1), 
        FV.styleFillColour = Just (0.8,0.2,0.2,0.1) 
    } 

green :: FV.FnPlotStyle
green = FV.defaultFnPlotStyle 
    { 
        FV.styleOutlineColour = Just (0.1,0.8,0.1,1), 
        FV.styleFillColour = Just (0.1,0.8,0.1,0.1) 
    } 
