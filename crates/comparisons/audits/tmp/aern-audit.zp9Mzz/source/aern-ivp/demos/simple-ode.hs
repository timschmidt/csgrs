{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
module Main where

import Numeric.AERN.IVP.Examples.ODE.Simple

import Numeric.AERN.IVP.Specification.ODE
import Numeric.AERN.IVP.Solver.Bisection
import Numeric.AERN.IVP.Solver.Picard.UncertainValue
import Numeric.AERN.IVP.Plot.UsingFnView 
    (plotODEIVPBisectionEnclosures, readIVPPlotArgs, 
     IVPPlotArgs(..), plotArgsHelpLines)

import Numeric.AERN.IVP.Solver.ShrinkWrap -- only for testing

import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.Evaluation
import Numeric.AERN.RmToRn.Integration
import Numeric.AERN.RmToRn.Differentiation

import qualified 
       Numeric.AERN.RmToRn.Plot.FnView 
       as FV

import Numeric.AERN.Poly.IntPoly
import Numeric.AERN.Poly.IntPoly.Interval ()
import Numeric.AERN.Poly.IntPoly.Plot ()

import Numeric.AERN.RealArithmetic.Basis.Double ()
import qualified 
       Numeric.AERN.DoubleBasis.Interval 
       as CF
--import Numeric.AERN.RealArithmetic.Basis.MPFR
--import qualified Numeric.AERN.MPFRBasis.Interval as MI

--import Numeric.AERN.Basics.Interval
import Numeric.AERN.RealArithmetic.Interval ()

import qualified 
       Numeric.AERN.RealArithmetic.RefinementOrderRounding 
       as ArithInOut
import Numeric.AERN.RealArithmetic.RefinementOrderRounding (dblToReal)
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators
--import Numeric.AERN.RealArithmetic.ExactOps

import qualified 
       Numeric.AERN.NumericOrder 
       as NumOrd
--import Numeric.AERN.NumericOrder.Operators

import qualified 
       Numeric.AERN.RefinementOrder 
       as RefOrd
--import Numeric.AERN.RefinementOrder.Operators

--import Numeric.AERN.Basics.Effort
import Numeric.AERN.Basics.SizeLimits
--import Numeric.AERN.Basics.ShowInternals

import Data.List (isSuffixOf)

import System.IO
import System.Environment
import System.Directory
import System.CPUTime

--import qualified Control.Concurrent as Concurrent

import Debug.Trace
_ = trace -- stop the unused warning

--import qualified Data.Map as Map
--import qualified Data.List as List

type CF = CF.DI

type Fn = IntPoly String CF
--type Fn = Interval (IntPoly String CF)

sampleCf :: CF
sampleCf = 0

sampleFn :: Fn
sampleFn = makeSampleWithVarsDoms 10 10 ["x"] [sampleCf]

main :: IO ()
main =
    do
    hSetBuffering stdout LineBuffering
    args <- getArgs
    runWithArgs args
        
usage :: IO ()
usage =
    do
--    putStrLn "Usage A: simple-uv-et <ivp name> <output>.csv [Wrap|ShrinkWrap]"
    putStrLn $ "Usage: simple-uv-et <ivp name> <end time> \"<PlotArgs>\" [<output>.pdf|GUI]" 
                ++ " [Wrap|ShrinkWrap] <maxDeg> <maxTermSize> <maxUnitSplitDepth> <minUnitSplitDepth>"
    putStr $ unlines $ plotArgsHelpLines 
    
runWithArgs :: [String] -> IO ()
runWithArgs [ivpName, tEndS, maybePlotDimensS, maybePDFfilenameS, 
                wrapTypeS, maxDegS, maxSizeS, maxDepthS, minDepthS] =
    do
    _ <- solveVtPrintSteps wrapType maybePlotDimens maybePDFfilename ivpTEnd (maxDeg, maxSize, maxDepth, minDepth)
    return ()
    where
    ivpTEnd = 
        ivp
        {
            odeivp_tEnd = dblToReal sampleCf tEndD,
            odeivp_maybeExactValuesAtTEnd = Nothing
        }
    ivp = ivpByNameReportError ivpName sampleFn
    wrapType = read wrapTypeS :: WrapType
    maybePlotDimens = readIVPPlotArgs maybePlotDimensS :: Maybe IVPPlotArgs
    maybePDFfilename = readPDFfilename maybePDFfilenameS :: Maybe String
    maxDeg = read maxDegS :: Int
    maxSize = read maxSizeS :: Int
    maxDepth = read maxDepthS :: Int
    minDepth = read minDepthS :: Int
    tEndD = read tEndS :: Double
    readPDFfilename "GUI" = Nothing
    readPDFfilename pdfilename
        | ".pdf" `isSuffixOf` pdfilename = Just pdfilename
    readPDFfilename filename =
        error $ "Unsupported output file format: " ++ filename

    
runWithArgs [ivpName, outputFileName, wrapTypeS] =
    writeCSV ivp outputFileName wrapType
    where
    wrapType = read wrapTypeS :: WrapType
    ivp = ivpByNameReportError ivpName sampleFn
runWithArgs _ = usage

data WrapType =
    Wrap | ShrinkWrap
    deriving (Show, Read)    
    
writeCSV :: ODEIVP Fn -> FilePath -> WrapType -> IO ()
writeCSV ivp outputFileName wrapType =
    do
    isClash <- doesFileExist outputFileName
    case isClash of
        True -> putStrLn $ "file " ++ outputFileName ++ " exists"
        False ->
            withFile outputFileName WriteMode $ \ handle ->
                do
                hSetBuffering handle LineBuffering
                writeCSVheader handle
                mapM_ (runSolverMeasureTimeMSwriteLine handle) paramCombinations
    where
    paramCombinations = 
        [(maxDegree, depth) | 
            maxDegree <- [0..15], depth <- [0..10]]
--            maxDegree <- [0..10], depth <- [0..5]]
    maxSize = 100
    writeCSVheader handle =
        do
        hPutStrLn handle $ "ivp: " ++ description
--        hPutStrLn handle $ "polynomial degree, min step size (2^(-n)), time (microseconds), error upper bound at t=1, error at t = 1"
        hPutStrLn handle $ "polynomial degree, min step size (2^(-n)), time (microseconds), error at t = 1"
    runSolverMeasureTimeMSwriteLine handle (maxDegree, maxDepth) =
        do
        resultsAndTimes <- mapM solveAndMeasure ([1..1] :: [Int])
        let ((result, _) : _)  = resultsAndTimes
        let averageTime = average $ map snd resultsAndTimes
        hPutStrLn handle $ makeCSVLine ((result, averageTime), (maxDegree, maxDepth))
        where
        average list = (2 * (sum list) + n) `div` (2 * n)
            where
            n = fromIntegral $ length list
        solveAndMeasure _ =
            do
            starttime <- getCPUTime
            solverResult <- solveVtPrintSteps wrapType Nothing Nothing ivp (maxDegree, maxSize, maxDepth, 0)
            endtime <- getCPUTime
            return $ (solverResult, (endtime - starttime) `div` 1000000000)
        
    description = odeivp_description ivp
    maybeVecExact = odeivp_maybeExactValuesAtTEnd ivp
    makeCSVLine (((maybeVec, _), execTimeMS), (maxDegree, depth)) =
        show maxDegree ++ "," 
        ++ show depth ++ ","
        ++ show execTimeMS ++ ","
--        ++ enclosureErrorBoundS ++ ","
        ++ enclosureErrorS
        where
        enclosureErrorS =
            case maybeVec of
                Nothing -> show "no solution"
                Just vecOut ->
                        case maybeVecExact of
                            Just vecExact 
                                | not (refinesVec vecExact vecOut) -> --  && refinesVec vecIn vecExact) ->
                                    error $ 
                                        "enclosure error:"
                                        ++ "\n vecOut = " ++ show vecOut
                                        ++ "\n vecExact = " ++ show vecExact
--                                        ++ "\n vecIn = " ++ show vecIn
                                | otherwise -> 
                                    computeMaxDiff vecOut vecExact
                            _ -> show "exact solution not known"
                where
                computeMaxDiff vecOut vecOther = 
                    removeBracks $
                    show $ 
                        snd $ RefOrd.getEndpointsOut $ 
                            foldl1 max $ zipWith (CF.<->) (map CF.width vecOut) (map CF.width vecOther)
        removeBracks ('<': rest1 ) =
            reverse $ removeR $ reverse rest1
            where
            removeR ('>' : rest2 ) = rest2
            removeR _ = error "internal error: removeR"
        removeBracks _ = error "internal error: removeBracks"

refinesVec :: [CF] -> [CF] -> Bool
refinesVec vec1 vec2 =
    and $ zipWith refines vec1 vec2
refines :: CF -> CF -> Bool
refines a1 a2 = (a2 CF.|<=? a1) == Just True
   
solveVtPrintSteps ::
    (solvingInfo ~ (Maybe ([Fn],[Fn]), (CF, Maybe [CF])))
    => 
    WrapType
    ->
    (Maybe IVPPlotArgs)
    ->
    (Maybe FilePath)
    ->
    ODEIVP Fn 
    -> 
    (Int, Int, Int, Int) 
    -> 
    IO 
    (
        Maybe [CF]
    ,
        (
            BisectionInfo solvingInfo (solvingInfo, Maybe CF)
        )
    )
solveVtPrintSteps wrapType maybePlotDimens maybePDFfilename ivp (maxdegParam, maxsizeParam, minDepthParam, maxDepthParam) =
    do
    putStrLn "--------------------------------------------------"
    putStrLn "demo of enclose-flow by (Konecny, Taha, Duracz 2014)"
    putStrLn "--------------------------------------------------"
    putStrLn $ "solving: " ++ description
    putStrLn "----------  parameters: -------------------------"
    putStrLn $ "maxdeg = " ++ show maxdeg
    putStrLn $ "maxsize = " ++ show maxsize
    putStrLn $ "delta = " ++ show delta
    putStrLn $ "m = " ++ show m
--    putStrLn $ "substSplitSizeLimit = " ++ show substSplitSizeLimit
    putStrLn $ "minimum step size = 2^{" ++ show minStepSizeExp ++ "}"
    putStrLn $ "maximum step size = 2^{" ++ show maxStepSizeExp ++ "}"
    putStrLn $ "split improvement threshold = " ++ show splitImprovementThreshold
    putStrLn $ "wrapping: " ++ (show wrapType)
    putStrLn $ "plotting: " ++ show maybePlotDimens
    case maybeExactResult of
        Just exactResult ->
            putStr $ showSegInfo "(almost) exact result = " (Nothing, (tEnd, Just exactResult))
        _ -> return ()
--    putStrLn "----------  steps: ---------------------------"
--    _ <- printStepsInfo (1:: Int) bisectionInfoOut
    putStrLn "----------  step summary: -----------------------"
    putStrLn $ "number of atomic segments = " ++ (show $ bisectionInfoCountLeafs bisectionInfoOut)
    putStrLn $ "smallest segment size: " ++ (show smallestSegSize)  
    putStrLn "----------  result: -----------------------------"
    putStr $ showSegInfo ">>> " (Nothing, (tEnd, endValues))
    case shouldShowSteps of
        True ->
            do
            putStrLn "----------  splitting info: -----------------"
            putStrLn $ showBisectionInfo showSegInfo showSplitReason "" bisectionInfoOut
        False -> return ()
    putStrLn "-------------------------------------------------"
    case maybePlotDimens of
        Nothing -> return ()
        Just (IVPPlotArgs rectDbl activevars shouldUseParamPlot isBW) -> 
            plotODEIVPBisectionEnclosures
                rect activevars isBW
                shouldUseParamPlot effCf (2^^(-8 :: Int) :: CF) ivp bisectionInfoOut
                maybePDFfilename
            where
            rect = fmap (dblToReal 0) rectDbl :: FV.Rectangle CF
    return (endValues, bisectionInfoOut)
    where
    shouldShowSteps = False
--    shouldShowSteps = True
    -- solver call:
    (endValues, bisectionInfoOut) =
        solveIVPWithUncertainValue wrapType
            sizeLimits effCf substSplitSizeLimit delta m 
                minStepSize maxStepSize splitImprovementThreshold
                    ivp
    -- parameters:
    delta = 1
    maxdeg = maxdegParam
    maxsize = maxsizeParam
    m = 200
    substSplitSizeLimit = undefined
--    minStepSizeExp = -4 :: Int
    minStepSizeExp = - minDepthParam
    maxStepSizeExp = - maxDepthParam
    minStepSize = 2^^minStepSizeExp
    maxStepSize = 2^^maxStepSizeExp
    splitImprovementThreshold = 2^^(-50 :: Int)
    
    -- auxiliary:
    description = odeivp_description ivp
    tStart = odeivp_tStart ivp
    tEnd = odeivp_tEnd ivp
    maybeExactResult = odeivp_maybeExactValuesAtTEnd ivp
    componentNames = odeivp_componentNames ivp
    
    effCf = ArithInOut.roundedRealDefaultEffort sampleCf
    sizeLimits =
        getSizeLimits $
            makeSampleWithVarsDoms maxdeg maxsize [] []
            
    (smallestSegSize, _) =
        aux tStart (tEnd CF.<-> tStart) bisectionInfoOut
        where
        aux tPrev tSmallestSoFar (BisectionNoSplit (_,(tNow,_))) =
            (CF.minOut tSmallestSoFar (tNow CF.<-> tPrev), tNow)
        aux tPrev tSmallestSoFar (BisectionSplit _ left Nothing) =
            aux tPrev tSmallestSoFar left
        aux tPrev tSmallestSoFar (BisectionSplit _ left (Just right)) =
            aux tPrevL tSmallestSoFarL right
            where
            (tSmallestSoFarL, tPrevL) =
                aux tPrev tSmallestSoFar left
    
--    showStepInfo (n, segInfo@(_, (t, _))) =
--        "step " ++ show n ++ ": t = " ++ show t ++ "\n" ++ (showSegInfo "    " segInfo)
--    showStepInfo _ =
--        error "internal error: showStepInfo"
--    printStepsInfo n (BisectionNoSplit segInfo) =
--        do
--        putStrLn $ showStepInfo (n, segInfo)
--        return $ n + 1
--    printStepsInfo n (BisectionSplit _ left maybeRight) =
--        do
--        n2 <- printStepsInfo n left
--        case maybeRight of
--            Just right -> printStepsInfo n2 right
--            Nothing -> return $ n2 + 1
    
    showSegInfo indent (_, (t, maybeValues)) =
        unlines $ map showComponent $ zip componentNames valueSs
        where
        showComponent (name, valueS) =
            indent ++ name ++ "("  ++ show t ++ ") ∊ " ++ valueS
        valueSs =
            case maybeValues of
                Just valuesOut -> map showValue valuesOut
                _ -> replicate (length componentNames) "<no result computed>"
        showValue valueOut =
            show valueOut 
    showSplitReason indent (segInfo, (Just improvement)) =
        showSegInfo indent segInfo ++ 
        "; but splitting improves by " ++ show improvement ++ ":"
    showSplitReason indent (segInfo, Nothing) =
        showSegInfo indent segInfo ++ 
        " - thus splitting"


solveIVPWithUncertainValue ::
    (solvingInfo ~ (Maybe ([Fn],[Fn]), (CF, Maybe [CF])))
    =>
    WrapType ->
    SizeLimits Fn -> 
    ArithInOut.RoundedRealEffortIndicator CF -> 
    Int -> 
    CF -> 
    Int -> 
    CF -> 
    CF -> 
    CF -> 
    ODEIVP Fn
    -> 
    (
     Maybe ([CF])
    ,
        (
            BisectionInfo solvingInfo (solvingInfo, Maybe CF)
        )
    )
solveIVPWithUncertainValue
        wrapType 
            sizeLimits effCf _substSplitSizeLimit
                delta m minStepSize maxStepSize splitImprovementThreshold
                    odeivp
    =
    solveUncertainValueExactTimeBisect2
        delta m minStepSize maxStepSize splitImprovementThreshold
            odeivp
    where
    solveUncertainValueExactTimeBisect2 =
        solveODEIVPUncertainValueExactTime_UsingPicard_Bisect shouldWrap shouldShrinkWrap
            sizeLimits effSizeLims effCompose effEval effInteg effDeriv effInclFn 
            effAddFn effMultFn effAbsFn effMinmaxFn 
            effDivFnInt effAddFnDom effMultFnDom effCf
    (shouldWrap, shouldShrinkWrap) =
        case wrapType of
            Wrap -> (True, False)
            ShrinkWrap -> (False, True)

--    substituteInitialValueUncertainty fn =
--        pEvalAtPointOutEff effEval initValDomBox fn
--        where
--        initValDomBox =
--            fromList $ zip componentNames initialValues
--        
    effSizeLims = sizeLimitsChangeDefaultEffort sampleFn
    effCompose =  compositionDefaultEffort sampleFn -- (effCf, Int1To10 substSplitSizeLimit)
    effEval = evaluationDefaultEffort sampleFn -- (effCf, Int1To10 substSplitSizeLimit)
    effInteg = integrationDefaultEffort sampleFn
    effDeriv = fakeDerivativeDefaultEffort sampleFn
    effAddFn = ArithInOut.addDefaultEffort sampleFn
    effMultFn = ArithInOut.multDefaultEffort sampleFn
    effAbsFn = ArithInOut.absDefaultEffort sampleFn
    effMinmaxFn = NumOrd.minmaxInOutDefaultEffort sampleFn
    effAddFnDom = ArithInOut.mixedAddDefaultEffort sampleFn sampleCf
    effMultFnDom = ArithInOut.mixedMultDefaultEffort sampleFn sampleCf
--        ArithInOut.fldEffortMult sampleCf $ ArithInOut.rrEffortField sampleCf effCf
    effDivFnInt = ArithInOut.mixedDivDefaultEffort sampleFn (0::Int)
    effInclFn = RefOrd.pCompareDefaultEffort sampleFn -- ((Int1To1000 0, (effCf, Int1To10 20)), ())
--    effCf = ArithInOut.roundedRealDefaultEffort sampleCf

makeSampleWithVarsDoms :: 
     Int -> Int -> [Var Fn] -> [CF] -> Fn
makeSampleWithVarsDoms maxdeg maxsize vars doms =
    newConstFn sizeLimits varDoms sampleCf
    where
    varDoms = zip vars doms 
    sizeLimits =
        defaultLimits
        {
            ipolylimits_maxdeg = maxdeg,
            ipolylimits_maxsize = maxsize,
            ipolylimits_effort =
                (ipolylimits_effort defaultLimits)
                {
                    ipolyeff_evalMaxSplitSize = 100
                }
        }
        where
        defaultLimits =
            defaultIntPolySizeLimits sampleCf cf_limits arity
        arity = length vars
        cf_limits = defaultSizeLimits sampleCf


testShrinkWrap :: IO ()
testShrinkWrap =
    do
    mapM_ putStrLn $ map show $ getSides result
    return ()
    where
    getSides [x,y] =
        [(xSL,ySL),(xTL,yTL),(xSR,ySR),(xTR,yTR)]
        where
        xSL = pEvalAtPointOutEff effPEval sL x
        xSR = pEvalAtPointOutEff effPEval sR x
        xTL = pEvalAtPointOutEff effPEval tL x
        xTR = pEvalAtPointOutEff effPEval tR x
        ySL = pEvalAtPointOutEff effPEval sL y
        ySR = pEvalAtPointOutEff effPEval sR y
        yTL = pEvalAtPointOutEff effPEval tL y
        yTR = pEvalAtPointOutEff effPEval tR y
        sL = fromList [("s", -1)]
        sR = fromList [("s", 1)]
        tL = fromList [("t", -1)]
        tR = fromList [("t", 1)]
    getSides _ = error "internal error: getSides"
    
    Just result =
        shrinkWrap 
            effCompose effEval effDeriv effAddFn effAbsFn effMinmaxFn 
                effDivFnInt effAddFnDom effMultFnDom effCf 
                    [xUsingTS, yUsingTS]
    xUsingTS = s <*> tPlus2 <+> pmeps -- s(t+2) +- eps
    yUsingTS = (c1 <-> s <*> s) <*> tPlus2 <+> pmeps -- (1-s^2)(t+2) +- eps
    tPlus2 = (c1 <+> c1 <+> t)
    t = newProjectionFromSample c1 "t"
    s = (0.5 :: CF) |<*> newProjectionFromSample c1 "s"
    pmeps = newConstFnFromSample c1 $ ((-eps) CF.</\> eps)
        where
        eps = 0.125
    c1 :: Fn
    c1 = newConstFn sizeLimits varDoms (1 :: CF)
    varDoms = [("t", unitDom),  ("s", unitDom)]
    unitDom = (-1) CF.</\> 1
    sizeLimits =
        (defaultIntPolySizeLimits sampleCf cf_limits arity)
        {
            ipolylimits_cf_limits = defaultSizeLimits sampleCf,
            ipolylimits_maxdeg = 3,
            ipolylimits_maxsize = 1000
        }
        where
        arity = 2
        cf_limits = defaultSizeLimits sampleCf
    effCf = ArithInOut.roundedRealDefaultEffort sampleCf
    effEval = evaluationDefaultEffort sampleFn -- (effCf, Int1To10 substSplitSizeLimit)
    effPEval = partialEvaluationDefaultEffort sampleFn -- (effCf, Int1To10 substSplitSizeLimit)
    effCompose =  compositionDefaultEffort sampleFn -- (effCf, Int1To10 substSplitSizeLimit)
    effDeriv = fakeDerivativeDefaultEffort sampleFn
    effAddFn = ArithInOut.addDefaultEffort sampleFn
    effAbsFn = ArithInOut.absDefaultEffort sampleFn
    effMinmaxFn = NumOrd.minmaxInOutDefaultEffort sampleFn
--    effMinmaxFn = minmaxInOutDefaultEffortIntPolyWithBezierDegree 4 sampleFn
    effAddFnDom = ArithInOut.mixedAddDefaultEffort sampleFn sampleCf
    effMultFnDom = ArithInOut.mixedMultDefaultEffort sampleFn sampleCf
    effDivFnInt = ArithInOut.mixedDivDefaultEffort sampleFn (0::Int)
 