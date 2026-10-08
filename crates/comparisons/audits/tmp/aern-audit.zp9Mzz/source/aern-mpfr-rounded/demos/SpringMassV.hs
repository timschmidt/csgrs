{-
  Solve the IVP y''=-100y with y(0) = 20, y'(0) = 0  
  using iterated interval Picard operator at each step
  over interval polynomials in three variables:
    u for time spanning the step (0 <= u <= h)
    y0 for the initial value of y to account for interval uncertainty
    y0Der for the initial value of y'

  best result so far: 
    y(36688) in 34 decimal digit precision in 50.5h on 2.8GHz i5
-}

module Main where

import Numeric.AERN.RealArithmetic.RefinementOrderRounding
import Numeric.AERN.RealArithmetic.Basis.MPFR
import Numeric.AERN.MPFRBasis.Interval
import Numeric.AERN.Basics.ShowInternals

import Numeric.AERN.Misc.Debug

import qualified Data.List as List

import System.IO

shouldPrintIterations = False
--shouldPrintIterations = True
shouldPrintPrecision = False
--shouldPrintPrecision = True

main =
    do
    hSetBuffering stdout NoBuffering
    print initParams 
    putStrLn $ "initial state: " ++ show (fst $ head steps)
    mapM_ printStep $ tail steps 
    print initParams 
--    mapM_ print $ paramSearch 0 [initParams]
    where
    steps = simulate initParams

data Params =
    Params
    {
        paramStepSize :: Int, -- 2^{-s} step size
        paramStepEpsilon :: Int, -- 2^{-n} stop iterating Picard when the width change falls below this
        paramPrecision :: Precision, -- floating point precision
        paramDegree :: Int, -- maximal polynomial degree
        paramRangeTolerance :: Int 
            -- 2^{-r} how hard to try to estimate ranges of polynomials 
            --        by splitting - currently unused
    }
    deriving (Show)

--initParams = Params 1 50 50 50 10 -- gets to time 6.5 in 0.62s (1 in 0.1s, 3.6 in 0.34s)
initParams = Params 0 100 100 80 10 -- gets to time 99 in 17.5s (36 in 6.4s)
--initParams = Params 0 200 200 160 10 -- gets to time 312 in 123s 
--initParams = Params (-2) 200 200 190 10 -- gets to time 564 in 198s (360 in 109s) drops at e-19
--initParams = Params (-3) 300 300 350 10 -- gets to time 3176 in 37.25min
--initParams = Params (-2) 300 300 300 10 -- gets to time 1400 in 15min
--initParams = Params (-3) 400 400 450 10 -- gets to time 8744 in 2.5h (3600 in 1h2min) starts at e-82 drops at e-35 using 2.2GB RAM
--initParams = Params (-3) 600 600 800 10 -- gets to time 19768 in 11h55min starts at e-142 drops at e-35 using 3.5GB RAM
--initParams = Params (-4) 600 600 800 10 -- gets to time 7712 in 6.5h (3600 in 3h) drops at e-70
--initParams = Params (-3) 700 700 850 10 -- gets to time 25448 in 20h using 4.25GB RAM
--initParams = Params (-3) 900 900 950 10 -- gets to time 36688 in 51.5h using 6GB RAM (36000 in 50.5h)

increaseParams (Params stp eps p dg rtol) =
    [
     Params stp eps p dgNew rtol, 
     Params stp epsNew pNew dg rtol, 
     Params stp epsNew pNew dgNew rtol
    ]
    where
    pNew = p + 2000
    dgNew = dg + 200
    epsNew = eps + 2000

paramSearch prevBest nextParamsOptions@(params : rest)
    | prevBest > 36000 = []
    | otherwise =
        (params, currentScoreSeq, currentScore) : 
            (case (rest, currentScore > prevBest + 1) of
                ([], _) -> paramSearch currentScore $ increaseParams params
                (_, True) -> paramSearch currentScore $ increaseParams params
                _ -> paramSearch prevBest rest
            )
         where
         currentScoreSeq =
            drop100ifAny 0 $ zip [0..] steps
         drop100ifAny n [] = []
         drop100ifAny n list 
            = 
            (n `div` stepsInOne) 
            : (drop100ifAny (n + 100) $ drop 100 list)
         currentScore =
            (length steps) `div` stepsInOne
         steps = simulate params
         stepsInOne = 2 ^ (paramStepSize params)

simulate params =
    waitTillPrecBelow (2^^(-30)) $ -- stop when diverging
        iterate (makeStep params stepSize epsilon) 
            ((i2mi 0, i2mi 20, i2mi 0),[]) -- initial values
    where
    waitTillPrecBelow eps (this@((_,yNmOne,_),_):rest)
        | precGood = this : (waitTillPrecBelow eps rest)
        | otherwise = [this]
        where
        precGood = 
            case width yNmOne <? eps of
                Just True -> True
                _ -> False
    epsilon =
        (i2mi 1) </> (i2mi $ 2^(paramStepEpsilon params))
    stepSize
        | stp >= 0 =(i2mi 1) </> (i2mi $ 2^stp)
        | otherwise = i2mi $ 2^(- stp)
        where
        stp = paramStepSize params
    i2mi :: Integer -> MI
    i2mi n =
        convertOutEff (paramPrecision params) n
    
printStep ((t,y,yDer), iters) =
    do
    putStrLn $ replicate 120 '*'
    putStrLn $ "iterating interval Picard for t = " ++ show t ++ ":"
    case shouldPrintIterations of
        True -> 
            mapM_ printIter $ zip [1..] $ zip (tail iters) iters
        False -> return ()
    putStrLn $ "after " ++ show (length iters) ++ " iterations stabilised on:" 
    putStrLn $ " t = " ++ show t ++ "; y(t) = " ++ showMI y ++ "; y'(t) = " ++ showMI yDer
    putStrLn $ " width of y = " ++ showMI (width y)
    where
    printIter (n, (((y,yDer),(yPoly,yDerPoly)), ((yPrev,yDerPrev),_))) =
        do
        putStrLn $ " >>>>>>>>>>>>>>>>> iteration " ++ show n ++ ":"
        putStrLn $ "  y = " ++ showPoly yPoly ++ "; "
        putStrLn $ "  y' = " ++ showPoly yDerPoly ++ "; "
        putStrLn $ "  y(h) = " ++ show y ++ "; " ++ "y'(h) = " ++ show yDer ++ "; "
        putStrLn $ "  width of y(h) = " ++ showMI (width y)
        putStrLn $ "  Improvement: " ++ show (yPrev |<=? y)
    showMI = showInternals (30,shouldPrintPrecision)
    
makeStep params h epsilon ((t, y0, y0Der), prevStepIters) =
--    unsafePrint
--    (
--        "makeStep:"
--        ++ "\n h = " ++ showMI h
--        ++ "\n t = " ++ showMI t
--        ++ "\n y0 = " ++ showMI y0
--        ++ "\n y0Der = " ++ showMI y0Der
--        ++ "\n yh = " ++ showMI yh
--        ++ "\n yhDer = " ++ showMI yhDer
--    )
    ((t + h, yh, yhDer), newStepIters)
    where
--    showMI = showInternals (20, shouldPrintPrecision)
    ((yh, yhDer), newStepIters) 
        = waitTillNoImprovementCheckInclusion False 1000 [] intPolySequence
    intPolySequence = map evalBoth polySequence
    polySequence
        =
        iterate (reduceBoth . picard zero (y0Poly, y0DerPoly)) $ 
        (yFirstEncl, yDerFirstEncl)
        
    yFirstEncl
        = mkConstPoly ["u","y0","y0Der"] $ y0 <+> hToMinusH
    yDerFirstEncl
        = mkConstPoly ["u","y0","y0Der"] $ y0Der <+> hToMinusH
    y0Poly 
        = V "u" [V "y0" [G "y0Der" [one], G "y0Der" [zero]]] 
        -- y0 as a polynomial of three variables u, y0 and y0Der 
    y0DerPoly 
        = V "u" [V "y0" [G "y0Der" [one,zero]]] 
        -- y0Der as a polynomial of three variables u, y0 and y0Der 
    reduceBoth (p1,p2) = (reduceOne p1, reduceOne p2)
        where
        reduceOne p 
            = reducePoly 0 0 rtol zero [hTo0,y0,y0Der] dg p 
    evalBoth (p1,p2) = ((evalOne p1, evalOne p2), (p1, p2))
        where
--        evalOne = evalPoly zero [h,y0,y0Der]
        evalOne = evalPolySplit 1 0 rtol zero [h,y0,y0Der]

--    -- uncomment this version to use univariate polynomials instead of the above: 
--    yFirstEncl
--        = mkConstPoly ["u"] $ y0 <+> hToMinusH
--    yDerFirstEncl
--        = mkConstPoly ["u"] $ y0Der <+> hToMinusH
--    y0Poly 
--        = G "u" [y0] 
--        -- y0 as a polynomial of three variables u, y0 and y0Der 
--    y0DerPoly 
--        = G "u" [y0Der] 
--        -- y0Der as a polynomial of three variables u, y0 and y0Der 
--    reduceBoth (p1,p2) 
--        = 
--        (reducePoly zero [hTo0] dg p1, 
--         reducePoly zero [hTo0] dg p2)
--    evalBoth (p1,p2) = ((evalOne p1, evalOne p2), (p1, p2))
--        where
--        evalOne = evalPoly zero [h]

    hTo0 = zero </\> h -- domain of u
    hToMinusH = (-h) </\> h

    dg = paramDegree params
    rtol = 
        (i2mi 1) </> (i2mi $ 2^(paramRangeTolerance params))
    i2mi :: Integer -> MI
    i2mi n = convertOutEff prec n
    prec = paramPrecision params
    zero = i2mi 0
    one = i2mi 1 
    waitTillNoImprovementCheckInclusion wasInclusion maxIters prevIters (this@(thisInts@(yNmOne,_),_):rest@(((yN,_),_):_)) 
        | maxIters < 0 = (thisInts, reverse currIters)
--        | wasInclusion && (not isInclusion) =
--            error "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX internal error: inclusion ceased"
        | (not isInclusion) || improvementAboveEpsilon  = 
            waitTillNoImprovementCheckInclusion isInclusion (maxIters - 1) currIters rest
        | otherwise = (thisInts, reverse currIters)
        where
        currIters = this : prevIters
        isInclusion =
            case yNmOne |<=? yN of
                Just r -> r
                _ -> False 
        improvementAboveEpsilon =
            case yNmOneWidth - yNWidth >? epsilon of
                Just True -> True
                _ -> False 
        yNmOneWidth = width yNmOne
        yNWidth = width yN         
    
    
picard ::
--    MI {-^ starting time -} -> -- autonomous system, always 0 
    MI {-^ zero with correct precision -} ->
    (Poly, Poly) {-^ initial values for y,y' -} -> 
    (Poly, Poly) {-^ approximates of y and y' -} -> 
    (Poly, Poly) {-^ improved approximates of y and y' -}
picard zero (y0, y0Der) (yPrev, yPrevDer) =
    (yNext, yNextDer)
    where
    yNext =
        integratePoly zero y0 yNextDer
    yNextDer =
        integratePoly zero y0Der (field yPrev)
    field y = scalePoly (-100) y
    
--stepOpt (Interval yPrevL yPrevR) (Interval yPrevDerL yPrevDerR) (Interval h _) =
--    (Interval yNextL yNextR, Interval yNextDerL yNextDerR)
--    where
--    yNextDerL
--        = yPrevDerL -. ((h *^ h +^ h) *^ yPrevR)
--    yNextDerR
--        = yPrevDerR -^ ((h *. h +. h) *. yPrevL)
    
    
{--- poor man's polynomials  ---}
data Poly = 
        G String [MI] -- uni-variate with constant term last
    |   V String [Poly] -- multi-variate poly
    deriving Show

--instance Show Poly where show = showPoly

showPoly :: Poly -> String
showPoly poly =
    sp "" poly
    where
    sp otherVars (G xName coeffs) 
        = List.intercalate " + " $ zipWith showTerm coeffs [degree,degree-1..0]
        where
        degree = length coeffs - 1
        showTerm coeff 0 
            = 
            case otherVars of
                "" -> showCoeff coeff
                _ -> showCoeff coeff ++ "*" ++ otherVars
        showTerm coeff n = showCoeff coeff ++ "*" ++ otherVars ++ (showVar n)
        showCoeff = showInternals (10,shouldPrintPrecision)
        showVar 0 = ""
        showVar 1 = xName
        showVar n = xName ++ "^" ++ show n
    sp otherVars (V xName polys) =
        List.intercalate " + " $ zipWith showTerm polys [degree,degree-1..0]
        where
        degree = length polys - 1
        showTerm p n = sp (otherVars ++ showVar n) p
        showVar 0 = ""
        showVar 1 = xName
        showVar n = xName ++ "^" ++ show n

polyDegree :: Poly -> Int
polyDegree (G _ coeffs) = length coeffs - 1
polyDegree (V _ polys@(_:_)) = 
    foldl1 max $ zipWith (+) (map polyDegree polys) [dg,dg-1..0]
    where
    dg = length polys - 1

mkConstPoly :: [String] -> MI -> Poly
mkConstPoly [xName] c = G xName [c]
mkConstPoly (xName:rest) c = V xName [mkConstPoly rest c]

mkConstFromPoly :: Poly -> MI -> Poly
mkConstFromPoly (G xName _) c = G xName [c]
mkConstFromPoly (V xName (rest:_)) c = V xName [mkConstFromPoly rest c]

polyNormalise :: Poly -> Poly
polyNormalise (G xName coeffs) 
    = G xName $ removeZeros coeffs
    where
    removeZeros [c] = [c]
    removeZeros coeffs@(c : rest)
        | isZero c = removeZeros rest
        | otherwise = coeffs
        where
        isZero c = (c |==? 0) == Just True
polyNormalise (V xName polys) 
    = V xName $ removeZeros $ map polyNormalise polys
    where
    removeZeros [p] = [p]
    removeZeros polys@(p : rest)
        | isZeroPoly p = removeZeros rest
        | otherwise = polys

isZeroPoly (G xName [c]) = (c |==? 0) == Just True
isZeroPoly (V xName [p]) = isZeroPoly p
isZeroPoly _ = False

evalPoly :: MI -> [MI] -> Poly -> MI
evalPoly zero [x] (G _ coeffs@(coeffHighestPower : rest)) 
    =
    aux rest x coeffHighestPower
    where
    aux [] _ acc = acc
    aux (coeffHighestPower : rest) x acc =
        aux rest x newAcc
        where
        newAcc = coeffHighestPower <+> (x <*> acc)
evalPoly zero (x : rest) (V _ polys) =
    foldl (<+>) zero $ zipWith (<*>) xPowers $ map (evalPoly zero rest) polys
    where
    xPowers = mkPowers (zero <+> 1) [] polys
    mkPowers xN acc [] = acc
    mkPowers xN acc (_:rest) = mkPowers (xN <*> x) (xN : acc) rest

evalPolyMono :: Int -> Int -> MI -> MI -> [MI] -> Poly -> MI
evalPolyMono maxDiffDepth maxSplitDepth tolerance zero doms p 
    =
--    unsafePrint
--    (
--        "evalPolyMono:"
--        ++ "\n p = " ++ showPoly p
--        ++ "\n doms = " ++ show doms
--        ++ "\n result = " ++ show result
--    )
    result
    where
    result
        | maxDiffDepth < 1 || degree < 2 = evalP doms
        | and $ map nonZero derivativeRanges
            = foldl1 (</\>) $ map evalP corners
        | otherwise = evalP doms
    degree = polyDegree p
    evalP doms = evalPoly zero doms p
    corners =
        allCombinations $ map getEndpointsAsInts doms
        where
        allCombinations :: [[a]] -> [[a]]
        allCombinations [] = [[]]
        allCombinations (options : rest) =
            [option : restCombination | 
                option <- options, 
                restCombination <- allCombinations rest] 
    getEndpointsAsInts x =
        [xLI, xRI]
        where
        xLI = Interval xL xL
        xRI = Interval xR xR
        (xL, xR) = getEndpoints x
    nonZero x =
        case x |<=? 0 of
            Just r -> not r; _ -> False
    derivativeRanges = 
        map (evalPolySplit (maxDiffDepth - 1) maxSplitDepth tolerance zero doms) pDers
    pDers = [diffPoly zero 0 p, diffPoly zero 1 p, diffPoly zero 2 p]
        

type Box = [MI]
type Partition = [Box]
        
evalPolySplit :: Int -> Int -> MI -> MI -> Box -> Poly -> MI
evalPolySplit maxDiffDepth maxSplitDepth tolerance zero doms p
    = 
--    unsafePrint
--    (
--        "evalPolySplit:"
--        ++ "\n p = " ++ showPoly p
--        ++ "\n doms = " ++ show doms
--        ++ "\n result = " ++ show result
--    )
    result
    where
    result
        = fst $ aux maxSplitDepth doms
--    = aux doms
    aux :: Int -> Box -> (MI, Partition)
    aux maxDepth doms
        | maxDepth < 1 || length allPartitions < 2 || not bestPartitionImproves =
            (evalPolyMono maxDiffDepth maxSplitDepth tolerance zero doms p, [doms])
        | otherwise =
            mergeResults $ unzip $ map (aux (maxDepth - 1)) bestPartition
        where
        mergeResults (values, domSeqs) =
            (foldl1 (</\>) values, concat domSeqs)
        value = evalP doms
        bestPartitionImproves =
            case (width value) - (bestPartitionValueWidth) >=? tolerance of
                Just r -> r; _ -> False
        (bestPartitionValueWidth, _, bestPartition) 
            = head $ List.sort $ map addValueWidth $ zip [1..] allPartitions
            where
            addValueWidth (n, partition) = (width pValue, n, partition)
                where
                pValue = foldl1 (</\>) $ map evalP partition 
        allPartitions :: [Partition]
        allPartitions = concat $ map splitDom $ isolateAll doms
            where
            isolateAll :: [a] -> [([a],a,[a])]
            isolateAll list = map splitN [0..length list-1]
                where
                splitN n = (pre, this, post)
                    where
                    (pre, this : post) = splitAt n list
            splitDom (pre, dom, post) 
                | domIsThin = []
                | otherwise 
                    = 
                    [[pre ++ [domL] ++ post, 
                      pre ++ [domR] ++ post]]
                where
                (domL, domR) = bisect Nothing dom
                domIsThin = (domL |==? domR) == Just True
                
    evalP doms = evalPoly zero doms p
        
negPoly :: Poly -> Poly
negPoly (G x coeffs) = G x $ map negate coeffs
negPoly (V x polys) = V x $ map negPoly polys

scalePoly :: MI -> Poly -> Poly
scalePoly c (G x coeffs) = G x $ map (\t -> t <*> c) coeffs
scalePoly c (V x polys) = V x $ map (scalePoly c) polys

scalePolyByInt :: Int -> Poly -> Poly
scalePolyByInt n (G x coeffs) = G x $ map (\t -> t <*>| n) coeffs
scalePolyByInt n (V x polys) = V x $ map (scalePolyByInt n) polys

divPolyByInt :: Poly -> Int -> Poly
divPolyByInt (G x coeffs) n = G x $ map (\cf -> cf </>| n) coeffs
divPolyByInt (V x polys) n = V x $ map (\p -> divPolyByInt p n) polys

integratePoly :: MI -> Poly -> Poly -> Poly
integratePoly zero initPoly (G x coeffs) =
    addPolys initPoly (G x $ coeffsFractions ++ [zero])
    where
    coeffsFractions = zipWith (</>|) coeffs [degreePlusOne,degreePlusOne-1..]
    degreePlusOne = length coeffs
integratePoly zero initPoly p@(V x polys@(polyHighest:_)) =
--    unsafePrint
--    (
--        "integratePoly:"
--        ++ "\n initPoly = " ++ showPoly initPoly
--        ++ "\n p = " ++ showPoly p
--        ++ "\n result = " ++ showPoly result
--    )
    result
    where
    result 
        = 
        addPolys initPoly (V x $ polysFractions ++ [mkConstFromPoly polyHighest zero]) 
    polysFractions = zipWith divPolyByInt polys [degreePlusOne,degreePlusOne-1..]
    degreePlusOne = length polys

diffPoly :: MI -> Int -> Poly -> Poly
diffPoly zero 0 (G x [c]) = G x [zero]
diffPoly zero 0 (G x coeffs) =
    G x $ coeffsMultiples
    where
    coeffsMultiples = zipWith (<*>|) coeffs [degree,degree-1..1]
    degree = length coeffs - 1
diffPoly zero 0 (V x [p]) = V x $ [mkConstFromPoly p zero]
diffPoly zero 0 (V x polys) =
    V x $ polysMultiples
    where
    polysMultiples = zipWith scalePolyByInt [degree,degree-1..1] polys
    degree = length polys - 1
diffPoly zero n (V x polys@(p:_)) =
    polyNormalise $
        V x $ map (diffPoly zero (n-1)) polys

{- 
    Reduce degree by replacing some occurences of variables
    in high degree terms by their domains.
-}
reducePoly :: 
    Int  ->
    Int ->
    MI ->
    MI ->
    [MI] {-^ the domain for each var -} -> 
    Int ->
    Poly -> Poly
reducePoly maxDiffDepth maxSplitDepth tolerance zero doms maxDeg poly
    =
--    unsafePrint
--    (
--        "reducePoly:"
--        ++ "\n doms = " ++ show doms
--        ++ "\n maxDeg = " ++ show maxDeg
--        ++ "\n poly = " ++ showPoly poly
--        ++ "\n reductionErrorBoundPoly = " ++ showPoly reductionErrorBoundPoly
--        ++ "\n result = " ++ showPoly result
--        ++ "\n eval over all domains poly:" ++ show (evalPoly zero doms poly)
--        ++ "\n eval over all domains result:" ++ show (evalPoly zero doms result)
--        ++ "\n inclusion test result: " ++ show (evalPoly zero doms result |<=? evalPoly zero doms poly)
--    )
    result
    where
    result
        | maxDeg >= (polyDegree poly) = poly
        | otherwise = reducedPoly
    reducedPoly
        = r doms maxDeg poly
    r [dom] maxDeg orig@(G x coeffs) 
        | maxDeg >= degree = orig
        | otherwise 
            =
            (G x $ newMaxDegreeCoeff : polyLowDegreeCoeffs)
        where
        degree = length coeffs - 1
        (polyHighDegreeCoeffs, polyMaxDegreeCoeff : polyLowDegreeCoeffs) 
            = splitAt (degree - maxDeg) coeffs
        newMaxDegreeCoeff 
            = polyMaxDegreeCoeff <+> highDegreeCoeffCompens
        highDegreeCoeffCompens 
            = dom <*> (evalPolySplit maxDiffDepth maxSplitDepth tolerance zero [dom] $ G x polyHighDegreeCoeffs)
            
    r doms@(dom:restDoms) maxDeg orig@(V x polys@(poly:_))
        | maxDeg >= degree 
            = 
            V x reducedPolys
        | otherwise
            = 
            V x reducedLowTermPolys
        where
        degree = length polys - 1
        reducedPolys
            = map rr $ zip polys [degree,degree-1..]
        rr (poly, termDegree) =
            r restDoms (maxDeg - termDegree) poly

        -- the following are only used when maxDeg < degree:
        (polyHighDegreePolys, polyMaxDegreePoly : polyLowDegreePolys)
            = splitAt (degree - maxDeg) polys
        reducedLowTermPolys
            = map rr $ zip (newMaxDegreePoly : polyLowDegreePolys) [maxDeg,maxDeg-1..]
        newMaxDegreePoly 
            = polyMaxDegreePoly `addPolys` highDegreePolysCompens
        highDegreePolysCompens 
            =  
            foldl1 addPolys $
                zipWith scalePoly domPowers polyHighDegreePolys
        domPowers = mkPowers dom [] polyHighDegreePolys
        mkPowers domN acc [] = acc
        mkPowers domN acc (_:rest) = mkPowers (domN <*> dom) (domN : acc) rest

{- 
    Reduce degree by removing all terms whose degree is too high
    and compensating for these tersm.
-}
reducePolySym :: 
    Int  ->
    Int ->
    MI ->
    MI ->
    [MI] {-^ the domain for each var -} -> 
    Int ->
    Poly -> Poly
reducePolySym maxDiffDepth maxSplitDepth tolerance zero doms maxDeg poly
    =
--    unsafePrint
--    (
--        "reducePoly:"
--        ++ "\n doms = " ++ show doms
--        ++ "\n maxDeg = " ++ show maxDeg
--        ++ "\n poly = " ++ showPoly poly
--        ++ "\n reductionErrorBoundPoly = " ++ showPoly reductionErrorBoundPoly
--        ++ "\n result = " ++ showPoly result
--        ++ "\n eval over all domains poly:" ++ show (evalPoly zero doms poly)
--        ++ "\n eval over all domains result:" ++ show (evalPoly zero doms result)
--        ++ "\n inclusion test result: " ++ show (evalPoly zero doms result |<=? evalPoly zero doms poly)
--    )
    result
    where
    result
        | maxDeg >= (polyDegree poly) = poly
        | otherwise =
            addPolyConst reducedPoly reductionErrorBound
    reductionErrorBound 
        =
        evalPolySplit maxDiffDepth maxSplitDepth tolerance zero doms reductionErrorBoundPoly
--        evalPoly zero doms reductionErrorBoundPoly
    (reducedPoly, reductionErrorBoundPoly)
        = r doms maxDeg poly
    r [dom] maxDeg orig@(G x coeffs) 
        | maxDeg >= degree = (orig, G x [zero])
        | otherwise 
            =
            (G x $ polyLowDegreeCoeffs
            ,
             G x $ polyHighDegreeCoeffs ++ (replicate (maxDeg + 1) zero)
            )
        where
        degree = length coeffs - 1
        (polyHighDegreeCoeffs, polyLowDegreeCoeffs) 
            = splitAt (degree - maxDeg) coeffs
    r doms@(dom:restDoms) maxDeg orig@(V x polys@(poly:_))
        | maxDeg >= degree 
            = (V x reducedPolys
              , 
               V x errorBounds
              )
        | otherwise
            = (V x reducedLowTermPolys
              , 
               omitedTermsError `addPolys` (V x lowTermErrorBounds)
              )
        where
        degree = length polys - 1
        (reducedPolys, errorBounds)
            = unzip $ map rr $ zip polys [degree,degree-1..]
        rr (poly, termDegree) =
            r restDoms (maxDeg - termDegree) poly
        -- the following are only used when maxDeg < degree:
        (polyHighDegreePolys, polyLowDegreePolys)
            = splitAt (degree - maxDeg) polys
        omitedTermsError 
            =
            V x $ polyHighDegreePolys 
                    ++ (replicate (maxDeg + 1) $ mkConstFromPoly poly zero)
        (reducedLowTermPolys, lowTermErrorBounds)
            = unzip $ map rr $ zip polyLowDegreePolys [maxDeg,maxDeg-1..]


--        domPowers = mkDomPowers [] 1 0 maxDeg
--        mkDomPowers prevPowers currPower n nLeft
--            | nLeft == 0 = prevPowers
--            | otherwise =
--                mkDomPowers currPowers nextPower (n+1) (nLeft-1)
--            where
--            currPowers = currPower : prevPowers
--            nextPower
--                | odd n = halfPower <*> halfPower
--                | otherwise = currPower <*> dom 
--                where
--                halfPower = currPowers !! ((n+1) `div` 2)
            
             
           
addPolys :: Poly -> Poly -> Poly
addPolys poly1@(G xName1 coeffs1) poly2@(G xName2 coeffs2) 
    | lengthPoly1 == lengthPoly2 =
        G xName1 $ zipWith (<+>) coeffs1 coeffs2
    | lengthPoly1 > lengthPoly2 =
        G xName1 $ highCoeffs1 ++ (zipWith (<+>) lowCoeffs1  coeffs2)
    | otherwise =
        G xName1 $ highCoeffs2 ++ (zipWith (<+>) lowCoeffs2  coeffs1)
    where
    (highCoeffs2, lowCoeffs2) = splitAt (lengthPoly2 - lengthPoly1) coeffs2
    (highCoeffs1, lowCoeffs1) = splitAt (lengthPoly1 - lengthPoly2) coeffs1
    lengthPoly1 = length coeffs1
    lengthPoly2 = length coeffs2
addPolys poly1@(V xName1 polys1) poly2@(V xName2 polys2) 
    | lengthPoly1 == lengthPoly2 =
        V xName1 $ zipWith addPolys polys1 polys2
    | lengthPoly1 > lengthPoly2 =
        V xName1 $ highPolys1 ++ (zipWith addPolys lowPolys1  polys2)
    | otherwise =
        V xName1 $ highPolys2 ++ (zipWith addPolys lowPolys2  polys1)
    where
    (highPolys2, lowPolys2) = splitAt (lengthPoly2 - lengthPoly1) polys2
    (highPolys1, lowPolys1) = splitAt (lengthPoly1 - lengthPoly2) polys1
    lengthPoly1 = length polys1
    lengthPoly2 = length polys2
--addPolys poly1@(V xName1 polys1) poly2@(V xName2 polys2) 
    

addPolyConst :: Poly -> MI -> Poly
addPolyConst (G x []) const = (G x [const])
addPolyConst (G x coeffs) const =
    G x $ aux coeffs
    where
    aux [c] = [c <+> const]
    aux (t : ts) = t : (aux ts)
addPolyConst (V x polys) const =
    V x $ aux polys
    where
    aux [p] = [addPolyConst p const]
    aux (t : ts) = t : (aux ts)  
    

xX = G "x" [1,0]
xX2MX = G "x" [1,-1,0]
xX3MX2X = G "x" [1,-1,1,0]

xyC1 = mkConstPoly ["x","y"] 1
xyX = V "x" [G "y" [1], G "y" [0]]
xyY = V "x" [G "y" [1,0]]
xyXY = addPolys xyX xyY
xyXY1 = addPolys xyXY xyC1
t = integratePoly 0 xyC1 xyXY1

