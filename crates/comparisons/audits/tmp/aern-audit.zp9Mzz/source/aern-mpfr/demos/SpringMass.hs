module Main where

import Numeric.AERN.RealArithmetic.RefinementOrderRounding
import Numeric.AERN.RealArithmetic.Basis.MPFR
import Numeric.AERN.MPFRBasis.Interval
import Numeric.AERN.Basics.ShowInternals

import qualified Data.List as List

import System.IO

main =
    do
    hSetBuffering stdout NoBuffering 
    mapM_ print $ simulate initParams
--    mapM_ print $ paramSearch 0 [initParams]

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

data Params =
    Params
    {
        paramPrecision :: Precision,
        paramDegree :: Int,
        paramStepEpsilon :: Int, -- 2^{-n}
        paramStepSize :: Int -- 2^{-s}
    }
    deriving (Show)

initParams = Params 100 18 100 3 -- gets to time 1 in < 30ms
--initParams = Params 600 110 600 3 -- gets to time 36 in 16s
--initParams = Params 600 110 600 5 -- gets to time 38 in 45s
--initParams = Params 6000 800 6000 3 -- gets to time 413 in 16h
--initParams = Params 6000 1200 6000 3

increaseParams (Params p dg eps sz) =
    [
     Params p dgNew eps sz, 
     Params pNew dg epsNew sz, 
     Params pNew dgNew epsNew sz
    ]
    where
    pNew = p + 2000
    dgNew = dg + 200
    epsNew = eps + 2000

simulate params =
    waitTillPrecBelow (2^^(-30)) $ -- stop when diverging
        iterate (makeStep (paramDegree params) stepSize epsilon) 
            (0, i2mi 20, i2mi 0) -- initial values
    where
    waitTillPrecBelow eps (this@(_,yNmOne,_):rest)
        | precGood = this : (waitTillPrecBelow eps rest)
        | otherwise = [this]
        where
        precGood = 
            case width yNmOne <? eps of
                Just True -> True
                _ -> False
    epsilon =
        (i2mi 1) </> (fromInteger $ 2^(paramStepEpsilon params))
    stepSize =
        (i2mi 1) </> (fromInteger $ 2^(paramStepSize params))
    i2mi :: Integer -> MI
    i2mi n =
        convertOutEff (paramPrecision params) n
    
makeStep dg h epsilon (t, y0, y0Der) =
    (t + h, yh, yhDer)
    where
    (yh, yhDer) =
        waitTillNoImprovement 1000 $
        map evalBoth $
        iterate (reduceBoth . picard (y0, y0Der)) $ 
        ([y0 + (plusMinus h)], [y0Der + (plusMinus h)]) 
    reduceBoth (p1,p2) = (reducePoly h dg p1, reducePoly h dg p2)
    evalBoth (p1,p2) = (evalPoly h p1, evalPoly h p2) 
    waitTillNoImprovement maxIters (this@(yNmOne,_):rest@((yN,_):_)) 
        | maxIters < 0 = this
        | improvementAboveEpsilon = 
            waitTillNoImprovement (maxIters - 1) rest
        | otherwise = this
        where
        improvementAboveEpsilon =
            case yNmOneWidth - yNWidth >? epsilon of
                Just True -> True
                _ -> False 
        yNmOneWidth = width yNmOne
        yNWidth = width yN         
    
    
plusMinus h = (-h) </\> h
    
picard ::
--    MI {-^ starting time -} -> -- autonomous system, always 0 
    (MI, MI) {-^ initial values for y,y' -} -> 
    (Poly, Poly) {-^ approximates of y and y' -} -> 
    (Poly, Poly) {-^ improved approximates of y and y' -}
picard (y0, y0Der) (yPrev, yPrevDer) =
    (yNext, yNextDer)
    where
    yNext =
        integratePoly y0 yNextDer
    yNextDer =
        integratePoly y0Der (field yPrev)
    field y = scalePoly (-100) y
    
--stepOpt (Interval yPrevL yPrevR) (Interval yPrevDerL yPrevDerR) (Interval h _) =
--    (Interval yNextL yNextR, Interval yNextDerL yNextDerR)
--    where
--    yNextDerL
--        = yPrevDerL -. ((h *^ h +^ h) *^ yPrevR)
--    yNextDerR
--        = yPrevDerR -^ ((h *. h +. h) *. yPrevL)
    
    
{--- poor man's polynomials  ---}
type Poly = [MI] -- uni-variate with constant term last

showPoly :: Poly -> String
showPoly [] = "0"
showPoly poly =
    List.intercalate " + " $ zipWith showTerm poly [degree,degree-1..0]
    where
    degree = length poly - 1
    showTerm coeff 0 = showCoeff coeff
    showTerm coeff n = showCoeff coeff ++ "*" ++ (showVar n)
    showCoeff = showInternals (10,False)
    showVar 1 = "x"
    showVar n = "x^" ++ show n 

evalPoly :: MI -> Poly -> MI
evalPoly x poly@(coeffHighestPower : rest) =
    aux rest x coeffHighestPower
    where
    aux [] _ acc = acc
    aux (coeffHighestPower : rest) x acc =
        aux rest x newAcc
        where
        newAcc = coeffHighestPower <+> (x <*> acc)

negPoly :: Poly -> Poly
negPoly poly = map negate poly

scalePoly :: MI -> Poly -> Poly
scalePoly c poly = map (\t -> t <*> c) poly

integratePoly :: MI -> Poly -> Poly
integratePoly const poly =
    polyFractions ++ [const]
    where
    polyFractions = zipWith (</>|) poly (reverse [1..degreePlusOne])
    degreePlusOne = length poly

reducePoly :: 
    MI {-^ @h@ right domain boundary, @h<1@ (left boundary is 0) -} -> 
    Int -> 
    Poly -> Poly
reducePoly h n poly@(highestDegreeCoeff : rest) 
    | n >= lengthPoly - 1 = poly
    | otherwise =
        addPolyConst polyLowDegreeCoeffs $
            (foldl1 (<+>) $ map range polyHighDegreeCoeffs)
    where
    lengthPoly = length poly
    (polyHighDegreeCoeffs, polyLowDegreeCoeffs) 
        = splitAt (lengthPoly - 1 - n) poly
    range coeff = 0 </\> (h <*> coeff)
        
--addPolys :: Poly -> Poly -> Poly
--addPolys poly1 poly2 
--    | lengthPoly1 == lengthPoly2 =
--        zipWith (<+>) poly1 poly2
--    | lengthPoly1 > lengthPoly2 =
--        zipWith (<+>) poly1  paddedPoly2
--    | otherwise =
--        zipWith (<+>) paddedPoly1  poly2
--    where
--    lengthPoly1 = length poly1
--    lengthPoly2 = length poly2
--    paddedPoly2 = (replicate (lengthPoly1 - lengthPoly2) 0) ++ poly2
--    paddedPoly1 = (replicate (lengthPoly2 - lengthPoly1) 0) ++ poly1
--    
addPolyConst :: Poly -> MI -> Poly
addPolyConst [] const = [const]
addPolyConst poly const =
    aux poly
    where
    aux [c] = [c <+> const]
    aux (t : ts) = t : (aux ts)  
    

    