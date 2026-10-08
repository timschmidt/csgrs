module PolynomialApproximation where

import FunctionAbstraction

import Numeric.AERN.RmToRn

import Numeric.AERN.Poly.IntPoly

import Numeric.AERN.MPFRBasis.Interval

import qualified Numeric.AERN.RefinementOrder as RefOrd (fromEndpointsOut)
import Numeric.AERN.RealArithmetic.Measures (imprecisionOf)
import Numeric.AERN.Basics.Effort

type Poly = IntPoly String MI

{-| An effective real function of one variable.  
    The Int parameter @n@ gives the following bound on the width 
    of the resulting polynomial enclosure: @ <= 2^(-n)@ 
-}
newtype PolyReal = PolyReal (Int -> Poly)

instance RF PolyReal where
    build = buildPolyReal
    evalMI = evalMIPolyReal
    
buildPolyReal :: (Rational, Rational) -> Expression -> PolyReal
buildPolyReal (lRat, rRat) expr = PolyReal $ aux expr
    where
    aux (ExprLit aRat) n =
        newConstFn limits varDoms aMI
        where
        aMI = aSeq !! n
        aSeq = fromRationalWithIncreasingAccuracy aRat
        (limits, varDoms) = buildLimitsVarDoms n (getPrec aMI) 
    aux (ExprVar) n =
        newProjection limits varDoms "x"
        where
        (limits, varDoms) = buildLimitsVarDoms n 100 

    lSeq = fromRationalWithIncreasingAccuracy lRat
    rSeq = fromRationalWithIncreasingAccuracy rRat

    buildLimitsVarDoms :: Int -> Precision -> (IntPolySizeLimits MI, [(String, MI)]) 
    buildLimitsVarDoms n coeffPrecision =
        (limits, varDoms)
        where
        limits =
            (defaultIntPolySizeLimits 0 coeffPrecision 1)
            {
                ipolylimits_maxdeg = 10,
                ipolylimits_maxsize = 11
            } 
        varDoms = zip vars doms
            where
            vars = ["x"]
            doms = [RefOrd.fromEndpointsOut (lMI, rMI)]
            lMI = lSeq !! n
            rMI = rSeq !! n
    
    
evalMIPolyReal :: PolyReal -> MI -> MI
evalMIPolyReal (PolyReal polySeq) x =
    -- evaluate on successive polynomials until:
    --   - the accuracy of the result has not improved significantly several times in a row (how many times?)
    --      - significant improvement = 
    --           for non-exact x: improvement of at least 1% of width of x
    --           for exact x: improvement of at least    
    undefined -- TODO
    
fromRationalWithIncreasingAccuracy :: Rational -> [MI]
fromRationalWithIncreasingAccuracy a =
    adjustRate (0 :: Int) $ 
        map withPrec precisionSequence
    where
    withPrec p = fromRationalWithPrec p a
    precisionSequence = effortIncrementSequence initPrec
        where
        initPrec = 50
    adjustRate i l@(h : t) 
        | hAccurateEnough = h : (adjustRate (i+1) l)
        | otherwise = adjustRate i t
        where
        hAccurateEnough 
            | i == 0 =
                (imprecisionOf h <=? one) == Just True
                -- this case is required to avoid the following trap: 
                --   two^^(0) = 1 with MPFR precision 100
            | otherwise =
                (imprecisionOf h <=? two^^(-i)) == Just True
            where
            one = fromRationalWithPrec (getPrec h) 1
            two = fromRationalWithPrec (getPrec h) 2
    adjustRate _ _ = error "internal error in fromRationalWithIncreasingAccuracy" 
        
    