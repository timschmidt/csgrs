{-|
    An encoding of the error function and plotting example enclosures.
-}
module TestSineCosine where

-- Doubles as interval endpoints:
import Numeric.AERN.RealArithmetic.Basis.Double ()

-- intervals generic in the type of its endpoints:
import Numeric.AERN.Basics.Interval 
    (Interval(..))

-- interval-coefficient polynomials:
import Numeric.AERN.Poly.IntPoly 
    (IntPoly, IntPolySizeLimits(..), IntPolyEffort(..), defaultIntPolySizeLimits)
import Numeric.AERN.Poly.IntPoly.Interval () 
import Numeric.AERN.Poly.IntPoly.Plot () 

-- abstract approximate real arithmetic operations:
import Numeric.AERN.RealArithmetic.RefinementOrderRounding
    (piOut, sqrtOut, expOutEff, expDefaultEffort, sincosDefaultEffort, sinOutEff, cosOutEff)
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators
    ((+|), (|*))

import Numeric.AERN.RealArithmetic.ExactOps (one)


-- ability to control the effort for elementary operations: 
import Numeric.AERN.RealArithmetic.Interval
    (SineCosineThinEffortIndicator(..))



-- abstract function processing operations:
import Numeric.AERN.RmToRn 
    (newProjection, newConstFn, primitiveFunctionOut)

import Numeric.AERN.RefinementOrder ((/\))

import qualified 
       Numeric.AERN.RmToRn.Plot.FnView 
    as FV

{----- type definitions -----}
type DI = Interval Double
type V = String -- names of variables
type Poly = IntPoly V DI 
type PI = Interval Poly

{----- using interval polynomials -----}

plotSineCosine :: IO ()
plotSineCosine = 
    FV.plotFns 
        [
--         ("sin^2(x) + cos^2(x) = 1", 
--            [
--             (("sin(x)", FV.blue, False), sineX),
--             (("cos(x)", FV.blue, False), cosineX)
--             ,
--             (("sin^2(x)", FV.blue, False), sineX * sineX),
--             (("cos^2(x)", FV.blue, False), cosineX * cosineX),
--             (("sin^2(x) + cos^2(x)", FV.blue, False), sineX * sineX + cosineX * cosineX)
--            ]
--         )
--         ,
         ("over thick argument",
            [
             (("x", FV.black, False), x),
             (("x+1", FV.black, False), xp1),
             (("x+(y\\in[0,1])", FV.black, False), xThickParam),
             (("[x,x+1]", FV.black, False), xThick),
             (("sin(x)", FV.black, False), sineX),
             (("sin(x+1)", FV.black, False), sinOutEff effSinCos xp1),
             (("sin(x+(y\\in[0,1]))", FV.red, False), sinOutEff effSinCos xThickParam),
             (("sin([x,x+1])", FV.green, False), sinOutEff effSinCos xThick),
             (("cos(x)", FV.black, False), cosineX),
             (("cos(x+1)", FV.black, False), cosOutEff effSinCos xp1),
             (("cos([x+(y\\in[0,1])])", FV.red, False), cosOutEff effSinCos xThickParam)
            ]
         )
--         ,
--         ("sin(x^2)",
--            [
--             (("x*x", FV.black, False), x * x),
--             (("sin(x*x)", FV.red, False), sineXSqr)
--            ]
--         )
        ]
    where
    maxdeg = 20
    taylorDeg = 10
    
    sineX =
        sinOutEff effSinCos x
    cosineX =
        cosOutEff effSinCos x
    sineXSqr =
        sinOutEff effSinCos (x * x)

    xp1 = x + (one x)
    xThickParam = x + y
    xThick = x +| (Interval 0 1 :: DI)

    x :: Poly
    x = newProjection sizeLimits doms "x"
    y = newProjection sizeLimits doms "y"
    doms = 
        [
            ("x", Interval (0) 2)
            ,
            ("y", Interval (0) 1)
        ]

    -- indicators controlling the approximation effort:
    effSinCos =
        (sincosDefaultEffort sampleFn)
        {
            sincoseff_taylorDeg = taylorDeg -- Taylor approximation degree
        }

    sizeLimits = -- restrictions on the size of the polynomials
        defaultSizeLimits
        {
            ipolylimits_maxdeg = maxdeg, -- maximum degree
            ipolylimits_maxsize = 1000, -- maximum number of terms
            ipolylimits_effort =
                (ipolylimits_effort defaultSizeLimits)
                {
                    ipolyeff_evalMaxSplitSize = 2000
                } 
        }
        where
        defaultSizeLimits = (defaultIntPolySizeLimits sampleCoeff () 1)
    sampleCoeff = 0
    sampleFn = x


plotSineOfCosine :: IO ()
plotSineOfCosine =
    FV.plotFns 
        [
         ("sin(cos(x))",
            [
--             (("x", FV.black, False), x),
             (("cos(x)", FV.black, False), cosX)
             ,
             (("sin(cos(x))", FV.black, False), sinOutEff effSinCos2 cosX)
--             ,
--             (("1+eps1DI", FV.black, False), eps1DIp1)
--             ,
--             (("(1+ePI)cos(x)", FV.red, True), eps1PIcosX)
             ,
             (("(1+eDI)cos(x)", FV.green, True), eps1DIcosX)
--             ,
--             (("sin((1+ePI)cos(x))", FV.red, True), Interval poly_sinEps1CosX poly_sinEps1CosX)
             ,
             (("sin((1+eDI)cos(x))", FV.blue, True), sinEps1CosX)
            ]
         )
        ]
    where
    maxdeg = 12
    maxsize = 100
    taylorDeg1 = 15
    taylorDeg2 = 20
    bernsteinDeg = 2
    evalMaxSplitSize = 2000
-- best so far:
--    maxdeg = 12
--    maxsize = 100
--    taylorDeg1 = 15
--    taylorDeg2 = 20
--    bernsteinDeg = 2
--    evalMaxSplitSize = 2000
    
    
    sinEps1CosX = sinOutEff effSinCos2 eps1DIcosX
    poly_sinEps1CosX = sinOutEff poly_effSinCos2 poly_eps1cosX
    
    poly_eps1cosX = poly_eps1p1 * poly_cosX
    poly_eps1p1 = poly_eps1 +| (1 :: Int)

    eps1DIcosX = eps1DIp1 * cosX
    eps1DIp1 = newConstFn sizeLimits doms (eps1DI +| (1 :: Int))

    cosX = cosOutEff effSinCos1 x
    poly_cosX = cosOutEff poly_effSinCos1 poly_x

    poly_eps1 = newProjection sizeLimits doms "eps1" :: Poly
    poly_x = newProjection sizeLimits doms "x" :: Poly
    x = newProjection sizeLimits doms "x" :: PI
    doms =
        [
            ("x", 0 /\ 2)
            ,
            ("eps1", eps1DI)
--            ,
--            ("eps2", eps2DI)
        ]
    eps1DI = (-eps1) /\ eps1
    eps2DI = (-eps2) /\ eps2
    eps1 = 0.125 :: DI
    eps2 = 0.125 :: DI
    
    -- indicators controlling the approximation effort:
    effSinCos1 =
        (sincosDefaultEffort x)
        {
            sincoseff_taylorDeg = taylorDeg1 -- Taylor approximation degree
        }
    poly_effSinCos1 =
        (sincosDefaultEffort poly_x)
        {
            sincoseff_taylorDeg = taylorDeg1 -- Taylor approximation degree
        }

    effSinCos2 =
        (sincosDefaultEffort x)
        {
            sincoseff_taylorDeg = taylorDeg2 -- Taylor approximation degree
        }
    poly_effSinCos2 =
        (sincosDefaultEffort poly_x)
        {
            sincoseff_taylorDeg = taylorDeg2 -- Taylor approximation degree
        }

    sizeLimits = -- restrictions on the size of the polynomials
        defaultSizeLimits
        {
            ipolylimits_maxdeg = maxdeg, -- maximum degree
            ipolylimits_maxsize = maxsize, -- maximum number of terms
            ipolylimits_effort =
                (ipolylimits_effort defaultSizeLimits)
                {
                    ipolyeff_evalMaxSplitSize = evalMaxSplitSize,
                    ipolyeff_minmaxBernsteinDegree = bernsteinDeg
                } 
        }
        where
        defaultSizeLimits = (defaultIntPolySizeLimits sampleCoeff () 1)
    sampleCoeff = 0
    sampleFn = x



{----- using polynomial intervals -----}

plotSineCosinePI :: IO ()
plotSineCosinePI = 
    FV.plotFns 
        [
--         ("sin^2(x) + cos^2(x) = 1", 
--            [
--             (("sin(x)", FV.blue, False), sineX),
--             (("cos(x)", FV.blue, False), cosineX),
--             (("sin^2(x)", FV.blue, False), sineX * sineX),
--             (("cos^2(x)", FV.blue, False), cosineX * cosineX),
--             (("sin^2(x) + cos^2(x)", FV.blue, False), sineX * sineX + cosineX * cosineX)
--            ]
--         )
--         ,
         ("over thick argument",
            [
             (("x", FV.black, False), x),
             (("x+1", FV.black, False), x + 1),
             (("[x,x+1]", FV.black, False), xThick),
             (("sin(x)", FV.black, False), sineX),
             (("sin(x+1)", FV.black, False), sinOutEff effSinCos xp1),
             (("sin([x,x+1])", FV.red, False), sinOutEff effSinCos xThick)
            ]
         )
--         ,
--         ("sin(x^2)",
--            [
--             (("x*x", FV.black, False), x * x),
--             (("sin(x*x)", FV.red, False), sineXSqr)
--            ]
--         )
        ]
    where
    maxdeg = 20
    taylorDeg = 20
    sineX =
        sinOutEff effSinCos x
    cosineX =
        cosOutEff effSinCos x
    sineXSqr =
        sinOutEff effSinCos (x * x)
    xThick = x /\ (x + 1)
    xp1 = x + 1
    x :: PI
    x = 
        -- ie the identity function \x:[0,1] -> x
        newProjection sizeLimits [("x", Interval (0) 2)] "x"

    -- indicators controlling the approximation effort:
    effSinCos =
        (sincosDefaultEffort sampleFn)
        {
            sincoseff_taylorDeg = taylorDeg -- Taylor approximation degree
        }

    sizeLimits = -- restrictions on the size of the polynomials
        defaultSizeLimits
        {
            ipolylimits_maxdeg = maxdeg, -- maximum degree
            ipolylimits_maxsize = 100, -- maximum number of terms
            ipolylimits_effort =
                (ipolylimits_effort defaultSizeLimits)
                {
                    ipolyeff_evalMaxSplitSize = 20000
                } 
        }
        where
        defaultSizeLimits = (defaultIntPolySizeLimits sampleCoeff () 1)
    sampleCoeff = 0
    sampleFn = x

