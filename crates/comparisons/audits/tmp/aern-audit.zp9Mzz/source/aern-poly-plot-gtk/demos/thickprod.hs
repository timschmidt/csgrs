{-|
    An encoding of a non-smooth thick function and plotting example outer and inner approximation.
-}
module ThickProd where

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
    (piOut, sqrtOut, expOutEff, expDefaultEffort)
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators
    ((|<*>), (<*>), (>*<))

-- ability to control the effort for elementary operations: 
import Numeric.AERN.RealArithmetic.Interval.ElementaryFromFieldOps
    (ExpThinEffortIndicator(..))

-- abstract function processing operations:
import Numeric.AERN.RmToRn 
    (newProjection, newConstFn, primitiveFunctionOut)
import qualified 
       Numeric.AERN.RmToRn.Plot.FnView 
    as FV

{----- type definitions -----}
type DI = Interval Double
type V = String -- names of variables
type Poly = IntPoly V DI 
type PI = Interval Poly

{----- the error function -----}

{-| An enclosure of the error function over the interval [0,2] -}
thickProd :: Bool -> Int -> Int -> PI
thickProd useInner maxdeg bernsteinDeg 
    | useInner =
        unitIntervalConstFn >*< x
    | otherwise =
        unitIntervalConstFn <*> x
--        (Interval (-1) 1 :: DI) |<*> x
    where
    unitIntervalConstFn = 
        newConstFn sizeLimits domains (Interval (-1) 1)
    x = 
        newProjection sizeLimits domains "x"
    domains = [("x", Interval (-1) 2)]
        -- ie the identity function \x:[-1,1] -> x

    -- indicators controlling the approximation effort:
    sizeLimits = -- restrictions on the size of the polynomials
        defaultSizeLimits
        {
            ipolylimits_maxdeg = maxdeg, -- maximum degree
            ipolylimits_maxsize = 100, -- maximum number of terms
            ipolylimits_effort =
                (ipolylimits_effort defaultSizeLimits)
                {
                    ipolyeff_minmaxBernsteinDegree = bernsteinDeg,
                    ipolyeff_evalMaxSplitSize = 10000
                } 
        }
        where
        defaultSizeLimits = (defaultIntPolySizeLimits sampleCoeff () 1)
    sampleCoeff = 0
    sampleFn = x

{-| An enclosure of the error function over the interval [0,2] -}
thickProdExact :: PI
thickProdExact =
    a <*> x -- parametrised the interval using a, thus approximating a binary real polynomial, which is trivial
    where
    x = newProjection sizeLimits domains "x"
        -- ie the projection function \(x,a):[-1,1]x[-1,1] -> x
    a = newProjection sizeLimits domains "a"
    domains = [("x", Interval (-1) 2), ("a", Interval (-1) 1)]

    -- indicators controlling the approximation effort:
    sizeLimits =
        (defaultIntPolySizeLimits sampleCoeff () 1)
    sampleCoeff = 0

{-| Show a plot of erf(x) over [-1,2] -}
plotThickProd :: IO ()
plotThickProd = 
    FV.plotFns 
        [("f(x) = [-1,1]*x", 
            [
             (("f(x) [outer approx.]", FV.blue, True), thickProd False 50 4),
             (("f(x) [inner approx.]", FV.green, True), thickProd True 10 2),
                                            -- deliberately using low precision for inner approx. 
                                            -- so that its inconsistent region is visible
             (("f(x)", FV.black, True), thickProdExact)
            ]
         )
        ]

plotThickProd2 :: IO ()
plotThickProd2 = 
    FV.plotFns 
        [("f(x) = [-1,1]*x", 
            [
             (("f(x) [outer approx.]", FV.blue, True), thickProd False 200 14),
             (("f(x) [inner approx.]", FV.green, True), thickProd True 200 14),
                                            -- deliberately using low precision for inner approx. 
                                            -- so that its inconsistent region is visible
             (("f(x)", FV.black, True), thickProdExact)
            ]
         )
        ]

