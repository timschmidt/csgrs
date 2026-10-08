{-|
    Code accompanying the quick start tutorial on
    using AERN to produce tight enclosures for real functions.
-}
module Quickstart1 where

-- Doubles as interval endpoints:
import Numeric.AERN.RealArithmetic.Basis.Double ()

-- intervals generic in the type of its endpoints:
import Numeric.AERN.Basics.Interval 
    (Interval)

-- interval-coefficient polynomials:
import Numeric.AERN.Poly.IntPoly 
    (IntPoly, IntPolySizeLimits(..), IntPolyEffort(..), defaultIntPolySizeLimits)
import Numeric.AERN.Poly.IntPoly.Interval () 
import Numeric.AERN.Poly.IntPoly.Plot ()

-- abstract approximate order operations:
import Numeric.AERN.RefinementOrder.Operators 
    ((</\>)) -- hull of interval union

-- abstract approximate real arithmetic operations:
import Numeric.AERN.RealArithmetic.RefinementOrderRounding
    (piOut, 
     sqrtOut, sqrtOutEff, sqrtDefaultEffort,  
     expOut, expOutEff, expDefaultEffort)
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators
    ((|<*>))

-- ability to control the effort for elementary operations: 
import Numeric.AERN.RealArithmetic.Interval.ElementaryFromFieldOps
    (ExpThinEffortIndicator(..), SqrtThinEffortIndicator(..))

-- abstract function processing operations:
import Numeric.AERN.RmToRn 
    (newConstFn, newProjection, primitiveFunctionOut)
import qualified 
       Numeric.AERN.RmToRn.Plot.FnView 
    as FV

{----- type definitions -----}
type DI = Interval Double
type V = String -- names of variables
type Poly = IntPoly V DI 
type PI = Interval Poly

{----- constructing basic functions -----}

{-| encoding of the domain box -}
varDoms :: [(V, DI)]
varDoms = [("x", (-1) </\> 1)]

{-| example size limits for polynomials -}
sizeLimits :: IntPolySizeLimits DI
sizeLimits =
    limitsDefault
    {
        ipolylimits_maxdeg = 30,
        ipolylimits_maxsize = 100,
        ipolylimits_effort =
            (ipolylimits_effort limitsDefault)
            {
                ipolyeff_recipTauDegree = 6
            }
    }
    where
    limitsDefault = 
        defaultIntPolySizeLimits sampleCf effortCf arity
    sampleCf = 0
    arity = 1
    effortCf = ()

{-| The identity @\x:[-1,1] -> x@.  This is a simple example of a projection.  -}
x :: PI
x = newProjection sizeLimits varDoms "x"

{-| The constant function @\x:[-1,1] -> 1@. -}
c1 :: PI
c1 = newConstFn sizeLimits varDoms 1

{-| The function @\x:[-1,1] -> exp(x)@. -}
expX :: PI
expX = expOut x

{-| The function @\x:[-1,1] -> exp(x)@ with adjustable effort. -}
expXDeg :: Int -> PI
expXDeg taylorDegree = expOutEff eff x
    where
    eff = effDefault
        {
            expeff_taylorDeg = taylorDegree
        }
    effDefault = expDefaultEffort x

{-
    To make the following plotting code work, the file FnView.glade
    has to be available in the current directory.  The master copy of this file
    is in the root folder of the aern-realfn-plot-gtk package.
    (https://aern.googlecode.com/hg/aern-realfn-plot-gtk/FnView.glade)
-}

{-| Show a plot of exp(x) over [-1,1] -}
plotExp :: IO ()
plotExp = 
    FV.plotFns 
        [("exp example", 
            [(("(\\x:[-1,1].x)", FV.black, True), x),
             (("(\\x:[-1,1].exp(x))", FV.green, True), expXDeg 7),
             (("(\\x:[-1,1].exp(x))", FV.blue, True), expXDeg 10)
            ]
         )
        ]

erf :: PI
erf =
    scale $
    primitiveFunctionOut
        (expOutEff effExp (- x * x))
        integrationVariable
    where
    integrationVariable = "x"
    x = -- the identity function \x:[0,2] -> x
        newProjection sizeLimits varDoms "x"
        where
        varDoms = [("x", 0 </\> 2)]

    scale = (c |<*>)
    c = 2 / (sqrtOut (piOut sample_coeff))
    sample_coeff = 0

    -- indicators controlling the approximation effort:
    effExp =
        (expDefaultEffort x)
        {
            expeff_taylorDeg = 20 -- Taylor approximation degree
        }
    sizeLimits = -- restrictions on the size of the polynomials
        (defaultIntPolySizeLimits sample_coeff () 1)
        {
            ipolylimits_maxdeg = 30, -- maximum degree
            ipolylimits_maxsize = 100 -- maximum number of terms
        }

{-| Show a plot of erf(x) over [-1,1] -}
plotERF :: IO ()
plotERF = 
    FV.plotFns 
        [("erf example", 
            [
             (("(\\x:[-1,1].integral(exp(-x^2)))", FV.blue, True), erf)
            ]
         )
        ]

{-| The function @\x:[-1,1] -> sqrt(x+2)@. -}
sqrtXplus2 :: PI
sqrtXplus2 = sqrtOut $ x + c1 + c1

{-| The function @\x:[-1,1] -> sqrt(x+2)@ with adjustable effort. -}
sqrtXplus2Iters :: Int -> PI
sqrtXplus2Iters iters = sqrtOutEff eff $ x + c1 + c1
    where
    eff = effDefault
        {
            sqrteff_newtonIters = iters
        }
    effDefault = sqrtDefaultEffort x

--effSqrt = effIP

{-| Show a plot of sqrt(x+2) over [-1,1] -}
plotSqrt :: IO ()
plotSqrt = 
    FV.plotFns 
        [("sqrt example", 
            [(("(\\x:[-1,1].x)", FV.black, True), x),
             (("(\\x:[-1,1].1)", FV.black, True), c1),
             (("(\\x:[-1,1].sqrt(x+2))", FV.green, True), sqrtXplus2Iters 2),
             (("(\\x:[-1,1].sqrt(x+2))", FV.blue, True), sqrtXplus2Iters 20)
            ]
         )
        ]

