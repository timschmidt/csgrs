{-|
-}
module Integrate where

-- interval-coefficient polynomials:
import Numeric.AERN.Poly.IntPoly 
    (IntPoly, IntPolySizeLimits(..), IntPolyEffort(..), defaultIntPolySizeLimits)
import Numeric.AERN.Poly.IntPoly.Interval () 
--import Numeric.AERN.Poly.IntPoly.Plot ()

-- intervals generic in the type of its endpoints:
import Numeric.AERN.Basics.Interval 
    (Interval)

-- Doubles as interval endpoints:
import Numeric.AERN.RealArithmetic.Basis.Double ()

-- abstract function processing operations:
import Numeric.AERN.RmToRn 
    (newProjection, primitiveFunctionOut, getDomainBox, evalAtPointOut, insertVar, lookupVar)
--import qualified 
--       Numeric.AERN.RmToRn.Plot.FnView 
--    as FV

-- abstract approximate real arithmetic operations:
--import Numeric.AERN.RealArithmetic.RefinementOrderRounding
--    (piOut, 
--     sqrtOut, sqrtOutEff, sqrtDefaultEffort,  
--     expOut, expOutEff, expDefaultEffort)
--import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators
--    ((|<*>))

-- ability to control the effort for elementary operations: 
--import Numeric.AERN.RealArithmetic.Interval.ElementaryFromFieldOps
--    (ExpThinEffortIndicator(..), SqrtThinEffortIndicator(..))

import Numeric.AERN.RealArithmetic.Measures (iterateUntilAccurate)

-- abstract approximate order operations:
import Numeric.AERN.RefinementOrder 
    (getEndpointsOut) -- endpoints of an interval-like value
import Numeric.AERN.RefinementOrder.Operators 
    ((</\>)) -- hull of interval union

-- ability to extract and change precision of numbers:
import Numeric.AERN.Basics.SizeLimits
    (getSizeLimits, changeSizeLimitsOut)

{----- type definitions -----}
type DI = Interval Double
type V = String -- names of variables
type Poly = IntPoly V DI 
type PI = Interval Poly

type R = DI
type RI = DI

{----- constructing basic functions -----}

integrate :: PI -> R -> (PI -> PI) -> [(Int, R, R)]
integrate x e computeF =
    iterateUntilAccurate 0 30 e $ 
            \ maxdeg -> resultForMaxdeg maxdeg
    where
    resultForMaxdeg maxdeg =
        (evalAtPointOut (domWith domR) pfDeg) - (evalAtPointOut (domWith domL) pfDeg) 
        where
        pfDeg = pf maxdeg
        domWith pt =
            insertVar "x" pt (getDomainBox x)
        (domL, domR) = getEndpointsOut dom
        dom = case lookupVar (getDomainBox x) "x" of Just d -> d; _ -> error ""
    pf maxdeg = 
        primitiveFunctionOut (computeF xEff) "x"
        where
        xEff = changeSizeLimitsOut sizeLimit x
        sizeLimit = 
            (getSizeLimits x)
            {
                ipolylimits_maxdeg = maxdeg,
                ipolylimits_maxsize = maxdeg + 1
            }
        
        
[test1, test2] =
    [
        integrate x 0.01 (\ y -> y * y)
    ,
        integrate x 0.0001 (\ y -> y * y)
    ]
    where
    x :: PI
    x = newProjection sizeLimits varDoms "x"

    varDoms = [("x", (0) </\> 1)]

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


{-
    To make the following plotting code work, the file FnView.glade
    has to be available in the current directory.  The master copy of this file
    is in the root folder of the aern-realfn-plot-gtk package.
    (https://aern.googlecode.com/hg/aern-realfn-plot-gtk/FnView.glade)
-}

--{-| Show a plot of exp(x) over [-1,1] -}
--plotExp :: IO ()
--plotExp = 
--    FV.plotFns 
--        [("exp example", 
--            [(("(\\x:[-1,1].x)", FV.black, True), x),
--             (("(\\x:[-1,1].exp(x))", FV.green, True), expXDeg 7),
--             (("(\\x:[-1,1].exp(x))", FV.blue, True), expXDeg 10)
--            ]
--         )
--        ]

