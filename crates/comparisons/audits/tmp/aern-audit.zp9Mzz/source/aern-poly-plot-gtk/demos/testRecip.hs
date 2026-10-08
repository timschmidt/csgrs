{-|
    Code accompanying the quick start tutorial on
    using AERN to produce tight enclosures for real functions.
-}
module TestRecip where

-- Doubles as interval endpoints:
import Numeric.AERN.RealArithmetic.Basis.Double ()

-- intervals generic in the type of its endpoints:
import Numeric.AERN.Basics.Interval

-- interval-coefficient polynomials:
import Numeric.AERN.Poly.IntPoly 
    (IntPoly, IntPolySizeLimits(..), defaultIntPolySizeLimits)
import Numeric.AERN.Poly.IntPoly.Plot ()

-- abstract approximate order operations:
import Numeric.AERN.RefinementOrder.Operators 
    ((</\>)) -- hull of interval union

-- abstract approximate real arithmetic operations:
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
import Numeric.AERN.RealArithmetic.RefinementOrderRounding ((<+>))

-- abstract function processing operations:
import Numeric.AERN.RmToRn (newConstFn, newProjection)
import qualified Numeric.AERN.RmToRn.Plot.FnView as FV

import Numeric.AERN.RmToRn.RefinementOrderRounding.Reciprocal (recipOutEff)

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
    (defaultIntPolySizeLimits sampleCf effortCf arity)
    {
        ipolylimits_maxdeg = 10
    }
    where
    sampleCf = 0
    arity = 1
    effortCf = ()

{-| The identity @\x:[0,1] -> x@.  This is a simple example of a projection.  -}
x :: Poly
x = newProjection sizeLimits varDoms "x"

{-| The constant function @\x:[0,1] -> 1@. -}
c1 :: Poly
c1 = newConstFn sizeLimits varDoms 1

--{-| The function @\x:[0,1] -> sqrt(x+1)@. -}
--sqrtXplus1 :: Poly
--sqrtXplus1 = ArithInOut.sqrtOut $ x + c1 + c1
--
--{-
--    To make the following plotting code work, the file FnView.glade
--    has to be available in the current directory.  The master copy of this file
--    is in the root folder of the aern-realfn-plot-gtk package.
--    (https://aern.googlecode.com/hg/aern-realfn-plot-gtk/FnView.glade)
---}
--plotSqrt :: IO ()
--plotSqrt = 
--    plotFns 
--        [("sqrt example", 
--            [("(\\x:[0,1].x)", x),
--             ("(\\x:[0,1].1)", c1),
--             ("(\\x:[0,1].sqrt(x+1))", sqrtXplus1)])]


f = c1 <+> c1 <+> x
sampleD = 1 :: DI
effRecip = 
    (ArithInOut.ringOpsDefaultEffort x, 
     ArithInOut.roundedRealDefaultEffort sampleD, 
     ArithInOut.mixedFieldOpsDefaultEffort x sampleD)

plotFR :: IO ()
plotFR = 
    FV.plotFns 
        [("recip example", 
            [(("(\\x:[-1,1].1)", FV.black, True), Interval c1 c1),
             (("(\\x:[-1,1].1/(x+2))[ord2]", FV.blue, True), fRI 2),
             (("(\\x:[-1,1].1/(x+2) * (x+2))[ord2]", FV.blue, True), fRfI 2),
             (("(\\x:[-1,1].1/(x+2))[ord2]", FV.blue, True), fRI 3),
             (("(\\x:[-1,1].1/(x+2) * (x+2))[ord2]", FV.blue, True), fRfI 3),
             (("(\\x:[-1,1].1/(x+2))[ord5]", FV.black, True), fRI 4),
             (("(\\x:[-1,1].1/(x+2) * (x+2))[ord5]", FV.black, True), fRfI 4)
            ])
        ]
    where
    fRI i = fst $ resultPairs !! (i-2) 
    fRfI i = snd $ resultPairs !! (i-2)
    resultPairs = map fRfRfI [2..] 
    fRfRfI i =
        (Interval fR fR, Interval fRf fRf)
        where
        fR = recipOutEff effRecip i f
        fRf = fR ArithInOut.<*> f 


