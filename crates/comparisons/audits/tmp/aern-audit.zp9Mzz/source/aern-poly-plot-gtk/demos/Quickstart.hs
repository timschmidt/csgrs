{-|
    Code accompanying the quick start tutorial on
    using AERN to produce tight enclosures for real functions.
    
    WARNING: This module is under construction.
    It is intended that this module will demonstrate
    how AERN has been refactored to be easier to use.
    This module draft serves as source of requirements
    for the changes. 
-}
module Quickstart where

{----- AERN imports -----}

-- Doubles as interval endpoints:
import Numeric.AERN.RealArithmetic.Basis.Double ()

-- intervals generic in the type of its endpoints:
import Numeric.AERN.Basics.Interval

-- interval-coefficient polynomials:
import Numeric.AERN.Poly.IntPoly 
    (IntPoly, IntPolySizeLimits(..), defaultIntPolySizeLimits)
import Numeric.AERN.Poly.IntPoly.Interval () 
import Numeric.AERN.Poly.IntPoly.Plot ()

-- abstract approximate order operations:
--import qualified Numeric.AERN.NumericOrder as NumOrd
--import Numeric.AERN.NumericOrder.Operators
--import qualified Numeric.AERN.RefinementOrder as RefOrd
import Numeric.AERN.RefinementOrder.Operators ((</\>))
import Numeric.AERN.Basics.SizeLimits (changeSizeLimitsOut)

-- abstract approximate real arithmetic operations:
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators 
    ((<+>), (<*>), (<^>), -- outer-rounded
     (<+>|), (<*>|), (</>|), -- outer-rounded mixed type
     (>+<), (>^<) -- inner-rounded
    )
--import Numeric.AERN.RealArithmetic.ExactOps

-- interval-specific versions of some real operations:
--import Numeric.AERN.RealArithmetic.Interval (intervalExpDefaultEffortWithIters, intervalSqrtDefaultEffortWithIters)


-- abstract function processing operations:
import Numeric.AERN.RmToRn (newConstFn, newProjection)
import qualified 
       Numeric.AERN.RmToRn.Plot.FnView as FV

{----- non-AERN imports -----}
-- none at the moment

{----- type definitions -----}
type DI = Interval Double
type V = String -- names of variables
type Poly = IntPoly V DI 
type PI = Interval Poly

{----- constructing basic functions -----}

{-
    To make the following plotting code work, the file FnView.glade
    has to be available in the current directory.  The master copy of this file
    is in the root folder of the aern-realfn-plot-gtk package.
    (https://aern.googlecode.com/hg/aern-realfn-plot-gtk/FnView.glade)
-}
basicFnPlot :: IO ()
basicFnPlot = FV.plotFns [("basic fns", [(("(\\x.x)", FV.black, True), x),(("(\\x.1)", FV.black, True), c1)])]

{-| The identity @\x:[0,1] -> x@.  This is a simple example of a projection.  -}
x :: PI
x = newProjection sizeLimits varDoms "x"

{-| The constant function @\x:[0,1] -> 1@. -}
c1 :: PI
c1 = newConstFn sizeLimits varDoms 1

{-| encoding of the domain box -}
varDoms :: [(V, DI)]
varDoms = zip vars doms

vars :: [V]
vars = ["x"]

doms :: [DI]
doms = [(-1) </\> 1]

{-| example size limits for polynomials -}
sizeLimits :: IntPolySizeLimits DI
sizeLimits =
    (defaultIntPolySizeLimits 0 () 1)
    {
        ipolylimits_maxdeg = 3,
        ipolylimits_maxsize = 40
    }

{----- basic arithmetic -----}

example10 :: PI
example10 = x <+> c1 -- outer-rounded addition

example11 :: PI
example11 = (x <*> x) -- outer-rounded multiplication

example12 :: PI
example12 = x <^> 2 -- outer-rounded power with natural exponent

basicArithPlot :: IO ()
basicArithPlot = FV.plotFns [("arith", [(("(\\x.x+1)", FV.black, True), x <+> c1),(("(\\x.x^2)", FV.black, True), x <^> 2)])]

example13 :: PI
example13 = (x <+>| (1 :: Int)) -- scalar shifting (mixed-type arithmetic)

example14 :: PI
example14 = (x </>| (2 :: Int)) -- scalar division

example15 :: PI
example15 = (x <*>| (0.5 :: Rational)) -- scaling

example16 :: PI
example16 = (((x </>| (2 :: Int)) <+> c1) <^> 4 ) -- rounding in action

example16E :: PI
example16E = (((changeSizeLimitsOut sizeLimitsD4 $ x </>| (2 :: Int)) <+> c1) <^> 4 ) -- same as above but without rounding

sizeLimitsD4 :: IntPolySizeLimits DI
sizeLimitsD4 = sizeLimits { ipolylimits_maxdeg = 4 }

roundingPlot :: IO ()
roundingPlot = FV.plotFns [("outer rounding", [(("rounded", FV.blue, True), example16),(("exact", FV.black, True), example16E)])]

example17 :: PI
example17 = (x + c1) -- Num instance is synonymous to outer-rounded operations

nonexample18 :: PI
nonexample18 = (x + 1) -- incompatible domains!

{----- some elementary functions -----}

example21 :: PI
example21 = ArithInOut.expOut x

example22 :: PI
example22 = ArithInOut.expOut $ ArithInOut.expOut x

nonexample23 :: PI
nonexample23 = ArithInOut.sqrtOut x -- includes sqrt of negative numbers

example24 :: PI
example24 = ArithInOut.sqrtOut $ x + c1 + c1
-- TODO: implement rudimentary division as interval sqrt needs it

{----- interval functions -----}

-- TODO

{----- inner-rounding -----}

-- TODO

--example40 :: PI
--example40 = ((x >+< c1) >^< 4) -- inner rounding




