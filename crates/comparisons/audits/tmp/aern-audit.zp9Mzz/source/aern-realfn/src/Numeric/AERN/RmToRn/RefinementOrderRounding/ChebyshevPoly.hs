{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-|
    Module      :  Numeric.AERN.RmToRn.RefinementOrderRounding.ChebyshevPoly
    Description :  approximations of Chebyshev polynomials  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Approximations of Chebyshev polynomials.
-}

module Numeric.AERN.RmToRn.RefinementOrderRounding.ChebyshevPoly where

import Numeric.AERN.RealArithmetic.ExactOps (HasOne(..))
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut

import Debug.Trace
_ = trace

chebyshevOut ::
    (HasOne f, ArithInOut.RoundedRing f, Show f) 
     =>
    (ArithInOut.RingOpsEffortIndicator f)
    {-^ Effort indicator to guide the effort used for approximately computing the Chebyshev polynomials -} ->
    f {-^ @x@ - A function to substitute in the Chebyshev polynomial  -} ->
    [f] {-^ @[T_0(x), T_1(x), T_2(x), ..]@ - The Chebyshev polynomials of all degrees -}
chebyshevOut effRingF x =
--    trace
--    (
--        "chebyshevOut:"
--        ++ "\n x = " ++ show x
--        ++ "\n T_2(x) = " ++ (show $ result !! 2)
--    )
    result
    where
    result = 
        chebyshevPolys c1 x 
    chebyshevPolys tNminus2 tNminus1 = 
        tNminus2 : (chebyshevPolys tNminus1 tN)
        where
        tN = (c2 ~<*>~ x ~<*>~ tNminus1) ~<->~ tNminus2
    c1 = (one sampleF)
    c2 = c1 ~<+>~ c1
    sampleF = x

    (~<+>~) = ArithInOut.addOutEff effAddF
    (~<->~) = ArithInOut.subtrOutEff effAddF
    (~<*>~) = ArithInOut.multOutEff effMultF

    effAddF = ArithInOut.ringEffortAdd sampleF effRingF
    effMultF = ArithInOut.ringEffortMult sampleF effRingF
    
    
    