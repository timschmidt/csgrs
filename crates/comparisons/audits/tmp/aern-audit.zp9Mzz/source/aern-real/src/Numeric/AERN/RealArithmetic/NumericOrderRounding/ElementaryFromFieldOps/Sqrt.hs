{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE UndecidableInstances #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE NoMonoLocalBinds #-}
{-# LANGUAGE StandaloneDeriving #-}
{-|
    Module      :  Numeric.AERN.RealArithmetic.NumericOrderRounding.ElementaryFromFieldOps.Sqrt
    Description :  generic up/down rounded sqrt
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable

    An generic implementation of up/down rounded sqrt.
-}

module Numeric.AERN.RealArithmetic.NumericOrderRounding.ElementaryFromFieldOps.Sqrt where

import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn
--import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
--import Numeric.AERN.RealArithmetic.RefinementOrderRounding.OpsImplicitEffort
import qualified Numeric.AERN.NumericOrder as NumOrd
--import qualified Numeric.AERN.RefinementOrder as RefOrd
--import Numeric.AERN.RefinementOrder.OpsImplicitEffort

import Numeric.AERN.RealArithmetic.ExactOps

--import Numeric.AERN.Basics.Consistency
import Numeric.AERN.Basics.Effort
--import Numeric.AERN.Basics.Mutable
import Numeric.AERN.Basics.Exception

import Test.QuickCheck (Arbitrary(..)) -- , arbitrary, vectorOf)

import Control.Applicative
import Control.Exception (throw)
--import Control.Monad.ST (ST)

data SqrtThinEffortIndicator e =
    SqrtThinEffortIndicator
    {
        sqrteff_arith :: ArithUpDn.RoundedRealEffortIndicator e,
        sqrteff_newtonIters :: Int -- ^ the highest number of iterations of Newton method to make 
    }

instance 
    (
        Arbitrary (ArithUpDn.RoundedRealEffortIndicator e)
    )
    =>
    Arbitrary (SqrtThinEffortIndicator e)
    where
    arbitrary =
        SqrtThinEffortIndicator <$> arbitrary <*> (fromInt1To10 <$> arbitrary)
        
deriving instance
    (
        Show (ArithUpDn.RoundedRealEffortIndicator e)
    )
    =>
    Show (SqrtThinEffortIndicator e)

instance
    (
        EffortIndicator (ArithUpDn.RoundedRealEffortIndicator e)
    )
    =>
    EffortIndicator (SqrtThinEffortIndicator e)
    where
    effortIncrementVariants (SqrtThinEffortIndicator e1O e2O) =
        [SqrtThinEffortIndicator e1 e2 | 
            (e1, e2) <- effortIncrementVariants (e1O, e2O) ]
    effortRepeatIncrement (SqrtThinEffortIndicator i1 i2, SqrtThinEffortIndicator j1 j2) = 
        SqrtThinEffortIndicator (effortRepeatIncrement (i1, j1)) (effortRepeatIncrement (i2, j2)) 
    effortIncrementSequence (SqrtThinEffortIndicator e1O e2O) =
        [SqrtThinEffortIndicator e1 e2 | 
            (e1, e2) <- effortIncrementSequence (e1O, e2O) ]
    effortCombine  (SqrtThinEffortIndicator i1 i2) (SqrtThinEffortIndicator j1 j2) =
        SqrtThinEffortIndicator (effortCombine i1 j1) (effortCombine i2 j2)

sqrtThinDefaultEffort :: 
   (ArithUpDn.RoundedReal e) 
   =>
   e -> Int -> SqrtThinEffortIndicator e
sqrtThinDefaultEffort x iters =
    SqrtThinEffortIndicator
    {
        sqrteff_arith = ArithUpDn.roundedRealDefaultEffort x,
        sqrteff_newtonIters = iters
    }

sqrtOutThinArg ::
    (Show e, ArithUpDn.RoundedReal e) 
    =>
    SqrtThinEffortIndicator e ->
    e {-^ @x@ as a singleton interval -} -> 
    (e, e) {-^ @sqrt(x)@ lower and upper bounds -}
sqrtOutThinArg eff x
    | sureIsZero x = (zero x, zero x)
    | not (sureAbove0 x) = 
        case (sureAbove0 (neg x)) of
            True -> 
                throw $ AERNDomViolationException $ 
                    "sqrtOutThinArg: applied to a negative argument " ++ show x
            _ ->
                throw $ AERNMaybeDomViolationException $ 
                    "sqrtOutThinArg: cannot check that sqrt is applied to a positive argument " ++ show x
    | xRecipSqrtDownInFastRegion =
--            unsafePrint ("AERN: sqrtOutThinArg: lower bound in fast region") $
        (x *. xRecipSqrtDown,
         x *^ xRecipSqrtUp) -- best upper bound estimate based on an error estimate of the lower bound
    | sureAbove0 xRecipSqrtDown =
--            unsafePrint ("AERN: sqrtOutThinArg: lower bound NOT in fast region, using division") $
        (x *. xRecipSqrtDown,
         recipUp xRecipSqrtDown) 
         -- an upper bound using division - introduces a fairly large error; used when iteration has not reached the fast region
    | otherwise =
--            unsafePrint ("AERN: sqrtOutThinArg: lower bound too close to zero, using dummy upper bound") $
        (x *. xRecipSqrtDown,
         NumOrd.maxUpEff effortMinmax x (one x))
         -- a dummy fallback upper bound where lower bound is too close to 0
    where
    (xRecipSqrtDownPrev, xRecipSqrtDown) = recipSqrtDown
    xRecipSqrtDownInFastRegion =
        case ArithUpDn.convertDnEff effortToDbl (0::Double) t of
            Just lowerBound -> lowerBound > 0.381966012 -- (3 - sqrt 5)/2
            Nothing -> False
        where
        t = (xRecipSqrtDownPrev *. xRecipSqrtDownPrev) *. x
    xRecipSqrtUp = 
         -- only valid in "fast" region, ie where the error is smaller 
         -- than the gap between the results of the last two iterations
        (xRecipSqrtLastUp +^ xRecipSqrtLastUp) +^ (neg xRecipSqrtDownPrev)
    xRecipSqrtLastUp = 
            (xRecipSqrtDownPrev /^| 2 )
            *^
            (3 |+^ (neg $ x *. (xRecipSqrtDownPrev *. xRecipSqrtDownPrev))) 
            
    sureAbove0 t = 
        case ArithUpDn.convertDnEff effortToDbl (0::Double) t of
            Just lowerBound -> lowerBound > 0
            Nothing -> False
            
    sureIsZero t = 
        case NumOrd.pEqualEff effortCompare t (zero t) of
            Just True -> True
            _ -> False
    
    x1 +^ x2 = ArithUpDn.addUpEff effortAdd x1 x2
    x1 *^ x2 = ArithUpDn.multUpEff effortMult x1 x2
    x1 *. x2 = ArithUpDn.multDnEff effortMult x1 x2
    recipUp x1 = ArithUpDn.recipUpEff effortDiv x1
    recipDn x1 = ArithUpDn.recipDnEff effortDiv x1
    n |+^ x2 = ArithUpDn.mixedAddUpEff effortAddInt x2 (n :: Int)
    n |+. x2 = ArithUpDn.mixedAddDnEff effortAddInt x2  (n :: Int)
    x1 /^| n = ArithUpDn.mixedDivUpEff effortDivInt x1  (n :: Int)
    x1 /.| n = ArithUpDn.mixedDivDnEff effortDivInt x1  (n :: Int)
    
    effortAdd = ArithUpDn.fldEffortAdd x effortField
    effortMult = ArithUpDn.fldEffortMult x effortField
    effortDiv = ArithUpDn.fldEffortDiv x effortField
    effortAddInt = ArithUpDn.mxfldEffortAdd x (0::Int) effortMixedField
    effortDivInt = ArithUpDn.mxfldEffortDiv x (0::Int) effortMixedField

    effortMinmax = ArithUpDn.rrEffortMinmax x effE
    effortToDbl = ArithUpDn.rrEffortToDouble x effE
    effortCompare = ArithUpDn.rrEffortComp x effE
    effortField = ArithUpDn.rrEffortField x effE
    effortMixedField = ArithUpDn.rrEffortIntMixedField x effE
    effE = sqrteff_arith eff
    maxIters = sqrteff_newtonIters eff

    recipSqrtDown
        | q0OK = -- computed an approximation in the stable region:
            iterRecipSqrt maxIters z q0 -- then iterate!
        | otherwise = (z,z) -- zero is an always correct lower approximation
        where
        z = zero q0
        (q0OK, q0) = 
            (sureAbove0 xPlusOneUp && sureAbove0 babylon2, 
             recipDn babylon2)
            where
            -- babylon2 = (x+5)/4 - 1/(x+1) rounded upwards
            --   ie two Babylonian iterations
            --     \ t -> (t + x/t)/2 
            --   starting with t_0 = x:
            --   t_1 = (x + 1)/2
            --   t_2 = ((x + 1)/2 + x/((x + 1)/2))/2 =
            --         ((x + 1)/2 + 2x/(x + 1))/2 =
            --         ((x + 1)^2 + 4x)/4(x+1) =
            --         (x^2 + 6x + 1)/4(x+1) =
            --         (x^2 + 6x + 5 - 4)/4(x+1) =
            --         ((x + 5)(x + 1) - 4)/4(x+1) =
            --         (x + 5) - 1/(x+1)
            babylon2 = xPlus5Div4Up +^ (neg xPlusOneRecipDn)
            xPlus5Div4Up = ((5::Int) |+^ x) /^| (4::Int)
            xPlusOneRecipDn = recipDn xPlusOneUp
            xPlusOneUp = (1::Int) |+^ x
        -- iteratively improve q, a lower bound on sqrt(1/x)
        --   using the formula q_{n+1} = (q_n / 2) * (3 - x * q_n * q_n)  
        --   quoted eg in http://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Iterative_methods_for_reciprocal_square_roots
        iterRecipSqrt maxIters2 qNm2 qNm1
            | maxIters2 > 0 && sureAbove0 qNm1 =
--                    unsafePrint ("AERN: sqrtOutThinArg: recipSqrtDown: iterRecipSqrt: maxIters2 = " ++ show maxIters2) $
                iterRecipSqrt (maxIters2 - 1) qNm1 qN
            | otherwise = (qNm2, qNm1)
            where
            qN =
                (qNm1 /.| (2::Int))
                *.
                ((3::Int) |+. (neg $ x *^ (qNm1 *^ qNm1))) 


--sqrtOutThinArgInPlace ::
--    (CanBeMutable e,
--     HasZero e, HasOne e, Show e,
--     NumOrd.RoundedLattice e,
--     NumOrd.RoundedLatticeInPlace e,
--     NumOrd.PartialComparison e,
--     ArithUpDn.Convertible e Double,
--     ArithUpDn.RoundedMixedField e Int,
--     ArithUpDn.RoundedMixedFieldInPlace e Int,
--     ArithUpDn.RoundedField e,
--     ArithUpDn.RoundedFieldInPlace e) =>
--    ArithUpDn.FieldOpsEffortIndicator e ->
--    ArithUpDn.MixedFieldOpsEffortIndicator e Int ->
--    NumOrd.MinmaxEffortIndicator e ->
--    NumOrd.PartialCompareEffortIndicator e ->
--    ArithUpDn.ConvertEffortIndicator e Double ->
--    Mutable (Interval e) s {-^ where to write the result @sqrt(x)@ -} ->
--    Int {-^ the highest number of iterations of Newton method to make -} ->
--    Mutable e s {-^ @x@ viewed as a singleton interval -} -> 
--    ST s ()
--sqrtOutThinArgInPlace
--        effortField
--        effortMixedField
--        effortMinmax
--        effortCompare
--        effortToDbl
--        resM@(MInterval resLM resRM)
--        maxIters
--        xM
--    =
--    do
--    -- we need x - a pure version of xM for branching conditions:
--    x <- unsafeReadMutable xM
--    -- unsafe is OK because we do not write into xM and we write to resM only as the last thing
--    --   and the value of x does not escape beyond this function
--    computeSqrt x
--    where
--    computeSqrt x =
--        do
--        continue
--        where
--        addUpDnEffort = ArithUpDn.fldEffortAdd x effortField
--        multUpDnEffort = ArithUpDn.fldEffortMult x effortField
--        mixedAddUpDnEffort = ArithUpDn.mxfldEffortAdd x (0::Int) effortMixedField
--        mixedMultUpDnEffort = ArithUpDn.mxfldEffortMult x (0::Int) effortMixedField
--        mixedDivUpDnEffort = ArithUpDn.mxfldEffortDiv x (0::Int) effortMixedField
--        
--
--        continue
--            | sureIsZero x = 
--                writeMutable resM $ zero $ Interval x x
--            | not (sureAbove0 x) = 
--                case (sureAbove0 (neg x)) of
--                    True -> 
--                        throw $ AERNDomViolationException $ 
--                            "sqrtOutThinArgInPlace: applied to a negative argument " ++ show x
--                    _ ->
--                        throw $ AERNMaybeDomViolationException $ 
--                            "sqrtOutThinArgInPlace: cannot check that sqrt is applied to a positive argument " ++ show x
--            | otherwise =
--                do
--                -- declare some variables:
--                temp1M <- makeMutable $ zero x
--                temp2M <- makeMutable $ zero x
--                -- iterate using Newton's method, assign results of last two iterations to the above vars: 
--                prevFirst <- recipSqrtDown temp1M temp2M
--                case prevFirst of
--                    True -> assignBounds temp1M temp2M
--                    False -> assignBounds temp2M temp1M
--                where
--                assignBounds xRecipSqrtDownPrevM xRecipSqrtDownM =
--                    do
--                    xRecipSqrtDown <- unsafeReadMutable xRecipSqrtDownM
--                    xRecipSqrtDownPrev <- unsafeReadMutable xRecipSqrtDownPrevM
--                    -- assign lower bound resL := x *. xRecipSqrtDown:
--                    ArithUpDn.multDnInPlaceEff effortMult resLM xM xRecipSqrtDownM
--                    -- assign upper bound the best applicable method out of three methods: 
--                    constructUpperBound xRecipSqrtDownM xRecipSqrtDown xRecipSqrtDownPrevM xRecipSqrtDownPrev
--                constructUpperBound xRecipSqrtDownM xRecipSqrtDown xRecipSqrtDownPrevM xRecipSqrtDownPrev 
--                    | xRecipSqrtDownInFastRegion =
--                        do
--                        -- in fast region, use the difference between the last two approx:
--    
--                        -- first, we need an upwards-rounded version of xRecipSqrtDown:                  
--                        -- xRecipSqrtLastUp := newton... xRecipSqrtDownPrev:
--                        let xRecipSqrtLastUpM = xRecipSqrtDownM -- safely reuse variable
--                        newtonIterateUp xRecipSqrtLastUpM xRecipSqrtDownPrevM
--                        
--                        -- now compute and use the difference:
--                        -- xRecipSqrtUp := 2*^xRecipSqrtLastUp -^ xRecipSqrtDownPrev:
--                        --   only valid in "fast" region, ie where the error is smaller 
--                        --   than the gap between the results of the last two iterations
--                        let xRecipSqrtUpM = xRecipSqrtLastUpM -- safely reuse variable
--                        xRecipSqrtLastUpM *^|= (2 :: Int)
--                        xRecipSqrtLastUpM -^= xRecipSqrtDownPrevM
--                        
--                        -- assign upper bound resR := x *^ xRecipSqrtUp:
--                        ArithUpDn.multUpInPlaceEff effortMult resRM xM xRecipSqrtUpM
--                    | sureAbove0 xRecipSqrtDown =
--                        do
--                        -- compute upper bound resR := 1 /^ xRecipSqrtDown:
--                        --   introduces a fairly large error; 
--                        --   used when iteration has not reached the fast region
--                        ArithUpDn.recipUpInPlaceEff effortDiv resRM xRecipSqrtDownM
--                    | otherwise =
--                        do
--                        -- a dummy fallback upper bound where lower bound is too close to 0:
--                        unsafeWriteMutable resRM $
--                            NumOrd.maxUpEff effortMinmax x (one x)
--                    where
--                    xRecipSqrtDownInFastRegion =
--                        case ArithUpDn.convertDnEff effortToDbl (0::Double) t of
--                            Just lowerBound -> lowerBound > 0.381966012 -- (3 - sqrt 5)/2
--                            Nothing -> False
--                        where
--                        t = 
--                            (xRecipSqrtDownPrev *. xRecipSqrtDownPrev) *. x
--                newtonIterateUp resM2 tM = 
--                    -- assumes no aliasing between resM2 and tM, does not change tM
--                    do
--                    -- res :=
--                    --    (t /^| 2 )
--                    --     *^
--                    --    (3 |+^ (neg $ x *. (t *. t))) 
--                    ArithUpDn.multUpInPlaceEff effortMult resM2 tM tM
--                    resM2 *^= xM
--                    negInPlace resM2 resM2
--                    resM2 +.|= (3 :: Int)
--                    resM2 *.= tM
--                    resM2 /.|= (2 :: Int)
--    
--                newtonIterateDn resM2 tM = 
--                    -- assumes no aliasing between resM2 and tM, does not change tM
--                    do
--                    -- res :=
--                    --    (t /.| 2 )
--                    --     *.
--                    --    (3 |+. (neg $ x *^ (t *^ t))) 
--                    ArithUpDn.multDnInPlaceEff effortMult resM2 tM tM
--                    resM2 *.= xM
--                    negInPlace resM2 resM2
--                    resM2 +^|= (3 :: Int)
--                    resM2 *^= tM
--                    resM2 /^|= (2 :: Int)
--    
--                sureAbove0 t = 
--                    case ArithUpDn.convertDnEff effortToDbl (0 :: Double) t of
--                        Just lowerBound -> lowerBound > 0
--                        Nothing -> False
--                        
--                sureIsZero t = 
--                    case NumOrd.pEqualEff effortCompare t (zero t) of
--                        Just True -> True
--                        _ -> False
--                
----                effortAdd = ArithUpDn.fldEffortAdd x effortField
--                effortMult = ArithUpDn.fldEffortMult x effortField
--                effortDiv = ArithUpDn.fldEffortDiv x effortField
----                effortMultInt = ArithUpDn.mxfldEffortMult x (0::Int) effortMixedField
----                effortAddInt = ArithUpDn.mxfldEffortAdd x (0::Int) effortMixedField
----                effortDivInt = ArithUpDn.mxfldEffortDiv x (0::Int) effortMixedField
--            
--                recipSqrtDown aM bM
--                    | q0OK = -- computed an approximation in the stable region:
--                        do
--                        writeMutable aM $ zero x
--                        writeMutable bM q0
--                        iterRecipSqrt maxIters True aM bM -- then iterate!
--                    | otherwise = 
--                        do
--                        writeMutable aM $ zero x -- zero is an always correct lower approximation
--                        writeMutable bM $ zero x
--                        return True
--                    where
--                    (q0OK, q0) = 
--                        (sureAbove0 xPlusOneUp && sureAbove0 babylon2, 
--                         recipDn babylon2)
--                        where
--                        -- babylon2 = (x+5)/4 - 1/(x+1) rounded upwards
--                        --   ie two Babylonian iterations
--                        --     \ t -> (t + x/t)/2 
--                        --   starting with t_0 = x:
--                        --   t_1 = (x + 1)/2
--                        --   t_2 = ((x + 1)/2 + x/((x + 1)/2))/2 =
--                        --         ((x + 1)/2 + 2x/(x + 1))/2 =
--                        --         ((x + 1)^2 + 4x)/4(x+1) =
--                        --         (x^2 + 6x + 1)/4(x+1) =
--                        --         (x^2 + 6x + 5 - 4)/4(x+1) =
--                        --         ((x + 5)(x + 1) - 4)/4(x+1) =
--                        --         (x + 5) - 1/(x+1)
--                        babylon2
--                            = xPlus5Div4Up +^ (neg xPlusOneRecipDn)
--                        xPlus5Div4Up 
--                            = ((5::Int) |+^ x) /^| (4::Int)
--                        xPlusOneRecipDn = recipDn xPlusOneUp
--                        xPlusOneUp 
--                            = (1::Int) |+^ x
--                        recipDn = ArithUpDn.recipDnEff effortDiv
--                    -- iteratively improve q, a lower bound on sqrt(1/x)
--                    --   using the formula q_{n+1} = (q_n / 2) * (3 - x * q_n * q_n)  
--                    --   quoted eg in http://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Iterative_methods_for_reciprocal_square_roots
--                    iterRecipSqrt maxIters2 prevFirst aM2 bM2 =
--                        -- assuming aM and bM do not alias
--                        do
--                        qNm1 <- unsafeReadMutable qNm1M
--                        case maxIters2 > 0 && sureAbove0 qNm1 of
--                            False -> -- should not or cannot continue iterating
--                                return prevFirst -- indicate which of the two variables has the older result
--                            True ->
--                                do
--                                newtonIterateDn qN qNm1M
--                                iterRecipSqrt (maxIters2 - 1) (not prevFirst) bM2 aM2 -- swap the variables
--                        where
--                        (qNm2M, qNm1M) 
--                            | prevFirst = (aM2,bM2)
--                            | otherwise = (bM2,aM2)
--                        qN = qNm2M
