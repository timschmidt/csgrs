{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE NoMonomorphismRestriction #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-|
    Module      :  Numeric.AERN.RmToRn.NumericOrder.FromInOutRingOps.Arbitrary
    Description :  approximation of min and max using ring operations
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable

    Approximation of min and max using ring operations.
    .
    The motivating use case for this module is where we compute min or max for a 
    /function/ pointwise over its domain.
-}

module Numeric.AERN.RmToRn.NumericOrder.FromInOutRingOps.Arbitrary where

import Prelude hiding (LT,GT,EQ)

import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Evaluation

import Numeric.AERN.RealArithmetic.ExactOps

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut

import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import qualified Numeric.AERN.RefinementOrder as RefOrd
import Numeric.AERN.RefinementOrder (IntervalLike(..))
----import Numeric.AERN.RefinementOrder.InPlace.OpsImplicitEffort


import qualified Numeric.AERN.NumericOrder as NumOrd
--import Numeric.AERN.NumericOrder.OpsImplicitEffort

--import Numeric.AERN.Basics.Effort
--import Numeric.AERN.Basics.Mutable
import Numeric.AERN.Basics.SizeLimits
import Numeric.AERN.Basics.PartialOrdering
import Numeric.AERN.Basics.Consistency

import Numeric.AERN.Misc.Debug

import Test.QuickCheck
import Numeric.AERN.Misc.QuickCheck
import qualified System.Random as R


type Area4FunFromRingOps f = 
    (f, -- sample value
     Maybe (Domain f)) -- maybe range

areaWhole4FunFromRingOps sampleFn =
    (sampleFn, Nothing)

{-|
   An arbitraryInArea implementation for almost any function type.  
   
   LIMITATION 1:
   Currently this function produces only elements
   that are consistent and close to thin.  The generated
   elements need to be further processed to get examples
   of thick, anticonsistent or inconsistent elements.
   
   LIMITATION 2:
   With requests for more than 2 elements, 
   this function currently always
   produces lists of elements that are linearly orderable
   and refuses requests to generate incomparable elements.
-}    
arbitraryInArea4FunFromRingOps ::
    (HasDomainBox fn,  HasConstFns fn, HasProjections fn, 
     ArithInOut.RoundedAdd fn,
     ArithInOut.RoundedMultiply fn,
     CanEvaluate fn,
     Show (Domain fn), IntervalLike (Domain fn),
     ArithInOut.RoundedReal (Domain fn),
     RefOrd.ArbitraryOrderedTuple (Domain fn),
     ArithInOut.RoundedMixedAdd fn (Domain fn),
     ArithInOut.RoundedMixedMultiply fn (Domain fn)
    )
    =>
    ((ArithInOut.RoundedRealEffortIndicator (Domain fn),
      GetEndpointsEffortIndicator (Domain fn),
      EvaluationEffortIndicator fn
     ),
     (ArithInOut.AddEffortIndicator fn,
      ArithInOut.MultEffortIndicator fn
     ),
     (ArithInOut.MixedAddEffortIndicator fn (Domain fn),
      ArithInOut.MixedMultEffortIndicator fn (Domain fn)
     )
    ) ->
    [fn] ->
    (Int -> Int) ->
    (Area4FunFromRingOps fn) ->
    (Gen fn)
arbitraryInArea4FunFromRingOps 
        ((effDom, effGetEndptsDom, effEval), 
         (effAddFn, effMultFn), 
         (effAddFnDFn, effMultFnDFn))
        fnSequence2
        fixedRandSeqQuantityOfSize
        area@(sampleFn, maybeRange)
    =
    arbitraryFnFromSequence
    where
    arbitraryFnFromSequence =
        arbitraryFromSequence fnSequence2
    arbitraryFromSequence seq 
        =
        sized $ \size ->
        do
        ix <- choose (0, fixedRandSeqQuantityOfSize size - 1)
        return $ 
--            unsafePrint ("arbitraryTupleInAreaRelatedBy4FunFromRingOps: size = " ++ show size ++ ", ix = " ++ show ix) $ 
                seq !! ix
    
    
{-|
   An arbitraryTupleInAreaRelatedBy implementation
   for almost any function type.  
   
   LIMITATION 1:
   Currently this function produces only elements
   that are consistent and close to thin.  The generated
   elements need to be further processed to get examples
   of thick, anticonsistent or inconsistent elements.
   
   LIMITATION 2:
   With requests for more than 2 elements, 
   this function currently always
   produces lists of elements that are linearly orderable
   and refuses requests to generate incomparable elements.
-}    
arbitraryTupleInAreaRelatedBy4FunFromRingOps ::
    (Show ix, Ord ix, 
     HasDomainBox fn,  HasConstFns fn, HasProjections fn, 
     ArithInOut.RoundedAdd fn,
     ArithInOut.RoundedMultiply fn,
     CanEvaluate fn,
     Show (Domain fn), IntervalLike (Domain fn),
     ArithInOut.RoundedReal (Domain fn),
     RefOrd.ArbitraryOrderedTuple (Domain fn),
     ArithInOut.RoundedMixedAdd fn (Domain fn),
     ArithInOut.RoundedMixedMultiply fn (Domain fn)
    )
    =>
    ((ArithInOut.RoundedRealEffortIndicator (Domain fn),
      GetEndpointsEffortIndicator (Domain fn),
      EvaluationEffortIndicator fn
     ),
     (ArithInOut.AddEffortIndicator fn,
      ArithInOut.MultEffortIndicator fn
     ),
     (ArithInOut.MixedAddEffortIndicator fn (Domain fn),
      ArithInOut.MixedMultEffortIndicator fn (Domain fn)
     )
    ) ->
    [fn] ->
    (Int -> Int) ->
    (Area4FunFromRingOps fn) ->
    [ix] -> 
    [((ix, ix),[PartialOrdering])]
    -> Maybe (Gen [fn])
arbitraryTupleInAreaRelatedBy4FunFromRingOps 
        ((effDom, effGetEndptsDom, effEval), 
         (effAddFn, effMultFn), 
         (effAddFnDFn, effMultFnDFn))
        (fnSequence2 :: [fn])
        fixedRandSeqQuantityOfSize
        area@(sampleFn, maybeRange)
        indices rels
    =
--    unsafePrint 
--        ("arbitraryTupleInAreaRelatedBy4FunFromRingOps: ids = " 
--         ++ show indices ++ "; rels = " ++ show rels ) $ 
    case (indices, rels) of
        ([_], _) ->
            Just $
                do
                fn <- arbitraryFnFromSequence
                return [fn]
        ([_i1,_i2], [((_i1a,_i2a),[NC])]) ->
            Just $
                do
                fn1 <- arbitraryFnFromSequence
                fn2 <- arbitraryFnFromSequence
                return $ ensureOverlap [fn1, fn2]
        _ ->
            NumOrd.forcedLinearArbitraryTupleRelatedBy
                arbitraryFnFromSequence 
                pickAndShiftGetSorted
                indices rels
    where
    arbitraryFnFromSequence =
        arbitraryInArea4FunFromRingOps 
            ((effDom, effGetEndptsDom, effEval), 
             (effAddFn, effMultFn), 
             (effAddFnDFn, effMultFnDFn))
            fnSequence2 fixedRandSeqQuantityOfSize area

    (.</>.) = ArithInOut.divOutEff effDivDom
    (.<*>.) = ArithInOut.multOutEff effMulDom
    (.<+>.) = ArithInOut.addOutEff effAddDom
    (.<->.) = ArithInOut.subtrOutEff effAddDom
    (~<+>.) = ArithInOut.mixedAddOutEff effAddFnDFn
    (~<*>.) = ArithInOut.mixedMultOutEff effMultFnDFn

    sampleDom = getSampleDomValue sampleFn
    effAddDom = ArithInOut.fldEffortAdd sampleDom $ ArithInOut.rrEffortField sampleDom effDom
    effMulDom = ArithInOut.fldEffortMult sampleDom $ ArithInOut.rrEffortField sampleDom effDom
    effDivDom = ArithInOut.fldEffortDiv sampleDom $ ArithInOut.rrEffortField sampleDom effDom
    effRefComp = ArithInOut.rrEffortRefComp sampleDom effDom
    effJoin = ArithInOut.rrEffortJoinMeet sampleDom effDom
    
    ensureOverlap :: [fn] -> [fn]
    ensureOverlap fns@[fn1,fn2] = [fn1,fn2Shifted]
        where
        fn2Shifted = fn2 ~<+>. ((evalAtPt fn1) .<->. (evalAtPt fn2))
        pt = getSampleFromInsideDomainBox fn1 domainBox 
        domainBox = getDomainBox fn1
        evalAtPt fn =
            evalAtPointOutEff effEval pt fn 
    addBounds fn = (fn, lower, upper)
        where
        (lower, upper) = getEndpointsOutEff effGetEndptsDom range
        range = evalAtPointOutEff effEval domainbox fn
        domainbox = getDomainBox fn
    pickAndShiftGetSorted seed n list 
        | n < 1 = []
        | otherwise 
            =
            shrinkToRange $
            pick n first (zip randomBools rest)
            where
            pick n (fnPrev, lowerPrev, upperPrev) ((copyPrev, current@(fn,lower,upper)):rest) 
                | n == 0 = []
                | copyPrev
                    =
                    (fnPrevTrCp, (lowerPrev, upperPrevTrCp)) : (pick (n-1) (fnPrevTrCp, lowerPrev, upperPrevTrCp) ((False, current):rest))
                | otherwise
                    =
                    (fnTr, (lowerTr, upperTr)) : (pick (n-1) (fnTr, lowerTr, upperTr) rest) 
                where
                fnPrevTrCp = fnPrev ~<+>. distCp
                upperPrevTrCp = upperPrev .<+>. distCp
                distCp = one sampleDom
                fnTr = fn ~<+>. dist
                upperPrevTr = upperPrev .<+>. dist
                upperTr = upper .<+>. dist
                lowerTr = lower .<+>. dist
                dist = (upperPrev .<->. lower) .<+>. (one sampleDom) 
                sampleDom = snd $ head $ toAscList $ getDomainBox fn
                
            fnBounds@(first:rest) = map addBounds list
            randomBools = 
                map even $ 
                    map fst $ 
                        drop 13 $ iterate (R.next . snd) (0,g)
            g = R.mkStdGen seed
            
            shrinkToRange list =
                case maybeRange of 
                    Nothing -> map fst list
                    Just range -> map (shrink range) list
                where
                shrink range (fn,_) =
                    (fn ~<*>. scalingFactor) ~<+>. shift
                    where
                    scalingFactor 
                        | ((z </\> listWidth) ⊒? (z </\> rangeWidth)) == Just True
                            -- ie listWidth <= rangeWidth
                            = one range
                        | (z ⊒? listWidth) == Just False
                            -- ie no division by zero
                            = rangeWidth .</>. listWidth
                        | otherwise
                            = one range 
                        where
                        (⊒?) = RefOrd.pGeqEff effRefComp
                        (</\>) = RefOrd.meetOutEff effJoin
                        z = zero range
                    rangeWidth = rangeR .<->. rangeL
                    (rangeL, rangeR) = RefOrd.getEndpointsOutEff effGetEndptsDom range
                    shift = rangeL .<->. (listL .<*>. scalingFactor)
                (_, (listL,_)) = head list 
                (_, (_,listR)) = last list
                listWidth = listR .<->. listL 

fnSequence ::
    (HasConstFns a, HasProjections a, ArithInOut.RoundedMultiply a,
     ArithInOut.RoundedMixedMultiply a (Domain a), HasOne (Domain a),
     HasInfinities (Domain a), RefOrd.ArbitraryOrderedTuple (Domain a),
     IntervalLike (Domain a), ArithInOut.RoundedAdd a) 
     =>
    (Int -> Int) -> a -> [a]
fnSequence fixedRandSeqQuantityOfSize sampleFn = 
    fixedRandSeq fixedRandSeqQuantityOfSize $ 
        arbitraryFn (effAddFn, effMultFn, effMultFnDFn, effGetEndptsDom) sampleFn
        where
        effAddFn = ArithInOut.addDefaultEffort sampleFn
        effMultFn = ArithInOut.multDefaultEffort sampleFn
        effMultFnDFn = ArithInOut.mixedMultDefaultEffort sampleFn sampleDom
        effGetEndptsDom = RefOrd.getEndpointsDefaultEffort sampleDom
        sampleDom = getSampleDomValue sampleFn

arbitraryFn ::
    (HasProjections fn, HasConstFns fn,
     ArithInOut.RoundedAdd fn,
     ArithInOut.RoundedMultiply fn,
     ArithInOut.RoundedMixedMultiply fn (Domain fn),
     RefOrd.IntervalLike (Domain fn),
     HasInfinities (Domain fn),
     RefOrd.ArbitraryOrderedTuple (Domain fn),
     HasOne (Domain fn)
    )
    =>
    (ArithInOut.AddEffortIndicator fn, 
     ArithInOut.MultEffortIndicator fn,
     ArithInOut.MixedMultEffortIndicator fn (Domain fn),
     RefOrd.GetEndpointsEffortIndicator (Domain fn)) ->
    fn ->
    Gen fn
arbitraryFn 
        (effAddFn, effMultFn, effMultFnDFn, effGetEndptsDom) 
        (sampleFn :: fn)
    =
    sized $ \size ->
    do
    -- choose polynomial generation parameters:
    degree <- choose (0,2 + (size `div` 7))
    constrTerms <- choose (1,3 + size)
    -- start building the result polynomial, first creating variables:
    varFns <- mapM (\v -> return $ newProjection sizeLimits varDoms v) vars
    -- now multiplying variables in various powers:  
    powerTerms <- mapM (const $ arbitraryPowerTerm varFns degree) [1..constrTerms]
    -- and combining them as a linear combination:
    coeffsL <- vectorOf (3 * constrTerms) ((\(Just a) -> a) $ RefOrd.arbitraryTuple 1)
    let coeffs = filter bounded $ map getEndpoint coeffsL
    let _ = sampleDom : coeffs
--    unsafePrint ("arbitraryFn: size = " ++ show size) $
    return $ 
        foldl1 (~<+>~) $ zipWith (~<*>.) powerTerms coeffs
    where
    getEndpoint [a] = fst $ RefOrd.getEndpointsOutEff effGetEndptsDom a
    bounded a = excludesInfinity a 
    arbitraryPowerTerm varFns degree
        | null varFns || degree == 0 
            = return $ newConstFn sizeLimits varDoms $ one sampleDom
        | otherwise = 
            do
            (n,varFn) <- elements $ zip [0..] varFns
            dg <- choose (1,degree)
            let remainingVarFns = take (n-1) varFns ++ drop (n+1) varFns
            restFn <- arbitraryPowerTerm remainingVarFns (degree - dg)
            let varFnPwr = foldl1 (~<*>~) $ replicate dg varFn 
            return $ varFnPwr ~<*>~ restFn -- rounding direction irrelevant
    sizeLimits = getSizeLimits sampleFn
    sampleDom = getSampleDomValue sampleFn
    varDoms = getVarDoms sampleFn
    vars :: [Var fn]
    (vars, _) = unzip varDoms
    
    (~<*>~) = ArithInOut.multOutEff effMultFn
    (~<+>~) = ArithInOut.addOutEff effAddFn
    (~<*>.) = ArithInOut.mixedMultOutEff effMultFnDFn
    

        