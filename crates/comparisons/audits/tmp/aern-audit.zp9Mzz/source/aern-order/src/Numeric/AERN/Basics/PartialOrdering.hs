{-|
    Module      :  Numeric.AERN.Basics.PartialOrdering
    Description :  extension of Prelude.Ordering with non-comparable variant  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Extension of 'Prelude.Ordering' with non-comparable variant.
-}
module Numeric.AERN.Basics.PartialOrdering 
(
    PartialOrderingPartialInfo(..),
    partialOrderingPartialInfoAllNothing,
    partialOrderingPartialInfoAllFalse,
    partialOrderingPartialInfoAnd,
    PartialOrdering(..),
    partialOrdering2PartialInfo,
    partialInfo2PartialOrdering,
    partialOrderingVariants,
    partialOrderingVariantsSet,
    permittedInReflexiveOrder,
    partialOrderingVariantsReflexive,
    permittedInLinearOrder,
    partialOrderingVariantsLinear,
    toPartialOrdering,
    fromPartialOrdering,
    partialOrderingTranspose,
    pickConsistentOrderings,
    transitivityConsequences,
    partialOrderingVariantsTriples
)
where

import qualified Prelude
import Prelude hiding (EQ, LT, GT)
import Test.QuickCheck

import Data.Maybe
import qualified Data.Map as Map
import qualified Data.Set as Set
import qualified Data.List as List

{-| A summary of /partial/ knowledge about the relative position
    of two elements of a partial order.
    
    This is more flexible than @Maybe PartialOrdering@ because,
    for example, sometimes one can establish that two elements are NOT leq
    (and thus also not EQ and not LESS) but not whether the elements
    are GT or NC.  This happens, in particular, when comparing functions
    and it is established that GT or NC holds in some part of the function's
    domain, such as at one point of the domain.
 -}
data PartialOrderingPartialInfo =
    PartialOrderingPartialInfo
    {
        pOrdInfLT :: Maybe Bool,
        pOrdInfLEQ :: Maybe Bool,
        pOrdInfEQ :: Maybe Bool,
        pOrdInfNC :: Maybe Bool,
        pOrdInfGEQ :: Maybe Bool,
        pOrdInfGT :: Maybe Bool
    }
    
instance Show PartialOrderingPartialInfo
    where
    show info =
        "PartialOrderingPartialInfo{"
        ++ "LT:" ++ showM (pOrdInfLT info) 
        ++ ",LEQ:" ++ showM (pOrdInfLEQ info) 
        ++ ",EQ:" ++ showM (pOrdInfEQ info) 
        ++ ",NC:" ++ showM (pOrdInfNC info) 
        ++ ",GEQ:" ++ showM (pOrdInfGEQ info) 
        ++ ",GT:" ++ showM (pOrdInfGT info) 
        ++ "}" 
        where
        showM (Just True) = "T"
        showM (Just False) = "F"
        showM _ = "?"

partialOrderingPartialInfoAllFalse :: PartialOrderingPartialInfo
partialOrderingPartialInfoAllFalse = 
    PartialOrderingPartialInfo jf jf jf jf jf jf

partialOrderingPartialInfoAllNothing :: PartialOrderingPartialInfo
partialOrderingPartialInfoAllNothing = 
    PartialOrderingPartialInfo Nothing Nothing Nothing Nothing Nothing Nothing

jf = Just False
jt = Just True

partialOrderingPartialInfoAnd :: 
    PartialOrderingPartialInfo -> PartialOrderingPartialInfo -> PartialOrderingPartialInfo
partialOrderingPartialInfoAnd comp1 comp2 =
    PartialOrderingPartialInfo (t i1 j1) (t i2 j2) (t i3 j3) (t i4 j4) (t i5 j5) (t i6 j6)
    where
    (PartialOrderingPartialInfo i1 i2 i3 i4 i5 i6) = comp1 
    (PartialOrderingPartialInfo j1 j2 j3 j4 j5 j6) = comp2
    t (Just a) (Just b) = Just $ a && b
    t _ _ = Nothing

{-| Like 'Prelude.Ordering' but with a non-comparable option
 -}
data PartialOrdering = 
        EQ -- equal 
      | LT -- less than
      | GT -- greater than
      | NC -- not comparable
    deriving (Eq, Ord, Show, Enum, Bounded)

partialOrdering2PartialInfo :: Maybe PartialOrdering -> PartialOrderingPartialInfo
partialOrdering2PartialInfo (Just EQ) =
    partialOrderingPartialInfoAllFalse
    {
        pOrdInfLEQ = jt, pOrdInfEQ = jt, pOrdInfGEQ = jt
    }
partialOrdering2PartialInfo (Just LT) =
    partialOrderingPartialInfoAllFalse
    {
        pOrdInfLEQ = jt, pOrdInfLT = jt
    }
partialOrdering2PartialInfo (Just GT) =
    partialOrderingPartialInfoAllFalse
    {
        pOrdInfGEQ = jt, pOrdInfGT = jt
    }
partialOrdering2PartialInfo (Just NC) =
    partialOrderingPartialInfoAllFalse
    {
        pOrdInfNC = jt
    }
partialOrdering2PartialInfo Nothing =
    partialOrderingPartialInfoAllNothing
    

{-|
    For a given record of partial information about partial ordering
    of two elements, give a list of all relations that are
    not ruled out by the partial information.
    
    (We assume that the partial information record is consistent.)
-}
partialInfo2PartialOrdering :: 
    PartialOrderingPartialInfo -> 
    [PartialOrdering]
partialInfo2PartialOrdering info 
    | infoEQ == jt = [EQ]
    | infoLT == jt = [LT]
    | infoGT == jt = [GT]
    | infoLEQ == jt = [LT, EQ]
    | infoGEQ == jt = [GT, EQ]
    | infoNC == jt = [NC]
    | otherwise = mnc ++ mlt ++ mgt ++ meq
    where
    infoEQ = pOrdInfEQ info
    infoNC = pOrdInfNC info
    infoLT = pOrdInfLT info
    infoLEQ = pOrdInfLEQ info
    infoGT = pOrdInfGT info
    infoGEQ = pOrdInfGEQ info
    mnc | infoNC /= jf = [NC]
        | otherwise = []
    mlt | (infoLT /= jf) && (infoLEQ /= jf) = [LT]
        | otherwise = []
    mgt | (infoGT /= jf) && (infoGEQ /= jf) = [GT]
        | otherwise = []
    meq | (infoEQ /= jf) && (infoLEQ /= jf) && (infoGEQ /= jf) = [EQ]
        | otherwise = []
    

instance Arbitrary PartialOrdering
    where
    arbitrary = elements partialOrderingVariants

partialOrderingVariants :: [PartialOrdering]
partialOrderingVariants = [minBound..maxBound]

partialOrderingVariantsSet :: Set.Set PartialOrdering
partialOrderingVariantsSet = Set.fromList partialOrderingVariants
        
permittedInReflexiveOrder :: PartialOrdering -> Bool
--permittedInReflexiveOrder GEE = False
--permittedInReflexiveOrder LEE = False
permittedInReflexiveOrder _ = True

partialOrderingVariantsReflexive = [EQ, LT, GT, NC]
    
permittedInLinearOrder :: PartialOrdering -> Bool
--permittedInLinearOrder GEE = False
--permittedInLinearOrder LEE = False
permittedInLinearOrder NC = False
permittedInLinearOrder _ = True

partialOrderingVariantsLinear = [EQ, LT, GT]
    
toPartialOrdering :: Ordering -> PartialOrdering
toPartialOrdering Prelude.EQ = EQ 
toPartialOrdering Prelude.LT = LT 
toPartialOrdering Prelude.GT = GT 

fromPartialOrdering :: PartialOrdering -> Ordering
fromPartialOrdering EQ = Prelude.EQ 
fromPartialOrdering LT = Prelude.LT 
fromPartialOrdering GT = Prelude.GT
fromPartialOrdering rel = error $ "cannot convert " ++ show rel ++ " to Prelude.Ordering" 

{-| flip an ordering relation -}
partialOrderingTranspose :: PartialOrdering -> PartialOrdering
partialOrderingTranspose LT = GT
--partialOrderingTranspose LEE = GEE
partialOrderingTranspose GT = LT
--partialOrderingTranspose GEE = LEE
partialOrderingTranspose a = a

{-
   From ambiguous contraints on the ordering of elements of a tuple of some size
   produce all consistent unambiguous orderings of all pairs in such tuples.
-}
pickConsistentOrderings ::
    (Eq ix, Ord ix, Show ix) =>
    (PartialOrdering -> Bool) {-^ filter of permissible orderings -} ->
    [ix] {-^ indices for elements of the resulting tuples -} ->
    [((ix, ix), [PartialOrdering])] {-^ constraints on the ordering of the tuples -} ->
    [[((ix, ix), PartialOrdering)]]
pickConsistentOrderings orderingsFilter indices constraints =
    depthFirst Map.empty [] allPairsAndTriples 
    where
    allPairsAndTriples = -- eg [((3,4),[]),((2,4),[]),((2,3),[4]),((1,4),[]),((1,3),[4]),((1,2),[3,4])]
        reverse $
        [((head postfix1, head postfix2), tail postfix2) | 
            postfix1 <- init (List.tails indices), -- eg [[1,2,3,4],[2,3,4],[3,4],[4]] 
            postfix2 <- init (List.tails (tail postfix1))]
    depthFirst pairLookupMap setPairs [] = [setPairs] 
    depthFirst pairLookupMap setPairs ((pair@(i1,i2), i3s) : remainingPairs) =
        concat $ map recurse allowedOrderings 
        where
        recurse ordering =
           depthFirst updatedPairLookupMap ((pair, ordering) : setPairs) remainingPairs   
           where
           updatedPairLookupMap =
               Map.insert (i2,i1) (partialOrderingTranspose ordering) $ 
               Map.insert (i1,i2) ordering pairLookupMap
        allowedOrderings =
            Set.toList $
                Set.intersection (lkConstraints pair) transitivityConstraints
        transitivityConstraints =
            foldl Set.intersection partialOrderingVariantsSet $ 
                map transitivityConstraint i3s
        transitivityConstraint i3 =
            case (Map.lookup (i1,i3) pairLookupMap, Map.lookup (i3,i2) pairLookupMap) of
                (Just rel13, Just rel32) -> transitivityConsequences rel13 rel32
                _ -> error $
                       "pickConsistentOrderings: transitivityConstraint:" 
                       ++ "\n pairLookupMap = " ++ show pairLookupMap
                       ++ "\n i1 = " ++ show i1 ++ "; i2 = " ++ show i2 ++ "; i3 = " ++ show i3
        constraintsMap = 
            Map.map (Set.fromList . filter orderingsFilter) $ Map.fromList $ constraints
        lkConstraints pair@(i1, i2) =
            case (Map.lookup pair constraintsMap, Map.lookup (i2,i1) constraintsMap) of
                (Just options, Just optionsT) ->
                    Set.intersection options $ Set.map partialOrderingTranspose optionsT
                (Just options, Nothing) ->
                    options
                (Nothing, Just optionsT) ->
                    Set.map partialOrderingTranspose optionsT
                (Nothing, Nothing) ->
                    Set.fromList $ filter orderingsFilter partialOrderingVariants

transitivityConsequences :: PartialOrdering -> PartialOrdering -> Set.Set PartialOrdering
transitivityConsequences rel12 rel23 = 
    case (rel12, rel23) of
        (EQ, rel23) -> Set.singleton rel23
        (rel12, EQ) -> Set.singleton rel12
        (LT, LT) -> Set.singleton LT
--        (LEE, LT) -> Set.singleton LT 
--        (LT, LEE) -> Set.singleton LT
--        (LEE, LEE) -> Set.fromList [LEE, LT]
        (GT, GT) -> Set.singleton GT
--        (GEE, GT) -> Set.singleton GT
--        (GT, GEE) -> Set.singleton GT
--        (GEE, GEE) -> Set.fromList [GEE, GT]
--        (LEE, GEE) -> noneOf [GT, LT]
--        (GEE, LEE) -> noneOf [GT, LT]
        (LT, NC) -> noneOf [GT, EQ]
        (GT, NC) -> noneOf [LT, EQ]
        (NC, LT) -> noneOf [GT, EQ]
        (NC, GT) -> noneOf [LT, EQ]

--        (LT, NC) -> noneOf [GT, GEE, EQ]
--        (LEE, NC) -> noneOf [GT, GEE, EQ]
--        (GT, NC) -> noneOf [LT, LEE, EQ]
--        (GEE, NC) -> noneOf [LT, LEE, EQ]
--        (NC, LT) -> noneOf [GT, GEE, EQ]
--        (NC, LEE) -> noneOf [GT, GEE, EQ]
--        (NC, GT) -> noneOf [LT, LEE, EQ]
--        (NC, GEE) -> noneOf [LT, LEE, EQ]
        _ -> partialOrderingVariantsSet
    where
    noneOf list = partialOrderingVariantsSet `Set.difference` (Set.fromList list) 

{-|
    All 29 triples of Comparison orderings @(r1, r2, r3)@ for which
    there could be elements satisfying
    @e1 `r1` e2 && e2 `r2` e3 && e1 `r3` e3@
    (ie not breaking transitivity).
-}
partialOrderingVariantsTriples :: [(PartialOrdering, PartialOrdering, PartialOrdering)]
partialOrderingVariantsTriples =
    map convertToTriple $
        pickConsistentOrderings (const True) [1,2,3] []
    where
    convertToTriple [((1,2),rel12), ((1,3),rel13), ((2,3),rel23)] = (rel12,rel23,rel13)

{- The following is useful for some manual testing -}



--allConsistentTriples = pickConsistentOrderings (const True) [1,2,3] [] 
--
--allConsistentTriplesReflexiveOrder = 
--    pickConsistentOrderings permittedInReflexiveOrder [1,2,3] [] 
--
--allConsistentTriplesLinearOrder = 
--    pickConsistentOrderings permittedInLinearOrder [1,2,3] [] 

--test =
--    Set.difference 
--        (Set.fromList partialOrderingVariantsTriples)
--        (Set.fromList partialOrderingVariantsTriples2)
--    where
--    convertToTriple assocList =
--        (rel12, rel23, rel13)
--        where
--        rel12 = fromJust $ Map.lookup (1,2) orderingMap
--        rel23 = fromJust $ Map.lookup (2,3) orderingMap
--        rel13 = fromJust $ Map.lookup (1,3) orderingMap
--        orderingMap = Map.fromList assocList
--
--{-|
--    All 29 triples of Comparison orderings @(r1, r2, r3)@ for which
--    there could be elements satisfying
--    @e1 `r1` e2 && e2 `r2` e3 && e1 `r3` e3@
--    (ie not breaking transitivity).
---}
--partialOrderingVariantsTriples2 :: [(PartialOrdering, PartialOrdering, PartialOrdering)]
--partialOrderingVariantsTriples2 =
--    [(r1,r2,r3)| 
--        r1 <- partialOrderingVariantsReflexive, 
--        r2 <- partialOrderingVariantsReflexive, 
--        r3 <- partialOrderingVariantsReflexive,
--        respectsTransitivity (r1, r2, r3)]
--
--{-|
--    Are there any elements satisfying
--    @e1 `r1` e2 && e2 `r2` e3 && e1 `r3` e3@
--    assuming equality and order are transitive.
---}
--respectsTransitivity :: 
--    (PartialOrdering, PartialOrdering, PartialOrdering) {-^ @(r1,r2,r3)@ -} -> 
--    Bool 
--respectsTransitivity rels =
--    case rels of
--        -- when a pair is equal:
--        (EQ,r2,r3) -> r2 == r3 -- e1 = e2
--        (r1,EQ,r3) -> r1 == r3 -- e2 = e3
--        (r1,r2,EQ) -> r1 == partialOrderingTranspose r2 -- e1 = e3
--        -- 6 permutations of strict inequalities:
--        (LT,LT,LT) -> True -- e1 < e2 < e3 (1)
--        (LT,LT,_ ) -> False -- but not e1 < e3
--        (LT,GT,LT) -> True -- e1 < e3 < e2 (2)
--        (_ ,GT,LT) -> False -- but not e1 < e2
--        (GT,LT,LT) -> True -- e2 < e1 < e3 (3)
--        (GT,_ ,LT) -> False -- but not e2 < e3
--        (GT,LT,GT) -> True -- e2 < e3 < e1 (4)
--        (_ ,LT,GT) -> False -- but not e2 < e1
--        (LT,GT,GT) -> True -- e3 < e1 < e2 (5)
--        (LT,_ ,GT) -> False -- but not e3 < e2
--        (GT,GT,GT) -> True -- e3 < e2 < e1 (6)
--        (GT,GT,_ ) -> False -- but not e3 < e1
--        --
--        _ -> True -- all else is OK


