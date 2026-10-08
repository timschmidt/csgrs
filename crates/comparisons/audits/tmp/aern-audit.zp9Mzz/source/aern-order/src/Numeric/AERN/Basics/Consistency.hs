{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-|
    Module      :  Numeric.AERN.Basics.Consistency
    Description :  types with consistent and inconsistent values
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Types with consistent and inconsistent values.
-}

module Numeric.AERN.Basics.Consistency where

import Numeric.AERN.Basics.Effort

import Numeric.AERN.Misc.Bool
import Numeric.AERN.Misc.Maybe

import Data.Maybe

import Test.QuickCheck
import Test.Framework (testGroup, Test)
import Test.Framework.Providers.QuickCheck2 (testProperty)

data ConsistencyStatus =
    Inconsistent -- ie neither Consistent nor Anticonsistent
    | Consistent 
    | Anticonsistent 
    | Exact -- ie both Consistent and Anticonsistent
    deriving (Eq, Show)
    
class
    (EffortIndicator (ConsistencyEffortIndicator t))
    => 
    HasConsistency t 
    where
    type ConsistencyEffortIndicator t
    consistencyDefaultEffort :: t -> ConsistencyEffortIndicator t
    getConsistencyEff :: (ConsistencyEffortIndicator t) -> t -> Maybe ConsistencyStatus
    isConsistentEff :: (ConsistencyEffortIndicator t) -> t -> Maybe Bool
    isConsistentEff eff e = 
        case getConsistencyEff eff e of
            Just Consistent -> Just True
            Just Exact -> Just True
            Just _ -> Just False
            _ -> Nothing
    isExactEff :: (ConsistencyEffortIndicator t) -> t -> Maybe Bool
    isExactEff eff e =
        case getConsistencyEff eff e of
            Just Exact -> Just True
            Just _ -> Just False
            _ -> Nothing

isExact :: (HasConsistency t) => t -> Maybe Bool
isExact e =
    isExactEff (consistencyDefaultEffort e) e
    
class (HasConsistency t) => HasAntiConsistency t where
    isAntiConsistentEff :: (ConsistencyEffortIndicator t) -> t -> Maybe Bool
    isAntiConsistentEff eff e = 
        case getConsistencyEff eff e of
            Just Anticonsistent -> Just True
            Just Exact -> Just True
            Just _ -> Just False
            _ -> Nothing
    flipConsistency :: t -> t
    
class HasThinRepresentative t where
    -- get a value that is both consistent and anticonsistent 
    -- as well as close to the argument value
    getThinRepresentative :: t -> t
    
propFlipConsistency :: 
    (HasAntiConsistency t, Eq t) => t -> (ConsistencyEffortIndicator t) -> t -> Bool
propFlipConsistency _ effort e =
    (defined eConsistent && defined eFAntiConsistent)
    ===>
    (fromJust eConsistent <===> fromJust eFAntiConsistent)
    where
    eF = flipConsistency e
    eConsistent = isConsistentEff effort e
    eFAntiConsistent =  isAntiConsistentEff effort eF

propConsistencyFlipSelfInverse :: 
    (HasAntiConsistency t, Eq t) => t -> t -> Bool
propConsistencyFlipSelfInverse _ e =
    e == (flipConsistency $ flipConsistency e)

testsConsistency :: 
    (Arbitrary t, Show t, Eq t,
     HasAntiConsistency t) 
     => 
    (String, t) -> Test
testsConsistency (name, sample) =
    testGroup (name ++ " consistency flip")
        [
         testProperty "con<->anticon" (propFlipConsistency sample)
        ,
         testProperty "self inverse" (propConsistencyFlipSelfInverse sample)
        ]
