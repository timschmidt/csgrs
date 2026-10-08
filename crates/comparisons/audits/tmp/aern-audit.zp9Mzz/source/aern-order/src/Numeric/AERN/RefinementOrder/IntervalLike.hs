{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-|
    Module      :  Numeric.AERN.RefinementOrder.IntervalLike
    Description :  types with endpoints
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Types whose values have endpoints.
-}

module Numeric.AERN.RefinementOrder.IntervalLike where

import Numeric.AERN.RefinementOrder.PartialComparison

import Numeric.AERN.Basics.Exception
import Numeric.AERN.Basics.Effort
import Numeric.AERN.Basics.Laws.RoundedOperation

import Numeric.AERN.Misc.Bool
import Numeric.AERN.Misc.Maybe

import Data.Maybe

import Test.QuickCheck
import Test.Framework (testGroup, Test)
import Test.Framework.Providers.QuickCheck2 (testProperty)

class
    (EffortIndicator (GetEndpointsEffortIndicator t), 
     EffortIndicator (FromEndpointsEffortIndicator t))
    => 
    IntervalLike t 
    where
    type GetEndpointsEffortIndicator t
    type FromEndpointsEffortIndicator t
    getEndpointsDefaultEffort :: t -> GetEndpointsEffortIndicator t
    fromEndpointsDefaultEffort :: t -> FromEndpointsEffortIndicator t
--    getEndpointsInEff :: (GetEndpointsEffortIndicator t) -> t -> (t,t)
-- getEndpointsInEff makes no sense as the endpoints are "thin" 
-- and an it makes little sense to approximate them inwards.
    getEndpointsOutEff :: (GetEndpointsEffortIndicator t) -> t -> (t,t)
    fromEndpointsInEff :: (FromEndpointsEffortIndicator t) -> (t,t) -> t 
    fromEndpointsOutEff :: (FromEndpointsEffortIndicator t) -> (t,t) -> t 

-- versions  with default effort
--getEndpointsIn :: (IntervalLike t) => t -> (t,t)
--getEndpointsIn a = getEndpointsInEff (getEndpointsDefaultEffort a) a
getEndpointsOut :: (IntervalLike t) => t -> (t,t)
getEndpointsOut a = getEndpointsOutEff (getEndpointsDefaultEffort a) a
fromEndpointsIn :: (IntervalLike t) => (t,t) -> t
fromEndpointsIn p@(l,r) = fromEndpointsInEff (fromEndpointsDefaultEffort l) p
fromEndpointsOut :: (IntervalLike t) => (t,t) -> t
fromEndpointsOut p@(l,r) = fromEndpointsOutEff (fromEndpointsDefaultEffort l) p
    
propEndpointsFromGet :: 
    (IntervalLike t, Show t, 
     HasLegalValues t, PartialComparison t) 
    => 
    t -> 
    (GetEndpointsEffortIndicator t) -> 
    (FromEndpointsEffortIndicator t) -> 
    (PartialCompareEffortIndicator t) -> 
    t -> Bool
propEndpointsFromGet _ effGet effFrom effComp e =
    outerFromGet && innerFromGet
    where
    outerFromGet =
        leqIfDefined "outer (from.get) endpoints" (pLeqEff effComp) 
            (fromEndpointsOutEff effFrom $ getEndpointsOutEff effGet e) e 
    innerFromGet =
        leqIfDefined "inner (from.get) endpoints" (pLeqEff effComp) 
            e (fromEndpointsInEff effFrom $ getEndpointsOutEff effGet e) 

testsEndpoints :: 
    (Arbitrary t, Show t, Eq t,
     IntervalLike t,
     HasLegalValues t, PartialComparison t) 
    => 
    (String, t) -> Test
testsEndpoints (name, sample) =
    testGroup (name ++ " consistency flip")
        [
         testProperty "interval <-> endpoints" (propEndpointsFromGet sample)
        ]
