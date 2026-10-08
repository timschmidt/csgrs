{-|
    Module      :  Numeric.AERN.Misc.Maybe
    Description :  utilities for partial predicates and partial operations
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Utilities for partial predicates and partial operations.
-}
module Numeric.AERN.Misc.Maybe where

import Data.Maybe

defined :: (Maybe t) -> Bool
defined = isJust

assumeTotal1 :: (t1 -> Maybe t) -> (t1 -> t)
assumeTotal1 f a =
    case f a of Just res -> res

assumeTotal2 :: (t2 -> t1 -> Maybe t) -> (t2 -> t1 -> t)
assumeTotal2 f a b =
    case f a b of Just res -> res

completeWith :: t -> (t2 -> t1 -> Maybe t) -> (t2 -> t1 -> t)
completeWith e (*?) e2 e1 =
    case e2 *? e1 of Just r -> r; Nothing -> e  

trueOrNothing :: Maybe Bool -> Bool 
trueOrNothing = nothingOr True

falseOrNothing :: Maybe Bool -> Bool
falseOrNothing = nothingOr False

nothingOr :: (Eq a) => a -> Maybe a -> Bool 
nothingOr _ Nothing = True
nothingOr e1 (Just e2) = e1 == e2     

justButNot :: (Eq a) => a -> Maybe a -> Bool
justButNot _ Nothing = False
justButNot e1 (Just e2) = e1 /= e2 

(||?) :: Maybe Bool -> Maybe Bool -> Maybe Bool 
(||?) b1@(Just True) _ = b1 
(||?) _ b2@(Just True) = b2
(||?) b1@(Just False) b2@(Just False) = b1
(||?) _ _ = Nothing

(&&?) :: Maybe Bool -> Maybe Bool -> Maybe Bool 
(&&?) b1@(Just False) _ = b1 
(&&?) _ b2@(Just False) = b2
(&&?) b1@(Just True) b2@(Just True) = b1
(&&?) _ _ = Nothing

 