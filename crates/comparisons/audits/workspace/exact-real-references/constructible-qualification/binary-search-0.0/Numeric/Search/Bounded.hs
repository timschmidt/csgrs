-----------------------------------------------------------------------------
-- |
-- Module      :  Numeric.Search.Bounded
-- Copyright   :  (c) Ross Paterson 2008
-- License     :  BSD-style
-- Maintainer  :  ross@soi.city.ac.uk
-- Stability   :  experimental
-- Portability :  portable
--
-- Searching unbounded intervals within bounded integral types for the
-- boundary of an upward-closed set, using a combination of exponential
-- and binary search.
--
-----------------------------------------------------------------------------
--
module Numeric.Search.Bounded (search, searchFrom, searchTo) where

import Numeric.Search.Range

-- | /O(log(abs n))/.
-- Search a bounded integral type.
--
-- If @p@ is an upward-closed predicate, @search p@ returns
-- @Just n@ if and only if @n@ is the least such satisfying @p@.
search :: (Bounded a, Integral a) => (a -> Bool) -> Maybe a
search p
  | p 0 = Just (searchDown p minBound 0)
  | otherwise = searchUp p 1 maxBound

-- | /O(log(abs n))/.
-- Search the part of a bounded integral type from a given point.
--
-- If @p@ is an upward-closed predicate, @searchFrom p l@ returns
-- @Just n@ if and only if @n@ is the least @n >= l@ satisfying @p@.
searchFrom :: (Bounded a, Integral a) => (a -> Bool) -> a -> Maybe a
searchFrom p l
  | l <= 0 && p 0 = Just (searchDown p l 0)
  | otherwise = searchUp p (max 1 l) maxBound

-- | /O(log(abs n))/.
-- Search the part of a bounded integral type up to a given point.
--
-- If @p@ is an upward-closed predicate, @searchTo p h@ returns
-- @Just n@ if and only if @n@ is the least @n <= h@ satisfying @p@.
searchTo :: (Bounded a, Integral a) => (a -> Bool) -> a -> Maybe a
searchTo p h
  | p h' = Just (searchDown p minBound h')
  | otherwise = searchUp p 1 h
  where h' = min 0 h

-- @h <= 0 && p h@
searchDown :: (Integral a) => (a -> Bool) -> a -> a -> a
searchDown p l h
  | l `quot` 2 >= h = searchSafeRange p l h
  | p h' = searchDown p l h'
  | otherwise = searchSafeRange p (h'+1) h
  where h' = h*2 - 1

-- @0 < l@
searchUp :: (Integral a) => (a -> Bool) -> a -> a -> Maybe a
searchUp p l h
  | h `div` 2 <= l = searchFromTo p l h
  | p l' = Just (searchSafeRange p l l')
  | otherwise = searchUp p (l'+1) h
  where l' = l*2 + 1

-- | Like 'search', but assuming @l <= h && p h@.
searchSafeRange :: Integral a => (a -> Bool) -> a -> a -> a
searchSafeRange p l h
  | l == h = l
  | p m = searchSafeRange p l m
  | otherwise = searchSafeRange p (m+1) h
  -- Stay within @min 0 l .. max 0 h@ to avoid overflow.
  where m = l `div` 2 + h `div` 2	-- If l < h, then l <= m < h
