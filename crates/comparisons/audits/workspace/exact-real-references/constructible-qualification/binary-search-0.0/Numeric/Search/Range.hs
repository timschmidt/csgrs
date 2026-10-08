-----------------------------------------------------------------------------
-- |
-- Module      :  Numeric.Search.Range
-- Copyright   :  (c) Ross Paterson 2008
-- License     :  BSD-style
-- Maintainer  :  ross@soi.city.ac.uk
-- Stability   :  experimental
-- Portability :  portable
--
-- Binary search of a bounded interval of an integral type for the
-- boundary of an upward-closed set, using a combination of exponential
-- and binary search.
--
-----------------------------------------------------------------------------

module Numeric.Search.Range (searchFromTo) where

-- | /O(log(h-l))/.
-- Search a bounded interval of some integral type.
--
-- If @p@ is an upward-closed predicate, @searchFromTo p l h == Just n@
-- if and only if @n@ is the least number @l <= n <= h@ satisfying @p@.
--
-- For example, the following function determines the first index (if any)
-- at which a value occurs in an ordered array:
--
-- > searchArray :: Ord a => a -> Array Int a -> Maybe Int
-- > searchArray x array = do
-- >   let (lo, hi) = bounds array
-- >   k <- searchFromTo (\ i -> array!i >= x) lo hi
-- >   guard (array!k == x)
-- >   return k
--
searchFromTo :: Integral a => (a -> Bool) -> a -> a -> Maybe a
searchFromTo p l h
  | l > h = Nothing
  | p h = Just (searchSafeRange p l h)
  | otherwise = Nothing

-- | Like 'searchFromTo', but assuming @l <= h && p h@.
searchSafeRange :: Integral a => (a -> Bool) -> a -> a -> a
searchSafeRange p l h
  | l == h = l
  | p m = searchSafeRange p l m
  | otherwise = searchSafeRange p (m+1) h
  -- Stay within @min 0 l .. max 0 h@ to avoid overflow.
  where m = l `div` 2 + h `div` 2	-- If l < h, then l <= m < h
