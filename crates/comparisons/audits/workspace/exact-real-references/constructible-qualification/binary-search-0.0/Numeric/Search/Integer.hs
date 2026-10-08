-----------------------------------------------------------------------------
-- |
-- Module      :  Numeric.Search.Integer
-- Copyright   :  (c) Ross Paterson 2008
-- License     :  BSD-style
-- Maintainer  :  ross@soi.city.ac.uk
-- Stability   :  experimental
-- Portability :  portable
--
-- Searching unbounded intervals of integers for the boundary of an
-- upward-closed set, using a combination of exponential and binary
-- search.
--
-----------------------------------------------------------------------------

module Numeric.Search.Integer (
	-- * One-dimensional searches
	search, searchFrom, searchTo,
	-- * Two-dimensional searches
	search2) where

import Data.Maybe (fromMaybe)

-- | /O(log(abs n))/.
-- Search the integers.
--
-- If @p@ is an upward-closed predicate, @search p@ returns the least
-- @n@ satisfying @p@.  If no such @n@ exists, either because no integer
-- satisfies @p@ or all do, @search p@ does not terminate.
--
-- For example, the following function computes discrete logarithms (base 2):
--
-- > discreteLog :: Integer -> Integer
-- > discreteLog n = search (\ k -> 2^k <= n)
--
search :: (Integer -> Bool) -> Integer
search p = fromMaybe (searchFrom p 1) (searchTo p 0)

-- | /O(log(n-l))/.
-- Search the integers from a given lower bound.
--
-- If @p@ is an upward-closed predicate,
-- @searchFrom p l = 'search' (\\ i -> i >= l && p i)@.
-- If no such @n@ exists (because no integer satisfies @p@),
-- @searchFrom p@ does not terminate.
searchFrom :: (Integer -> Bool) -> Integer -> Integer
searchFrom p = search_from 1
  where search_from step l
	  | p l' = searchIntegerRange p l (l'-1)
	  | otherwise = search_from (2*step) (l'+1)
	  where l' = l + step

-- | /O(log(h-n))/.
-- Search the integers up to a given upper bound.
--
-- If @p@ is an upward-closed predicate, @searchTo p h == 'Just' n@
-- if and only if @n@ is the least number @n <= h@ satisfying @p@.
searchTo :: (Integer -> Bool) -> Integer -> Maybe Integer
searchTo p h0
  | p h0 = Just (search_to 1 h0)
  | otherwise = Nothing
  where search_to step h		-- @step >= 1 && p h@
	  | p h' = search_to (2*step) h'
	  | otherwise = searchSafeRange p (h'+1) h
	  where h' = h - step

-- | /O(m log(n\/m))/.
-- Two-dimensional search, using an algorithm due described in
--
-- * Richard Bird, /Saddleback search: a lesson in algorithm design/,
--   in /Mathematics of Program Construction/ 2006,
--   Springer LNCS vol. 4014, pp82-89.
--
-- If @p@ is closed upwards in each argument on non-negative integers,
-- @search2 p@ returns the minimal non-negative pairs satisfying @p@,
-- listed in order of increasing x-coordinate.
--
-- /m/ and /n/ refer to the smaller and larger dimensions of the
-- rectangle containing the boundary.
--
-- For example,
--
-- > search2 (\ x y -> x^2 + y^2 >= 25)  ==  [(0,5),(3,4),(4,3),(5,0)]
--
search2 :: (Integer -> Integer -> Bool) -> [(Integer,Integer)]
search2 p = search2Rect p 0 0 hx hy []
  where	hx = searchFrom (\ x -> p x 0) 0
	hy = searchFrom (\ y -> p 0 y) 0

search2Rect :: (Integer -> Integer -> Bool) ->
	Integer -> Integer -> Integer -> Integer ->
	[(Integer,Integer)] -> [(Integer,Integer)]
search2Rect p lx ly hx hy
  | lx > hx || ly > hy = id
  | lx == hx && ly == hy = if p lx ly then ((lx, ly) :) else id
  | hx-lx > hy-ly =
	let	mx = (lx+hx) `div` 2
		my = searchIntegerRange (\ y -> p mx y) ly hy
	in search2Rect p lx my mx hy . search2Rect p (mx+1) ly hx (my-1)
  | otherwise =
	let	mx = searchIntegerRange (\ x -> p x my) lx hx
		my = (ly+hy) `div` 2
	in search2Rect p lx (my+1) (mx-1) hy . search2Rect p mx ly hx my

-- | Search a bounded interval of integers.
--
-- If @p@ is an upward-closed predicate,
--
-- > searchIntegerRange p l h = 'search' (\ i -> i >= l && p i || i > h)
--
-- Cost: /O(log(h-l))/ calls to @p@.
searchIntegerRange :: (Integer -> Bool) -> Integer -> Integer -> Integer
searchIntegerRange p l h
  | h < l = h+1
  | p m = searchIntegerRange p l (m-1)
  | otherwise = searchIntegerRange p (m+1) h
  where m = (l+h) `div` 2

-- | Like 'search', but assuming @l <= h && p h@.
searchSafeRange :: (Integer -> Bool) -> Integer -> Integer -> Integer
searchSafeRange p l h
  | l == h = l
  | p m = searchSafeRange p l m
  | otherwise = searchSafeRange p (m+1) h
  where m = (l + h) `div` 2	-- If l < h, then l <= m < h
