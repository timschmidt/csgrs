module Data.Number.IReal.Powers where

-- | Common functions collected to allow for instances which
-- handle dependency problems for intervals, and for automatic
-- differentiation.
class Num a => Powers a where
   -- squaring function; in @sq x@, there is only one occurrence of @x@ (as opposed to @x * x@)
   sq :: a -> a
   -- power function; @pow n x@ computes @x^n@, but can be implemented for 'IReal' with correct
   -- treatment of dependency.
   pow :: a -> Int -> a
   sq = flip pow 2
   pow x n = x ^ n

instance Powers Double  

instance Powers Integer 
