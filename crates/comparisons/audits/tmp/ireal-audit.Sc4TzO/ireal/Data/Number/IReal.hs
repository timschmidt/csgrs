-- | This module provides the type 'IReal', the values of which are real numbers and intervals, with 
-- potentially unbounded precision arithmetic and elementary functions.
-- 
-- 'IReal' is an instance of the standard numeric classes, so we can interact in ghci as follows:
--
-- >>> exp 0.5 + pi * sqrt ( 2 + sin 1) ? 50
-- 6.94439823755032768935865535478209938180612180886848
--
-- The right operand to the operator '?' indicates the number of decimals to display in the result.
-- Using '?' is the default way to print values; the 'Show' instance is not recommended to use, since
-- the redundant rounding policy implies that we cannot guarantee to generate equal string representations
-- for equal values.
--
-- For simple expressions like the above,
-- one can request a thousand decimals with more or less instantaneous result; also ten thousand decimals is easy 
-- (less than a second on a typical laptop).
--
-- Here is an example with interval arguments:
-- 
-- >>> exp (0.5 +- 0.001) + pi * sqrt ( 2 + sin (1 +- 0.003)) ? 30
-- 6.94[| 1236147625 .. 7554488225 |]
-- 
-- The result is displayed in a non-standard but hopefully easily interpreted notation. 
-- We will not get the requested 30 decimals here; interval upper and lower bounds are displayed
-- with at most 10 distinguishing digits. The result of an interval computation is conservative; 
-- it includes all possible values of the expression for inputs in the given intervals. As always in 
-- interval arithmetic, results may be unduly pessimistic because of the dependency problem.
-- 
-- As a third example, consider 
-- 
-- >>> log (2 +- 1e-50) ? 30
-- 0.693147180559945309417232121458
-- 
-- The result is obviously an interval, not a number, but displayed with 30 decimals it looks just like a real number. Conversely,
-- a real number is an infinite object and we can only ever compute an approximation to it. So a finitely printed 'IReal' value
-- can always be thought of as denoting an interval; there is an error margin of one unit in the last displayed digit.
-- These remarks give a first intuition for why it may be fruitful to merge real numbers and intervals into one type. 
--
-- 'IReal' is also an instance of 'Eq' and 'Ord'; these are, however, non-total for computability reasons;
-- evaluation of e.g.  @sin pi == 0@ at type 'IReal' will not terminate.
--
-- At the github site <https://github.com/sydow/ireal.git> one can find a QuickCheck testsuite (in directory tests), a paper with documentation (in directory doc) and a number of 
-- small applications (in directory applications).
module Data.Number.IReal (
  -- * The type of real numbers and intervals
  IReal,
  toDouble,
  -- * Printing 'IReal's 
  (?),
  (??),
  showIReal,
  -- * Total comparison operators
  (<!),
  (>!),
  (=?=),
  atDecimals,
  -- * Intervals
  -- ** Constructing interval values
  (+-),
  (-+-),
  hull,
  intersection,
  -- ** Selectors
  lower, 
  upper,
  mid,
  rad,
  containedIn,
  -- * Balanced folds
  foldb,
  foldb1,
  bsum,
  foldb',
  isumN',
  isum',
  -- * Automatic Differentiation
  Dif(..),
  deriv,
  derivs,
  con,
  var,
  unDif,
  val,
  -- * QuickCheck support
  -- ** Generators
  uniformNum,
  uniformIval,
  exprGen,
  -- ** Properties 
  propIsRealNum,
  propIsRealIval,
  -- * Auxiliary functions and type classes
  force,
  dec2bits,
  lg2,
  Powers(..),
  Scalable(..),
  VarPrec(..)
  ) where 

import Data.Number.IReal.IReal
import Data.Number.IReal.Powers
import Data.Number.IReal.IntegerInterval
import Data.Number.IReal.Scalable
import Data.Number.IReal.UnsafeMemo
import Data.Number.IReal.IRealOperations
import Data.Number.IReal.Generators
import Data.Number.IReal.Auxiliary
import Data.Number.IReal.FoldB
import Data.Number.IReal.FAD


