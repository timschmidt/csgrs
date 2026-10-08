{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE RankNTypes #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE ConstrainedClassMethods #-}

-- | The Converge type class.
module Data.CReal.Converge
  ( Converge(..)
  ) where

import Control.Arrow ((&&&))
import Data.Coerce (coerce)
import Data.CReal.Internal (CReal(..), atPrecision, crMemoize)
import Data.Function (on)
import Data.Proxy (Proxy)
import GHC.TypeLits (someNatVal, SomeNat(..))

-- $setup
-- >>> :set -XFlexibleContexts
-- >>> import Data.CReal.Internal

-- | If a type is an instance of Converge then it represents a stream of values
-- which are increasingly accurate approximations of a desired value
class Converge a where
  -- | The type of the value the stream converges to.
  type Element a

  -- | 'converge' is a function that returns the value the stream is converging
  -- to. If given a stream which doens't converge to a single value then
  -- 'converge' will not terminate.
  --
  -- If the stream is empty then it should return nothing.
  --
  -- >>> let initialGuess = 1 :: Double
  -- >>> let improve x = (x + 121 / x) / 2
  -- >>> converge (iterate improve initialGuess)
  -- Just 11.0
  --
  -- >>> converge [] :: Maybe [Int]
  -- Nothing
  converge :: a -> Maybe (Element a)

  -- | 'convergeErr' is a function that returns the value the stream is
  -- converging to. It also takes a function err which returns a value which
  -- varies monotonically with the error of the value in the stream. This can
  -- be used to ensure that when 'convergeErr' terminates when given a
  -- non-converging stream or a stream which enters a cycle close to the
  -- solution. See the documentation for the CReal instance for a caveat with
  -- that implementation.
  --
  -- It's often the case that streams generated with approximation
  -- functions such as Newton's method will generate worse approximations for
  -- some number of steps until they find the "zone of convergence". For these
  -- cases it's necessary to drop some values of the stream before handing it
  -- to convergeErr.
  --
  -- For example trying to find the root of the following funciton @f@ with a
  -- poor choice of starting point. Although this doesn't find the root, it
  -- doesn't fail to terminate.
  --
  -- >>> let f  x = x ^ 3 - 2 * x + 2
  -- >>> let f' x = 3 * x ^ 2 - 2
  -- >>> let initialGuess = 0.1 :: Float
  -- >>> let improve x = x - f x / f' x
  -- >>> let err x = abs (f x)
  -- >>> convergeErr err (iterate improve initialGuess)
  -- Just 1.0142132
  convergeErr :: Ord (Element a) => (Element a -> Element a) -> a -> Maybe (Element a)

-- | Every list of equatable values is an instance of 'Converge'. 'converge'
-- returns the first element which is equal to the succeeding element in the
-- list. If the list ends before the sequence converges the last value is
-- returned.
instance {-# OVERLAPPABLE #-} Eq a => Converge [a] where
  type Element [a] = a

  converge = lastMay . takeWhilePairwise (/=)
  {-# INLINE converge #-}

  convergeErr err xs = fmap snd . lastMay . takeWhilePairwise ((>) `on` fst) $ es
    where es = (err &&& id) <$> xs
  {-# INLINE convergeErr #-}

-- | The overlapping instance for @'CReal' n@ has a slightly different
-- behavior. The instance for 'Eq' will cause 'converge' to return a value when
-- the list converges to within 2^-n (due to the 'Eq' instance for @'CReal' n@)
-- despite the precision the value is requested at by the surrounding
-- computation. This instance will return a value approximated to the correct
-- precision.
--
-- It's important to note when the error function reaches zero this function
-- behaves like 'converge' as it's not possible to determine the precision at
-- which the error function should be evaluated at.
--
-- Find where log x = π using Newton's method
--
-- >>> let initialGuess = 1
-- >>> let improve x = x - x * (log x - pi)
-- >>> let Just y = converge (iterate improve initialGuess)
-- >>> showAtPrecision 10 y
-- "23.1406"
-- >>> showAtPrecision 50 y
-- "23.1406926327792686"
instance {-# OVERLAPPING #-} Converge [CReal n] where
  type Element [CReal n] = CReal n

  converge [] = Nothing
  converge xs =
    Just $ crMemoize (\p ->
      case someNatVal (toInteger p) of
           Nothing -> error "Data.CReal.Converge p should be non negative"
           Just (SomeNat (_ :: Proxy p')) ->
             let modifyPrecision = coerce :: [CReal n] -> [CReal p']
             in (last . takeWhilePairwise (/=) . modifyPrecision $ xs) `atPrecision` p)
  {-# INLINE converge #-}

  convergeErr _ [] = Nothing
  convergeErr err xs =
    Just $ crMemoize (\p ->
      case someNatVal (toInteger p) of
           Nothing -> error "Data.CReal.Converge p should be non negative"
           Just (SomeNat (_ :: Proxy p')) ->
             let modifyPrecision = coerce :: [CReal n] -> [CReal p']
                 modifyFunPrecision = coerce :: (CReal n -> CReal n) -> CReal p' -> CReal p'
                 es = (modifyFunPrecision err &&& id) <$> modifyPrecision xs
                 continue (e1, x1) (e2, x2) = if e1 == 0 then x1 /= x2 else e1 > e2
             in (snd . last . takeWhilePairwise continue $ es) `atPrecision` p)
  {-# INLINE convergeErr #-}

takeWhilePairwise :: (a -> a -> Bool) -> [a] -> [a]
takeWhilePairwise p (x1:x2:xs) = if x1 `p` x2
                                   then x1 : takeWhilePairwise p (x2:xs)
                                   else [x1]
takeWhilePairwise _ xs = xs

lastMay :: [a] -> Maybe a
lastMay [] = Nothing
lastMay xs = Just (last xs)


