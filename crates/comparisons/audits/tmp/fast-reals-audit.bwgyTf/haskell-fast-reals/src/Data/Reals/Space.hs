{-# LANGUAGE MultiParamTypeClasses, FunctionalDependencies, TypeSynonymInstances, FlexibleInstances #-}

-- | Basic definitions od spaces and their properties

module Data.Reals.Space (
             sor, sand, force,
             Hausdorff  (..),
             Discrete (..),
             Compact (..),
             Overt (..),
             LinearOrder (..)
) where

import Data.Reals.Staged

-- | The Sierpinski space @Sigma@ is represented by staged booleans
type Sigma = StagedWithFun Bool

-- | Disjunction for Sierpinski space
sor :: Sigma -> Sigma -> Sigma
sor = lift2 (\s p q -> p || q)

-- | Conjunction for Sierpinski space
sand :: Sigma -> Sigma -> Sigma
sand = lift2 (\s p q -> p && q)

-- | Force a value in the Sierpinski space into Booleans. This may diverge as bottom cannot be
-- reliably detected.
force :: Sigma -> Bool
force p = loop 0
          where loop k
                   | approximate p (prec RoundDown k) = True -- lower approximation is True, the value is top
                   | not (approximate p (prec RoundUp k)) = False -- upper approximation is False, the value is bottom
                   | otherwise = loop (k+1)

-- | The Show instance may cause divergence because 'force' could diverge. An alternative
-- implementation would give up after a while, and the user would have to use 'force' explicitly to
-- get the exact results (or divergence).

instance Show Sigma where
    show p = show $ force p

-- | A space is Hausdorff if inequality, here called 'apart', is an open relation.
class Hausdorff t where
  apart :: t -> t -> Sigma

-- | A space is Discrete if equality, here called 'equal', is an open relation.
class Discrete t where
  equal :: t -> t -> Sigma

-- | Suppose the type 's' represents a family of subspaces of 't'. The typical example is
-- that 't' is the type of reals and 's' is the type of closed intervals. Then the subspaces
-- represented by 's' are compact subspaces of 't' if the universal quantifier is a continuous
-- map from @t -> 'Sigma'@ to 'Sigma'.
class Compact s t | s -> t where
  forall :: s -> (t -> Sigma) -> Sigma

-- | Suppose the type 's' represents a family of subspaces of 't'. The typical example is
-- that 't' is the type of reals and 's' is the type of closed intervals. Then the subspaces
-- represented by 's' are overt subspaces of 't' if the existential quantifier is a continuous
-- map from @t -> 'Sigma'@ to 'Sigma'.
class Overt s t | s -> t where
  exists :: s -> (t -> Sigma) -> Sigma

-- | The real numbers are strictly linearly ordered by open relation <, we define
-- a class that expresses that fact.
class LinearOrder t where
  less :: t -> t -> Sigma
  more :: t -> t -> Sigma

  -- default implemetnation of 'more' in terms of 'less'
  more x y = less y x
