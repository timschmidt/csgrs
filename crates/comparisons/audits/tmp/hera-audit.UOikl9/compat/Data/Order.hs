module Data.Order where

-- | Partial ordering.
data POrdering = Less | Greater | Incomparable deriving (Eq, Show, Ord)

-- | Partial booleans
data PBool = PTrue -- ^ equivalent to True
             | PFalse -- ^ equivalent to False
             | Indeterminate -- ^ neither True nor False.
               deriving(Eq, Show, Ord)
