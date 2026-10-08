{- |The Extended module allows real-valued numeric data types to be extended by
   positive and negative infinity.
-}
{-# LANGUAGE DeriveFunctor #-}
module Data.CDAR.Extended (Extended(..)) where

import Control.Monad

-- |Extended numbers are either finite numbers or one of the two infinities.
data Extended a = PosInf | NegInf | Finite a deriving (Eq,Read,Show,Functor)

instance Applicative Extended where
    pure = Finite
    (Finite f) <*> (Finite x) = Finite $ f x
    (Finite _) <*> PosInf = PosInf
    (Finite _) <*> NegInf = NegInf
    PosInf <*> _ = PosInf
    NegInf <*> _ = NegInf

instance Monad Extended where
    return = pure
    (Finite x) >>= f = f x
    PosInf >>= _ = PosInf
    NegInf >>= _ = NegInf

instance Ord a => Ord (Extended a) where
    compare PosInf PosInf = EQ
    compare NegInf NegInf = EQ
    compare _ PosInf = LT
    compare NegInf _ = LT
    compare PosInf _ = GT
    compare _ NegInf = GT
    compare (Finite a) (Finite b) = compare a b

instance (Ord a, Num a) => Num (Extended a) where
    -- PosInf + NegInf should be undefined, but here it is the first argument
    (+) = liftM2 (+)
    -- 0 * ???Inf should be undefined, but here it is PosInf
    a * PosInf = if a < 0 then NegInf else PosInf
    PosInf * a = if a < 0 then NegInf else PosInf
    a * NegInf = if a < 0 then PosInf else NegInf
    NegInf * a = if a < 0 then PosInf else NegInf
    a * b = (*) <$> a <*> b
    negate = fmap negate
    abs = fmap abs
    signum PosInf = Finite 1
    signum NegInf = Finite (-1)
    signum a = signum <$> a
    fromInteger i = Finite $ fromInteger i

instance Real a => Real (Extended a) where
    toRational (Finite x) = toRational x
    toRational _ = undefined
