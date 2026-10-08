-- |
-- Module:      Math.NumberTheory.TestUtils
-- Copyright:   (c) 2016 Andrew Lelechenko
-- Licence:     MIT
-- Maintainer:  Andrew Lelechenko <andrew.lelechenko@gmail.com>
--
-- Utils to test Math.NumberTheory
--

{-# LANGUAGE ConstraintKinds            #-}
{-# LANGUAGE CPP                        #-}
{-# LANGUAGE DataKinds                  #-}
{-# LANGUAGE FlexibleContexts           #-}
{-# LANGUAGE FlexibleInstances          #-}
{-# LANGUAGE KindSignatures             #-}
{-# LANGUAGE MultiParamTypeClasses      #-}
{-# LANGUAGE RankNTypes                 #-}
{-# LANGUAGE ScopedTypeVariables        #-}
{-# LANGUAGE TypeFamilies               #-}
{-# LANGUAGE TypeOperators              #-}
{-# LANGUAGE UndecidableInstances       #-}
{-# LANGUAGE UndecidableSuperClasses    #-}

{-# OPTIONS_GHC -fconstraint-solver-iterations=0 #-}

{-# OPTIONS_GHC -fno-warn-orphans #-}

module Math.NumberTheory.TestUtils
  ( Positive(..)
  , NonNegative(..)
  , AnySign(..)
  , Power(..)
  , Huge(..)
  , testIntegralProperty
  , testIntegral2Property
  , testSmallAndQuick
  ) where

import Test.SmallCheck.Series (Positive(..), NonNegative(..), Serial(..))
import Test.Tasty
import Test.Tasty.SmallCheck as SC
import Test.Tasty.QuickCheck as QC hiding (Positive(..), NonNegative(..), generate)

import Data.Bits
import Data.Int
import Data.Kind
import Data.Word
import Numeric.Natural

import Math.NumberTheory.TestUtils.Wrappers

#if !MIN_VERSION_QuickCheck(2,17,0)
instance Arbitrary Natural where
  arbitrary = fromInteger <$> (arbitrary `suchThat` (>= 0))
  shrink = map fromInteger . filter (>= 0) . shrink . toInteger
#endif

#if !MIN_VERSION_smallcheck(1,2,0)
instance Functor NonNegative where
  fmap f (NonNegative x) = NonNegative (f x)

instance (Num a, Bounded a) => Bounded (NonNegative a) where
  minBound = NonNegative 0
  maxBound = NonNegative (maxBound :: a)

instance Functor Positive where
  fmap f (Positive x) = Positive (f x)

instance (Num a, Bounded a) => Bounded (Positive a) where
  minBound = Positive 1
  maxBound = Positive (maxBound :: a)
#endif

-------------------------------------------------------------------------------

-- https://www.cs.ox.ac.uk/projects/utgp/school/andres.pdf, p. 21
-- :k Compose = (k1 -> Constraint) -> (k2 -> k1) -> (k2 -> Constraint)
class    (f (g x)) => (f `Compose` g) x
instance (f (g x)) => (f `Compose` g) x

type family ConcatMap (w :: Type -> Constraint) (cs :: [Type]) :: Constraint
  where
    ConcatMap w '[] = ()
    ConcatMap w (c ': cs) = (w c, ConcatMap w cs)

type family Matrix (as :: [Type -> Constraint]) (w :: Type -> Type) (bs :: [Type]) :: Constraint
  where
    Matrix '[] w bs = ()
    Matrix (a ': as) w bs = (ConcatMap (a `Compose` w) bs, Matrix as w bs)

type TestableIntegral wrapper =
  ( Matrix '[Arbitrary, Show, Serial IO] wrapper '[Int, Int8, Int16, Int32, Int64, Word, Word8, Word16, Word32, Word64, Integer, Natural]
  , Matrix '[Arbitrary, Show] wrapper '[Large Int, Large Word, Huge Integer, Huge Natural]
  , Matrix '[Bounded, Integral] wrapper '[Int, Word]
  , Num (wrapper Integer)
  , Num (wrapper Natural)
  , Functor wrapper
  )

testIntegralProperty
  :: forall wrapper bool. (TestableIntegral wrapper, SC.Testable IO bool, QC.Testable bool)
  => String -> (forall a. (Integral a, Bits a, Show a) => wrapper a -> bool) -> TestTree
testIntegralProperty name f = testGroup name
  [ SC.testProperty "smallcheck Int"     (f :: wrapper Int     -> bool)
  , SC.testProperty "smallcheck Word"    (f :: wrapper Word    -> bool)
  , SC.testProperty "smallcheck Integer" (f :: wrapper Integer -> bool)
  , SC.testProperty "smallcheck Natural" (f :: wrapper Natural -> bool)
  , QC.testProperty "quickcheck Int"     (f :: wrapper Int     -> bool)
  , QC.testProperty "quickcheck Int8"    (f :: wrapper Int8    -> bool)
  , QC.testProperty "quickcheck Int16"   (f :: wrapper Int16   -> bool)
  , QC.testProperty "quickcheck Int32"   (f :: wrapper Int32   -> bool)
  , QC.testProperty "quickcheck Int64"   (f :: wrapper Int64   -> bool)
  , QC.testProperty "quickcheck Word"    (f :: wrapper Word    -> bool)
  , QC.testProperty "quickcheck Word8"   (f :: wrapper Word8   -> bool)
  , QC.testProperty "quickcheck Word16"  (f :: wrapper Word16  -> bool)
  , QC.testProperty "quickcheck Word32"  (f :: wrapper Word32  -> bool)
  , QC.testProperty "quickcheck Word64"  (f :: wrapper Word64  -> bool)
  , QC.testProperty "quickcheck Integer" (f :: wrapper Integer -> bool)
  , QC.testProperty "quickcheck Natural" (f :: wrapper Natural -> bool)
  , QC.testProperty "quickcheck Large Int"     ((f :: wrapper Int     -> bool) . getLarge)
  , QC.testProperty "quickcheck Large Word"    ((f :: wrapper Word    -> bool) . getLarge)
  , QC.testProperty "quickcheck Huge  Integer" ((f :: wrapper Integer -> bool) . getHuge)
  , QC.testProperty "quickcheck Huge  Natural" ((f :: wrapper Natural -> bool) . getHuge)
  ]

testIntegral2Property
  :: forall wrapper1 wrapper2 bool. (TestableIntegral wrapper1, TestableIntegral wrapper2, SC.Testable IO bool, QC.Testable bool)
  => String -> (forall a1 a2. (Integral a1, Integral a2, Bits a1, Bits a2, Show a1, Show a2) => wrapper1 a1 -> wrapper2 a2 -> bool) -> TestTree
testIntegral2Property name f = testGroup name
  [ SC.testProperty "smallcheck Int Int"         (f :: wrapper1 Int     -> wrapper2 Int     -> bool)
  , SC.testProperty "smallcheck Int Word"        (f :: wrapper1 Int     -> wrapper2 Word    -> bool)
  , SC.testProperty "smallcheck Int Integer"     (f :: wrapper1 Int     -> wrapper2 Integer -> bool)
  , SC.testProperty "smallcheck Int Natural"     (f :: wrapper1 Int     -> wrapper2 Natural -> bool)
  , SC.testProperty "smallcheck Word Int"        (f :: wrapper1 Word    -> wrapper2 Int     -> bool)
  , SC.testProperty "smallcheck Word Word"       (f :: wrapper1 Word    -> wrapper2 Word    -> bool)
  , SC.testProperty "smallcheck Word Integer"    (f :: wrapper1 Word    -> wrapper2 Integer -> bool)
  , SC.testProperty "smallcheck Word Natural"    (f :: wrapper1 Word    -> wrapper2 Natural -> bool)
  , SC.testProperty "smallcheck Integer Int"     (f :: wrapper1 Integer -> wrapper2 Int     -> bool)
  , SC.testProperty "smallcheck Integer Word"    (f :: wrapper1 Integer -> wrapper2 Word    -> bool)
  , SC.testProperty "smallcheck Integer Integer" (f :: wrapper1 Integer -> wrapper2 Integer -> bool)
  , SC.testProperty "smallcheck Integer Natural" (f :: wrapper1 Integer -> wrapper2 Natural -> bool)
  , SC.testProperty "smallcheck Natural Int"     (f :: wrapper1 Natural -> wrapper2 Int     -> bool)
  , SC.testProperty "smallcheck Natural Word"    (f :: wrapper1 Natural -> wrapper2 Word    -> bool)
  , SC.testProperty "smallcheck Natural Integer" (f :: wrapper1 Natural -> wrapper2 Integer -> bool)
  , SC.testProperty "smallcheck Natural Natural" (f :: wrapper1 Natural -> wrapper2 Natural -> bool)

  , QC.testProperty "quickcheck Int Int"         (f :: wrapper1 Int     -> wrapper2 Int     -> bool)
  , QC.testProperty "quickcheck Int Int8"        (f :: wrapper1 Int     -> wrapper2 Int8    -> bool)
  , QC.testProperty "quickcheck Int Word"        (f :: wrapper1 Int     -> wrapper2 Word    -> bool)
  , QC.testProperty "quickcheck Int Word8"       (f :: wrapper1 Int     -> wrapper2 Word8   -> bool)
  , QC.testProperty "quickcheck Int Integer"     (f :: wrapper1 Int     -> wrapper2 Integer -> bool)
  , QC.testProperty "quickcheck Int Natural"     (f :: wrapper1 Int     -> wrapper2 Natural -> bool)
  , QC.testProperty "quickcheck Int8 Int"        (f :: wrapper1 Int8    -> wrapper2 Int     -> bool)
  , QC.testProperty "quickcheck Int8 Int8"       (f :: wrapper1 Int8    -> wrapper2 Int8    -> bool)
  , QC.testProperty "quickcheck Int8 Word"       (f :: wrapper1 Int8    -> wrapper2 Word    -> bool)
  , QC.testProperty "quickcheck Int8 Word8"      (f :: wrapper1 Int8    -> wrapper2 Word8   -> bool)
  , QC.testProperty "quickcheck Int8 Integer"    (f :: wrapper1 Int8    -> wrapper2 Integer -> bool)
  , QC.testProperty "quickcheck Int8 Natural"    (f :: wrapper1 Int8    -> wrapper2 Natural -> bool)
  , QC.testProperty "quickcheck Word Int"        (f :: wrapper1 Word    -> wrapper2 Int     -> bool)
  , QC.testProperty "quickcheck Word Int8"       (f :: wrapper1 Word    -> wrapper2 Int8    -> bool)
  , QC.testProperty "quickcheck Word Word"       (f :: wrapper1 Word    -> wrapper2 Word    -> bool)
  , QC.testProperty "quickcheck Word Word8"      (f :: wrapper1 Word    -> wrapper2 Word8   -> bool)
  , QC.testProperty "quickcheck Word Integer"    (f :: wrapper1 Word    -> wrapper2 Integer -> bool)
  , QC.testProperty "quickcheck Word Natural"    (f :: wrapper1 Word    -> wrapper2 Natural -> bool)
  , QC.testProperty "quickcheck Word8 Int"       (f :: wrapper1 Word8   -> wrapper2 Int     -> bool)
  , QC.testProperty "quickcheck Word8 Int8"      (f :: wrapper1 Word8   -> wrapper2 Int8    -> bool)
  , QC.testProperty "quickcheck Word8 Word"      (f :: wrapper1 Word8   -> wrapper2 Word    -> bool)
  , QC.testProperty "quickcheck Word8 Word8"     (f :: wrapper1 Word8   -> wrapper2 Word8   -> bool)
  , QC.testProperty "quickcheck Word8 Integer"   (f :: wrapper1 Word8   -> wrapper2 Integer -> bool)
  , QC.testProperty "quickcheck Word8 Natural"   (f :: wrapper1 Word8   -> wrapper2 Natural -> bool)
  , QC.testProperty "quickcheck Integer Int"     (f :: wrapper1 Integer -> wrapper2 Int     -> bool)
  , QC.testProperty "quickcheck Integer Int8"    (f :: wrapper1 Integer -> wrapper2 Int8    -> bool)
  , QC.testProperty "quickcheck Integer Word"    (f :: wrapper1 Integer -> wrapper2 Word    -> bool)
  , QC.testProperty "quickcheck Integer Word8"   (f :: wrapper1 Integer -> wrapper2 Word8   -> bool)
  , QC.testProperty "quickcheck Integer Integer" (f :: wrapper1 Integer -> wrapper2 Integer -> bool)
  , QC.testProperty "quickcheck Integer Natural" (f :: wrapper1 Integer -> wrapper2 Natural -> bool)
  , QC.testProperty "quickcheck Natural Int"     (f :: wrapper1 Natural -> wrapper2 Int     -> bool)
  , QC.testProperty "quickcheck Natural Int8"    (f :: wrapper1 Natural -> wrapper2 Int8    -> bool)
  , QC.testProperty "quickcheck Natural Word"    (f :: wrapper1 Natural -> wrapper2 Word    -> bool)
  , QC.testProperty "quickcheck Natural Word8"   (f :: wrapper1 Natural -> wrapper2 Word8   -> bool)
  , QC.testProperty "quickcheck Natural Integer" (f :: wrapper1 Natural -> wrapper2 Integer -> bool)
  , QC.testProperty "quickcheck Natural Natural" (f :: wrapper1 Natural -> wrapper2 Natural -> bool)

  , QC.testProperty "quickcheck Large Int Int"         ((f :: wrapper1 Int     -> wrapper2 Int     -> bool) . fmap getLarge)
  , QC.testProperty "quickcheck Large Int Word"        ((f :: wrapper1 Int     -> wrapper2 Word    -> bool) . fmap getLarge)
  , QC.testProperty "quickcheck Large Int Integer"     ((f :: wrapper1 Int     -> wrapper2 Integer -> bool) . fmap getLarge)
  , QC.testProperty "quickcheck Large Int Natural"     ((f :: wrapper1 Int     -> wrapper2 Natural -> bool) . fmap getLarge)
  , QC.testProperty "quickcheck Large Word Int"        ((f :: wrapper1 Word    -> wrapper2 Int     -> bool) . fmap getLarge)
  , QC.testProperty "quickcheck Large Word Word"       ((f :: wrapper1 Word    -> wrapper2 Word    -> bool) . fmap getLarge)
  , QC.testProperty "quickcheck Large Word Integer"    ((f :: wrapper1 Word    -> wrapper2 Integer -> bool) . fmap getLarge)
  , QC.testProperty "quickcheck Large Word Natural"    ((f :: wrapper1 Word    -> wrapper2 Natural -> bool) . fmap getLarge)
  , QC.testProperty "quickcheck Huge  Integer Int"     ((f :: wrapper1 Integer -> wrapper2 Int     -> bool) . fmap getHuge)
  , QC.testProperty "quickcheck Huge  Integer Word"    ((f :: wrapper1 Integer -> wrapper2 Word    -> bool) . fmap getHuge)
  , QC.testProperty "quickcheck Huge  Integer Integer" ((f :: wrapper1 Integer -> wrapper2 Integer -> bool) . fmap getHuge)
  , QC.testProperty "quickcheck Huge  Integer Natural" ((f :: wrapper1 Integer -> wrapper2 Natural -> bool) . fmap getHuge)
  , QC.testProperty "quickcheck Huge  Natural Int"     ((f :: wrapper1 Natural -> wrapper2 Int     -> bool) . fmap getHuge)
  , QC.testProperty "quickcheck Huge  Natural Word"    ((f :: wrapper1 Natural -> wrapper2 Word    -> bool) . fmap getHuge)
  , QC.testProperty "quickcheck Huge  Natural Integer" ((f :: wrapper1 Natural -> wrapper2 Integer -> bool) . fmap getHuge)
  , QC.testProperty "quickcheck Huge  Natural Natural" ((f :: wrapper1 Natural -> wrapper2 Natural -> bool) . fmap getHuge)
  ]

testSmallAndQuick
  :: (SC.Testable IO a, QC.Testable a)
  => String
  -> a
  -> TestTree
testSmallAndQuick name f = testGroup name
  [ SC.testProperty "smallcheck" f
  , QC.testProperty "quickcheck" f
  ]
