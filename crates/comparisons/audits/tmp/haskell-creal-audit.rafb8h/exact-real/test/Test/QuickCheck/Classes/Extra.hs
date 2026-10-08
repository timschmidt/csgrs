{-# LANGUAGE ScopedTypeVariables #-}

-- | Add a bunch of checkers for testing properties of different algebraic
-- structures and relations
module Test.QuickCheck.Classes.Extra
  ( module Test.QuickCheck.Classes
  -- | Algebraic structures
  , group
  , abelian
  , ring
  , commutativeRing
  , field

  -- | Relations
  , complement
  , strictTotalOrd
  ) where

import Data.Group (invert, Group, Abelian)
import Data.Monoid (Sum(..), Product)
import Test.QuickCheck.Extra (Arbitrary, (<=>), (==>))
import Test.QuickCheck.Modifiers (NonZero)
import Test.QuickCheck.Checkers (commutes, transitive, EqProp, (=-=), BinRel)
import Test.QuickCheck.Classes
import Test.Tasty.Extra (testGroup, TestTree, testTreeFromBatch, testTreeFromNamedBatch)
import Test.Tasty.QuickCheck (testProperty, Property, Gen, property, forAll)

distributesL :: EqProp a => (a -> a -> a) -> (a -> a -> a) -> a -> a -> a -> Property
distributesL (*:) (+:) a b c = a *: (b +: c) =-= (a *: b) +: (a *: c)

distributesR :: EqProp a => (a -> a -> a) -> (a -> a -> a) -> a -> a -> a -> Property
distributesR (*:) = distributesL (flip (*:))

distributes :: (Arbitrary a, EqProp a, Show a) => String -> (a -> a -> a) -> (a -> a -> a) -> TestTree
distributes s (*:) (+:) = testGroup s ts
  where ts = [testProperty "left distributes" (distributesL (*:) (+:)),
              testProperty "right distributes" (distributesR (*:) (+:))]

group :: forall a. (Arbitrary a, EqProp a, Group a, Show a) => String -> a -> TestTree
group s _ = testGroup s ts
  where
    ts = [ testTreeFromBatch (monoid (undefined :: a))
         , testProperty "left inverse element" (\(x :: a) -> x <> invert x =-= mempty)
         , testProperty "right inverse element" (\(x :: a) -> invert x <> x =-= mempty)
         ]

abelian :: forall a. (Arbitrary a, EqProp a, Abelian a, Show a) => String -> a -> TestTree
abelian s _ = testGroup s ts
  where
    ts = [ group "group" (undefined :: a)
         , testProperty "commutative" (commutes ((<>) :: a -> a -> a))
         ]

ring :: forall a. (Arbitrary a, EqProp a, Num a, Show a) => String -> a -> TestTree
ring s _ = testGroup s ts
  where
    ts = [ abelian "abelian under Sum" (undefined :: Sum a)
         , testTreeFromNamedBatch "monoid under product" (monoid (undefined :: Product a))
         , distributes "* distributes over +" (*) ((+) :: a -> a -> a)
         ]

commutativeRing :: forall a. (Arbitrary a, EqProp a, Num a, Show a) => String -> a -> TestTree
commutativeRing s _ = testGroup s ts
  where ts = [ring "ring" (undefined :: a),
             testProperty "* commutes" (commutes ((*) :: a -> a -> a))]

-- TODO: Reduce the Ord constraint to an Eq constraint on the new quickcheck
-- release
field :: forall a. (Arbitrary a, EqProp a, Fractional a, Show a, Ord a) => String -> a -> TestTree
field s _ = testGroup s ts
  where ts = [abelian "Abelian under Sum" (undefined :: Sum a),
              abelian "Abelian under Product NonZero" (undefined :: Product (NonZero a)),
              distributes "* distributes over +" (*) ((+) :: a -> a -> a)]

complement :: forall a. (Arbitrary a, EqProp a, Show a) =>
              String -> (a -> Gen a) -> BinRel a -> BinRel a -> TestTree
complement s gen r1 r2 = testGroup s ts
  where ts = [testProperty "strictOrd"
              (property $ \ a ->
               forAll (gen a) $ \ b ->
               a `r1` b <=> not (a `r2` b))
             ]

strictTotalOrd
  :: forall a
   . (Arbitrary a, EqProp a, Eq a, Show a)
  => String
  -> (a -> Gen a)
  -> BinRel a
  -> TestTree
strictTotalOrd s gen r = testGroup s ts
 where
  ts =
    [ testProperty "irreflexive" (property $ \a -> not (a `r` a))
    , testProperty "transitive" $ transitive r gen
    , testProperty
      "connected"
      ( property
      $ \a -> forAll (gen a) $ \b -> (a /= b) ==> (a `r` b) || (b `r` a)
      )
    ]
