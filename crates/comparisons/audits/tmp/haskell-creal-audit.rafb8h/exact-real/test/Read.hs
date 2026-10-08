{-# LANGUAGE ScopedTypeVariables #-}

module Read
  ( read'
  ) where

import Test.QuickCheck.Checkers (EqProp, inverseL)
import Test.Tasty (testGroup, TestTree)
import Test.Tasty.QuickCheck (testProperty, Arbitrary)

read' :: forall a. (Arbitrary a, EqProp a, Show a, Read a) => a -> TestTree
read' _ = testGroup "Test Read instance" ts
  where ts = [ testProperty "read show left inverse"
                            (inverseL read (show :: a -> String))
             ]
