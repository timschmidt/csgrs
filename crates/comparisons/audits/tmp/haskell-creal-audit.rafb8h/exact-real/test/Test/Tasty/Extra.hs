module Test.Tasty.Extra
  ( module Test.Tasty
  , testTreeFromBatch
  , testTreeFromNamedBatch) where

import Test.Tasty (testGroup, TestTree)
import Test.Tasty.QuickCheck (testProperty)
import Test.QuickCheck.Checkers (TestBatch)

testTreeFromBatch :: TestBatch -> TestTree
testTreeFromBatch (n, ts) = testTreeFromNamedBatch n (n, ts)

testTreeFromNamedBatch :: String -> TestBatch -> TestTree
testTreeFromNamedBatch n (_, ts) = testGroup n ts'
  where ts' = uncurry testProperty <$> ts

