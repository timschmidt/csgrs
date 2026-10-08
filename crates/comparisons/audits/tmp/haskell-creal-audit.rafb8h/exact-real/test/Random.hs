{-# LANGUAGE ScopedTypeVariables #-}

module Random
  ( random
  ) where

import Test.Tasty (testGroup, TestTree)
import Test.Tasty.QuickCheck (testProperty, Arbitrary, (==>))
import System.Random (Random, randoms, randomRs, mkStdGen)

random :: forall a. (Arbitrary a, Show a, Ord a, Fractional a, Random a) => a -> TestTree
random _ = testGroup "Test Random instance" ts
  where ts = [ testProperty "randomR range"
                 (\s l u -> let rs = take 100 (randomRs (l :: a, u) (mkStdGen s))
                            in l <= u ==> (all (>= l) rs && all (<= u) rs))
             , testProperty "randomR zero bounds"
                 (\s l -> let rs = take 100 (randomRs (l :: a, l) (mkStdGen s))
                          in all (== l) rs)
             , testProperty "random range"
                 (\s -> let rs = take 100 (randoms (mkStdGen s)) :: [a]
                        in all (>= 0) rs && all (< 1) rs)
             ]
