{-# LANGUAGE MultiParamTypeClasses, FlexibleInstances #-}
{-|
    Module      :  Numeric.AERN.Misc.QuickCheck
    Description :  miscellaneous utilities for QuickCheck  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    miscellaneous utilities for QuickCheck
-}
module Numeric.AERN.Misc.QuickCheck where

import qualified Data.List as List
import Test.QuickCheck
import Test.QuickCheck.Gen (unGen)
import qualified System.Random as R
import qualified Test.QuickCheck.Random as QCR

sampleOfLength ::
    (Gen a) ->
    Int ->
    IO [a]
sampleOfLength gen n =
    do
    rnd0 <- QCR.newQCGen
    return [(m r n) | (r,n) <- rnds rnd0 `zip` [0,2..2*n] ]
    where
    m = unGen gen
    rnds rnd = rnd1 : rnds rnd2 
        where (rnd1,rnd2) = R.split rnd

{-| Run the generator with size increased by 1 (useful for avoiding 
    too narrow selection at size 0 - in particular Double "randomly" generates 
    0 with probability 1 at size 0. 
-}
incrSize :: Gen t -> Gen t
incrSize gen = sized (\size -> resize (size + 1) gen)

{-|
    Probability of True is @n/m@.  Precondition: @0<n<m@    
-}
arbitraryBoolRatio :: Int {-^ @n@ -} -> Int {-^ @m@ -} -> Gen Bool 
arbitraryBoolRatio n m | 0 < n && n < m =
    frequency [(n, return True), (m - n, return False)]

    
arbitraryOrder :: (Ord t) => [t] -> Gen [t]
arbitraryOrder elems =
    do
    nums <- mapM (const arbitrary) elems
    return $ permuteBy (nums :: [Int]) elems
    where
    permuteBy nums elems =
        map snd $ List.sort $ zip nums elems

{-|
  Have a fairly long and hairy sequence of elements of increasing complexity
  pre-generated and fixed and then pick from it randomly.
  This deals with the problem that the random generation takes a long time
  when the elements' construction is expensive, eg when functions are built
  using a fairly large sequence of multiplications and additions.
-}
fixedRandSeq ::
    (Int -> Int) -> Gen a -> [a]
fixedRandSeq fixedRandSeqQuantityOfSize gen =
    aux 0 0
    where
    aux prevQuantity size 
        = newSeqPortion ++ (aux currQuantity (size + 1))
        where
        newSeqPortion 
            =
            take (currQuantity - prevQuantity) $ 
                map (\g -> unGen gen g size) 
                    randomGens
        currQuantity = fixedRandSeqQuantityOfSize size
    randomGens 
        = map snd $ drop 13 $ iterate (R.next . snd) (0,g)
    g = QCR.mkQCGen 754657854089 -- no magic, just bashed at the keyboard at random

class ArbitraryWithParam t param
    where
    arbitraryWithParam :: param -> Gen t

instance 
    (ArbitraryWithParam t param, 
     Show t, 
     Testable prop) 
    => 
    Testable (param, t -> prop)
    where
    property (param, fn) =
        forAll (arbitraryWithParam param) fn
        



