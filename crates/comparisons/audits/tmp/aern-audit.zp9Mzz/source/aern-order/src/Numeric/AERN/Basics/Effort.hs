{-|
    Module      :  Numeric.AERN.Basics.Effort
    Description :  indicating computational effort for approximate computations
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Indicating computational effort for approximate computations.
-}
module Numeric.AERN.Basics.Effort where

import Numeric.AERN.Misc.List

import Test.QuickCheck

class
    (Show t, Arbitrary t) 
    => 
    EffortIndicator t 
    where
    {-| get a range of independent increments to a given effort indicator (may be empty) -}
    effortIncrementVariants :: t -> [t]
    {-| repeat the increment present in the given pair for the larger effort indicator -}
    effortRepeatIncrement :: (t,t) -> t
    {-| get an increasing sequence of effort indicators of the same type 
        as long as possible, ideally infinite -}
    effortIncrementSequence :: t -> [t]
    {-| a combination of two effort indicators, a supremum in the strength order -}
    effortCombine :: t -> t -> t

newtype Int1To5 = Int1To5 { fromInt1To5 :: Int }
newtype Int1To10 = Int1To10 { fromInt1To10 :: Int }
newtype Int1To20 = Int1To20 { fromInt1To20 :: Int }
newtype Int1To50 = Int1To50 { fromInt1To50 :: Int }
newtype Int1To100 = Int1To100 { fromInt1To100 :: Int }
newtype Int1To1000 = Int1To1000 { fromInt1To1000 :: Int }

instance Show Int1To5 where
    show (Int1To5 i) = show i

instance Show Int1To10 where
    show (Int1To10 i) = show i

instance Show Int1To20 where
    show (Int1To20 i) = show i

instance Show Int1To50 where
    show (Int1To50 i) = show i

instance Show Int1To100 where
    show (Int1To100 i) = show i

instance Show Int1To1000 where
    show (Int1To1000 i) = show i

instance Arbitrary Int1To5 where
    arbitrary =
        do
        i <- choose (1, 5)
        return $ Int1To5 i

instance Arbitrary Int1To10 where
    arbitrary =
        do
        i <- choose (1, 10)
        return $ Int1To10 i

instance Arbitrary Int1To20 where
    arbitrary =
        do
        i <- choose (1, 20)
        return $ Int1To20 i

instance Arbitrary Int1To50 where
    arbitrary =
        do
        i <- choose (1, 50)
        return $ Int1To50 i

instance Arbitrary Int1To100 where
    arbitrary =
        do
        i <- choose (1, 100)
        return $ Int1To100 i

instance Arbitrary Int1To1000 where
    arbitrary =
        do
        i <- choose (1, 1000)
        return $ Int1To1000 i

instance EffortIndicator Int where
    effortIncrementVariants i = [i + 1]
    effortRepeatIncrement (i1, i2) = i2 + (i2 - i1)
    effortIncrementSequence i =
        map (i +) $ fibs12
        where
        fibs12 = scanl (+) 1 (1:fibs12)
    effortCombine = max

instance EffortIndicator Int1To5 where
    effortIncrementVariants (Int1To5 i) = [Int1To5 $ i + 1]
    effortRepeatIncrement (Int1To5 i1, Int1To5 i2) = Int1To5 $ i2 + (i2 - i1)
    effortIncrementSequence (Int1To5 i) =
        map Int1To5 $ map (i +) $ [1..]
    effortCombine (Int1To5 i1) (Int1To5 i2) = Int1To5 $ max i1 i2

instance EffortIndicator Int1To10 where
    effortIncrementVariants (Int1To10 i) = [Int1To10 $ i + 1]
    effortRepeatIncrement (Int1To10 i1, Int1To10 i2) = Int1To10 $ i2 + (i2 - i1)
    effortIncrementSequence (Int1To10 i) =
        map Int1To10 $ map (i +) $ [1..]
    effortCombine (Int1To10 i1) (Int1To10 i2) = Int1To10 $ max i1 i2

instance EffortIndicator Int1To20 where
    effortIncrementVariants (Int1To20 i) = [Int1To20 $ i + 2]
    effortRepeatIncrement (Int1To20 i1, Int1To20 i2) = Int1To20 $ i2 + (i2 - i1)
    effortIncrementSequence (Int1To20 i) =
        map Int1To20 $ map (i +) $ [2,4..]
    effortCombine (Int1To20 i1) (Int1To20 i2) = Int1To20 $ max i1 i2

instance EffortIndicator Int1To50 where
    effortIncrementVariants (Int1To50 i) = [Int1To50 $ i + 5]
    effortRepeatIncrement (Int1To50 i1, Int1To50 i2) = Int1To50 $ i2 + (i2 - i1)
    effortIncrementSequence (Int1To50 i) =
        map Int1To50 $ map (i +) $ [3,6..]
    effortCombine (Int1To50 i1) (Int1To50 i2) = Int1To50 $ max i1 i2

instance EffortIndicator Int1To100 where
    effortIncrementVariants (Int1To100 i) = [Int1To100 $ i + 7]
    effortRepeatIncrement (Int1To100 i1, Int1To100 i2) = Int1To100 $ i2 + (i2 - i1)
    effortIncrementSequence (Int1To100 i) =
        map Int1To100 $ map (i +) $ fibs12
        where
        fibs12 = scanl (+) 1 (1:fibs12)
    effortCombine (Int1To100 i1) (Int1To100 i2) = Int1To100 $ max i1 i2

instance EffortIndicator Int1To1000 where
    effortIncrementVariants (Int1To1000 i) = [Int1To1000 $ i + 13]
    effortRepeatIncrement (Int1To1000 i1, Int1To1000 i2) = Int1To1000 $ i2 + (i2 - i1)
    effortIncrementSequence (Int1To1000 i) =
        map Int1To1000 $ map (i +) $ fibs12
        where
        fibs12 = scanl (+) 1 (1:fibs12)
    effortCombine (Int1To1000 i1) (Int1To1000 i2) = Int1To1000 $ max i1 i2

instance EffortIndicator () where
    effortIncrementVariants _ = []
    effortRepeatIncrement _ = ()
    effortIncrementSequence i = []
    effortCombine _ _ = ()

instance (EffortIndicator t1, EffortIndicator t2) => EffortIndicator (t1, t2)
    where
    effortIncrementVariants (i1, i2) = 
        i1Variants ++ i2Variants
        where
        i1Variants =
            map (\i -> (i, i2)) (effortIncrementVariants i1)
        i2Variants =
            map (\i -> (i1, i)) (effortIncrementVariants i2)
    effortRepeatIncrement ((i1, i2), (j1, j2)) = 
        (effortRepeatIncrement (i1, j1), effortRepeatIncrement (i2, j2)) 
    effortIncrementSequence (i1, i2) =
        case (effortIncrementSequence i1, effortIncrementSequence i2) of
            ([], []) -> []
            (s1, []) -> map (\e -> (e, i2)) s1 
            ([], s2) -> map (\e -> (i1, e)) s2
            (s1, s2) -> zipFill s1 s2
    effortCombine (i1, i2) (j1, j2) = (effortCombine i1 j1, effortCombine i2 j2)
        
instance (EffortIndicator t1, EffortIndicator t2, EffortIndicator t3) => EffortIndicator (t1, t2, t3)
    where
    effortIncrementVariants (i1, i2, i3) = 
        i1Variants ++ i2Variants ++ i3Variants
        where
        i1Variants =
            map (\i -> (i, i2, i3)) (effortIncrementVariants i1)
        i2Variants =
            map (\i -> (i1, i, i3)) (effortIncrementVariants i2)
        i3Variants =
            map (\i -> (i1, i2, i)) (effortIncrementVariants i3)
    effortRepeatIncrement ((i1, i2, i3), (j1, j2, j3)) = 
        (effortRepeatIncrement (i1, j1), 
         effortRepeatIncrement (i2, j2), 
         effortRepeatIncrement (i3, j3)) 
    effortIncrementSequence (i1, i2, i3) =
        case (effortIncrementSequence i1, effortIncrementSequence i2, effortIncrementSequence i3) of
            ([], [], []) -> []
            (s1, [], []) -> map (\e -> (e, i2, i3)) s1 
            ([], s2, []) -> map (\e -> (i1, e, i3)) s2
            ([], [], s3) -> map (\e -> (i1, i2, e)) s3
            (s1, s2, []) -> map (\(e1,e2) -> (e1, e2, i3)) $ zipFill s1 s2 
            (s1, [], s3) -> map (\(e1,e3) -> (e1, i2, e3)) $ zipFill s1 s3 
            ([], s2, s3) -> map (\(e2,e3) -> (i1, e2, e3)) $ zipFill s2 s3 
            (s1, s2, s3) -> zipFill3 s1 s2 s3
    effortCombine (i1, i2, i3) (j1, j2, j3) = 
        (effortCombine i1 j1, effortCombine i2 j2, effortCombine i3 j3)
        
instance 
    (EffortIndicator t1, EffortIndicator t2, EffortIndicator t3, EffortIndicator t4)
    => 
    EffortIndicator (t1, t2, t3, t4)
    where
    effortIncrementVariants (i1, i2, i3, i4) =
        map regroup $
            effortIncrementVariants ((i1,i2),(i3,i4))
        where
        regroup ((a,b),(c,d)) = (a,b,c,d) 
    effortRepeatIncrement ((i1, i2, i3, i4), (j1, j2, j3, j4)) = 
        (effortRepeatIncrement (i1, j1), 
         effortRepeatIncrement (i2, j2), 
         effortRepeatIncrement (i3, j3),
         effortRepeatIncrement (i4, j4)) 
    effortIncrementSequence (i1, i2, i3, i4) =
        map regroup $
            effortIncrementSequence ((i1,i2),(i3,i4))
        where
        regroup ((a,b),(c,d)) = (a,b,c,d) 
    effortCombine (i1, i2, i3, i4) (j1, j2, j3, j4) = 
        (effortCombine i1 j1, effortCombine i2 j2, effortCombine i3 j3, effortCombine i4 j4)
        