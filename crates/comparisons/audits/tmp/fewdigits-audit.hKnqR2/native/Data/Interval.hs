{-# OPTIONS_GHC -fglasgow-exts #-}
module Data.Interval 
                (Interval(Interval), point, isIn,
                 intervalNegate, intervalAbs,
                 intervalTranslate, intervalScale,
                 intervalRecip, intervalPower,
                 intervalPlus, intervalMult,
                 bound, union, unionList ) where

import Test.QuickCheck

newtype Interval a = Interval (a,a)
                     deriving (Show, Eq)

point x = Interval (x,x)

x `isIn` (Interval (a,b)) = a <= x && x <= b

intervalNegate (Interval (lb, ub)) = Interval (-ub, -lb)

intervalAbs x@(Interval (lb, ub)) | 0 < lb = x
                                  | ub < 0 = intervalNegate x
                                  | otherwise = Interval (0, max ub (-lb))

intervalTranslate a (Interval (lb, ub)) = Interval (a+lb, a+ub)

intervalScale 0 _ = Interval (0, 0)
intervalScale a (Interval (lb, ub)) | 0 < a = Interval (a*lb, a*ub)
                                    | a < 0 = Interval (a*ub, a*lb)

intervalRecip (Interval (lb, ub)) | 0 < lb || ub < 0 = Just $ Interval (recip ub, recip lb)
                                  | otherwise = Nothing

intervalPower (Interval (lb, ub)) n | odd n = Interval (lb^n, ub^n)
                                    | 0 < lb = Interval (lb^n, ub^n)
                                    | ub < 0 = Interval (ub^n, lb^n)
                                    | otherwise = Interval (0, max (ub^n) (lb^n))


intervalPlus (Interval (xlb, xub)) (Interval (ylb, yub)) = Interval (xlb+ylb, xub+yub)

intervalMult x@(Interval (xlb, xub)) y@(Interval (ylb, yub))
  | xub < 0 = intervalNegate (intervalMult (intervalNegate x) y)
  | yub < 0 = intervalMult y x
  | 0 < xlb && 0 < ylb = Interval (xlb*ylb, xub*yub)
  | 0 < xlb = Interval (xub*ylb, xub*yub)
  | otherwise = Interval (min (xlb*yub) (xub*ylb), max (xlb*ylb) (xub*yub))

intersect (Interval (xlb, xub)) (Interval (ylb, yub)) = 
  Interval (max xlb ylb, min xub yub)

bound (Interval (lb, ub)) = max ub (-lb)

union (Interval (xlb, xub)) (Interval (ylb, yub)) =
 Interval (min xlb ylb, max xub yub)

unionList :: (Ord a) => [Interval a] -> Interval a
unionList = foldr1 union

instance (Arbitrary a, Ord a) => Arbitrary (Interval a) where
 arbitrary = do x <- arbitrary
                y <- arbitrary
                if x < y then return $ Interval (x,y)
                         else return $ Interval (y,x)
 coarbitrary (Interval (x,y)) = coarbitrary x .
                                coarbitrary y