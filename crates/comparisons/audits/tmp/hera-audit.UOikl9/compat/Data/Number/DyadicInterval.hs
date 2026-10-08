{-# LANGUAGE BangPatterns #-}
module Data.Number.DyadicInterval(Interval,
                      fromBallA, fromBall, make, makeA,
                      below, contains, includes, 
                      intersectA, intersect, 
                      neg, add, mul, sub, div, sqrt, exp, log,
                      compareI, maxI, minI,
                      center, radius, lower, upper, width,
                      fromDyadic, fromString, fromInt, fromWord,
                      toString)
 where

import qualified Data.Number.Ball as B
import qualified Data.Number.Dyadic as D

import Data.Order

import Prelude hiding (div, sqrt, exp, log)
import Data.Word(Word)

import Control.Monad
    
-- | A wrapper around Ball allowing the results of operations like division
-- by interval containing zero to be represented and do not cause errors.
--
-- Nothing represents undefined interval.
type Interval = Maybe B.Ball

data Inclusion = Above | Below | NoInclusion

-- | Make an interval from a ball and normalize it to specified precision.
fromBallA    :: D.Precision -> B.Ball -> Interval
fromBallA p b = Just (B.normalizeBall p b)

-- | Just make an interval from a ball.
fromBall :: B.Ball -> Interval
fromBall b = Just b

-- | Make an interval from two endpoints.
makeA       :: D.Precision -- ^ precision of the interval's center
               -> D.Dyadic -- ^ left endpoint
               -> D.Dyadic -- ^ right endpoint
               -> Interval
makeA p l u = Just (B.makeA p l u)

-- | Make an interval from two endpoints so that no precision is lost.
make     :: D.Dyadic -> D.Dyadic -> Interval
make l u = Just (B.make l u)

-- | Checks if second interval is inside the first. _|_ is above all.
below                            :: Interval -> Interval -> Bool
below (Just b) (Just b') = B.below b b'
below Nothing _                = True
below _ Nothing                = False

-- | Checks if interval contains dyadic. _|_ contains everything.
contains                :: Interval -> D.Dyadic -> Bool
contains Nothing _    = True
contains (Just b) d = B.contains b d

-- | Returns Below if second interval is inside first, Above if converse, NoInclusion otherwise.
includes                   :: Interval -> Interval -> Inclusion
includes i i' | below i i' = Below
              | below i' i = Above
              | otherwise  = NoInclusion

-- | Return the intersection of two intervals. The resulting interval's center has specified precision.
-- 
--  If one of the intervals is _|_ then just return the other (even if it is _|_).
intersectA                      :: D.Precision -> Interval -> Interval -> Interval
intersectA p (Just b) (Just b') = B.intersectA p b b'
intersectA _ Nothing i          = i
intersectA _ i Nothing          = i

-- | Return the intersection of two intervals so that no precision is lost.
intersect                    :: Interval -> Interval -> Interval
intersect (Just b) (Just b') = B.intersect b b'
intersect Nothing x          = x
intersect x Nothing          = x

-- | Negate the interval. neg _|_ = _|_.
neg     :: D.Precision -> Interval -> Interval
neg p i = do b <- i
             return $! B.neg p b

{-# INLINE wrap #-}
wrap :: (D.Precision -> B.Ball -> B.Ball -> B.Ball)
        -> D.Precision -> Interval -> Interval -> Interval
wrap f p i i' = do !b <- i
                   !b' <- i'
                   return $! f p b b'

-- | Addition. If one of the arguments is _|_, so is the result.
add :: D.Precision -> Interval -> Interval -> Interval
add = wrap B.add

-- | Multiplication. If one of the arguments is _|_, so is the result
mul :: D.Precision -> Interval -> Interval -> Interval
mul = wrap B.mul

-- | Subtraction. If one of the arguments is _|_, so is the result
sub :: D.Precision -> Interval -> Interval -> Interval
sub = wrap B.sub

-- | Division. If one of the arguments is _|_ or divisor contains 0 then result is _|_.
div          :: D.Precision -> Interval -> Interval -> Interval
div p i i' = do  !b <- i
                 !b' <- i'
                 B.div p b b'
-- | Square root. If one argument is _|_ or interval contains 0 then result is _|_.
sqrt     :: D.Precision -> Interval -> Interval
sqrt p i = do !b <- i
              B.sqrt p b

-- | Natural logarithm. If one argument is _|_ or interval contains 0 then result is _|_.
log     :: D.Precision -> Interval -> Interval
log p i = do !b <- i
             B.log p b

-- | @ e ^ i @ If argument is _|_ so is the result.
exp     :: D.Precision -> Interval -> Interval
exp p i = do !b <- i
             return $! (B.exp p b)

-- | Compare two intervals. If one of them is _|_ the result is incomparable, 
-- otherwise result is comparison of balls.
compareI                      :: Interval -> Interval -> POrdering
compareI (Just b) (Just b')   = B.compareB b b'
compareI _ _                  = Incomparable

-- | Maximum of intervals. If one interval is _|_ so is the result.
maxI :: D.Precision -> Interval -> Interval -> Interval
maxI = wrap B.maxB

-- | Similar to maxI.
minI :: D.Precision -> Interval -> Interval -> Interval
minI = wrap B.minB

-- | Center of interval. Center on _|_ will result in fail.
center                     :: (MonadFail m) => Interval -> m D.Dyadic
center Nothing             = fail "center of _|_ is not defined"
center (Just (B.Ball c _)) = return c

-- | Radius of interval. Radius on _|_ will result in fail.
radius                     :: (MonadFail m) => Interval -> m D.Dyadic
radius Nothing             = fail "radius of _|_ is infinity"
radius (Just (B.Ball _ r)) = return r

-- | Lower endpoint of interval with precision of the center. 
-- Lower on _|_ will result in fail.
lower              :: (MonadFail m) => Interval -> m D.Dyadic
lower Nothing  = fail "lower bound of _|_ is -infinity"
lower (Just b) = return (B.lower_ b)

-- | Upper endpoint of interval with precision of the center. 
-- Upper on _|_ will result in fail.
upper              :: (MonadFail m) => Interval -> m D.Dyadic
upper Nothing  = fail "upper bound of _|_ is +infinity"
upper (Just b) = return (B.upper_ b)


-- | Width of the interval. Widht on _|_ will result in fail.
width          :: (MonadFail m) => Interval -> m D.Dyadic
width Nothing  = fail "width of _|_ is infinity"
width (Just b) = return (B.width b)

fromDyadic     :: D.Precision -> D.Dyadic -> Interval
fromDyadic p d = Just (B.fromDyadic p d)

fromString     :: D.Precision -> String -> Interval
fromString p s = Just (B.fromString p s)

fromInt     :: D.Precision -> Int -> Interval
fromInt p d = Just (B.fromInt p d)

fromWord     :: D.Precision -> Word -> Interval
fromWord p d = Just (B.fromWord p d)

toString          :: Interval -> String
toString Nothing  = "_|_"
toString (Just b) = show b
