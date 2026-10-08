{-# INCLUDE <mpfr.h> #-}
{-# INCLUDE <hsmpfr.h> #-}

module Data.Number.Ball (Ball(..), makeA, make, 
             normalizeBall,
             lower, upper, lower_, upper_,
             sgnLower, sgnUpper,
             width, compareB,
             below, contains, 
             intersectA, intersect,
             add, sub, neg, absB, mul, div, sqrt, exp, log,
             maxB, minB,
             fromDyadic, fromString, fromInt, fromWord )
where

import qualified Data.Number.Dyadic as D
import Data.Order

import Prelude hiding (div, sqrt, exp, log)
import Data.Word(Word)

-- | Ball represents a closed interval @[center-radius, center+radius] @
data Ball = Ball { center :: !D.Dyadic, -- ^ center of the ball
                   radius :: !D.Dyadic -- ^ radius of the ball 
                 }
{-
instance Show Ball where
    show b@(Ball c r) = "\ncenter = " ++ D.toString dc c ++ "\n" ++ "radius = " ++ 
                        D.toString dr r
                            where dc = min 60 $ (decimalPrec . correctDigits) b
                                  dr = D.getPrec r
  -}
instance Show Ball where
    show b@(Ball c r) = s ++ "[" ++ show go ++ "]"
                        where go' = decimalPrec . correctDigits $ b
                              go  = let r' = D.getExp r in 
                                    if go' == 0 && r' < 0 then decimalPrec . fromIntegral . negate . succ $ r' else go'
                              s = D.toString go c

-- | Precision of ball\'s radius. 
radPrec :: D.Precision
radPrec = 32

-- | Create epsilon neighbourhood of d according to the number of accurate digits of d.
-- Specifically return m * 2 ^ (e - p - 1) 
createEpsilon      :: Int -- ^ m
                      -> D.Dyadic -- ^ dyadic with magnitude e and precision p
                      -> D.Dyadic
createEpsilon i d = D.int2i D.Zero radPrec i (if d == 0 then 0 else D.getExp d - (fromIntegral $ D.getPrec d) - 1)

-- | If first arugment \/= 0 then add to second argument the epsilon of the third.
addEpsilon       :: Int -- ^ indicates whether correction is necessary 
                    -> D.Dyadic -- ^ dyadic to be corrected
                    -> D.Dyadic -- ^ dyadic which indicates the magnitude of correction
                    -> D.Dyadic
addEpsilon e d x = if e /= 0 then D.add D.Up (D.getPrec d) d (createEpsilon 1 x)
                   else d

-- | Make a ball from endpoints
makeA           :: D.Precision -- ^ desired precision of the center
                  -> D.Dyadic -- ^ left endpoint
                  -> D.Dyadic -- ^ right endpoint
                  -> Ball 
makeA p d1 d2 = Ball cen rad
               where (c,e) = D.add_ D.Near p d1 d2
                     cen   = D.div2w D.Near p c 1
                     r     = D.sub D.Up radPrec d2 d1
                     r'    = D.div2w D.Up radPrec r 1
                     rad   = addEpsilon e r' c


-- | Make a ball from endpoints so that no precision is lost. 
make       :: D.Dyadic -- ^ left endpoint
              -> D.Dyadic -- ^ right endpoint
              -> Ball 
make d1 d2 = makeA (D.addPrec d1 d2) d1 d2
 
-- | Normalize the given ball's center to the specified precision.
-- Resulting ball might be larger.
normalizeBall              :: D.Precision -> Ball -> Ball
normalizeBall p (Ball c r) = Ball c' r'
                             where (c',e) = D.set_ D.Near p c
                                   r''    = D.set D.Up radPrec r
                                   r'     = addEpsilon e r'' c'

-- | MakeA a ball from dyadic. Radius is 0 if desired precision is not smaller
-- than precision of dyadic.
fromDyadic      :: D.Precision -> D.Dyadic -> Ball
fromDyadic p d = Ball c r
                     where (c, e) = D.set_ D.Near p d
                           r'     = D.fromWord D.Up radPrec 0
                           r      = addEpsilon e r' c

-- | Similar to fromDyadic.
fromInt     :: D.Precision -> Int -> Ball
fromInt p d = Ball c r
                  where (c, e) = D.fromInt_ D.Near p d
                        r'     = D.fromWord D.Up radPrec 0
                        r      = addEpsilon e r' c

-- | Similar to fromInt.
fromWord   :: D.Precision -> Word -> Ball
fromWord p = fromInt p . fromIntegral

-- | Lower endpoint of the ball rounded down to specified precision.
lower              :: D.Precision -> Ball -> D.Dyadic 
lower p (Ball c r) = D.sub D.Down p c r

-- | Upper endpoint of the ball rounded up to specified precision.
upper              :: D.Precision -> Ball -> D.Dyadic 
upper p (Ball c r) = D.add D.Up p c r

-- | Lower endpoint with precision of the center
lower_              :: Ball -> D.Dyadic
lower_ b@(Ball c _) = lower (D.getPrec c) b

-- | Upper endpoint with precision of the center
upper_              :: Ball -> D.Dyadic
upper_ b@(Ball c _) = upper (D.getPrec c) b

-- | Sign of lower endpoint of the ball. This should be faster than using @ signum (center b - radius b) @ 
sgnLower            :: Ball -> Int
sgnLower (Ball c r) = case compare c r of
                        LT -> -1
                        EQ -> 0
                        _  -> 1

-- | Analogous to sgnLower.
sgnUpper            :: Ball -> Int
sgnUpper (Ball c r) = case compare (D.neg D.Near (D.getPrec r) r) c of 
                        LT -> 1
                        EQ -> 0
                        _  -> -1

-- | Upper bound on the width of the ball. @ 2 * radius b @ rounded up.
width            :: Ball -> D.Dyadic
width (Ball _ r) = D.mul2w D.Up radPrec r 1

-- | Check if second ball is included in the first
below     :: Ball -> Ball -> Bool
below a b = lower_ a <= lower_ b && upper_ a >= upper_ b

-- | Check if dyadic is element of the ball.
contains     :: Ball -> D.Dyadic -> Bool
contains b d = lower_ b <= d && upper_ b >= d

-- | Returns an intersection of two balls. If balls are disjoint then computation fails with fail.
intersectA         :: MonadFail m => D.Precision -- ^ precision of the resulting ball's center
                      -> Ball -> Ball -> m Ball
intersectA p b1 b2 | l <= u = return $ makeA p l u
                   | otherwise = fail "cannot intersect disjoint intervals"
                     where l = D.maxD D.Down p (lower p b1) (lower p b2)
                           u = D.minD D.Up p (upper p b1) (upper p b2)

-- | Intersection of two balls exactly (no precision is lost).
intersect                              :: MonadFail m => Ball -> Ball -> m Ball
intersect b1@(Ball c _) b2@(Ball c' _) = intersectA (D.addPrec c c') b1 b2

-- | Addition of two balls.
--
-- - @ center = center a + center b @
-- 
-- - @ radius = radius a + radius b @
--
-- Rounding errors are added to the radius.
add :: D.Precision -> Ball -> Ball -> Ball
add p (Ball c r) (Ball c' r') = Ball cen rad
                                where (cen, e) = D.add_ D.Near p c c'
                                      rad      = D.add D.Up radPrec r' (addEpsilon e r cen)

-- | Subtraction of two balls.
--
-- - @ center = center a - center b @
-- 
-- - @ radius = radius a + radius b @
--
-- Rounding errors are added to the radius.
sub :: D.Precision -> Ball -> Ball -> Ball
sub p (Ball c r) (Ball c' r') = Ball cen rad
                                where (cen, e) = D.sub_ D.Near p c c'
                                      rad      = D.add D.Up radPrec r' (addEpsilon e r cen)
-- | Negation of the ball. 
--
-- - center = - center b rounded to specified precision.
-- 
-- - radius is only modified for the rounding error.
neg              :: D.Precision -> Ball -> Ball
neg p (Ball c r) = Ball c' r'
                   where (c',e) = D.neg_ D.Near p c
                         r'     = addEpsilon e r c'

absB     :: D.Precision -> Ball -> Ball
absB p b = if lb > 0 then normalizeBall p b
             else if ub < 0 then neg p b
                    else makeA p 0 (max (negate lb) (ub))
           where lb = lower_ b
                 ub = upper_ b


-- | Multiplication of two balls. (centers of both balls are assumed positive)
--
-- - If none of the balls contains 0 then
--
-- @ center = center a * center b + radius a * radius b @
--
-- @ radius = center a * radius b + radius a * center b @
-- 
-- - If one of the operands (left) contains 0
-- 
-- @ center = center a * upper b @
--
-- @ radius = radius a * upper b @
-- 
-- - If both of the balls contain 0
--
-- @ lower =  min ((lower a) * (upper b)) ((lower b) * (upper a)) @
-- 
-- @ upper =  max ((lower a) * (lower b)) ((upper b) * (upper a)) @
--
-- Rounding errors are added to the radius.
mul         :: D.Precision -> Ball -> Ball -> Ball
mul p b1 b2 = if D.sgn (center b1) * D.sgn (center b2) < 0 then neg p ret else ret
               where ret = mul' (absB p b1) (absB p b2)
                     mul' b b' = case (sgnLower b, sgnLower b') of 
                                   (1, 1) -> nonzero b b'
                                   (1, _) -> leftzero b' b
                                   (_, 1) -> leftzero b b'
                                   _      -> bothzero b b'
                     nonzero (Ball c r) (Ball c' r') = Ball cen rad 
                                                       where r'' = D.fma D.Up radPrec c r' (D.mul D.Up radPrec c' r)                                      
                                                             cr  = D.mul D.Near (2 * radPrec) r r' 
                                                             (cen, e) = D.fma_ D.Near p c c' cr
                                                             rad      = addEpsilon e r'' cen
                     leftzero (Ball c r) b = Ball cen rad
                                             where (cen, e) = D.mul_ D.Near p c up
                                                   rad      = addEpsilon e (D.mul D.Up radPrec r up) cen
                                                   up       = upper p b
                     bothzero b b' = makeA p l u
                                      where l  = D.minD D.Down p l1 l2
                                            u  = D.maxD D.Up p u1 u2
                                            l1 = D.mul D.Down p (lower p b) (upper p b')
                                            l2 = D.mul D.Down p (upper p b) (lower p b')
                                            u1 = D.mul D.Up p (lower p b) (lower p b')
                                            u2 = D.mul D.Up p (upper p b) (upper p b')

-- | Division of two balls
-- 
-- - If radius is \"large\" then divide endpoints and makeA a ball from them.
--
-- - If radius is \"small\" then division can be optimized
--
-- - @ center = center a / center b @
-- 
-- - @ (radius = radius a * center b + center a * radius b) / (center b * center b) + 2 * 2 ^ (e1 - e2 - p)@ 
--  where @ p @ is precision of the result, @ e1 = getExp c1, e2 = getExp c2 @. This way the resulting interval is 
--  guaranteed to be correct.
--
-- Rounding errors are added to the radius.
--
-- If divisor ball contains zero compuatation fails with fail.
div         :: (MonadFail m) => D.Precision -> Ball -> Ball -> m Ball
div p b1 b2 = if sgnLower b2 > 0 then return (div' b1 b2)
              else if sgnUpper b2 < 0 then return (neg p (div' b1 (neg p b2)))
                   else fail "Division by interval containing zero"
              where div' b b' = if smallRad b && smallRad b' then divSmall b b'
                                else divLarge b b'
                    -- radius is small if (a) it is 0 or if number of correct digits is 
                    -- more than half of the end precision
                    smallRad (Ball c r) = D.sgn r == 0 || 2 * (D.getExp c - D.getExp r) > fromIntegral p + 2
                    divSmall (Ball c r) (Ball c' r') = Ball cen rad
                                                       where cen = D.div D.Near p c c'
                                                             r'' = D.fma D.Up radPrec c r' (D.mul D.Up radPrec c' r)
                                                             (bsq, e) = D.sqr_ D.Down radPrec c'
                                                             bsq' = if e == 0 then D.nextBelow bsq else bsq
                                                             r''' = D.div D.Up radPrec r'' bsq'
                                                             -- now r''' is at most 2 * 2 ^ (e1 - e2 - p) too small
                                                             rad = D.add D.Up radPrec r''' (createEpsilon 3 cen)
                    divLarge b b' = makeA p l u
                                    where l = D.div D.Down p l1 (if D.sgn l1 < 0 then l2 else u2)
                                          u = D.div D.Up p u1 (if D.sgn u1 < 0 then u2 else l2)
                                          l1 = lower p b
                                          l2 = lower p b'
                                          u1 = upper p b
                                          u2 = upper p b'

-- | Square root of a ball. If interval contains 0 then computation fails.
sqrt                :: MonadFail m => D.Precision -> Ball -> m Ball
sqrt p b@(Ball c r) = if lower_ b < 0 then fail "Sqrt of a interval containing negative numbers"
                        else if radSmall then return sqrt_small
                               else return sqrt_large
                      where sqrt_large = makeA p l u
                              where l = D.sqrt D.Down p (D.sub D.Down p c r)
                                    u = D.sqrt D.Up p (D.add D.Up p c r)
                            radSmall = D.sgn c /= 0 && (D.sgn r == 0 || D.getExp c `quot` 2 - D.getExp r > fromIntegral p)
                            sqrt_small = Ball cen rad
                              where (cen,e) = D.sqrt_ D.Near p c
                                    rad'    = D.div D.Up radPrec r cen
                                    rad     = addEpsilon e rad' cen

-- | @ e ^ b @
exp              :: D.Precision -> Ball -> Ball
exp p (Ball c r) = makeA p l u
                   where l = D.exp D.Down p (D.add D.Down p c r)
                         u = D.exp D.Up p (D.add D.Up p c r)

-- | Natural logarithm of a ball. If interval contains 0 then computation fails.
log                :: MonadFail m => D.Precision -> Ball -> m Ball
log p b@(Ball c r) = if lower_ b <= 0 then fail "Domain of log is R+"
                       else return (makeA p l u)
                     where l = D.log D.Down p (D.add D.Down p c r)
                           u = D.log D.Up p (D.add D.Up p c r)

-- | Compare two balls.
--
-- - if upper a < lower b then Less
--
-- - if upper b < lower a then Greater 
--
-- - otherwise balls are incomparable.
compareB      :: Ball -> Ball -> POrdering
compareB b b' = if upper_ b < lower_ b' then Less
                else if lower_ b > upper_ b' then Greater
                else Incomparable

-- | Maximum of two balls, meaning:
--
-- - lower = max (lower a) (lower b) rounded down
--
-- - upper = max (upper a) (upper b) rounded up
maxB        :: D.Precision -> Ball -> Ball -> Ball
maxB p b b' = makeA p l u
              where l = D.maxD D.Down p (lower p b) (lower p b')
                    u = D.maxD D.Up p (upper p b) (upper p b')

-- | Analogous to maxB.
minB        :: D.Precision -> Ball -> Ball -> Ball
minB p b b' = makeA p l u
              where l = D.minD D.Down p (lower p b) (lower p b')
                    u = D.minD D.Up p (upper p b) (upper p b')

-- | Similar to fromDyadic.
fromString     :: D.Precision -> String -> Ball
fromString p s = Ball cen rad
                   where cen = D.fromString s p 10
                         rad = createEpsilon 1 cen

decimalPrec :: Word -> Word
decimalPrec d = floor (fromIntegral d * logBase 10 2 :: Double)

correctDigits :: Ball -> Word
correctDigits (Ball c r) =  case compare D.zero r of
                             EQ -> (fromIntegral . D.getPrec) c
                             LT -> let cd = D.getExp c - D.getExp r in fromIntegral (max 0 cd)
                             _  -> error "Ball.correctDigits: radius should be a nonnegative, non-degenerate number"
